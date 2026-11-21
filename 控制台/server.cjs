const http = require('node:http');
const net = require('node:net');
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const {ControlRoom}=require('./control-room.cjs');
const {shareAddresses,deviceKey}=require('./network.cjs');
const { execFile } = require('node:child_process');
const {ExperimentTasks}=require('./experiments.cjs');
const {Trajectory}=require('./trajectory.cjs');
const robotAddress=require('./robot-address.cjs');
const gazeboView=require('./gazebo-view.cjs');
const {MazeMap,distance}=require('../迷宫/maze-map.cjs');
const {MazeScanStream}=require('../迷宫/scan-stream.cjs');
const {MazeMission}=require('../迷宫/mission.cjs');
const {RegisteredMap}=require('../迷宫/registered-map.cjs');

const COMMANDS = new Set(['status','start','pause','resume','stop','manual','auto','w','s','a','d','drive','zero','heartbeat']);
const MANUAL_ERRORS={odometry_unavailable:'定位未就绪，请先准备传感器',lidar_unavailable:'雷达未就绪，请先准备传感器',feedback_unavailable:'底盘反馈超时',chassis_fault:'底盘报告故障',chassis_mode:'底盘尚未处于网络/CAN 模式',remote_takeover:'遥控器开关正在接管底盘',not_stopped:'请先停车，再恢复手动驾驶',heartbeat_missing:'控制心跳已过期，请重新启用键盘驾驶'};
const experiments = JSON.parse(fs.readFileSync(path.join(__dirname,'experiments.json'),'utf8'));
const availableExperiments = experiments.map(item=>({...item,enabled:item.enabled === true && ['lab1','lab2','lab2-eight','lab3','lab3-xy','maze'].includes(item.id)}));
function createConsole({ port = 8765, tcpPort = 18766, ping = null, sshRunner, leaseMs=1000, cloudStream=null, deferredHardware=false, mazeStreamFactory=fn=>new MazeScanStream(fn) } = {}) {
  const simDir=[path.join(__dirname,'../实验三扩展/仿真'),path.join(__dirname,'../仿真')].find(p=>fs.existsSync(path.join(p,'web/index.html')));
  const sim=simDir?new (require('./sim-bridge.cjs').SimulationBridge)(simDir):null;
  const displays=new Set();
  const simFiles={'/simulation/gazebo/':['web/gazebo.html','text/html; charset=utf-8'],'/simulation/field.js':['web/field.js','text/javascript; charset=utf-8'],'/simulation/native.js':['web/native.js','text/javascript; charset=utf-8'],'/simulation/field.css':['web/field.css','text/css; charset=utf-8']};
  const addresses=Object.values(os.networkInterfaces()).flat().filter(x=>x.family==='IPv4'&&!x.internal).map(x=>x.address);
  const hosts=new Set(['127.0.0.1','localhost',...addresses].map(ip=>`${ip}:${port}`));
  const urls=shareAddresses(robotAddress.address()).map(ip=>`http://${ip}:${port}/hardware/`);
  const localRequest=req=>['127.0.0.1','::1','::ffff:127.0.0.1'].includes(req.socket.remoteAddress)&&['127.0.0.1:'+port,'localhost:'+port].includes(req.headers.host);
  let socket, connected = false, status = {}, statusAt = 0, buffer = '', closing = false;
  let transportStarted=false, linkRequest=null;
  let hardwareLink={phase:'idle',message:'可以连接真机'};
  function startTransport(){
    if(transportStarted||closing)return;
    transportStarted=true;connect();cloudStream?.start();
  }
  function connectHardware(){
    if(connected||linkRequest)return;
    const request=linkRequest=new AbortController();hardwareLink={phase:'connecting',message:'正在建立机器人连接'};
    require('./launch.cjs').ensureRobotLink(request.signal).then(()=>{
      if(linkRequest!==request)return;
      hardwareLink={phase:'connecting',message:'已建立隧道，等待底盘反馈'};
      startTransport();
    },error=>{if(linkRequest===request)hardwareLink={phase:'error',message:error.message};}).finally(()=>{if(linkRequest===request)linkRequest=null;});
  }
  function detachHardware(){
    mazeStream.stop();resetMaze();
    transportStarted=false;linkRequest?.abort();linkRequest=null;clearTimeout(retry);
    connected=false;status={};statusAt=0;cloud={points:[],frame:''};cloudAt=0;trajectory.clear();
    cloudStream?.stop();socket?.destroy();socket=null;require('./launch.cjs').closeRobotLink();
    hardwareLink={phase:'idle',message:'可以连接真机'};
  }
  let manualAck=null;
  const send = command => { if (!connected || !socket?.writable) return false; socket.write(command + '\n'); return true; };
  let cloud={points:[],frame:''},cloudAt=0;
  const trajectory=new Trajectory();
  const registeredMap=new RegisteredMap();
  const mazeMap=new MazeMap();
  const mission=new MazeMission(mazeMap,send);
  let mazeSeen=0,lidarOffset=null,pendingCloud=null;
  function registerPending(){
    if(!pendingCloud||!lidarOffset)return;
    if(Date.now()-cloudAt>500){pendingCloud=null;return;}
    if(registeredMap.accept(pendingCloud,trajectory,status.map_generation,Date.now(),lidarOffset))pendingCloud=null;
  }
  const mazeStream=mazeStreamFactory(sample=>{
    if(!connected)return false;
    const offset=sample.lidarOffset;
    const calibrated=Array.isArray(offset)&&offset.length===3&&offset.every(v=>Number.isFinite(v)&&Math.abs(v)<2);
    if(sample.type==='lidar_mount'){lidarOffset=calibrated?offset:null;return calibrated;}
    if(!connected||!trajectory.pose||Date.now()-trajectory.at>650||!status.feedback_fresh||
       sample.generation!==status.map_generation||sample.frame!==trajectory.frame||
       Math.abs(trajectory.stamp-sample.stamp)>.45||!Array.isArray(sample.pose)||distance(trajectory.pose,sample.pose)>.3||
       Math.abs(Math.atan2(Math.sin(trajectory.pose[2]-sample.pose[2]),Math.cos(trajectory.pose[2]-sample.pose[2])))>.35)return false;
    lidarOffset=calibrated?offset:null;
    if(!status.maze_height_observations)sample.heightHits=[];
    return mazeMap.accept(sample);
  });
  function resetMaze(){
    mazeStream.stop();
    mazeMap.reset();registeredMap.reset();lidarOffset=null;pendingCloud=null;
    if(tasks.prepared==='maze')tasks.prepared=null;
  }
  function mazeUnavailable(){
    const now=Date.now();
    if(!connected||now-statusAt>=700||status.feedback_fresh!==true)return 'feedback';
    if(status.chassis_mode!==1||status.chassis_errors!==0||status.manual_block_reason==='chassis_fault'||
       ![0,2].includes(status.remote_switch)||status.mode==='manual')return 'control';
    if(status.maze_protocol!==2||status.xy_planner!=='navfn_pursuit')return 'controller';
    if(!mazeMap.generation||mazeMap.generation!==status.map_generation)return 'map';
    if(status.manual_block_reason==='lidar_unavailable')return 'lidar';
    if(status.manual_block_reason==='odometry_unavailable'||now-trajectory.at>=600)return 'odometry';
    if(now-mazeMap.at>=850||status.sensors_ready!==true)return 'perception';
    if(status.dwa_ready!==true)return 'planner';
    return '';
  }
  const mazeHealthy=()=>!mazeUnavailable();
  const stopped=()=>connected && Date.now()-statusAt<500 && status.feedback_fresh===true &&
      [status.v,status.w,status.feedback_v,status.feedback_w].every(Number.isFinite)&&
      Math.abs(status.v)<0.001&&Math.abs(status.w)<0.001&&Math.abs(status.feedback_v)<0.02&&Math.abs(status.feedback_w)<0.02&&
      (status.paused || ['FAULT','DONE','IDLE'].includes(status.stage));
  const tasks=new ExperimentTasks({send,run:sshRunner,stopped,
    prepareMaze:()=>{
      if(status.maze_protocol!==2)throw Error('车端迷宫控制程序尚未就绪');
      if(status.chassis_mode!==1||![0,2].includes(status.remote_switch)||status.chassis_errors!==0)
        throw Error('请确认底盘处于 CAN 控制档，且遥控器已释放控制');
      const reuse=status.sensors_ready&&status.dwa_ready&&status.xy_planner==='navfn_pursuit'&&Boolean(status.map_generation);
      if(status.mode==='manual')send('auto');
      mazeMap.reset();registeredMap.reset();pendingCloud=null;mazeSeen=Date.now();mazeStream.start();
      return reuse;
    },
    startMaze:(task,alive,options)=>{
      if(!mazeHealthy()||options.mapId!==mazeMap.id||!stopped())throw Error('请确认本轮地图、迷宫控制器和停车反馈已就绪');
      if(options.hint!=null){
        if(!Array.isArray(options.hint)||options.hint.length!==2)throw Error('搜索区需要前方和左侧两个坐标');
        mazeMap.goals({kind:'hint',id:options.mapId,x:options.hint[0],y:options.hint[1]});
      }
      mission.start(task,alive,Date.now(),status.stage==='FAULT'?status.reason:'');
    },
    ready:experiment=>experiment==='maze'?mazeHealthy():connected&&Date.now()-statusAt<1500&&status.sensors_ready===true&&status.feedback_fresh===true&&(experiment!=='lab3-xy'||status.dwa_enabled!==true||status.dwa_ready===true)});
  const room=new ControlRoom({leaseMs,connected:()=>connected,
    stopped:()=>stopped()&&(!room.barrier||statusAt>room.barrier.at)&&tasks.task?.phase!=='stopping',
    stop:()=>{send('zero');send('pause');tasks.stop();}});
  const sessions=room.sessions;
  function connect() {
    if (closing) return;
    const peer=socket=net.createConnection({host:'127.0.0.1',port:tcpPort});
    socket.on('connect',()=> { connected = true; buffer = '';hardwareLink={phase:'ready',message:'底盘通道已连接'}; room.beginStop('连接已建立，正在确认停车',{release:true});send('status'); });
    socket.on('data', data => {
      if(socket!==peer)return;
      buffer += data.toString();
      if (buffer.length > 1048576) return socket.destroy();
      let index;
      while ((index = buffer.indexOf('\n')) >= 0) {
        const line = buffer.slice(0,index); buffer = buffer.slice(index+1);
        try { const value = JSON.parse(line); if (value && typeof value === 'object' && !Array.isArray(value)) {
          if(value.type==='cloud'){
            cloud={points:Array.isArray(value.points)?value.points.filter(point=>Array.isArray(point)&&point.length===3&&point.every(Number.isFinite)).slice(0,8000):[],frame:String(value.frame||'').replace(/^\//,'').slice(0,100),registered:value.registered===true,stamp:Number.isFinite(value.stamp)?value.stamp:null};cloudAt=Date.now();
            pendingCloud=cloud.registered?cloud:null;registerPending();
          }else if(value.type==='pose'){const epoch=trajectory.epoch;trajectory.add(value);if(epoch!==trajectory.epoch)resetMaze();else registerPending();}
          else if(value.type==='ack'){
            if(value.command==='manual'&&manualAck){const pending=manualAck;manualAck=null;clearTimeout(pending.timer);pending.resolve(value);}
          }
          else{status = {...value,manual_block_message:MANUAL_ERRORS[value.manual_block_reason]||''}; statusAt = Date.now();tasks.observe(status);
            if(mazeMap.generation&&mazeMap.generation!==status.map_generation||registeredMap.generation&&registeredMap.generation!==status.map_generation)resetMaze();}
        } } catch {}
      }
    });
    socket.on('error',()=>{});
    socket.on('close',()=> {if(socket!==peer)return;connected = false;hardwareLink={phase:'error',message:'机器人连接中断，可以重新连接'};room.disconnected();trajectory.clear();mazeStream.stop();resetMaze();if(manualAck){clearTimeout(manualAck.timer);manualAck.resolve({ok:false,reason:'控制连接已断开'});manualAck=null;} if (!closing&&transportStarted) retry = setTimeout(connect,1000); });
  }
  let retry;
  const timer = setInterval(()=> {
    room.tick();
    if(connected&&(Date.now()-mazeSeen<10000||tasks.task?.experiment==='maze'&&(tasks.busy||tasks.prepared==='maze')))mazeStream.start();
    else mazeStream.stop();
    if(mission.run){
      const unavailable=mazeUnavailable();
      mission.tick({status,statusAt,unavailable,healthy:!unavailable,stopped:stopped()});
      if(tasks.task?.phase==='failed')tasks.prepared=null;
    }
  },100);
  const reply = (res, code, data) => { res.writeHead(code, {'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'}); res.end(JSON.stringify(data)); };
  const server = http.createServer(async (req,res)=> {
    if (!hosts.has(req.headers.host)) return reply(res,403,{error:'无效访问地址'});
    const origin=`http://${req.headers.host}`;
    if(req.headers.origin&&req.headers.origin!==origin)return reply(res,403,{error:'拒绝跨站控制'});
    room.tick();
    if (req.method === 'GET') {
      if(req.url.startsWith('/gazebo/')||req.url==='/api/simulation/viewer'){
        if(!localRequest(req))return reply(res,403,{error:'请在控制台主机查看 Gazebo 原生窗口'});
        return req.url==='/api/simulation/viewer'?gazeboView.status(res):gazeboView.serveAsset(req,res);
      }
      if(sim && req.url==='/api/simulation/status')return reply(res,200,sim.status());
      if(sim && req.url==='/api/simulation/events')return sim.subscribe(res);
      if(req.url==='/' && sim){res.writeHead(302,{Location:'/simulation/','Cache-Control':'no-store'});return res.end();}
      if (simDir && req.url==='/simulation/') {
        const original=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
        const symbols=original.match(/<svg class="symbols"[\s\S]*?<\/defs><\/svg>/)[0];
        const drive=original.match(/<div class="drive-surface">[\s\S]*?<\/small><\/div><\/div>/)[0];
        const driveSettings=original.match(/<details class="drive-settings">[\s\S]*?<\/details>/)[0];
        const page=fs.readFileSync(path.join(simDir,'web/index.html'),'utf8').replace('<!--SHARED_SYMBOLS-->',symbols).replace('<!--SHARED_DRIVE-->',drive).replace('<!--SHARED_DRIVE_SETTINGS-->',driveSettings);
        res.writeHead(200,{'Content-Type':'text/html; charset=utf-8','Cache-Control':'no-store'});res.end(page);return;
      }
      if (simDir && Object.hasOwn(simFiles,req.url)) {
        const [file,type]=simFiles[req.url];res.setHeader('Content-Type',type);
        fs.createReadStream(path.join(simDir,file)).on('error',()=>res.destroy()).pipe(res);return;
      }
      if (req.url === '/api/experiments') return reply(res,200,{experiments:availableExperiments});
      if(req.url==='/api/cloud-map'||req.url.startsWith('/api/cloud-map?')){
        const q=new URL(req.url,origin).searchParams;mazeSeen=Date.now();
        return reply(res,200,{...registeredMap.chunk(q.get('id'),Number(q.get('from')),Date.now(),Number(q.get('removed'))),connected});
      }
      if(req.url==='/api/maze'||req.url.startsWith('/api/maze?')){
        const q=new URL(req.url,origin).searchParams;mazeSeen=Date.now();
        const unavailable=mazeUnavailable();
        const healthReason={feedback:'等待底盘反馈',control:'确认 CAN 控制档与遥控器开关',controller:'等待车端迷宫控制器',
          map:'等待本轮地图',lidar:'等待车载雷达数据',odometry:'等待定位更新',perception:'等待雷达与定位配对',planner:'等待路径规划'}[unavailable]||'';
        return reply(res,200,{...mazeMap.snapshot(q.get('id'),Number(q.get('since'))||0),
          streamError:mazeStream.error,healthy:!unavailable,healthReason,paused:mission.run?.paused||false,mode:mission.run?.mode||null});
      }
      if (req.url === '/api/session') {
        try{
          const token=room.session(localRequest(req),deviceKey(req.socket.remoteAddress,addresses));return reply(res,200,{token,room:room.snapshot(token),urls});
        }
        catch(error){return reply(res,429,{error:error.message});}
      }
      if (req.url === '/api/status') {const token=req.headers['x-control-token'];room.touch(token);return reply(res,200,{service:'robot-lab-console',sharedVersion:2,hardwareLink,robotIp:robotAddress.address(),connected,status,age:statusAt?Date.now()-statusAt:null,owned:!!room.owner,room:room.snapshot(token),urls,task:tasks.task,preparedExperiment:tasks.prepared});}
      if (req.url === '/api/cloud' || req.url.startsWith('/api/cloud?')) {
        const source=new URL(req.url,origin).searchParams.get('source');
        if(source&&!['aggregate','raw'].includes(source))return reply(res,400,{error:'未知点云来源'});
        if(source&&cloudStream)return reply(res,200,{...cloudStream.snapshot(source),trajectory:trajectory.snapshot()});
        return reply(res,200,{...cloud,age:cloudAt?(Date.now()-cloudAt)/1000:null,connected,trajectory:trajectory.snapshot(),source:'registered',frames:1});
      }
      if (req.url === '/api/trajectory') return reply(res,200,{...trajectory.snapshot(),connected});
      const files = {'/':'index.html','/hardware/':'index.html','/app.js':'app.js','/ui.js':'ui.js','/room.js':'room.js','/liquid.js':'liquid.js','/cloud.js':'cloud.js','/style.css':'style.css','/glass.css':'glass.css','/vendor/three.module.min.js':'vendor/three.module.min.js','/vendor/three.core.min.js':'vendor/three.core.min.js'};
      Object.assign(files,{'/maze.js':'../迷宫/view.js','/maze.css':'../迷宫/view.css'});
      if (!Object.hasOwn(files,req.url)) return reply(res,404,{error:'未找到'});
      res.writeHead(200,{'Content-Type':req.url.endsWith('.js')?'text/javascript; charset=utf-8':req.url.endsWith('.css')?'text/css; charset=utf-8':'text/html; charset=utf-8','Cache-Control':'no-store','Content-Security-Policy':"default-src 'self'; connect-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; frame-ancestors 'none'",'X-Content-Type-Options':'nosniff'});
      return fs.createReadStream(path.join(__dirname,files[req.url])).pipe(res);
    }
    if (req.method !== 'POST' || req.headers.origin !== origin) return reply(res,403,{error:'拒绝跨站控制'});
    if(sim && req.url==='/api/simulation/viewer/start'){
      if(!localRequest(req))return reply(res,403,{error:'请在控制台主机恢复原生窗口'});
      try{await gazeboView.restore(simDir);return reply(res,200,{ok:true});}
      catch(error){return reply(res,503,{error:'原生视窗恢复失败 · '+error.message});}
    }
    if(req.url==='/api/shutdown'&&localRequest(req)){
      if(connected&&!stopped())return reply(res,409,{error:'请先停止真实机器人'});
      reply(res,200,{ok:true});setImmediate(close);return;
    }
    if(req.url==='/api/hardware/connect'){
      if(!localRequest(req))return reply(res,403,{error:'请在控制台主机连接真机'});
      connectHardware();return reply(res,connected?200:202,hardwareLink);
    }
    if(req.url==='/api/hardware/leave'){
      const token=req.headers['x-control-token'];
      if(!sessions.has(token))return reply(res,403,{error:'真机会话已失效'});
      if(room.owner?.token===token)return reply(res,409,{error:'请先停车并释放控制权'});
      const other=[...sessions].some(([id,s])=>id!==token&&Date.now()-s.seen<5000);
      if(localRequest(req)&&!other&&!room.owner){
        if(connected&&!stopped())return reply(res,409,{error:'等待底盘确认停稳后再切换'});
        detachHardware();
      }
      room.cancel(token);sessions.delete(token);
      return reply(res,200,{ok:true});
    }
    if(sim && req.url==='/api/simulation/start'){
      if(!localRequest(req)&&!sim.live())return reply(res,403,{error:'请先在控制台主机启动仿真'});
      const state=sim.start();return reply(res,state.phase==='ready'?200:202,state);
    }
    if(sim && req.url==='/api/simulation/command'){
      let payload='';
      try{for await(const chunk of req){payload+=chunk;if(payload.length>16384)return reply(res,413,{error:'场景数据过大'});}
        sim.command(JSON.parse(payload));return reply(res,202,{ok:true});
      }catch(error){return reply(res,409,{error:error.message});}
    }
    const token=req.headers['x-control-token']; const session=sessions.get(token);
    if(!session) return reply(res,403,{error:'会话失效，请刷新'});
    let body='';
    try { for await (const chunk of req) { body+=chunk; if(body.length>1024) return reply(res,413,{error:'请求过长'}); } body=JSON.parse(body); }
    catch { return reply(res,400,{error:'请求格式错误'}); }
    if (!body||!Number.isSafeInteger(body.seq) || (body.seq<=session.seq&&body.command!=='stop')) return reply(res,409,{error:'过期指令'});
    session.seq=Math.max(session.seq,body.seq);room.touch(token);
    if(req.url==='/api/room'){
      try{
        switch(body.action){
          case 'join':room.join(token,body.name);break;
          case 'queue':room.enqueue(token);break;
          case 'cancel':room.cancel(token);break;
          case 'release':room.release(token);break;
          case 'takeover':if(!localRequest(req)||!session.admin)return reply(res,403,{error:'只有本机管理员可以接管'});room.takeover(token);break;
          default:return reply(res,400,{error:'未知操作'});
        }
        return reply(res,200,{ok:true,room:room.snapshot(token)});
      }catch(error){return reply(res,409,{error:error.message});}
    }
    if(req.url === '/api/ping') {
      let ip=robotAddress.address();
      if(!ping){try{ip=await robotAddress.discover();}catch(error){return reply(res,200,{ok:false,output:error.message});}}
      const run = ping || ((done)=>execFile('ping.exe',['-n','1','-w','1000',ip],{timeout:2500,windowsHide:true,encoding:'buffer'},(err,stdout)=> {
        const output = new TextDecoder('gb18030').decode(stdout);
        done(!err && /TTL[=：:]/i.test(output),output);
      }));
      return run((ok,output)=>reply(res,200,{ok,ip,output:String(output).slice(0,2000)}));
    }
    if(req.url==='/api/maze/goal'){
      if(!room.valid(token,body.lease)||!connected)return reply(res,409,{error:'请先取得控制权'});
      if(tasks.prepared!=='maze'||!mazeHealthy())return reply(res,409,{error:'请先准备本轮迷宫地图'});
      try{mazeMap.goals(body);mission.retarget();return reply(res,200,{ok:true});}
      catch(error){return reply(res,409,{error:error.message});}
    }
    if(req.url !== '/api/command') return reply(res,404,{error:'未找到'});
    const { command }=body;
    if(command!=='claim' && !COMMANDS.has(command)) return reply(res,400,{error:'未知指令'});
    if(command==='start' && !availableExperiments.some(item=>item.enabled && item.id===body.experiment)) return reply(res,400,{error:'实验未接入或未启用'});
    if(command==='stop'){room.emergency(token);return reply(res,200,{ok:true,room:room.snapshot(token),task:tasks.task});}
    if(!connected) return reply(res,503,{error:'机器人控制连接未建立'});
    if(command==='claim') {
      try{room.claim(token);return reply(res,200,{ok:true,room:room.snapshot(token)});}
      catch(error){return reply(res,409,{error:error.message});}
    }
    if(command==='heartbeat'){
      try{room.heartbeat(token,body.lease);send('heartbeat');return reply(res,200,{ok:true});}
      catch(error){return reply(res,409,{error:error.message});}
    }
    // Every movement is fenced by the current lease, including queued HTTP requests.
    if(!room.valid(token,body.lease))return reply(res,409,{error:'控制权已变更或正在停车，请等待并重新操作'});
    if(mission.run){
      if(command==='pause'){mission.pause();return reply(res,200,{ok:true});}
      if(command==='resume'){mission.resume();return reply(res,200,{ok:true});}
      if(!['status','zero'].includes(command))return reply(res,409,{error:'请先停止迷宫任务'});
    }
    if(command==='drive'){
      const {linear,angular,speed,turn}=body;
      if(![-1,0,1].includes(linear)||![-1,0,1].includes(angular)||!Number.isFinite(speed)||speed<0.1||speed>1.5||!Number.isFinite(turn)||turn<0.05||turn>0.50)
        return reply(res,400,{error:'驾驶参数超出范围'});
      if(status.manual_vector!==true)return reply(res,409,{error:'机器人端尚未支持组合驾驶，请更新控制程序'});
      if(status.mode!=='manual'||Date.now()-statusAt>=1500)return reply(res,409,{error:'请先启用键盘驾驶并确认连接'});
      send(`drive ${linear} ${angular} ${speed} ${turn}`);return reply(res,200,{ok:true});
    }
    if(command==='manual'){
      if(manualAck)return reply(res,409,{error:'正在等待机器人确认，请稍候'});
      const result=await new Promise(resolve=>{
        const timer=setTimeout(()=>{manualAck=null;resolve({ok:false,reason:'机器人未确认手动模式，请检查部署版本或连接'});},1000);
        manualAck={resolve,timer};
        if(!send('manual')){clearTimeout(timer);manualAck=null;resolve({ok:false,reason:'控制连接已断开'});}
      });
      if(!room.valid(token,body.lease))return reply(res,409,{error:'控制权已变更'});
      return reply(res,result.ok?200:409,result.ok?{ok:true}:{error:MANUAL_ERRORS[result.reason]||result.reason});
    }
    if(command==='start'){
      if(body.experiment==='lab3-xy'&&tasks.prepared==='lab3-xy'){
        const p=trajectory.snapshot(),o=body.options||{};
        if(!connected||!p.pose||p.age===null||p.age>.5||p.frame!==o.goalFrame||!Number.isFinite(o.goalX)||!Number.isFinite(o.goalY)||Math.hypot(o.goalX-p.pose[0],o.goalY-p.pose[1])>30)return reply(res,409,{error:'终点或定位无效，请重新选点（距离不超过 30 m）'});
      }
      try{tasks.start(body.experiment,()=>room.valid(token,body.lease),body.options);return reply(res,200,{ok:true,task:tasks.task});}
      catch(error){return reply(res,409,{error:error.message});}
    }
    if(['pause','resume'].includes(command)&&body.experiment==='lab1')return reply(res,400,{error:'实验一使用启动与停止，不支持暂停续跑'});
    send(command); return reply(res,200,{ok:true});
  });
  server.on('upgrade',(req,socket,head)=>{
    if(req.url!=='/gazebo/websockify'||!localRequest(req)||req.headers.origin!==`http://${req.headers.host}`||
       req.headers.upgrade?.toLowerCase()!=='websocket'||!req.headers['sec-websocket-key']){socket.destroy();return;}
    displays.add(socket);socket.on('close',()=>displays.delete(socket));gazeboView.upgrade(req,socket,head);
  });
  server.once('listening',()=>{if(!deferredHardware)startTransport();});
  function close(){closing=true;clearInterval(timer);for(const display of displays)display.destroy();sim?.close();send('zero');send('pause');detachHardware();server.close();}
  return {server,room,close};
}
if(require.main===module) {
  const portIndex=process.argv.indexOf('--port');
  const port=portIndex<0?8765:Number(process.argv[portIndex+1]);
  if(port!==8765)throw Error('Unexpected console port');
  const {CloudStream}=require('./cloud-stream.cjs');
  const app=createConsole({port,deferredHardware:true,cloudStream:new CloudStream()});
  app.server.on('error',error=> { console.error(error.message);app.close();process.exitCode=1; });
  app.server.listen(port,'0.0.0.0',()=> {
    console.log('Admin: http://127.0.0.1:'+port);
    for(const ip of shareAddresses(robotAddress.address()))console.log('Share: http://'+ip+':'+port);
    if(process.argv.includes('--open')) execFile('rundll32.exe',['url.dll,FileProtocolHandler','http://127.0.0.1:'+port],{windowsHide:true},()=>{});
  });
  process.on('SIGINT',()=>{app.close();process.exitCode=0;});
}
module.exports={createConsole};
