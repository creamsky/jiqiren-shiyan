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
const {MazeMap,distance,BASE_EXTENSION}=require('../迷宫/maze-map.cjs');
const {LidarReader}=require('./point-cloud/lidar-reader.cjs');
const {MazeMission}=require('../迷宫/mission.cjs');
const {RegisteredMap}=require('./point-cloud/registered-map.cjs');

const COMMANDS = new Set(['status','start','pause','resume','stop','manual','auto','w','s','a','d','drive','zero','heartbeat']);
const MANUAL_ERRORS={odometry_unavailable:'定位未就绪，请先准备传感器',lidar_unavailable:'雷达未就绪，请先准备传感器',feedback_unavailable:'底盘反馈超时',chassis_fault:'底盘报告故障',chassis_mode:'底盘尚未处于网络/CAN 模式',remote_takeover:'遥控器开关正在接管底盘',not_stopped:'请先停车，再恢复手动驾驶',heartbeat_missing:'控制心跳已过期，请重新启用键盘驾驶'};
const experiments = JSON.parse(fs.readFileSync(path.join(__dirname,'experiments.json'),'utf8'));
const availableExperiments = experiments.map(item=>({...item,enabled:item.enabled === true && ['lab1','lab2','lab2-eight','lab3','lab3-xy','maze'].includes(item.id)}));
function createConsole({ port = 8765, tcpPort = 18766, ping = null, sshRunner, leaseMs=1000, cloudStream=null, deferredHardware=false, robotLink=require('./launch.cjs'), lidarReaderFactory=fn=>new LidarReader(fn) } = {}) {
  const simDir=[path.join(__dirname,'../实验三扩展/仿真'),path.join(__dirname,'../仿真')].find(p=>fs.existsSync(path.join(p,'web/index.html')));
  const sim=simDir?new (require('./sim-bridge.cjs').SimulationBridge)(simDir):null;
  const displays=new Set();
  const simFiles={'/simulation/gazebo/':['web/gazebo.html','text/html; charset=utf-8'],'/simulation/field.js':['web/field.js','text/javascript; charset=utf-8'],'/simulation/native.js':['web/native.js','text/javascript; charset=utf-8'],'/simulation/field.css':['web/field.css','text/css; charset=utf-8']};
  const addresses=Object.values(os.networkInterfaces()).flat().filter(x=>x.family==='IPv4'&&!x.internal).map(x=>x.address);
  const hosts=new Set(['127.0.0.1','localhost',...addresses].map(ip=>`${ip}:${port}`));
  const urls=shareAddresses(robotAddress.address()).map(ip=>`http://${ip}:${port}/hardware/`);
  const localRequest=req=>['127.0.0.1','::1','::ffff:127.0.0.1'].includes(req.socket.remoteAddress)&&['127.0.0.1:'+port,'localhost:'+port].includes(req.headers.host);
  let socket, connected = false, status = {}, statusAt = 0, buffer = '', closing = false;
  let transportStarted=false, linkRequest=null, retry, retryDelay=1000;
  let hardwareLink={phase:'idle',message:'可以连接真机'};
  function startTransport(){
    if(closing)return;
    transportStarted=true;
    connect();
  }
  function retryTransport(){
    if(closing||!transportStarted)return;
    clearTimeout(retry);
    retry=setTimeout(()=>deferredHardware?connectHardware(true):connect(),retryDelay);
    retryDelay=Math.min(10000,retryDelay*2);
  }
  function connectHardware(recover=false){
    if(closing||socket||linkRequest)return;
    clearTimeout(retry);
    const request=linkRequest=new AbortController();hardwareLink={phase:'connecting',message:'正在建立机器人连接'};
    robotLink.ensureRobotLink(request.signal,{recover}).then(()=>{
      if(linkRequest!==request)return;
      hardwareLink={phase:'connecting',message:'已建立隧道，等待底盘反馈'};
      startTransport();
    },error=>{
      if(linkRequest!==request)return;
      hardwareLink={phase:'error',message:error.message};
      if(recover&&!error.manual)retryTransport();
    }).finally(()=>{if(linkRequest===request)linkRequest=null;});
  }
  function detachHardware(){
    resetObservations();
    transportStarted=false;linkRequest?.abort();linkRequest=null;clearTimeout(retry);
    connected=false;status={};statusAt=0;cloud={points:[],frame:''};cloudAt=0;trajectory.clear();
    cloudStream?.stop();socket?.destroy();socket=null;robotLink.closeRobotLink();retryDelay=1000;
    hardwareLink={phase:'idle',message:'可以连接真机'};
  }
  let manualAck=null;
  const send = command => { if (!connected || !socket?.writable) return false; socket.write(command + '\n'); return true; };
  let cloud={points:[],frame:''},cloudAt=0;
  const trajectory=new Trajectory();
  const registeredMap=new RegisteredMap();
  const mazeMap=new MazeMap();
  const mission=new MazeMission(mazeMap,send);
  const cloudGeneration=()=>status.map_generation||`pose:${trajectory.epoch}`;
  let mazeSeen=0,lidarOffset=null,pendingCloud=null;
  function registerPending(){
    if(!pendingCloud||!lidarOffset)return;
    if(Date.now()-cloudAt>500){pendingCloud=null;return;}
    if(registeredMap.accept(pendingCloud,trajectory,cloudGeneration(),Date.now(),lidarOffset))pendingCloud=null;
  }
  const lidarReader=lidarReaderFactory(sample=>{
    if(!connected)return false;
    const offset=sample.lidarOffset;
    const calibrated=Array.isArray(offset)&&offset.length===3&&offset.every(v=>Number.isFinite(v)&&Math.abs(v)<2);
    if(sample.type==='lidar_mount'){lidarOffset=calibrated?offset:null;registerPending();return calibrated;}
    if(!trajectory.pose||Date.now()-trajectory.at>650||!status.feedback_fresh||
       sample.generation!==status.map_generation||sample.frame!==trajectory.frame||
       Math.abs(trajectory.stamp-sample.stamp)>.45||!Array.isArray(sample.pose)||distance(trajectory.pose,sample.pose)>.3||
       Math.abs(Math.atan2(Math.sin(trajectory.pose[2]-sample.pose[2]),Math.cos(trajectory.pose[2]-sample.pose[2])))>.35)return false;
    lidarOffset=calibrated?offset:null;
    if(!status.maze_height_observations)sample.heightHits=[];
    return mazeMap.accept(sample);
  });
  function resetObservations(){
    lidarReader.stop();
    cloud={points:[],frame:''};cloudAt=0;
    mazeMap.reset();registeredMap.reset();lidarOffset=null;pendingCloud=null;
    if(['maze','lab3-xy'].includes(tasks.prepared)){
      tasks.prepared=null;
      if(tasks.task?.phase==='ready'){tasks.task.phase='stopped';tasks.task.log='定位坐标系已更新，请重新准备本轮实验';}
    }
  }
  function mazeUnavailable(forPreparation=false){
    const now=Date.now();
    if(!connected||now-statusAt>=700||status.feedback_fresh!==true)return 'feedback';
    if(!forPreparation&&(status.chassis_mode!==1||status.chassis_errors!==0||status.manual_block_reason==='chassis_fault'||
       ![0,2].includes(status.remote_switch)||status.mode==='manual'))return 'control';
    if(status.maze_protocol!==2||status.xy_planner!=='navfn_pursuit')return 'controller';
    if(!mazeMap.generation||mazeMap.generation!==status.map_generation)return 'map';
    if(status.manual_block_reason==='lidar_unavailable')return 'lidar';
    if(status.manual_block_reason==='odometry_unavailable'||now-trajectory.at>=600)return 'odometry';
    if(now-mazeMap.at>=850||status.sensors_ready!==true)return 'perception';
    if(status.dwa_ready!==true)return 'planner';
    return '';
  }
  const mazeHealthy=()=>!mazeUnavailable();
  const navigationOnline=()=>connected&&Date.now()-statusAt<700&&status.feedback_fresh===true&&status.sensors_ready===true&&
    status.dwa_ready===true&&status.xy_planner==='navfn_pursuit'&&Boolean(status.map_generation)&&Date.now()-trajectory.at<600;
  function startBlock(experiment){
    if(!['maze','lab3-xy'].includes(experiment))return status.control_permitted===false?'当前仅观察，请确认网络 / CAN 模式和底盘反馈':'';
    if(!connected||Date.now()-statusAt>=700||status.feedback_fresh!==true)return '等待新鲜底盘反馈';
    if(status.chassis_errors!==0||status.manual_block_reason==='chassis_fault')return '底盘报告故障，请检查后再开始';
    if(status.chassis_mode===3||[1,3].includes(status.remote_switch))return '遥控器正在接管。切回网络 / CAN 档即可开始';
    if(status.chassis_mode!==1||![0,2].includes(status.remote_switch))return '等待底盘进入网络 / CAN 模式';
    if(status.mode==='manual')return '请先返回自动模式';
    if(experiment==='maze'){
      if(status.gap_roam!==true)return '车端需要更新自主漫游控制器';
      if(!status.sensors_ready)return '等待读取本轮雷达与定位';
    }else if(!navigationOnline())return '等待雷达、定位与导航规划器恢复';
    if(!stopped())return '等待车端确认停稳';
    return '';
  }
  const stopped=()=>connected && Date.now()-statusAt<500 && status.feedback_fresh===true &&
      [status.v,status.w,status.feedback_v,status.feedback_w].every(Number.isFinite)&&
      Math.abs(status.v)<0.001&&Math.abs(status.w)<0.001&&Math.abs(status.feedback_v)<0.02&&Math.abs(status.feedback_w)<0.02&&
      (status.paused || ['FAULT','DONE','IDLE'].includes(status.stage));
  const monitors=new Map();
  let pendingStopId=null;
  const tasks=new ExperimentTasks({send,run:sshRunner,stopped,startBlock,onboardCapable:()=>status.onboard_protocol===1,
    prepareMaze:()=>{
      if(status.gap_roam!==true)throw Error('车端需要更新自主漫游控制器');
      if(status.chassis_errors!==0)throw Error('底盘报告故障，请检查后再准备');
      if(status.mode==='manual')send('auto');
      mazeMap.reset();registeredMap.reset();pendingCloud=null;
    },
    startMaze:(task,alive,options)=>{
      if(!options.mapId){
        if(!alive()||!stopped())throw Error('漫游启动条件已改变');
        tasks.dispatch(task,'startroam '+BASE_EXTENSION);
        Object.assign(task,{nativeRoam:true,awaitingActive:true,sentAt:Date.now()});
        return;
      }
      if(!mazeHealthy()||options.mapId!==mazeMap.id||!stopped())throw Error('请确认本轮地图、迷宫控制器和停车反馈已就绪');
      if(options.hint!=null){
        if(!Array.isArray(options.hint)||options.hint.length!==2)throw Error('搜索区需要前方和左侧两个坐标');
        mazeMap.goals({kind:'hint',id:options.mapId,x:options.hint[0],y:options.hint[1]});
      }
      mission.start(task,alive,Date.now(),status.stage==='FAULT'?status.reason:'');
    },
    ready:experiment=>experiment==='lab3-xy'?navigationOnline():connected&&Date.now()-statusAt<1500&&status.sensors_ready===true&&status.feedback_fresh===true&&(experiment!=='maze'||status.gap_roam===true)});
  const room=new ControlRoom({leaseMs,connected:()=>connected,onboard:()=>tasks.onboard||tasks.vehicleJob,canManageMoving:()=>tasks.onboard,present:token=>monitors.has(token),
    stopped:()=>stopped()&&(!room.barrier||statusAt>room.barrier.at)&&tasks.task?.phase!=='stopping',
    stop:keepReady=>{if(tasks.onboard)pendingStopId=tasks.task.onboardId;send('zero');send('pause');tasks.stop(keepReady);}});
  const sessions=room.sessions;
  function connect() {
    if (closing||socket) return;
    const peer=socket=net.createConnection({host:'127.0.0.1',port:tcpPort});
    let previewSeq=0;
    const previewCells=new Map();
    const openedAt=Date.now();
    const watchdog=setInterval(()=>{
      if(Date.now()-(statusAt||openedAt)>5000)peer.destroy();
    },500);
    buffer='';
    peer.on('connect',()=>{if(socket===peer)peer.write('cloudkey\nstatus\n');});
    socket.on('data', data => {
      if(socket!==peer)return;
      buffer += data.toString();
      if (buffer.length > 1048576) return socket.destroy();
      let index;
      while ((index = buffer.indexOf('\n')) >= 0) {
        const line = buffer.slice(0,index); buffer = buffer.slice(index+1);
        try { let value = JSON.parse(line); if (value && typeof value === 'object' && !Array.isArray(value)) {
          if(value.type==='cloud_delta'){
            const valid=cells=>Array.isArray(cells)&&cells.length<=1200&&cells.every(p=>Array.isArray(p)&&p.length===3&&p.every(v=>Number.isSafeInteger(v)&&Math.abs(v)<=1000000));
            if(!Number.isSafeInteger(value.seq)||value.seq<=0||!Number.isFinite(value.stamp)||!valid(value.add)||!valid(value.del)||
               (value.base!==0&&value.base!==previewSeq)){peer.write('cloudkey\n');continue;}
            if(!value.base)previewCells.clear();
            for(const p of value.del)previewCells.delete(p.join(','));
            for(const p of value.add)previewCells.set(p.join(','),p.map(n=>n/100));
            if(previewCells.size>1200){previewSeq=0;previewCells.clear();peer.write('cloudkey\n');continue;}
            previewSeq=value.seq;
            value={type:'cloud',registered:true,stamp:value.stamp,frame:value.frame,points:[...previewCells.values()]};
          }
          if(value.type==='cloud'){
            cloud={points:Array.isArray(value.points)?value.points.filter(point=>Array.isArray(point)&&point.length===3&&point.every(Number.isFinite)).slice(0,8000):[],frame:String(value.frame||'').replace(/^\//,'').slice(0,100),registered:value.registered===true,stamp:Number.isFinite(value.stamp)?value.stamp:null,generation:cloudGeneration(),epoch:trajectory.epoch};cloudAt=Date.now();
            pendingCloud=cloud.registered?cloud:null;registerPending();
          }else if(value.type==='pose'){const epoch=trajectory.epoch;trajectory.add(value);if(epoch!==trajectory.epoch)resetObservations();else registerPending();}
          else if(value.type==='ack'){
            if(value.command==='manual'&&manualAck){const pending=manualAck;manualAck=null;clearTimeout(pending.timer);pending.resolve(value);}
          }
          else if(!value.type||value.type==='status'){
            const reconnect=!connected;
            tasks.reconcile(value,reconnect);
            if(!connected){
              connected=true;retryDelay=1000;hardwareLink={phase:'ready',message:'已收到车端状态'};
              if(pendingStopId&&value.onboard_id===pendingStopId)send('stop');
              else if(!tasks.onboard&&!tasks.vehicleJob&&!value.onboard_id)room.beginStop('连接已建立，正在确认停车',{release:!room.owner||!monitors.has(room.owner.token),keepReady:true});
            }
            if(value.onboard_id!==pendingStopId||['DONE','FAULT'].includes(value.stage))pendingStopId=null;
            const previousGeneration=cloudGeneration();status = {...value,manual_block_message:MANUAL_ERRORS[value.manual_block_reason]||''}; statusAt = Date.now();tasks.observe(status);
            if(previousGeneration!==cloudGeneration()||mazeMap.generation&&mazeMap.generation!==status.map_generation)resetObservations();}
        } } catch {}
      }
    });
    socket.on('error',()=>{});
    peer.on('close',()=>{
      clearInterval(watchdog);
      if(socket!==peer)return;
      socket=null;
      const wasConnected=connected;connected=false;
      hardwareLink={phase:'error',message:'车端状态中断，正在重连'};
      if(wasConnected){room.disconnected();trajectory.clear();status={};statusAt=0;resetObservations();}
      if(manualAck){clearTimeout(manualAck.timer);manualAck.resolve({ok:false,reason:'控制连接已断开'});manualAck=null;}
      retryTransport();
    });
  }
  const timer = setInterval(()=> {
    room.tick();
    const watchingMaze=Date.now()-mazeSeen<10000||!!mission.run;
    if(connected&&(watchingMaze||trajectory.frame==='camera_init'))lidarReader.start(mission.run?'planner':watchingMaze?'maze':'mount');
    else lidarReader.stop();
    if(mission.run){
      const unavailable=mazeUnavailable();
      mission.tick({status,statusAt,unavailable,healthy:!unavailable,stopped:stopped()});
      if(!mission.run)tasks.prepared=null;
    }
  },100);
  const reply = (res, code, data) => { res.writeHead(code, {'Content-Type':'application/json; charset=utf-8','Cache-Control':'no-store'}); res.end(JSON.stringify(data)); };
  const server = http.createServer(async (req,res)=> {
    if (!hosts.has(req.headers.host)) return reply(res,403,{error:'无效访问地址'});
    const origin=`http://${req.headers.host}`;
    if(req.headers.origin&&req.headers.origin!==origin)return reply(res,403,{error:'拒绝跨站控制'});
    room.tick();
    if (req.method === 'GET') {
      if(req.url==='/api/session/monitor'){
        const token=req.headers['x-control-token'];
        if(!sessions.has(token))return reply(res,403,{error:'会话已失效'});
        res.writeHead(200,{'Content-Type':'text/event-stream','Cache-Control':'no-store'});res.write(': connected\n\n');
        const previous=monitors.get(token);monitors.set(token,res);previous?.end();
        res.on('close',()=>{if(monitors.get(token)===res){monitors.delete(token);if(room.owner?.token===token)room.owner.reconnectUntil=Date.now()+15000;}});
        return;
      }
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
        const q=new URL(req.url,origin).searchParams;
        return reply(res,200,{...registeredMap.chunk(q.get('id'),Number(q.get('from')),Date.now(),Number(q.get('removed'))),
          epoch:trajectory.epoch,connected,error:connected?(lidarOffset?'正在累计已配准点云':lidarReader.error):'点云 · 等待真机连接'});
      }
      if(req.url==='/api/maze'||req.url.startsWith('/api/maze?')){
        const q=new URL(req.url,origin).searchParams;mazeSeen=Date.now();
        const unavailable=mazeUnavailable();
        const healthReason={feedback:'等待底盘反馈',control:'确认 CAN 控制档与遥控器开关',controller:'等待车端迷宫控制器',
          map:'等待本轮地图',lidar:'等待车载雷达数据',odometry:'等待定位更新',perception:'等待雷达与定位配对',planner:'等待路径规划'}[unavailable]||'';
        return reply(res,200,{...mazeMap.snapshot(q.get('id'),Number(q.get('since'))||0),
          streamError:lidarReader.error,healthy:!unavailable,healthReason,paused:mission.run?.paused||false,mode:mission.run?.mode||null});
      }
      if (req.url === '/api/session') {
        try{
          const token=room.session(localRequest(req),deviceKey(req.socket.remoteAddress,addresses));return reply(res,200,{token,room:room.snapshot(token),urls});
        }
        catch(error){return reply(res,429,{error:error.message});}
      }
      if (req.url === '/api/status') {const token=req.headers['x-control-token'];room.touch(token);return reply(res,200,{service:'robot-lab-console',sharedVersion:2,hardwareLink,robotIp:robotAddress.address(),connected,status,age:statusAt?Date.now()-statusAt:null,owned:!!room.owner,room:room.snapshot(token),urls,task:tasks.task,preparedExperiment:tasks.prepared,startBlockReason:tasks.task?.phase==='ready'?startBlock(tasks.prepared):''});}
      if (req.url === '/api/cloud' || req.url.startsWith('/api/cloud?')) {
        const query=new URL(req.url,origin).searchParams,source=query.get('source');
        if(source&&!['aggregate','raw'].includes(source))return reply(res,400,{error:'未知点云来源'});
        const sample=source&&cloudStream?cloudStream.snapshot(source,connected):{...cloud,age:cloudAt?(Date.now()-cloudAt)/1000:null,connected,source:'registered',frames:1};
        const brief=query.has('brief'),tag=JSON.stringify([sample.source,sample.generation,sample.epoch,sample.frame,sample.stamp]);
        if(brief&&query.get('seen')===tag)return reply(res,200,{unchanged:true,tag,connected:sample.connected,age:sample.age});
        return reply(res,200,{...sample,tag,trajectory:brief?{stamp:trajectory.stamp,age:trajectory.at?(Date.now()-trajectory.at)/1000:null}:trajectory.snapshot()});
      }
      if (req.url === '/api/trajectory') return reply(res,200,{...trajectory.snapshot(),generation:cloudGeneration(),connected});
      if(req.url.startsWith('/api/trajectory?')){
        const q=new URL(req.url,origin).searchParams;
        return reply(res,200,{...trajectory.delta(q.get('id'),Number(q.get('from'))),generation:cloudGeneration(),connected});
      }
      const files = {'/':'index.html','/hardware/':'index.html','/app.js':'app.js','/ui.js':'ui.js','/room.js':'room.js','/liquid.js':'liquid.js','/cloud.js':'cloud.js','/style.css':'style.css','/glass.css':'glass.css','/vendor/three.module.min.js':'vendor/three.module.min.js','/vendor/three.core.min.js':'vendor/three.core.min.js'};
      Object.assign(files,{'/ground-filter.js':'point-cloud/ground-filter.js','/point-cloud-view.js':'point-cloud/view-controls.js','/point-cloud-view.css':'point-cloud/view.css','/maze.js':'../迷宫/view.js','/maze.css':'../迷宫/view.css'});
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
      if(connected&&!tasks.onboard&&!tasks.vehicleJob&&!stopped())return reply(res,409,{error:'请先停止真实机器人'});
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
        if(connected&&!tasks.onboard&&!tasks.vehicleJob&&!stopped())return reply(res,409,{error:'等待底盘确认停稳后再切换'});
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
    if(command==='stop'){
      if(!connected&&!(tasks.vehicleJob&&tasks.task.experiment==='lab1'))return reply(res,503,{error:'监控链路中断，停车指令未送达。请使用现场遥控器或急停'});
      room.emergency(token);return reply(res,200,{ok:true,room:room.snapshot(token),task:tasks.task});
    }
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
    if(status.control_permitted===false&&['resume','manual','drive','w','s','a','d'].includes(command))
      return reply(res,409,{error:'真机已连接，当前仅观察。请检查底盘反馈和网络 / CAN 档位'});
    if((tasks.onboard||tasks.task?.nativeRoam&&tasks.task.phase==='running')&&!['pause','resume','zero','status'].includes(command))
      return reply(res,409,{error:'请先停止当前自主任务'});
    if(['resume','manual','auto','drive','w','a','s','d'].includes(command))send('heartbeat');
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
      if(body.intent!=='prepare'&&body.experiment==='lab3-xy'&&tasks.prepared==='lab3-xy'){
        const blocked=startBlock('lab3-xy');if(blocked)return reply(res,409,{error:blocked});
        const p=trajectory.snapshot(),o=body.options||{};
        if(!connected||!p.pose||p.age===null||p.age>.5||p.frame!==o.goalFrame||!Number.isFinite(o.goalX)||!Number.isFinite(o.goalY)||Math.hypot(o.goalX-p.pose[0],o.goalY-p.pose[1])>30)return reply(res,409,{error:'终点或定位无效，请重新选点（距离不超过 30 m）'});
      }
      try{send('heartbeat');tasks.start(body.experiment,()=>room.valid(token,body.lease),body.options,body.intent,body.prepareId);return reply(res,200,{ok:true,task:tasks.task});}
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
  function close(){closing=true;clearInterval(timer);for(const display of displays)display.destroy();for(const res of monitors.values())res.end();monitors.clear();sim?.close();if(!tasks.onboard&&!tasks.vehicleJob){send('zero');send('pause');}detachHardware();server.close();}
  return {server,room,close};
}
if(require.main===module) {
  const portIndex=process.argv.indexOf('--port');
  const port=portIndex<0?8765:Number(process.argv[portIndex+1]);
  if(port!==8765)throw Error('Unexpected console port');
  const {CloudViews}=require('./cloud-stream.cjs');
  const app=createConsole({port,deferredHardware:true,cloudStream:new CloudViews()});
  app.server.on('error',error=> { console.error(error.message);app.close();process.exitCode=1; });
  app.server.listen(port,'0.0.0.0',()=> {
    console.log('Admin: http://127.0.0.1:'+port);
    for(const ip of shareAddresses(robotAddress.address()))console.log('Share: http://'+ip+':'+port);
    if(process.argv.includes('--open')) execFile('rundll32.exe',['url.dll,FileProtocolHandler','http://127.0.0.1:'+port],{windowsHide:true},()=>{});
  });
  process.on('SIGINT',()=>{app.close();process.exitCode=0;});
}
module.exports={createConsole};
