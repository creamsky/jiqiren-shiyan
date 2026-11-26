const {spawn}=require('node:child_process');
const {randomUUID}=require('node:crypto');
const path=require('node:path');
const robotAddress=require('./robot-address.cjs');
const REMOTE={lab3:'bash /home/nvidia/lab26/select_experiment.sh lab2',lab1:'bash /home/nvidia/lab1/start.sh 30',stopLab1:'bash /home/nvidia/lab1/stop.sh',lab2:'bash /home/nvidia/lab26/select_experiment.sh lab2','lab2-eight':'bash /home/nvidia/lab26/select_experiment.sh lab2'};
REMOTE['lab3-xy']=REMOTE.lab3;
REMOTE.maze=REMOTE.lab3+' --roam';
function jobSSH(command,onData){
  return new Promise((resolve,reject)=>{
    const executable=path.join(process.env.WINDIR||'C:\\Windows','System32','OpenSSH','ssh.exe');
    const key=path.join(process.env.USERPROFILE||'C:\\Users\\asus','.ssh','robot_lab_ed25519');
    const child=spawn(executable,['-i',key,'-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o',`HostKeyAlias=${robotAddress.hostKeyAlias}`,'-o','ConnectTimeout=8','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3',`nvidia@${robotAddress.address()}`,command],{windowsHide:true});
    const timer=setTimeout(()=>{child.kill();reject(Error('远程实验超过 4 分钟，需检查实车状态'));},240000);
    child.stdout.on('data',data=>onData(data.toString('utf8')));
    child.stderr.on('data',data=>onData(data.toString('utf8')));
    child.on('error',error=>{clearTimeout(timer);reject(error);});
    child.on('close',code=>{clearTimeout(timer);resolve(code);});
  });
}
async function runSSH(command,onData,kind){
  kind=kind||Object.keys(REMOTE).find(key=>REMOTE[key]===command);
  if(!kind||REMOTE[kind]!==command)throw Error('未知车端实验操作');
  const request='bash /home/nvidia/lab26/console_job.sh '+randomUUID()+' '+kind;
  let code=await jobSSH(request,onData);
  const deadline=Date.now()+240000;
  while(code===255&&Date.now()<deadline){
    await new Promise(resolve=>setTimeout(resolve,2000));
    code=await jobSSH(request+' observe',onData);
  }
  return code;
}
class ExperimentTasks {
  constructor({send,stopped,ready=()=>true,startBlock=()=>'',onboardCapable=()=>false,run=runSSH,waitMs=2000,readyMs=15000,startMaze,prepareMaze}){Object.assign(this,{send,stopped,ready,startBlock,onboardCapable,run,waitMs,readyMs,startMaze,prepareMaze});this.task=null;this.prepared=null;this.job=null;}
  get onboard(){return !!this.task?.onboardId&&this.task.phase==='running';}
  get vehicleJob(){return this.task?.vehicleJob===true&&['preparing','running'].includes(this.task.phase);}
  dispatch(task,command){
    if(this.onboardCapable())task.onboardId=task.prepareId;
    if(this.send(task.onboardId?`onboard ${task.onboardId} ${command}`:command)===false)throw Error('底盘连接中断，启动指令未送达');
  }
  reconcile(s,reconnect){
    const valid=s.onboard_protocol===1&&/^[a-f0-9-]{36}$/.test(s.onboard_id)&&['lab2','lab2-eight','lab3','lab3-xy','maze'].includes(s.onboard_experiment);
    const terminal=['DONE','FAULT'].includes(s.stage);
    if(valid&&this.task?.onboardId===s.onboard_id){
      this.task.awaitingActive=false;
      if(terminal&&this.task.phase==='running'){
        this.task.phase=s.stage==='DONE'?'completed':s.reason==='Operator stop'?'stopped':'failed';
        this.task.log=s.reason||'车端已完成';this.prepared=null;
      }
    }else if(valid&&(!terminal||reconnect&&!this.task)){
      this.prepared=null;this.task={experiment:s.onboard_experiment,prepareId:s.onboard_id,onboardId:s.onboard_id,
        nativeRoam:s.onboard_experiment==='maze',phase:terminal?(s.stage==='DONE'?'completed':s.reason==='Operator stop'?'stopped':'failed'):'running',
        log:terminal?s.reason||'车端已完成':'已同步车端任务',exitCode:null,options:{}};
    }else if(reconnect&&this.onboard){
      this.task.phase='failed';this.task.log='车端没有报告原任务，需重新准备';this.prepared=null;
    }else if(!valid&&this.onboard&&!this.task.awaitingActive){
      this.task.phase='failed';this.task.log='车端任务已结束或控制器已重新初始化';this.prepared=null;
    }else if(this.onboard&&this.task.awaitingActive&&Date.now()-this.task.sentAt>2500){
      this.task.phase='failed';this.task.log='车端未确认本轮任务';this.prepared=null;
    }
  }
  get busy(){return !!this.task && ['preparing','running','stopping'].includes(this.task.phase);}
  observe(status){
    if(this.task?.onboardId)return;
    if((!this.task?.nativeRoam&&!['lab2','lab2-eight','lab3','lab3-xy'].includes(this.task?.experiment))||this.task.phase!=='running')return;
    if(this.task.awaitingActive){
      const active=['TURNING','DRIVING'].includes(status.stage)||status.stage==='WAITING'&&status.paused===false;
      if(active&&(!this.task.nativeRoam||status.roaming===true))this.task.awaitingActive=false;
      else if(Date.now()-this.task.sentAt>2500){
        this.send('stop');this.prepared=null;this.task.phase='failed';this.task.log='车端未确认启动，已请求停车。请检查底盘反馈后重试。';
      }
      return;
    }
    if(['DONE','FAULT'].includes(status.stage)){
      this.prepared=null;
      this.task.phase=status.stage==='DONE'?'completed':'failed';
    }
  }
  async waitStopped(){const deadline=Date.now()+this.waitMs;while(Date.now()<deadline){if(this.stopped())return;await new Promise(r=>setTimeout(r,25));}throw Error('未确认机器人静止，拒绝切换实验');}
  start(experiment,ownerAlive,options={},intent,prepareId){
    options=options||{};
    if(intent!==undefined&&!['prepare','run'].includes(intent))throw Error('未知启动操作');
    if(intent==='prepare'&&this.task?.experiment===experiment&&['preparing','ready'].includes(this.task.phase)&&(this.prepared===experiment||this.task.phase==='preparing'))return;
    if(intent==='run'&&experiment!=='lab1'&&this.prepared!==experiment)throw Error('本轮准备已失效，请重新准备');
    if(intent==='run'&&experiment!=='lab1'&&prepareId!==this.task?.prepareId)throw Error('准备轮次已更新，请确认当前实验后再开始');
    if(experiment==='lab2-eight'&&(!Number.isFinite(options.radius)||options.radius<0.5||options.radius>10||!['cross','outer'].includes(options.startPoint)))throw Error('半径须为 0.5–10 m，起点须为交叉点或外侧');
    if(this.busy)throw Error('实验正在准备、运行或停止，请稍候');
    if(intent!=='prepare'&&experiment!=='lab1'&&this.prepared===experiment){
      if(!ownerAlive())throw Error('控制权已失效');
      const block=this.startBlock(experiment);
      if(block)throw Error(block);
      if(!this.ready(experiment))throw Error('本轮准备已保留，等待定位、雷达与规划器就绪后再开始');
      if(experiment==='lab3-xy'&&(!Number.isFinite(options.goalX)||!Number.isFinite(options.goalY)||!(/^[A-Za-z0-9_/]+$/).test(options.goalFrame||'')))throw Error('请在已配准点云中选取终点，或输入终点坐标');
      if(experiment==='maze'){
        const task={experiment,prepareId:this.task.prepareId,phase:'running',exitCode:null,log:'',options};
        this.startMaze(task,ownerAlive,options);this.task=task;return;
      }
      const command=experiment==='lab2-eight'?`start8 ${options.radius} ${options.startPoint}`:experiment==='lab3-xy'?`startxy ${options.goalX} ${options.goalY} ${options.goalFrame}`:experiment==='lab3'?'start3':'start';
      const task={experiment,prepareId:this.task.prepareId,phase:'running',exitCode:null,log:'',awaitingActive:true,sentAt:Date.now(),options};
      this.dispatch(task,command);this.task=task;return;
    }
    const task=this.task={experiment,prepareId:randomUUID(),phase:'preparing',exitCode:null,log:'',cancelled:false,progress:0,progressLabel:'准备启动'};
    let pending='';
    const onData=text=>{
      task.log=(task.log+text).slice(-4000);pending+=text;
      const lines=pending.split('\n');pending=lines.pop().slice(-4000);
      for(const line of lines){const match=/^EXPERIMENT_PROGRESS (\d+) (.+)$/.exec(line);if(match){task.progress=Math.max(task.progress,Math.min(100,Number(match[1])));task.progressLabel=match[2];}}
    };
    this.send('stop');
    this.job=(async()=>{
      try{
        await this.waitStopped();
        if(task.cancelled||!ownerAlive())throw Error('控制权已失效或任务已取消，请重新操作');
        this.prepared=null;
        task.phase=experiment==='lab1'?'running':'preparing';
        if(experiment==='maze')this.prepareMaze?.();
        task.vehicleJob=this.onboardCapable();
        const code=await this.run(REMOTE[experiment],onData,experiment);
        task.exitCode=code;
        if(task.cancelled)return;
        if(code!==0)throw Error(`远程实验命令失败 (${code})`);
        if(experiment!=='lab1') {
          const deadline=Date.now()+this.readyMs;
          while(!this.ready(experiment)&&Date.now()<deadline&&!task.cancelled)await new Promise(r=>setTimeout(r,100));
          if(task.cancelled)return;
          if(!this.ready(experiment))throw Error('准备失败：定位、雷达、底盘或 DWA 规划器未就绪，请检查机器人日志');
        }
        if(!task.vehicleJob&&!ownerAlive())throw Error('操作窗口已失去控制权，请重新取得控制权');
        this.prepared=experiment;
        task.phase=experiment==='lab1'?'completed':'ready';
        if(experiment==='lab1')task.progress=100;
      }catch(error){if(!task.cancelled){task.phase='failed';task.log=(task.log+'\n'+error.message).slice(-4000);}}
    })();
  }
  stop(keepReady=false){
    this.send('stop');
    const task=this.task;
    if(keepReady&&task?.phase==='ready'&&this.prepared===task.experiment)return;
    this.prepared=null;
    if(!task)return;
    if(task.phase==='stopping')return;
    const wasPreparing=task.phase==='preparing';
    const stopLab1=task.experiment==='lab1'&&['running','preparing','completed'].includes(task.phase);
    task.cancelled=true;
    if(!stopLab1){
      if(wasPreparing){task.phase='stopping';const pending=this.job;this.job=(async()=>{await pending;task.phase='stopped';this.prepared=null;})();}
      else task.phase='stopped';
      return;
    }
    task.phase='stopping';
    const previous=this.job;
    this.job=(async()=>{
      try{
        const append=text=>task.log=(task.log+text).slice(-4000);
        await this.run(REMOTE.stopLab1,append);
        await previous;
        // The initial stop can precede initialization of the remote start job.
        // Stop once more after that job exits to avoid orphan sensor processes.
        const code=await this.run(REMOTE.stopLab1,append);
        task.exitCode=code;task.phase=code===0?'stopped':'failed';
      }
      catch(error){task.phase='failed';task.log=(task.log+'\n'+error.message).slice(-4000);}
      this.prepared=null;
    })();
  }
}
module.exports={ExperimentTasks,REMOTE};
