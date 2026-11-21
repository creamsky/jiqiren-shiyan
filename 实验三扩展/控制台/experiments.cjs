const {spawn}=require('node:child_process');
const path=require('node:path');
const robotAddress=require('./robot-address.cjs');
const REMOTE={lab3:'bash /home/nvidia/lab26/select_experiment.sh lab2',lab1:'bash /home/nvidia/lab1/start.sh 30',stopLab1:'bash /home/nvidia/lab1/stop.sh',lab2:'bash /home/nvidia/lab26/select_experiment.sh lab2','lab2-eight':'bash /home/nvidia/lab26/select_experiment.sh lab2'};
REMOTE['lab3-xy']=REMOTE.lab3;
function runSSH(command,onData){
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
class ExperimentTasks {
  constructor({send,stopped,ready=()=>true,run=runSSH,waitMs=2000,readyMs=15000}){Object.assign(this,{send,stopped,ready,run,waitMs,readyMs});this.task=null;this.prepared=null;this.job=null;}
  get busy(){return !!this.task && ['preparing','running','stopping'].includes(this.task.phase);}
  observe(status){
    if(!['lab2','lab2-eight','lab3','lab3-xy'].includes(this.task?.experiment)||this.task.phase!=='running')return;
    if(this.task.awaitingActive){
      if(['WAITING','TURNING','DRIVING'].includes(status.stage))this.task.awaitingActive=false;
      return;
    }
    if(['DONE','FAULT'].includes(status.stage))this.task.phase=status.stage==='DONE'?'completed':'failed';
  }
  async waitStopped(){const deadline=Date.now()+this.waitMs;while(Date.now()<deadline){if(this.stopped())return;await new Promise(r=>setTimeout(r,25));}throw Error('未确认机器人静止，拒绝切换实验');}
  start(experiment,ownerAlive,options={}){
    options=options||{};
    if(experiment==='lab3-xy'&&this.prepared===experiment&&(!Number.isFinite(options.goalX)||!Number.isFinite(options.goalY)||!(/^[A-Za-z0-9_/]+$/).test(options.goalFrame||'')))throw Error('请在已配准点云中选取终点，或输入终点坐标');
    if(experiment==='lab2-eight'&&(!Number.isFinite(options.radius)||options.radius<0.5||options.radius>10||!['cross','outer'].includes(options.startPoint)))throw Error('半径须为 0.5–10 m，起点须为交叉点或外侧');
    if(this.busy)throw Error('实验正在准备、运行或停止，请稍候');
    if(experiment!=='lab1'&&this.prepared===experiment){
      if(!this.ready(experiment)){this.prepared=null;throw Error('定位、雷达、底盘或导航规划器未就绪，请重新准备实验');}
      if(!ownerAlive())throw Error('控制权已失效');
      this.task={experiment,phase:'running',exitCode:null,log:'',awaitingActive:true,options};this.send(experiment==='lab2-eight'?`start8 ${options.radius} ${options.startPoint}`:experiment==='lab3-xy'?`startxy ${options.goalX} ${options.goalY} ${options.goalFrame}`:experiment==='lab3'?'start3':'start');return;
    }
    const task=this.task={experiment,phase:'preparing',exitCode:null,log:'',cancelled:false,progress:0,progressLabel:'准备启动'};
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
        const code=await this.run(REMOTE[experiment],onData);
        task.exitCode=code;
        if(task.cancelled)return;
        if(code!==0)throw Error(`远程实验命令失败 (${code})`);
        if(experiment!=='lab1') {
          const deadline=Date.now()+this.readyMs;
          while(!this.ready(experiment)&&Date.now()<deadline&&!task.cancelled)await new Promise(r=>setTimeout(r,100));
          if(task.cancelled)return;
          if(!this.ready(experiment))throw Error('准备失败：定位、雷达、底盘或 DWA 规划器未就绪，请检查机器人日志');
        }
        this.prepared=experiment;
        task.phase=experiment==='lab1'?'completed':'ready';
        if(experiment==='lab1')task.progress=100;
      }catch(error){if(!task.cancelled){task.phase='failed';task.log=(task.log+'\n'+error.message).slice(-4000);}}
    })();
  }
  stop(){
    this.prepared=null;
    this.send('stop');
    const task=this.task;
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
