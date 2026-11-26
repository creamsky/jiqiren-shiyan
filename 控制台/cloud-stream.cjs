'use strict';
const {spawn}=require('node:child_process');
const path=require('node:path');
const os=require('node:os');
const robotAddress=require('./robot-address.cjs');

class CloudStream {
  constructor({run=spawn,now=Date.now,source=null}={}) {this.run=run;this.now=now;this.source=source;this.frames=new Map();this.child=null;this.retry=null;this.active=false;this.error='正在连接点云';}
  accept(line) {
    let data;try{data=JSON.parse(line);}catch{return false;}
    if(data?.type!=='cloud'||!['aggregate','raw'].includes(data.source)||!Number.isFinite(data.stamp)||
      !Array.isArray(data.points)||data.points.length>1000000||!data.points.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite))||
      !Number.isInteger(data.frames)||data.frames<1||!Number.isSafeInteger(data.sourcePoints)||data.sourcePoints<data.points.length||
      (data.source==='aggregate'&&data.registered!==true))return false;
    this.frames.set(data.source,{data:{...data,frame:String(data.frame||'').replace(/^\//,'').slice(0,100)},at:this.now()});this.error='';return true;
  }
  snapshot(source) {
    const frame=this.frames.get(source),age=frame?(this.now()-frame.at)/1000:null;
    return {...(frame?.data||{points:[],frame:'',source,frames:0}),age,connected:!!this.child&&age!==null&&age<3,error:this.error};
  }
  start() {if(this.active)return;this.active=true;this.connect();}
  connect() {
    if(!this.active)return;
    const key=path.join(os.homedir(),'.ssh','robot_lab_ed25519');
    const ssh=path.join(process.env.WINDIR||'C:\\Windows','System32','OpenSSH','ssh.exe');
    const args=['-n','-T','-C','-i',key,'-o','BatchMode=yes','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes',
      '-o',`HostKeyAlias=${robotAddress.hostKeyAlias}`,'-o','ConnectTimeout=5','-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2',
      `nvidia@${robotAddress.address()}`,'source /home/nvidia/lab26/env.sh && exec nice -n 5 /home/nvidia/robot-lab-viewer/'+(this.source?'cloud-preview '+this.source:'cloud-stream')];
    let buffer='';const child=this.run(ssh,args,{windowsHide:true,stdio:['ignore','pipe','pipe']});this.child=child;
    child.stdout.setEncoding('utf8');child.stdout.on('data',chunk=>{
      if(this.child!==child)return;
      buffer+=chunk;
      if(buffer.length>64*1024*1024){this.error='点云数据过大，正在重新连接';child.kill();return;}
      let index;while((index=buffer.indexOf('\n'))>=0){const line=buffer.slice(0,index);buffer=buffer.slice(index+1);this.accept(line);}
    });
    child.stderr.on('data',()=>{this.error='点云连接未就绪，正在重试';});
    child.on('error',()=>{this.error='无法启动点云 SSH 连接';});
    child.on('close',()=>{
      if(this.child!==child)return;
      this.child=null;
      if(this.active){this.error='点云连接已断开，正在重试';this.retry=setTimeout(()=>this.connect(),3000);this.retry.unref();}
    });
  }
  stop() {this.active=false;clearTimeout(this.retry);this.child?.kill();this.child=null;this.frames.clear();this.error='等待连接真机';}
}
class CloudViews {
  constructor(make=source=>new CloudStream({source})){
    this.streams=new Map(['raw','aggregate'].map(source=>[source,make(source)]));this.timers=new Map();
  }
  snapshot(source,online){
    const stream=this.streams.get(source);
    if(online){
      stream.start();clearTimeout(this.timers.get(source));
      const idle=setTimeout(()=>{stream.stop();this.timers.delete(source);},3500);
      idle.unref();this.timers.set(source,idle);
    }
    return stream.snapshot(source);
  }
  stop(){for(const timer of this.timers.values())clearTimeout(timer);this.timers.clear();for(const stream of this.streams.values())stream.stop();}
}
module.exports={CloudStream,CloudViews};
