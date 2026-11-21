'use strict';
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const {spawn} = require('node:child_process');
const {createInterface} = require('node:readline');
const address = require('../控制台/robot-address.cjs');
const sourceErrors={
  scan_pair:'等待车端同步的雷达标记和清除射线',
  pose:'收到雷达，等待相同时刻的定位',
  generation:'等待车端地图编号',
};

class MazeScanStream {
  constructor(onScan) {
    this.onScan=onScan; this.child=null; this.nextTry=0;
    this.sourceAt=0; this.lastSampleAt=0; this.error='等待读取实车雷达';
  }
  accept(line,now=Date.now()) {
    if(line.length>1500000)return false;
    let scan;
    try {scan=JSON.parse(line);} catch {return false;}
    if(scan?.type==='scan_status'){
      this.error=Object.hasOwn(sourceErrors,scan.issue)?sourceErrors[scan.issue]:'等待读取实车雷达';
      return false;
    }
    if(scan?.type==='lidar_mount'){this.onScan(scan);return false;}
    if(!Number.isFinite(scan?.stamp)||typeof scan.generation!=='string')return false;
    this.sourceAt=now;
    if(this.onScan(scan)){
      this.lastSampleAt=now;this.error='';return true;
    }
    this.error='收到雷达和定位，等待与控制台当前地图对齐';
    return false;
  }
  start() {
    const now=Date.now();
    if(this.child&&now-Math.max(this.startedAt,this.sourceAt)>10000){
      this.stop();this.nextTry=now+1000;return;
    }
    if(this.child || now<this.nextTry)return;
    const args=['-T','-i',path.join(os.homedir(),'.ssh','robot_lab_ed25519'),
      '-o','BatchMode=yes','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes',
      '-o','HostKeyAlias='+address.hostKeyAlias,'-o','ConnectTimeout=5','-o','ServerAliveInterval=3',
      '-o','ServerAliveCountMax=2','nvidia@'+address.address(),
      'source /home/nvidia/lab26/env.sh && exec /usr/bin/python3 -u -'];
    const child=this.child=spawn(path.join(process.env.WINDIR,'System32','OpenSSH','ssh.exe'),args,{windowsHide:true});
    this.startedAt=now;
    this.error='等待同一时刻的雷达与定位';
    child.stdin.on('error',()=>{});
    child.stdin.end(fs.readFileSync(path.join(__dirname,'scan_stream.py')));
    const lines=createInterface({input:child.stdout});
    lines.on('line',line=>{
      if(this.child!==child)return;
      try {this.accept(line);} catch {this.error='本帧雷达数据未能合入当前地图';}
    });
    child.stderr.on('data',buf=>{
      if(this.child===child&&Date.now()-this.lastSampleAt>800&&/ssh:|permission denied|traceback/i.test(buf.toString()))
        this.error='迷宫雷达转发出错，正在重新连接';
    });
    child.on('error',()=>{if(this.child===child)this.error='无法启动迷宫雷达转发';});
    child.on('close',()=>{
      lines.close();
      if(this.child===child){this.child=null;this.nextTry=Date.now()+1200;this.error='迷宫雷达转发已断开，正在重新连接';}
    });
  }
  stop() { const child=this.child;this.child=null;child?.kill(); }
}
module.exports={MazeScanStream};
