'use strict';
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const {spawn} = require('node:child_process');
const {createInterface} = require('node:readline');
const address = require('../控制台/robot-address.cjs');

class MazeScanStream {
  constructor(onScan) { this.onScan=onScan; this.child=null; this.nextTry=0; this.error='等待读取实车雷达'; }
  start() {
    if(this.child&&Date.now()-this.lastSampleAt>10000)this.stop();
    if (this.child || Date.now()<this.nextTry) return;
    const args=['-T','-i',path.join(os.homedir(),'.ssh','robot_lab_ed25519'),
      '-o','BatchMode=yes','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes',
      '-o','HostKeyAlias='+address.hostKeyAlias,'-o','ConnectTimeout=5','-o','ServerAliveInterval=3',
      '-o','ServerAliveCountMax=2','nvidia@'+address.address(),
      'source /home/nvidia/lab26/env.sh && exec /usr/bin/python3 -u -'];
    const child=this.child=spawn(path.join(process.env.WINDIR,'System32','OpenSSH','ssh.exe'),args,{windowsHide:true});
    this.lastSampleAt=Date.now();
    this.error='等待同一时刻的雷达与定位';
    child.stdin.on('error',()=>{});
    child.stdin.end(fs.readFileSync(path.join(__dirname,'scan_stream.py')));
    const lines=createInterface({input:child.stdout});
    lines.on('line',line=>{
      if(this.child!==child || line.length>500000) return;
      try { const scan=JSON.parse(line); if(this.onScan(scan)){this.lastSampleAt=Date.now();this.error='';} } catch {}
    });
    child.stderr.on('data',buf=>{if(this.child===child)this.error=buf.toString().trim().slice(-210);});
    child.on('error',error=>{this.error=error.message;});
    child.on('close',()=>{lines.close();if(this.child===child){this.child=null;this.nextTry=Date.now()+2500;}});
  }
  stop() { const child=this.child;this.child=null;child?.kill(); }
}
module.exports={MazeScanStream};
