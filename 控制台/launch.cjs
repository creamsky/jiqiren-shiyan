const {spawn, execFile} = require('node:child_process');
const path = require('node:path');
const fs = require('node:fs');
const net = require('node:net');
const robotAddress=require('./robot-address.cjs');
const port=8765;
const origin='http://127.0.0.1:'+port;
const logs = path.join(__dirname, 'logs');
fs.mkdirSync(logs, {recursive:true});
function background(file, args, name) {
  const out = fs.openSync(path.join(logs, name+'.log'), 'a');
  const child = spawn(file, args, {cwd:__dirname, detached:true, windowsHide:true, stdio:['ignore',out,out]});
  child.on('error', error=>console.error(error.message));
  child.unref(); fs.closeSync(out); return child;
}
async function status() {
  try {return await (await fetch(origin+'/api/status',{signal:AbortSignal.timeout(800)})).json();}
  catch {return null;}
}
function occupied(value){return new Promise(resolve=>{const socket=net.connect({host:'127.0.0.1',port:value});const finish=result=>{socket.destroy();resolve(result);};socket.once('connect',()=>finish(true));socket.once('error',()=>finish(false));socket.setTimeout(500,()=>finish(true));});}
async function launch(){
  const hardware=process.argv.includes('--real');
  const url=origin+(hardware?'/hardware/':'/simulation/');
  let current=await status();
  // A single fixed gateway prevents two independent ownership locks for one robot.
  if(current?.service!=='robot-lab-console' && await occupied(port)) throw Error('Port 8765 is occupied. Close the conflicting service before starting the shared console.');
  if(current?.service==='robot-lab-console'&&current.sharedVersion!==2)throw Error('请先双击“关闭控制台.cmd”退出旧版本，再打开新控制台。');
  if(!current){
    background(process.execPath,[path.join(__dirname,'server.cjs'),'--port',String(port)],'frontend');
    for(let n=0;n<25;n++){
      await new Promise(resolve=>setTimeout(resolve,200));
      current=await status(); if(current)break;
    }
  }
  if(current?.service!=='robot-lab-console') throw Error('Local console did not start. See logs/frontend.log.');
  if(!process.argv.includes('--no-open'))execFile('rundll32.exe',['url.dll,FileProtocolHandler',url],{windowsHide:true},()=>{});
  console.log('Console opened: '+url);
  if(hardware)for(const address of current.urls||[])console.log('Share with teammates: '+address);
}
let robotTunnel=null;
function closeRobotLink(){robotTunnel?.kill();robotTunnel=null;}
function tunnelRunning(){return robotTunnel?.exitCode===null && robotTunnel.signalCode===null && !robotTunnel.killed;}
async function ensureRobotLink(signal){
  if(!tunnelRunning() && await occupied(18766))throw Error('真机转发端口 18766 已在使用，请先关闭以前的真机控制台');
  const key=path.join(process.env.USERPROFILE,'.ssh','robot_lab_ed25519');
  try{fs.accessSync(key,fs.constants.R_OK);}catch{throw Error('未找到机器人 SSH 密钥');}
  const ssh=path.join(process.env.WINDIR,'System32','OpenSSH','ssh.exe');
  const ip=robotAddress.address();
  const auth=['-n','-i',key,'-o','BatchMode=yes','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes','-o',`HostKeyAlias=${robotAddress.hostKeyAlias}`,'-o','ConnectTimeout=8'];
  // Older bootstraps actively request CAN mode. Only launch the observation-capable version.
  const command="cd /home/nvidia/lab26 && if test -f console_watch.sh; then for i in 1 2 3 4 5; do ss -ltnH 'sport = :8766' | grep -q . && exit 0; sleep 1; done; exit 43; elif grep -qx '# OBSERVE_ONLY_CONNECT_V1' launch_console_service.sh; then bash ./launch_console_service.sh; else ss -ltnH 'sport = :8766' | grep -q . || exit 42; fi";
  await new Promise((resolve,reject)=>execFile(ssh,[...auth,`nvidia@${ip}`,command],{signal,windowsHide:true,timeout:65000},(error,stdout,stderr)=>{
    fs.writeFileSync(path.join(logs,'robot-connect.log'),stdout+'\n'+stderr+(error?'\n'+error.message:''));
    if(!error)return resolve();
    const canReason=(stdout+'\n'+stderr).match(/^CAN:\s*([^\r\n]+)/m);
    if(error.code===42)return reject(Object.assign(Error('车载电脑在线，请先更新车端观察模式启动脚本'),{manual:true}));
    if(error.code===43)return reject(Error('车载电脑在线，正在等待车端常驻服务就绪'));
    if(canReason)return reject(Error(canReason[1]));
    reject(Error(error.code===255?'无法连接机器人 SSH，请检查机器人地址和无线网络':'车端控制服务启动或检查失败，详见 logs/robot-connect.log'));
  }));
  signal.throwIfAborted();
  if(tunnelRunning())return;
  robotTunnel=background(ssh,[...auth,'-N','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=2','-o','ServerAliveCountMax=3','-L','127.0.0.1:18766:127.0.0.1:8766',`nvidia@${ip}`],'robot-tunnel');
  for(let n=0;n<30;n++){
    signal.throwIfAborted();
    if(!tunnelRunning())break;
    if(await occupied(18766))return;
    await new Promise(resolve=>setTimeout(resolve,200));
  }
  closeRobotLink();
  throw Error('机器人 SSH 转发未建立，详见 logs/robot-tunnel.log');
}
if(require.main===module)launch().catch(error=>{console.error(error.message);process.exitCode=1;});
module.exports={ensureRobotLink,closeRobotLink};
