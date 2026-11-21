const fs=require('node:fs');
const path=require('node:path');
const os=require('node:os');
const net=require('node:net');
const {execFile}=require('node:child_process');
const state=path.join(__dirname,'robot-address.json');
const hostKeyAlias='10.226.235.72';
function address(){try{const ip=JSON.parse(fs.readFileSync(state,'utf8')).ip;if(net.isIPv4(ip))return ip;}catch{}return '10.107.60.72';}
function verify(ip){return new Promise(resolve=>execFile(path.join(process.env.WINDIR||'C:\\Windows','System32','OpenSSH','ssh.exe'),['-n','-i',path.join(os.homedir(),'.ssh','robot_lab_ed25519'),'-o','BatchMode=yes','-o','IdentitiesOnly=yes','-o','StrictHostKeyChecking=yes','-o',`HostKeyAlias=${hostKeyAlias}`,'-o','ConnectTimeout=2',`nvidia@${ip}`,'test -d /home/nvidia/lab26 && echo ROBOT_VERIFIED'],{windowsHide:true,timeout:6000},(err,out)=>resolve(!err&&out.trim()==='ROBOT_VERIFIED')));}
function open(ip){return new Promise(resolve=>{const s=net.connect({host:ip,port:22});const finish=v=>{s.destroy();resolve(v);};s.once('connect',()=>finish(true));s.once('error',()=>finish(false));s.setTimeout(450,()=>finish(false));});}
function candidates(){const result=new Set();for(const [name,entries] of Object.entries(os.networkInterfaces())){if(/vmware|vethernet|loopback/i.test(name))continue;for(const item of entries||[]){if(item.internal||item.family!=='IPv4'||item.netmask!=='255.255.255.0')continue;const prefix=item.address.split('.').slice(0,3).join('.');if(!/^(10\.|192\.168\.|172\.(1[6-9]|2\d|3[01])\.)/.test(prefix+'.'))continue;for(let n=1;n<255;n++){const ip=prefix+'.'+n;if(ip!==item.address)result.add(ip);}}}return [...result];}
let pending;
function discover(){if(pending)return pending;pending=(async()=>{const previous=address();if(await verify(previous))return previous;const list=candidates();let found=null;let next=0;await Promise.all(Array.from({length:24},async()=>{while(!found&&next<list.length){const ip=list[next++];if(ip===previous)continue;if(await open(ip)&&await verify(ip))found=ip;}}));if(!found)throw Error('当前网络未找到通过主机密钥核验的机器人，请检查热点和电源');fs.writeFileSync(state,JSON.stringify({ip:found},null,2)+'\n');return found;})().finally(()=>{pending=null;});return pending;}
module.exports={address,discover,hostKeyAlias};
