// Pixels and pointer input belong to gzclient running inside the project VM.
const viewport=document.getElementById('nativeViewport');
const status=document.getElementById('nativeStatus');
let display=null, retry=null, active=false, attempt=0,frame=null,canvasObserver=null;
const recovery=document.getElementById('nativeRecovery'),reason=document.getElementById('nativeReason'),retryButton=document.getElementById('retryViewer');
let unavailableSince=0,lastRepair=0,repairJob=null,handshake=null;

function fitViewer() {
  if(!frame)return;
  const width=viewport.clientWidth,height=viewport.clientHeight;
  if(!width||!height)return;
  const canvas=frame.querySelector('canvas');
  const ratio=canvas?.width&&canvas.height?canvas.width/canvas.height:1.6;
  const fitWidth=document.fullscreenElement===viewport||document.body.classList.contains('standalone-gazebo')?Math.min(width,height*ratio):Math.max(width,height*ratio);
  const fitHeight=fitWidth/ratio;
  Object.assign(frame.style,{width:fitWidth+'px',height:fitHeight+'px',left:(width-fitWidth)/2+'px',top:(height-fitHeight)/2+'px'});
  if(display)display.scaleViewport=true;
}
new ResizeObserver(fitViewer).observe(viewport);
window.addEventListener('resize',fitViewer);
document.addEventListener('fullscreenchange',fitViewer);

function message(value) {
  status.textContent=value;
  status.hidden=!value;
  recovery.hidden=!active||!value;
  reason.textContent=value;
}
async function restore(){
  if(repairJob)return repairJob;
  if(!active)return;
  retryButton.disabled=true;lastRepair=Date.now();
  repairJob=(async()=>{
    const response=await fetch('/api/simulation/viewer/start',{method:'POST',signal:AbortSignal.timeout(105000)});
    if(!response.ok)throw Error((await response.json()).error||'重连画面失败');
  })().catch(error=>{if(!viewport.dataset.connected)message(error.message);}).finally(()=>{
    repairJob=null;retryButton.disabled=false;schedule();
  });
  return repairJob;
}
retryButton.onclick=()=>{message('正在重连原生画面…');restore();};
function waitForViewer(){
  if(!unavailableSince)unavailableSince=Date.now();
  if(Date.now()-unavailableSince>7000&&Date.now()-lastRepair>40000)restore();
}
function schedule() {
  if(active&&!retry)retry=setTimeout(()=>{retry=null;connect();},1800);
}
async function connect() {
  if(!active||display)return;
  const generation=++attempt;
  try {
    const response=await fetch('/api/simulation/viewer',{cache:'no-store',signal:AbortSignal.timeout(2200)});
    if(response.status===403){message('请在控制台主机打开 Gazebo 原生视窗');return;}
    if(!response.ok||!(await response.json()).available)throw Error('正在等待虚拟机的 Gazebo 原生窗口…');
    const {default:RFB}=await import('/gazebo/core/rfb.js');
    if(!active||generation!==attempt)return;
    const url=new URL('/gazebo/websockify',location.href);
    url.protocol=location.protocol==='https:'?'wss:':'ws:';
    canvasObserver?.disconnect();
    frame=document.createElement('div');frame.className='native-rfb-frame';
    viewport.replaceChildren(frame);fitViewer();
    const session=display=new RFB(frame,url.href);
    session.scaleViewport=true;
    session.resizeSession=false;
    session.focusOnClick=false;
    session.background='transparent';
    handshake=setTimeout(()=>{if(display===session&&!viewport.dataset.connected){display=null;session.disconnect();waitForViewer();schedule();}},8500);
    session.addEventListener('connect',()=>{if(display===session){
      clearTimeout(handshake);unavailableSince=0;
      message('');viewport.dataset.connected='true';fitViewer();
      const canvas=frame.querySelector('canvas');
      if(canvas){canvasObserver=new MutationObserver(fitViewer);canvasObserver.observe(canvas,{attributes:true,attributeFilter:['width','height']});}
    }});
    session.addEventListener('disconnect',()=>{
      if(display!==session)return;
      clearTimeout(handshake);display=null;delete viewport.dataset.connected;waitForViewer();
      message('原生视窗已断开，正在重新连接…');schedule();
    });
    session.addEventListener('securityfailure',()=>message('原生视窗握手失败，请查看仿真运行日志'));
  }catch(error){
    if(!active||generation!==attempt)return;
    message(error.message.startsWith('正在')?error.message:'正在连接 Gazebo 原生视窗…');
    waitForViewer();
    schedule();
  }
}
message('启动仿真后显示虚拟机内的 Gazebo 原生窗口');
const full=document.getElementById('fullNative');
full.onclick=async()=>{
  if(!viewport.dataset.connected){message('请先启动仿真，再全屏观察');return;}
  try{await viewport.requestFullscreen();}catch{message('浏览器未允许全屏，可在当前视窗观察');}
};
function enable(){
  active=true;message('正在连接 Gazebo 原生窗口…');connect();
}
window.addEventListener('gazebo-connect',enable);
if(document.body.classList.contains('standalone-gazebo'))enable();
window.addEventListener('pagehide',()=>{
  active=false;attempt++;clearTimeout(retry);clearTimeout(handshake);retry=null;
  canvasObserver?.disconnect();
  const old=display;display=null;old?.disconnect();
});
