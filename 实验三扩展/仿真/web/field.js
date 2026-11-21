const $=id=>document.getElementById(id);
const presets={'交错桩桶':[['cone',1.8,.12],['cone',3.1,-.55],['cone',4.25,.5]],'双侧通道':[-1.26,1.26].flatMap(y=>[1.9,2.38,2.86,3.34,3.82].map(x=>['block',x,y])),'空场':[]};
let objects=structuredClone(presets['交错桩桶']),goal=[5.5,0],tool='pan',state=null,lastRx=0,history=[],pending=null,err='';
let sceneEdited=false,goalEdited=false,firstObservation=true,savedDraftSignature='';
let runtime={phase:'idle',message:''},events=null,starting=false,requestSequence=0,posted=Promise.resolve(),driveSending=false,switching=false,switchMessage='',commandEpoch=0;
let navigation=null;
const owner=crypto.randomUUID?.()||Array.from(crypto.getRandomValues(new Uint8Array(16)),n=>n.toString(16).padStart(2,'0')).join(''),held=new Set();
try{
 const saved=JSON.parse(localStorage.scoutScene);
 if(Array.isArray(saved.objects)&&saved.objects.length<=40&&saved.objects.every(o=>['cone','block'].includes(o[0])&&o.length===3&&o.slice(1).every(n=>Number.isFinite(n)&&Math.abs(n)<=12))&&saved.goal?.length===2&&saved.goal.every(n=>Number.isFinite(n)&&Math.abs(n)<=12)){
  objects=saved.objects;goal=saved.goal;sceneEdited=saved.sceneEdited===true;goalEdited=saved.goalEdited===true;$('preset').value='自定义';
 }
}catch{}
const canvas=$('mapCanvas'),context=canvas.getContext('2d');
let map={width:1,height:1,cx:2.5,cy:0,scale:95},drag=null,currentView='map',openingNative=false;
const live=()=>!!(state&&performance.now()-lastRx<1600);
function screen(x,y){return [map.width/2+(x-map.cx)*map.scale,map.height/2-(y-map.cy)*map.scale]}
function world(x,y){return [map.cx+(x-map.width/2)/map.scale,map.cy-(y-map.height/2)/map.scale]}
function fit(){
 const points=[[0,0],goal,...objects.map(o=>o.slice(1))];
 if(state?.pose)points.push(state.pose.slice(0,2));
 if(live()){points.push(...(state.objects||[]).map(o=>o.slice(1)));if(state.phase==='running'&&state.goal)points.push(state.goal)}
 const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
 const panel=innerWidth>800?document.querySelector('.inspector').getBoundingClientRect().width+36:0;
 map.scale=Math.min((map.width-panel)/(Math.max(...xs)-Math.min(...xs)+2.4),Math.max(180,map.height-220)/(Math.max(...ys)-Math.min(...ys)+3.2));
 map.cx=(Math.max(...xs)+Math.min(...xs))/2+panel/(2*map.scale);map.cy=(Math.max(...ys)+Math.min(...ys))/2;
 draw();
}
function resize(){
 const bounds=$('mapViewport').getBoundingClientRect();
 if(!bounds.width||!bounds.height)return;
 map.width=bounds.width;map.height=bounds.height;
 const ratio=Math.min(devicePixelRatio||1,2);
 canvas.width=Math.round(map.width*ratio);canvas.height=Math.round(map.height*ratio);
 context.setTransform(ratio,0,0,ratio,0,0);draw();
}
new ResizeObserver(resize).observe($('mapViewport'));
function view(name){
 currentView=name;$('nativeViewport').hidden=name!=='native';$('mapViewport').hidden=name!=='map';
 document.body.dataset.simulationView=name;
 $('nativeView').classList.toggle('active',name==='native');$('nativeView').setAttribute('aria-pressed',String(name==='native'));
 $('mapView').classList.toggle('active',name==='map');$('mapView').setAttribute('aria-pressed',String(name==='map'));
 $('fitMap').hidden=name!=='map';$('fullNative').hidden=name!=='native';
 if(name==='map'){resize();fit()}
 window.dispatchEvent(new Event('resize'));
}
$('nativeView').onclick=()=>{
 view('native');
 if(live()||runtime.phase==='ready')window.dispatchEvent(new Event('gazebo-connect'));
 else{err='请先启动仿真，再点击“嵌入原生”';buttons()}
};
$('mapView').onclick=()=>view('map');$('fitMap').onclick=fit;
function line(points,color,width=1.5){
 if(points.length<2)return;
 context.strokeStyle=color;context.lineWidth=width;context.beginPath();
 points.forEach((p,i)=>{const [x,y]=screen(p[0],p[1]);if(i)context.lineTo(x,y);else context.moveTo(x,y)});context.stroke();
}
function circle(x,y,r,color,fill=true){
 const p=screen(x,y);context.beginPath();context.arc(p[0],p[1],r*map.scale,0,Math.PI*2);
 if(fill){context.fillStyle=color;context.fill()}else{context.strokeStyle=color;context.lineWidth=2;context.stroke()}
}
function draw(){
 if(!context)return;context.clearRect(0,0,map.width,map.height);
 const colors={grid:'#46697425',text:'#101619',path:'#286b48',trail:'#8b591e',cloud:'#277b88'};
 context.lineWidth=1;context.strokeStyle=colors.grid;
 const lo=world(0,map.height),hi=world(map.width,0);
 for(let x=Math.ceil(lo[0]);x<=hi[0];x++){const p=screen(x,0);context.beginPath();context.moveTo(p[0],0);context.lineTo(p[0],map.height);context.stroke()}
 for(let y=Math.ceil(lo[1]);y<=hi[1];y++){const p=screen(0,y);context.beginPath();context.moveTo(0,p[1]);context.lineTo(map.width,p[1]);context.stroke()}
 context.fillStyle=colors.text;context.font='11px Consolas,monospace';context.fillText('1 m 网格 · X → / Y ↑',16,map.height-15);
 if(live()){
  line(state.path||[],colors.path);line(state.trail||[],colors.trail,2);
  context.fillStyle=colors.cloud;
  for(const p of state.points||[]){const [x,y]=screen(p[0],p[1]);context.fillRect(x-1,y-1,2,2)}
 }
 const actual=live()?state.objects||[]:[];
 for(const [kind,x,y] of actual){
  if(kind==='cone')circle(x,y,.16,'#e68743');
  else{const p=screen(x-.24,y+.55);context.fillStyle='#758d99';context.fillRect(...p,.48*map.scale,1.10*map.scale)}
 }
 if(!live()||sceneEdited&&!sceneMatches()){
  context.setLineDash([5,4]);context.lineWidth=1.5;
  for(const [kind,x,y] of objects){
   if(kind==='cone')circle(x,y,.16,'#96682d',false);
   else{const p=screen(x-.24,y+.55);context.strokeStyle='#627b86';context.strokeRect(...p,.48*map.scale,1.10*map.scale)}
   const p=screen(x,y);context.fillStyle=colors.text;context.fillText('草稿',p[0]+12,p[1]-10);
  }
  context.setLineDash([]);
 }
 const actualGoal=live()&&state.phase==='running'&&Array.isArray(state.goal)?state.goal:null;
 if(actualGoal){
  circle(...actualGoal,.20,colors.path,false);const p=screen(...actualGoal);context.fillStyle=colors.text;context.fillText('执行目标',p[0]+12,p[1]-12);
 }
 if(!actualGoal||goalEdited&&JSON.stringify(goal)!==JSON.stringify(actualGoal)){
  context.setLineDash(goalEdited?[5,4]:[]);circle(...goal,.20,colors.path,false);context.setLineDash([]);
  const p=screen(...goal);context.fillStyle=colors.text;context.fillText(goalEdited?'草稿终点':'待执行终点',p[0]+12,p[1]-12);
 }
 if(live()&&state.pose){
  const [x,y,h]=state.pose,p=screen(x,y);
  circle(x,y,.06,colors.trail);line([[x,y],[x+.32*Math.cos(h),y+.32*Math.sin(h)]],colors.trail,2.5);
  context.fillStyle=colors.text;context.fillText('实时位姿',p[0]+10,p[1]+16);
 }
}
document.addEventListener('themechange',draw);
function redraw(){
 $('xy').textContent=goal.map(n=>n.toFixed(2)).join(' / ')+' m'+(goalEdited?' · 草稿':'');
 $('count').textContent=live()?(state.objects||[]).length+(sceneEdited?' / 草稿 '+objects.length:''):objects.length+' · 草稿';
 const saved=JSON.stringify({objects,goal,sceneEdited,goalEdited});
 if(saved!==savedDraftSignature){localStorage.scoutScene=saved;savedDraftSignature=saved}
 draw();buttons();
}
function synchronizeEdit(){sceneEdited=true;redraw();if(live())beginRequest('scene')}
canvas.onpointerdown=e=>{drag={x:e.clientX,y:e.clientY,cx:map.cx,cy:map.cy};canvas.setPointerCapture(e.pointerId)};
canvas.onpointermove=e=>{
 if(!drag||tool!=='pan')return;
 map.cx=drag.cx-(e.clientX-drag.x)/map.scale;map.cy=drag.cy+(e.clientY-drag.y)/map.scale;draw();
};
canvas.onpointerup=e=>{
 if(!drag)return;const click=Math.hypot(e.clientX-drag.x,e.clientY-drag.y)<5;drag=null;
 if(!click||tool==='pan'||pending||switching||(live()&&['running','manual','scene_updating'].includes(state.phase)))return;
 const b=canvas.getBoundingClientRect(),p=world(e.clientX-b.left,e.clientY-b.top).map(n=>+n.toFixed(2));
 if(p.some(n=>Math.abs(n)>12)){err='请在 ±12 m 场地内编辑';buttons();return}
 if(tool==='goal'){goal=p;goalEdited=true;redraw();return}
 if(tool==='erase'){
  const distances=objects.map(o=>Math.hypot(o[1]-p[0],o[2]-p[1])),i=distances.indexOf(Math.min(...distances));
  if(i<0||distances[i]>.6)return;objects.splice(i,1);
 }else if(objects.length<40)objects.push([tool,...p]);
 $('preset').value='自定义';synchronizeEdit();
};
canvas.onpointercancel=()=>drag=null;
canvas.onwheel=e=>{e.preventDefault();map.scale=Math.min(450,Math.max(25,map.scale*Math.exp(-e.deltaY*.001)));draw()};
document.querySelectorAll('[data-tool]').forEach(button=>button.onclick=()=>{
 tool=button.dataset.tool;document.querySelectorAll('[data-tool]').forEach(b=>b.classList.toggle('active',b===button));
 $('editHint').textContent=tool==='pan'?'拖动平移，滚轮缩放。':tool==='goal'?'在点云图上选取目标，再点击开始导航。':'在图上编辑障碍，在线时自动同步至 Gazebo。';
 view('map');
});
$('preset').onchange=()=>{objects=structuredClone(presets[$('preset').value]);view('map');synchronizeEdit()};
function ownsDrive(){return !switching&&live()&&state.phase==='manual'&&state.manual_owner===owner}
function ownsNavigation(){
 return !!(state&&state.phase==='running'&&navigation?.accepted&&navigation.epoch===state.control_epoch&&
  (state.navigation_owner?state.navigation_owner===owner:JSON.stringify(state.goal)===JSON.stringify(navigation.goal)));
}
function hasOwnWork(){
 if(state?.phase==='manual')return state.manual_owner===owner;
 if(state?.phase==='running')return ownsNavigation();
 return !!pending;
}
function sceneMatches(expected=objects){return state&&JSON.stringify(expected)===JSON.stringify(state.objects)}
function buttons(){
 const ok=live(),active=ok&&['running','manual','scene_updating'].includes(state.phase),busy=!!pending||switching;
 $('apply').disabled=!ok||active||busy;$('go').disabled=!ok||!state.ready||active||busy;$('stop').disabled=!ok&&!busy;
 $('preset').disabled=active||busy;$('reset').disabled=!ok||active||busy;$('manual').disabled=!ok||!state.ready||active||busy;
 $('startSimulation').disabled=ok||starting||switching||runtime.phase==='starting';
 $('startSimulation').textContent=ok?'仿真已连接':starting||runtime.phase==='starting'?'正在启动…':'启动仿真';
 $('endManual').disabled=!ownsDrive();$('zero').disabled=!ownsDrive();
 document.querySelectorAll('[data-tool]').forEach(b=>b.disabled=active||busy);
 document.querySelectorAll('[data-direction]').forEach(b=>{b.disabled=!ownsDrive();b.classList.toggle('active',held.has(b.dataset.direction))});
 paintDriveControls();
 $('engine').textContent=ok?(state.ready?'本地仿真 · 在线':'本地仿真 · 等待就绪'):runtime.phase==='starting'?'本地仿真 · 启动中':'本地仿真 · 离线';
 $('engine').classList.toggle('live',ok);
 $('note').textContent=err||(switching?switchMessage:pending?(pending.op==='scene'?'正在同步场景…':pending.op==='go'?'正在下发目标…':'正在复位…'):ok?((!state.ready&&state.ready_reason?state.ready_reason:state.note)+(sceneMatches()?'':' · 当前布置待同步')):runtime.message||'点击“启动仿真”，连接 Gazebo 后可运行。');
 for(const [id,n] of [['v',state?.speed[0]],['w',state?.speed[1]],['gap',state?.gap],['distance',state?.distance]])$(id).textContent=ok&&Number.isFinite(n)?n.toFixed(2):'—';
}
async function jsonRequest(url,body){
 const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),keepalive:true,signal:AbortSignal.timeout(5000)});
 const result=await response.json();if(!response.ok)throw Error(result.error||result.message||'请求失败');return result;
}
function publish(msg){
 // Preserve the order of goal/stop requests when a page is left during submission.
 const epoch=commandEpoch;
 const action=()=>epoch!==commandEpoch&&msg.op!=='stop'?Promise.resolve({cancelled:true}):jsonRequest('/api/simulation/command',msg);
 const result=posted.then(action,action);posted=result.catch(()=>{});return result;
}
function cancelPending(){pending=null;requestSequence++}
function fail(message){cancelPending();err=message;buttons()}
function beginRequest(op,thenGo=false,expectedEpoch=state?.control_epoch){
 if(switching)return;
 if(!live()){fail('仿真尚未连接，请先启动 Gazebo');return}
 if((op==='go'||thenGo)&&(!Number.isSafeInteger(expectedEpoch)||expectedEpoch!==state.control_epoch)){fail('控制状态已更新，请重新确认目标后启动');return}
 err='';const id=owner+':'+(++requestSequence);
 pending={id,op,thenGo,controlEpoch:expectedEpoch,objects:structuredClone(objects),goal:[...goal],started:performance.now()};
 if(op==='go')navigation={id,epoch:expectedEpoch,goal:[...goal],accepted:false};
 publish({op,objects:pending.objects,goal:pending.goal,owner,control_epoch:expectedEpoch,request_id:id}).catch(error=>{if(pending?.id===id){if(navigation?.id===id)navigation=null;fail(error.message)}});buttons();
}
function stop(explicit=false){
 commandEpoch++;cancelPending();held.clear();navigation=null;err='';const result=publish({op:'stop',owner,explicit}).catch(error=>{err=error.message;buttons()});buttons();return result;
}
function drive(){
 paintDriveControls();
 if(!ownsDrive()||driveSending||switching)return;
 driveSending=true;
 publish({op:'drive',owner,axes:[Number(held.has('w'))-Number(held.has('s')),Number(held.has('a'))-Number(held.has('d'))],speed:Number($('manualSpeed').value),turn:Number($('manualTurn').value)})
  .catch(error=>{if(!switching)fail(error.message)}).finally(()=>{driveSending=false});
}
function release(){held.clear();drive()}
function paintDriveControls(){
 document.querySelectorAll('.drive-settings input,.drive-settings button').forEach(input=>input.disabled=held.size>0||switching);
 const linear=Number(held.has('w'))-Number(held.has('s')),angular=Number(held.has('a'))-Number(held.has('d'));
 $('keyState').textContent=[linear>0?'W':linear<0?'S':'',angular>0?'A':angular<0?'D':''].filter(Boolean).join(' + ')||'双轴组合';
}
function commitDriveValue(input,value){
 const min=Number(input.min),max=Number(input.max),step=Number(input.step)||1,parsed=Number(value);
 const safe=String(value).trim()!==''&&Number.isFinite(parsed)?parsed:Number(input.dataset.last||input.defaultValue||min);
 input.value=Number(Math.max(min,Math.min(max,min+Math.round((safe-min)/step)*step)).toFixed(6));input.dataset.last=input.value;
 document.querySelectorAll('[data-sync]').forEach(slider=>{if(slider.dataset.sync===input.id)slider.value=input.value});
}
document.addEventListener('click',event=>{const button=event.target.closest('.drive-settings [data-step]');if(!button||button.disabled)return;const input=$(button.dataset.step);if(input&&!input.disabled)commitDriveValue(input,Number(input.value)+Number(button.dataset.delta))});
document.addEventListener('input',event=>{if(event.target.matches('.drive-settings input[data-sync]')){const input=$(event.target.dataset.sync);if(!input.disabled)commitDriveValue(input,event.target.value)}});
document.addEventListener('change',event=>{if(event.target.matches('.drive-settings input[type=number]'))commitDriveValue(event.target,event.target.value)});
document.addEventListener('keydown',event=>{if(event.target.matches('.drive-settings input[type=number]')&&event.key==='Enter'){commitDriveValue(event.target,event.target.value);event.target.blur()}});
document.addEventListener('focusin',event=>{if(event.target.matches('.drive-settings input'))release()});
$('apply').onclick=()=>beginRequest('scene');
$('go').onclick=()=>beginRequest(sceneMatches()?'go':'scene',!sceneMatches());
$('reset').onclick=()=>beginRequest('reset');$('stop').onclick=()=>stop(true);$('endManual').onclick=()=>stop(true);
$('manual').onclick=()=>{release();if(!Number.isSafeInteger(state?.control_epoch)){fail('控制状态尚未就绪，请等待后重试');return}beginRequest('manual')};$('zero').onclick=release;
$('startSimulation').onclick=async()=>{
 starting=true;err='';buttons();
 try{runtime=await jsonRequest('/api/simulation/start',{});subscribe()}catch(error){err=error.message}finally{starting=false;buttons()}
};
document.querySelectorAll('[data-direction]').forEach(b=>{b.onpointerdown=e=>{if(!ownsDrive())return;e.preventDefault();b.setPointerCapture(e.pointerId);held.add(b.dataset.direction);drive()};b.onpointerup=b.onpointercancel=b.onlostpointercapture=()=>{held.delete(b.dataset.direction);drive()}});
window.addEventListener('keydown',e=>{if(e.target.matches('input,select,textarea'))return;if(e.code==='Space'){e.preventDefault();if(!e.repeat)stop(true);return}const k=e.key.toLowerCase();if(ownsDrive()&&['w','a','s','d'].includes(k)){e.preventDefault();held.add(k);drive()}});
window.addEventListener('keyup',e=>{held.delete(e.key.toLowerCase());drive()});
function leave(){if(hasOwnWork())stop();else{commandEpoch++;cancelPending();held.clear()}}
$('openNative').addEventListener('click',()=>{openingNative=true});
window.addEventListener('focus',()=>openingNative=false);
window.addEventListener('blur',()=>{if(!switching&&!(openingNative&&ownsNavigation()))leave()});
window.addEventListener('pagehide',()=>{if(!switching)leave()});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&!switching&&!(openingNative&&ownsNavigation()))leave()});
async function switchTarget(event){
 event.preventDefault();if(switching)return;
 const active=state&&(['running','manual','scene_updating'].includes(state.phase)||(state.speed||[]).some(n=>Math.abs(n)>=.025));
 const own=hasOwnWork();
 if(!live()){
  if(active||pending){err='仿真连接中断，无法确认车辆停稳，请恢复连接后切换';buttons();return}
  commandEpoch++;cancelPending();held.clear();location.assign(event.currentTarget.href);return;
 }
 if(active&&!own){err='当前仿真由其他页面控制，请等待车辆停稳后切换';commandEpoch++;cancelPending();held.clear();buttons();return}
 if(!active&&!own){commandEpoch++;cancelPending();held.clear();location.assign(event.currentTarget.href);return}
 const url=event.currentTarget.href;
 switching=true;switchMessage='正在停止本地仿真，等待车辆停稳…';err='';commandEpoch++;cancelPending();held.clear();buttons();
 const id=owner+':stop:'+ ++requestSequence;
 try{
  await publish({op:'stop',owner,request_id:id});
  const deadline=performance.now()+8000;let stableSince=0,lastFrame=0,samples=0;
  while(performance.now()<deadline){
   if(!live())throw Error('仿真连接中断，无法确认停稳，已保留当前页面');
   const ack=state.ack?.id===id?state.ack:null;
   if(ack?.ok===false)throw Error(ack.message||'仿真停车请求失败');
   const stopped=ack?.ok===true&&!['running','manual','scene_updating'].includes(state.phase)&&Array.isArray(state.speed)&&state.speed.length===2&&state.speed.every(Number.isFinite)&&Math.abs(state.speed[0])<.025&&Math.abs(state.speed[1])<.04;
   if(stopped){if(!stableSince)stableSince=performance.now();if(lastFrame!==lastRx){lastFrame=lastRx;samples++}if(samples>=3&&performance.now()-stableSince>=300){tool='pan';location.assign(url);return}}
   else{stableSince=0;samples=0}
   await new Promise(resolve=>setTimeout(resolve,100));
  }
  throw Error('未收到连续停稳反馈，当前页面已保留，请先停止车辆再切换');
 }catch(error){err=error.message}finally{switching=false;switchMessage='';buttons()}
}
document.querySelectorAll('[data-target="hardware"]').forEach(link=>link.addEventListener('click',switchTarget));
setInterval(()=>{if(!document.hidden&&document.hasFocus())drive()},120);
function observe(d){
 if(d.engine!=='gazebo-classic')return;
 state=d;lastRx=performance.now();
 if(sceneEdited&&sceneMatches())sceneEdited=false;
 if(!sceneEdited&&Array.isArray(d.objects)){
  const changed=JSON.stringify(objects)!==JSON.stringify(d.objects);objects=structuredClone(d.objects);
  if(changed)$('preset').value=Object.keys(presets).find(name=>JSON.stringify(presets[name])===JSON.stringify(objects))||'自定义';
 }
 if(d.phase==='running'&&Array.isArray(d.goal)){
  if(goalEdited&&JSON.stringify(goal)===JSON.stringify(d.goal))goalEdited=false;
  if(!goalEdited)goal=[...d.goal];
 }
 if(navigation){
  if(navigation.epoch!==d.control_epoch||(d.ack?.id===navigation.id&&d.ack.ok===false))navigation=null;
  else if(d.ack?.id===navigation.id&&d.ack.ok===true&&d.phase==='running')navigation.accepted=true;
  else if(navigation.accepted&&d.phase!=='running')navigation=null;
 }
 if(pending&&(pending.op==='go'||pending.thenGo)&&pending.controlEpoch!==d.control_epoch)fail('车辆已停止或复位，旧导航请求已取消，请重新确认目标');
 if(pending){
  const ack=d.ack?.id===pending.id?d.ack:null;
  if(ack?.ok===false)fail(ack.message||'Gazebo 请求执行失败');
  else if(pending.op==='scene'&&sceneMatches(pending.objects)&&(!d.ack||ack?.ok)){
   const go=pending.thenGo,epoch=pending.controlEpoch;pending=null;if(go)beginRequest('go',false,epoch);
  }else if(pending?.op==='go'&&d.phase==='running'&&JSON.stringify(d.goal)===JSON.stringify(pending.goal))pending=null;
  else if(pending?.op==='manual'&&ack?.ok&&d.phase==='manual'&&d.manual_owner===owner)pending=null;
  else if(pending?.op==='reset'&&d.phase==='ready'&&(!d.ack||ack?.ok))pending=null;
 }
 if(firstObservation){firstObservation=false;fit()}
 redraw();
 if(d.phase==='running'||history.at(-1)?.phase!==d.phase){history.push({time:d.time,phase:d.phase,pose:d.pose,command:d.command,speed:d.speed,gap:d.gap,contact:d.contact});if(history.length>12000)history.shift()}
 buttons();
}
function subscribe(){
 if(events)return;
 events=new EventSource('/api/simulation/events');
 events.onmessage=e=>{try{observe(JSON.parse(e.data))}catch(error){err=error.message;buttons()}};
 events.onerror=()=>{lastRx=-1e8;if(pending)fail('仿真连接中断，当前请求已取消');buttons()};
}
window.addEventListener('pagehide',()=>{events?.close();events=null;lastRx=-1e8;});
async function runtimeStatus(){try{const response=await fetch('/api/simulation/status',{signal:AbortSignal.timeout(1500)});if(response.ok)runtime=await response.json()}catch{}buttons()}
setInterval(()=>{
 if(pending&&performance.now()-pending.started>12000){const owned=hasOwnWork();fail('Gazebo 请求超时，请查看连接状态后重试');if(owned)publish({op:'stop',owner}).catch(()=>{})}
 if(pending&&!live())fail('点云或仿真状态已过期，请重新启动任务');buttons();
},500);
setInterval(runtimeStatus,1600);runtimeStatus();
$('save').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify({schema:1,objects,goal,engine:'gazebo-classic',samples:history,latest:state},null,2)],{type:'application/json'}));a.download='scout-field.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};
redraw();view('map');
