const $=id=>document.getElementById(id);
let token='', seq=0, mine=false, online=false, manual=false, direction='', busy=false, taskActive=false, chain=Promise.resolve();
let vectorSupported=false;
let submittingStart=false;
let lease='',armed=false,roomState=null,renewing=null,switching=false,switchHint='',controlEpoch=0,lastSnapshot=null;
const held=new Map();
const message=text=>{$('message').textContent=text;};
function applyRoom(room){
  if(!room)return;
  const changed=lease!==room.lease;
  roomState=room;lease=room.lease||'';mine=room.mine;
  if(changed||!mine||room.stopping){held.clear();direction='';armed=false;manual=false;paintKeys();}
  window.RoomUI?.render(room,online);
}
function post(url,data,urgent=false){
  const expectedLease=lease,expectedToken=token,epoch=controlEpoch;
  const run=async()=>{if(token!==expectedToken)throw Error('连接已更新，请重新操作');if(epoch!==controlEpoch&&['start','drive','manual','auto','w','a','s','d','resume'].includes(data.command))throw Error('控制目标已切换，旧运动请求已取消');const payload=JSON.stringify({...data,lease:expectedLease,seq:++seq});const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:payload,signal:AbortSignal.timeout(url==='/api/ping'?120000:1800)});const result=await response.json();if(!response.ok)throw Error(result.error||'请求失败');if(result.room)applyRoom(result.room);return result;};
  if(urgent)return run();
  const next=chain.then(run);chain=next.catch(()=>{});return next;
}
async function command(value,intent,prepareId){
  if(switching&&!['zero','stop','pause'].includes(value))return false;
  const epoch=controlEpoch,experiment=$('experiment').value;
  try{
    const payload={command:value};
    if(['start','pause','resume','stop'].includes(value))payload.experiment=experiment;
    if(value==='start'){
      payload.intent=intent;payload.prepareId=prepareId;payload.options=window.Workbench.options();
      if(intent==='run'&&experiment==='lab3-xy')Object.assign(payload.options,window.NavigationGoal?.());
      if(intent==='run'&&experiment==='maze')Object.assign(payload.options,window.MazeView?.options());
    }
    await post('/api/command',payload,['zero','stop','pause'].includes(value));return epoch===controlEpoch;
  }catch(error){message(error.message);return false;}
}
function paintKeys(){
  const keys=new Set(held.values());
  document.querySelectorAll('[data-direction]').forEach(b=>b.classList.toggle('active',keys.has(b.dataset.direction)));
  $('keyState').textContent=direction?direction.toUpperCase().split('').join(' + '):'双轴组合';
  document.querySelectorAll('.drive-settings input,.drive-settings button').forEach(input=>input.disabled=held.size>0||(online&&!vectorSupported));
}
function zero(){held.clear();direction='';paintKeys();if(mine&&online)command('zero');}
async function sendDrive(){
  if(!mine||!online||!manual||switching)return;
  const keys=new Set(held.values()),linear=Number(keys.has('w'))-Number(keys.has('s')),angular=Number(keys.has('a'))-Number(keys.has('d'));
  direction=(linear>0?'w':linear<0?'s':'')+(angular>0?'a':angular<0?'d':'');paintKeys();
  if(!direction){await command('zero');return;}
  if(!vectorSupported){
    if(direction.length===1&&Number($('manualSpeed').value)===1.5&&Number($('manualTurn').value)===.3){await command(direction);return;}
    zero();message('组合驾驶和自定义速度需要更新机器人端控制程序');return;
  }
  try{await post('/api/command',{command:'drive',linear,angular,speed:Number($('manualSpeed').value),turn:Number($('manualTurn').value)},true);}
  catch(error){zero();message(error.message);}
}
function hold(key,source='key:'+key){if(!mine||!online||!manual||switching)return;held.set(source,key);sendDrive();}
function release(source){if(held.delete(source))sendDrive();}
$('claim').onclick=async()=>{if(await command('claim')){mine=true;message('已取得控制权');}};
$('ping').onclick=async()=>{
  const label=$('ping').querySelector('span');
  try{$('ping').disabled=true;$('ping').setAttribute('aria-busy','true');label.textContent='检测中…';$('pingState').textContent='正在检测机器人网络';
    const result=await post('/api/ping',{},true);const latency=String(result.output).match(/(?:time|时间)\s*([=<＝＜])\s*(\d+)\s*ms/i);
    if(result.ip)document.querySelector('.address').textContent=result.ip;
    $('pingState').textContent=result.ok?'网络可达'+(latency?' · '+(latency[1]==='<'||latency[1]==='＜'?'<':'')+latency[2]+' ms':''):'网络不可达 · 检查电源与网络';
    $('pingState').dataset.state=result.ok?'ok':'error';message(result.ok?'网络检测完成；控制服务连接状态单独显示':'网络检测未收到机器人回应');
  }catch(error){$('pingState').textContent='检测失败 · '+error.message;message(error.message);}
  finally{$('ping').disabled=false;$('ping').setAttribute('aria-busy','false');label.textContent='检测网络';}
};
const hardwareButton=$('connectHardware');
if(hardwareButton)hardwareButton.onclick=async()=>{
  hardwareButton.disabled=true;$('pingState').textContent='正在连接真机…';
  try{
    const response=await fetch('/api/hardware/connect',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',signal:AbortSignal.timeout(5000)});
    const result=await response.json();if(!response.ok)throw Error(result.error||result.message||'连接失败');
    $('pingState').textContent=result.message||'正在建立机器人连接';
  }catch(error){$('pingState').textContent=error.message;hardwareButton.disabled=false;}
};
document.querySelectorAll('[data-command]').forEach(button=>button.onclick=async()=>{
  const c=button.dataset.command;
  if(c==='start'){
    if(submittingStart||!button.dataset.intent)return;
    submittingStart=true;button.disabled=true;
    const {intent,prepareId}=button.dataset;
    try{
      zero();
      if(await command(c,intent,prepareId))message(intent==='prepare'?'已请求准备实验':'已发送启动请求，等待车端反馈');
      await refresh();
    }finally{submittingStart=false;}
    return;
  }
  zero();
  if(c==='manual'){
    if(!mine){if(!await command('claim'))return;mine=true;}
    if(await command('manual')){armed=true;manual=true;message('键盘驾驶已启用：W/S 前后，A/D 转向，可组合按下');}
  }else if(await command(c))message('已发送：'+button.textContent);
});
$('zero').onclick=zero;
$('stop').onclick=()=>{zero();command('stop');};
document.querySelectorAll('[data-direction]').forEach(button=>{
  button.addEventListener('pointerdown',event=>{event.preventDefault();button.setPointerCapture(event.pointerId);hold(button.dataset.direction,'pointer:'+event.pointerId);});
  for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,e=>release('pointer:'+e.pointerId));
});
document.addEventListener('keydown',event=>{
  if(event.ctrlKey||event.altKey||event.metaKey||event.target.matches('input,textarea,[contenteditable="true"]')||$('experimentLibrary').open||$('joinDialog')?.open||$('shareDialog')?.open||$('takeoverDialog')?.open)return;
  const key=event.key.toLowerCase();
  if(key===' '){event.preventDefault();zero();if(online||taskActive)command('stop');return;}
  if('wasd'.includes(key)&&key.length===1){event.preventDefault();if(!event.repeat)hold(key);}
});
document.addEventListener('keyup',event=>release('key:'+event.key.toLowerCase()));
function suspend(){if(manual||held.size)zero();armed=false;manual=false;}
function leaveControl(){
  if(switching)return;
  suspend();if(mine&&token){fetch('/api/room',{method:'POST',headers:{'Content-Type':'application/json','X-Control-Token':token},body:JSON.stringify({action:'release',seq:++seq}),keepalive:true}).catch(()=>{});}mine=false;lease='';
}
window.addEventListener('blur',suspend);
document.addEventListener('visibilitychange',()=>{if(document.hidden)suspend();});
window.addEventListener('pagehide',leaveControl);
function hardwareActive(data){
  const state=data?.status||{};
  return !!(data?.task&&['preparing','running','stopping'].includes(data.task.phase))||
    ['v','w','feedback_v','feedback_w'].some(key=>Number.isFinite(state[key])&&Math.abs(state[key])>=.025)||
    (!state.paused&&(['DRIVING','TURNING'].includes(state.stage)||state.mode==='manual'));
}
async function readHardware(){
  const response=await fetch('/api/status',{headers:{'X-Control-Token':token},signal:AbortSignal.timeout(1000),cache:'no-store'});
  if(!response.ok)throw Error('无法读取真实机器人状态');
  const data=await response.json();lastSnapshot=data;return data;
}
function hardwareStopped(data){
  const state=data.status||{};
  return data.connected&&Number.isFinite(data.age)&&data.age<500&&state.feedback_fresh===true&&
    [state.v,state.w,state.feedback_v,state.feedback_w].every(Number.isFinite)&&
    Math.abs(state.v)<.001&&Math.abs(state.w)<.001&&Math.abs(state.feedback_v)<.025&&Math.abs(state.feedback_w)<.04&&
    (state.paused||['FAULT','DONE','IDLE'].includes(state.stage))&&
    !['preparing','running','stopping'].includes(data.task?.phase);
}
async function switchToSimulation(event){
  event.preventDefault();if(switching)return;
  const url=event.currentTarget.href;
  switching=true;switchHint='正在检查真实机器人是否停稳…';message(switchHint);
  controlEpoch++;held.clear();direction='';armed=false;manual=false;paintKeys();
  async function leaveHardware(){
    if(!token)await openSession();
    const response=await fetch('/api/hardware/leave',{method:'POST',headers:{'X-Control-Token':token},signal:AbortSignal.timeout(2000)});
    if(!response.ok)throw Error((await response.json()).error||'退出真机会话失败');
    mine=false;lease='';document.dispatchEvent(new Event('control-mode-reset'));location.assign(url);
  }
  try{
    const previous=lastSnapshot;
    let data=await readHardware();
    if(data.task?.onboardId&&data.task.phase==='running'||data.task?.vehicleJob&&['preparing','running'].includes(data.task.phase)){
      if(data.room?.mine)await post('/api/room',{action:'release'},true);
      await leaveHardware();return;
    }
    if(!data.connected){
      if(hardwareActive(data)||hardwareActive(previous))throw Error('真实机器人连接中断，无法确认停车，请恢复连接后切换');
      await leaveHardware();return;
    }
    if(hardwareActive(data)||data.room?.stopping){
      if(!data.room?.mine)throw Error('真实机器人正在由其他页面控制，请等待对方停稳后切换');
      applyRoom(data.room);switchHint='正在停止真实机器人，等待底盘反馈…';message(switchHint);
      await post('/api/command',{command:'stop',experiment:$('experiment').value},true);
    }
    const deadline=performance.now()+8000;let stable=0;
    while(performance.now()<deadline){
      data=await readHardware();
      if(!data.connected)throw Error('真实机器人失联，无法确认停稳，已保留当前页面');
      if(hardwareActive(data)&&data.room?.owner&&!data.room?.mine)throw Error('其他页面正在驾驶真实机器人，请稍后切换');
      if(hardwareStopped(data)){if(!stable)stable=performance.now();if(performance.now()-stable>=300){
        if(data.room?.mine)await post('/api/room',{action:'release'},true);
        await leaveHardware();return;
      }}else stable=0;
      await new Promise(resolve=>setTimeout(resolve,120));
    }
    throw Error('未收到连续停稳反馈，请先停止真实机器人后重试切换');
  }catch(error){switchHint=error.message;message(switchHint)}finally{switching=false;}
}
document.querySelectorAll('[data-target="simulation"]').forEach(link=>link.addEventListener('click',switchToSimulation));
document.addEventListener('control-suspend',suspend);
const labels={WAITING:'等待任务',TURNING:'原地转向',DRIVING:'直线行驶',DONE:'任务完成',FAULT:'故障停车',PAUSED:'已暂停',IDLE:'待命'};
let lastTaskView='';
async function refresh(){
  try{
    const response=await fetch('/api/status',{headers:{'X-Control-Token':token},signal:AbortSignal.timeout(1000)});if(!response.ok)throw Error('状态读取失败');const data=await response.json();
    lastSnapshot=data;
    if(token&&data.room&&!data.room.self){mine=false;armed=false;lease='';held.clear();direction='';await openSession();return;}
    online=data.connected && data.age!==null && data.age<1500;
    if(hardwareButton){
      const link=data.hardwareLink;
      hardwareButton.disabled=online||link?.phase==='connecting';
      hardwareButton.textContent=online?'真机已连接':link?.phase==='connecting'?'连接中…':'连接真机';
      if(link&&['connecting','error','ready'].includes(link.phase))$('pingState').textContent=link.message||({connecting:'连接中…',error:'连接失败，请重试',ready:'机器人连接已建立'})[link.phase];
    }
    if(data.robotIp)document.querySelector('.address').textContent=data.robotIp;
    if(data.preview)$('previewBadge').hidden=false;
    applyRoom(data.room);
    if(data.task&&window.Workbench.items().length){
      const key=data.task.experiment+'/'+data.task.phase;
      const active=['preparing','running','stopping'].includes(data.task.phase);
      if(active||key!==lastTaskView)window.Workbench.syncTask?.(data.task);
      lastTaskView=key;
    }
    if(!online)suspend();
    const state=data.status;manual=state.mode==='manual'&&armed;vectorSupported=state.manual_vector===true;
    const navigationTask=data.task?.experiment==='lab3-xy'?data.task:null;
    const canPickGoal=online&&navigationTask?.phase==='ready'&&data.preparedExperiment==='lab3-xy';
    window.NavigationMap?.(state.map_generation,data.task?.phase==='preparing',navigationTask?.prepareId,canPickGoal);
    paintKeys();
    if(!manual&&held.size)zero();
    $('chassis').textContent=!online?'未连接':state.feedback_fresh!==true||!Number.isInteger(state.chassis_mode)?'等待底盘反馈':state.chassis_mode===1?'网络 / CAN':state.chassis_mode===3?'遥控器接管':state.chassis_mode===0?'待切入 CAN（模式 0）':`底盘模式 ${state.chassis_mode}`;
    $('connection').textContent=online?'真实机器人 · 在线':'真实机器人 · 离线';$('connection').classList.toggle('online',online);
    if(!data.room){$('claim').disabled=!online||mine;$('claim').textContent=mine?'已取得控制权':'取得控制权';$('lease').textContent=mine?'本窗口正在控制':data.owned?'其他窗口正在控制':'未取得控制权';}
    $('stage').textContent=online?(labels[state.stage]||state.stage||'待命'):'等待连接';$('reason').textContent=online?(state.reason||'控制器在线'):'界面已就绪，等待机器人上线';
    if(online&&state.reason==='Operator stop'){$('stage').textContent='已停止 / 待命';$('reason').textContent='已按操作请求停车；准备实验后可重新启动';}
    $('mode').textContent=!online?'未启用':state.control_permitted===false?'仅观察':manual?'手动模式':'自动模式';
    const remoteActive=state.feedback_fresh===true&&(state.chassis_mode===3||[1,3].includes(state.remote_switch));
    $('driveStatus').textContent=!online?'控制连接未建立':remoteActive?'遥控器已接管，CAN 反馈在线。请停稳并将遥控器切回网络控制档。':state.manual_block_message?'驾驶受阻：'+state.manual_block_message:!mine?'取得控制权后，可启用键盘驾驶':manual?'键盘驾驶已启用 · 按住 WASD，松开停车':'点击“启用键盘驾驶”恢复或切换手动模式';
    let progress=Number(state.progress)||0,total=Number(state.total_distance)||5;
    for(const key of ['v','w'])$(key).textContent=online&&Number.isFinite(Number(state[key]))?Number(state[key]).toFixed(2):'—';
    $('obstacle').textContent=online?(state.obstacle?'有障碍':'畅通'):'—';$('paused').textContent=online?((state.paused||state.obstacle)?'暂停中':'等待 / 运行'):'等待反馈';
    document.querySelectorAll('[data-command],[data-direction],#stop,#zero').forEach(b=>b.disabled=!online||!mine||!!data.room?.stopping||(b.dataset.direction&&!manual));
    document.querySelector('[data-command="manual"]').disabled=!online||!mine||!!data.room?.stopping;
    const selected=$('experiment').value,lab1=selected==='lab1',eight=selected==='lab2-eight',task=data.task;
    const titles=Object.fromEntries(window.Workbench.items().map(item=>[item.id,item.title]));
    const taskBusy=task&&['preparing','running','stopping'].includes(task.phase);
    taskActive=!!taskBusy;
    const startButton=document.querySelector('[data-command="start"]');
    const prepared=data.preparedExperiment===selected;
    startButton.dataset.intent=lab1||prepared?'run':'prepare';
    startButton.dataset.prepareId=prepared&&task?.experiment===selected?task.prepareId||'':'';
    startButton.textContent=taskBusy&&task.experiment===selected?(task.phase==='preparing'?'准备中…':task.phase==='stopping'?'停止中…':task.awaitingActive?'等待车端确认':'运行中'):lab1?'开始建图 · 30 秒':prepared?'开始实验':'准备实验';
    startButton.disabled=submittingStart||!online||!mine||taskBusy||!selected||!!data.room?.stopping||!!(prepared&&data.startBlockReason);
    document.querySelectorAll('[data-command="pause"],[data-command="resume"]').forEach(b=>b.disabled=b.disabled||lab1||!!(task&&['preparing','stopping'].includes(task.phase)));
    $('taskBadge').textContent=window.Workbench.selected()?.badge||'待命';
    if(eight&&state.stage==='DRIVING')$('stage').textContent='八字轨迹跟踪';
    if(['lab3','lab3-xy'].includes(selected)&&task?.phase==='running'&&state.stage!=='FAULT'){
      const phases={waiting_for_data:'等待感知',straight:'沿初始路线直行',aligning:'低速转向绕行',aligning_goal:'终点对齐初始朝向',settling_goal:'终点前后微调',following_detour:'沿规划路线绕障',rejoining:'尽快回到原路线',recover_forward:'前进通过受限位置',recover_turn:'优先转向找路',recover_reverse:'短距离后退让位',returning:'回归目标点',blocked:'当前运动范围受阻',exploring:'转向探索',clear_hold:'确认前方畅通',avoid_forward:'绕障直行',finished:'避障任务完成',fault:'避障停车'};
      Object.assign(phases,{aligning_start:'起步对准目标',settling_start:'转向减速中',settling_goal:'已到目标，确认停稳',waiting_route:'通路受阻，等待恢复'});
      Object.assign(phases,{braking_for_reverse:'停稳后准备短退',replan_after_reverse:'短退完成，重新找路'});
      Object.assign(phases,{braking_for_turn:'停稳后原地找路'});
      Object.assign(phases,{pivot_to_path:'原地转向路线方向'});
      Object.assign(phases,{path_tracking:'沿全局路线跟踪',path_braking:'车身制动轨迹受阻'});
      Object.assign(phases,{dwa_tracking:'DWA 连续避障',dwa_replanning:'DWA 正在寻找可行路线'});
      Object.assign(phases,{dwa_slowing:'根据可用制动距离减速',dwa_braking:'制动范围受阻'});
      phases.waiting_handover='等待停稳以切换 XY 规划器';
      Object.assign(phases,{side_pivot:'原地转向侧方通道',side_pass:'沿已检查的侧方通道前进',side_settle:'停稳后接续目标路线'});
      Object.assign(phases,{route_retreat:'沿刚驶过的路线回退',retreat_replan:'停车重新选路'});
      phases.forward_escape='沿已检查的前方通道直行';
      $('stage').textContent=(state.xy_local_recovery?'XY 接续 · ':'')+(phases[state.avoidance_state]||$('stage').textContent);
      const reasons={'No route in the observed local window':'当前观测范围内没有可通行路线','Waiting to replan around changed obstacles':'障碍变化，等待重新规划','Swept body or braking path is obstructed':'车身或停车范围受阻','Final heading rotation is obstructed':'终点回正的车身旋转范围受阻','Final heading alignment timeout':'终点朝向未能及时对齐，已停车','No safe route for 12 seconds; reposition or clear the obstacle':'连续 12 秒无安全路线，请清理障碍或重新选择起点','No translational progress for 18 seconds':'连续 18 秒未前进，已停止任务'};
      Object.assign(reasons,{'Seeking a new route after limited progress':'通过转向或倒车寻找新路线','Searching around an obstructed route':'原路受阻，继续搜索','Searching while the route is replanned':'调整姿态并重新规划','Seeking a collision-free direction':'正在寻找可行运动方向','No collision-free forward, turning or reverse motion':'前进、转向和倒车均无法通过车身检查','Goal position adjustment is obstructed':'终点前后微调的车身范围受阻'});
      Object.assign(reasons,{'Route blocked; waiting for a clear route or a new goal':'暂时没有可行通路，已停车等待；通路恢复后继续，也可停止并更换目标','No safe forward, turning or recent-path retrace motion':'前进、转向及沿最近路径回退均暂不可行'});
      Object.assign(reasons,{'Stopping before recent-path retreat':'先停稳，再沿刚走过的直线路径短退','Retracing recent path before replanning':'正在低速短退，为重新规划留出空间','Short retreat complete; replanning':'短退完成，重新选择可行路线'});
      Object.assign(reasons,{'Stopping before pivot search':'先停稳，再原地转向寻找通路','Pivoting to observe and replan':'原地转向观察新方向，持续重新规划'});
      Object.assign(reasons,{'Pivoting toward the planned route':'停止前进，原地对准规划路线后继续','Planned pivot is obstructed':'原地旋转范围受阻，寻找其他动作或短退腾位'});
      Object.assign(reasons,{'DWA is replanning around obstacles':'根据当前障碍地图重新规划通路'});
      Object.assign(reasons,{'Waiting for current global path':'等待本轮地图上的全局路径','Waiting for a footprint-clear path':'当前路径的车身通行空间不足，正在重新选路'});
      reasons['Advancing straight to clear the rear turning envelope']='车尾限制转向，正在沿前方通道直行腾位';
      Object.assign(reasons,{'Insufficient rear braking clearance':'后方实测净距不足以停稳','Rear corridor lacks covering returns':'后方车宽内的雷达覆盖不足','Backing to an observed turning space':'后方有可用转向位，正在直退腾位','Observed reverse corridor changed':'已检查的后退通道发生变化，正在复核'});
      Object.assign(reasons,{'No continuous measured approach to retrace':'缺少连续的实走来路','Recorded turn is obstructed':'原路转角受阻，等待重新检查','Reached the start of the recorded route':'已回到这段实走轨迹起点，重新选路'});
      Object.assign(reasons,{'No recent straight approach to retrace':'最近没有足够长的直行轨迹','Rear corridor obstructed or unobserved':'后方有障碍或雷达覆盖不足','Retracing the measured approach':'沿有行驶记录的通道低速回退','Backing stopped at a possible side exit':'已发现侧向出口，停稳复核','Retreat complete; checking another exit':'已退到记录位置，重新检查其他出口','Retreat departed from recorded approach':'偏离原路，结束回退','No measured retreat progress':'回退无实测进展，已停车','Retreat interrupted':'回退已中断'});
      Object.assign(reasons,{'Measured braking path obstructed':'当前实速的停车范围内有回波','Slowing for proposed braking path':'按较低速度可在观测净距内停稳','No clear braking path in reachable velocity window':'本周期同一弧线上的可达速度都未通过制动检查','Invalid control interval':'控制周期异常'});
      Object.assign(reasons,{'Braking for observed side passage':'已看到侧方通道，减速准备转向','Reserving clearance to turn ahead':'前方近障，正在留出转向距离','Straightening in observed forward corridor':'车后限制转向，沿已观测的前方通道直行'});
      $('reason').textContent=state.avoidance_version>=2?(reasons[state.avoidance_reason]||state.reason||`距目标 ${Number(state.goal_distance??10).toFixed(2)} m · 已规划 ${state.avoidance_replans??0} 次`):`前方左侧 ${state.front_left_points??0} 点 · 右侧 ${state.front_right_points??0} 点`;
    }
    if(online && state.sensors_ready && 'front_gap_m' in state &&
       ['blocked','waiting_route','retreat_replan','dwa_braking','forward_escape','route_retreat'].includes(state.avoidance_state)){
      const gap=value=>Number.isFinite(value)?Math.round(value*100)+' cm':'本帧无通道内回波';
      $('reason').textContent+=' · 前向净距 '+gap(state.front_gap_m)+'，后向净距 '+gap(state.rear_gap_m);
    }
    window.Workbench.lock(taskBusy||!!(data.room?.owner&&!mine));
    if(!lab1&&task?.phase==='preparing'){$('stage').textContent='准备实验中';$('reason').textContent='正在启动传感器、定位和机器人屏幕 RViz';}
    if(!lab1&&task?.phase==='ready'&&state.sensors_ready){$('stage').textContent='准备完成';$('reason').textContent='定位和雷达就绪，点击启动开始新任务';}
    const phases={preparing:'准备中',running:'运行中',stopping:'停止中',stopped:'已停止',completed:'已完成',ready:'准备完成，请再次点击启动',failed:'失败'};
    $('taskLog').textContent=task?`${titles[task.experiment]||task.experiment} · ${phases[task.phase]||task.phase}${task.exitCode!==null?' · 退出码 '+task.exitCode:''}\n${task.log||''}`:'';
    if(lab1&&task?.experiment==='lab1'){$('stage').textContent=phases[task.phase]||task.phase;$('reason').textContent='30 秒建图 → 保存地图 → 验证定位；可独立切换手动驾驶';}
    if(online&&state.stage==='FAULT'&&state.reason!=='Operator stop'){
      const faults={'CAN bus-off':'CAN 总线离线，已停车，请检查底盘通信','CAN receive error':'CAN 接收异常，已停车','CAN feedback timeout':'底盘反馈超时，已停车','Odometry timeout':'定位数据超时，已停车','Lidar timeout':'雷达数据超时，已停车','Chassis fault or control-mode change':'底盘故障或控制模式改变，已停车','No chassis motion response':'已发出指令但底盘未运动，请检查控制模式和底盘'};
      $('stage').textContent='故障停车';
      $('reason').textContent=String(state.reason||'').startsWith('CAN transmit error')?`CAN 指令发送失败，已停车 · ${state.reason}`:(faults[state.reason]||state.reason||'控制器故障，请查看日志');
    }
    if(lab1){progress=task?.experiment==='lab1'?task.progress||0:0;total=100;}
    else if(task?.experiment!==selected||['preparing','ready'].includes(task?.phase)){progress=0;total=eight?4*Math.PI*(window.Workbench.options().radius||2):selected==='lab3'?10:5;}
    else if(task?.phase==='completed')progress=total;
    const percent=Math.max(0,Math.min(100,progress/total*100));
    $('progress').textContent=progress.toFixed(lab1?0:2);$('total').textContent=lab1?'100 %':total.toFixed(2)+' m';
    $('progressLabel').textContent=lab1?(task?.progressLabel||'实验阶段'):eight?'八字路线进度':'直线行驶进度';
    if(selected==='lab3')$('progressLabel').textContent='沿原路线进度';
    $('bar').style.width=percent+'%';$('progressPercent').textContent=percent.toFixed(0)+'%';
    $('taskProgress').setAttribute('aria-valuenow',percent.toFixed(0));
    window.MazeView?.status(data,online,mine);
    if(prepared&&task?.phase==='ready'&&data.startBlockReason){$('stage').textContent='已准备 · 等待开始';$('reason').textContent=data.startBlockReason;}
    else if(prepared&&selected==='lab3-xy'&&task?.phase==='ready'&&!window.hasNavigationGoal?.()){
      startButton.disabled=true;$('reason').textContent='准备完成。请在点云中选终点，或填写 X 和 Y 坐标。';
    }
    if(task?.phase==='failed'&&task.experiment===selected&&task.log){$('reason').textContent=task.log.trim().split('\n').at(-1);}
    // Other authenticated tabs may still issue an emergency task stop.
    $('stop').disabled=!online&&!taskActive;
    window.RoomUI?.render(data.room,online,data.urls);
    if(switching){document.querySelectorAll('[data-command],[data-direction],#zero,#claim,#connectHardware').forEach(button=>button.disabled=true);message(switchHint);}
  }catch{online=false;mine=false;armed=false;zero();$('connection').textContent='服务连接中断';document.querySelectorAll('[data-command],[data-direction],#stop,#zero,#claim').forEach(b=>b.disabled=true);window.MazeView?.status({},false,false);window.RoomUI?.render(roomState,false);}
}
setInterval(async()=>{
  if(!direction||!manual||busy||switching||!mine||!online||document.hidden||!document.hasFocus())return;
  busy=true;
  try{await sendDrive();}
  catch(error){mine=false;zero();message(error.message);}finally{busy=false;}
},100);
setInterval(refresh,300);
let monitorRequest=null;
async function watchSession(session){
  monitorRequest?.abort();
  const request=monitorRequest=new AbortController();
  while(token===session&&!request.signal.aborted){
    try{
      const response=await fetch('/api/session/monitor',{headers:{'X-Control-Token':session},signal:request.signal,cache:'no-store'});
      if(response.status===403)return;
      if(response.ok)await response.text();
    }catch{}
    if(!request.signal.aborted)await new Promise(resolve=>setTimeout(resolve,1000));
  }
}
function openSession(){
  if(renewing)return renewing;
  renewing=fetch('/api/session').then(r=>r.json()).then(data=>{if(!data.token)throw Error('无法建立会话');token=data.token;watchSession(token);applyRoom(data.room);window.RoomUI?.init({act:(action,extra)=>{if(['release','takeover'].includes(action))zero();return post('/api/room',{action,...extra},true);},online:()=>online,message},data);}).finally(()=>{renewing=null;});return renewing;
}
openSession().then(refresh).catch(()=>message('无法连接共享服务，请刷新页面'));
fetch('/api/experiments').then(r=>r.json()).then(data=>window.Workbench.load(data.experiments)).catch(()=>message('无法加载实验列表'));
