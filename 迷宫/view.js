'use strict';
// Local ground is fitted only where independent patches support a continuous surface.
// The output is a display attribute. The input XYZ and the navigation map are untouched.
function classifyMazeCloud(xyz,count,body,classes){
  classes.fill(0,0,count);
  if(!body||!Number.isFinite(body[2]))return;
  const tiles=new Map(),bins=new Map(),cell=1;
  const key=(x,y)=>x+','+y;
  for(let i=0;i<count;i++){
    const x=xyz[3*i],y=xyz[3*i+1],z=xyz[3*i+2];
    if(!Number.isFinite(z)||z<body[2]-.8||z>body[2]+.12)continue;
    const tx=Math.floor(x/cell),ty=Math.floor(y/cell),id=key(tx,ty);
    let t=tiles.get(id);
    if(!t){t={x:tx,y:ty,low:[],plane:null,ground:false};tiles.set(id,t);}
    const quadrant=(x/cell-tx>=.5?1:0)+(y/cell-ty>=.5?2:0);
    if(!t.low[quadrant]||z<t.low[quadrant][2])t.low[quadrant]=[x,y,z];
  }
  for(const t of tiles.values())for(const p of t.low){
    if(!p||Math.hypot(p[0]-body[0],p[1]-body[1])>6||p[2]>=body[2])continue;
    const bin=Math.floor(p[2]/.04);
    bins.set(bin,(bins.get(bin)||0)+1);
  }
  let anchor=null,best=0;
  for(const [bin,n] of bins){
    const support=n+.5*((bins.get(bin-1)||0)+(bins.get(bin+1)||0));
    if(support>best){best=support;anchor=(bin+.5)*.04;}
  }
  if(best<9)return;
  const neighbors=[[1,0],[-1,0],[0,1],[0,-1]];
  function floorAt(t,x,y){const p=t.plane;return p.z+p.a*(x-p.x)+p.b*(y-p.y);}
  const seeds=[];
  for(const t of tiles.values()){
    let support=[];
    for(let oy=-1;oy<=1;oy++)for(let ox=-1;ox<=1;ox++){
      const near=tiles.get(key(t.x+ox,t.y+oy));if(near)for(const p of near.low)if(p)support.push(p);
    }
    if(support.length<8)continue;
    support.sort((a,b)=>a[2]-b[2]);const median=support[support.length>>1][2];
    const n0=support.length;
    support=support.filter(p=>Math.abs(p[2]-median)<=.12);
    if(support.length<Math.max(7,n0*.65))continue;
    let x=0,y=0,z=0;
    for(const p of support){x+=p[0];y+=p[1];z+=p[2];}
    x/=support.length;y/=support.length;z/=support.length;
    let xx=0,yy=0,xy=0,xz=0,yz=0;
    for(const p of support){const dx=p[0]-x,dy=p[1]-y,dz=p[2]-z;xx+=dx*dx;yy+=dy*dy;xy+=dx*dy;xz+=dx*dz;yz+=dy*dz;}
    const det=xx*yy-xy*xy;if(det<.045*support.length*support.length)continue;
    const a=(yy*xz-xy*yz)/det,b=(xx*yz-xy*xz)/det;
    if(Math.hypot(a,b)>.22)continue;
    const good=support.filter(p=>Math.abs(p[2]-z-a*(p[0]-x)-b*(p[1]-y))<=.042).length;
    if(good<support.length*.78)continue;
    t.plane={x,y,z,a,b};
    if(Math.hypot(t.x+.5-body[0],t.y+.5-body[1])<=4&&Math.abs(floorAt(t,t.x+.5,t.y+.5)-anchor)<=.075){t.ground=true;seeds.push(t);}
  }
  for(let i=0;i<seeds.length;i++){
    const t=seeds[i];
    for(const [dx,dy] of neighbors){
      const n=tiles.get(key(t.x+dx,t.y+dy));if(!n?.plane||n.ground)continue;
      const x=t.x+.5+dx/2,y=t.y+.5+dy/2;
      if(Math.abs(floorAt(n,x,y)-floorAt(t,x,y))>.055)continue;
      n.ground=true;seeds.push(n);
    }
  }
  for(let i=0;i<count;i++){
    const x=xyz[3*i],y=xyz[3*i+1],z=xyz[3*i+2];if(!Number.isFinite(z))continue;
    const t=tiles.get(key(Math.floor(x/cell),Math.floor(y/cell)));
    if(!t?.ground){if(z>anchor+2.15)classes[i]=4;continue;}
    const h=z-floorAt(t,x,y);
    if(Math.abs(h)<=.04)classes[i]=1;
    else if(h>.04&&h<=.90)classes[i]=2;
    else if(h>.90)classes[i]=h>2.15?4:3;
  }
}
if(typeof module!=='undefined'&&module.exports)module.exports={classifyMazeCloud};
if(typeof window!=='undefined')
window.MazeView=(()=>{
  const el=id=>document.getElementById(id);
  const scene=document.querySelector('.scene'), experiment=el('experiment');
  const workspace=scene.closest('.workspace');
  const panel=document.createElement('section');
  panel.id='mazeAtlas';panel.hidden=true;
  panel.setAttribute('aria-label','实车迷宫累计地图');
  panel.innerHTML=`<div class="maze-key"><span><i class="seen"></i>已观测</span><span><i class="mark"></i>障碍回波</span><span><i class="pad"></i>估计底座</span><span><i class="unknown"></i>未知</span><span><i class="trail"></i>实走轨迹</span><span><i class="route"></i>规划路线</span></div>
    <canvas id="mazeCanvas" aria-label="累计俯视地图，滚轮缩放，拖动平移"></canvas>
    <div id="mazeEmpty"><strong>等待本轮地图</strong><span>取得控制权后，点「准备迷宫」开始建图</span></div>
    <div class="maze-map-controls"><div class="view-dock glass"><div class="segmented"><button id="mazeFit" type="button">查看全图</button><button id="mazeFollow" type="button">跟随车辆</button></div></div><output id="mazeScale"></output></div>`;
  scene.append(panel);
  const goals=document.createElement('section');goals.id='mazeGoals';goals.hidden=true;
  goals.innerHTML=`<button id="mazePick" type="button" class="secondary"><svg aria-hidden="true"><use href="#i-target"/></svg><span>在图上选出口</span></button>
    <p id="mazeGoalState" role="status"></p>
    <details id="mazeAdvanced"><summary>探索方式与搜索区 <span>展开</span></summary>
      <fieldset id="mazeModes"><legend>行驶策略</legend><label><input type="radio" name="mazeMode" value="explore" checked>地图探索</label><label><input type="radio" name="mazeMode" value="roam">自主漫游</label></fieldset>
      <p id="mazeModeHelp" class="maze-help">优先去累计地图里还没探索过的通道。</p>
      <div class="maze-inputs"><label>前方 <input id="mazeForward" type="number" step="0.1" inputmode="decimal" placeholder="米"></label><label>左侧 <input id="mazeLeft" type="number" step="0.1" inputmode="decimal" placeholder="米"></label></div>
      <p class="maze-help">选填。以本轮起点为准，右侧和后方填负数。</p>
      <button id="mazeHint" type="button" class="secondary">标记搜索区</button><small id="mazeCalibration"></small>
    </details>
    <details id="mazeGuide"><summary>怎么用 <span>展开</span></summary>
      <ol><li>连接真机，取得控制权，点「准备迷宫」。</li><li>地图就绪后，点「开始探索」。看到出口时，可点「在图上选出口」，再点出口内侧的已观测空地。</li><li>暂停后可继续同一轮。点「停止运动」结束本轮。重新准备后，旧出口需要重选。</li></ol>
      <p class="maze-help">滚轮缩放通行图，拖动平移。累计点云看整体，实时点云看当前一帧。</p>
    </details>`;
  document.querySelector('.task-panel .actions').after(goals);
  const signal=document.createElement('span');
  signal.className='maze-signal';signal.hidden=true;
  scene.querySelector('.scene-caption').append(signal);
  const canvas=el('mazeCanvas'),ctx=canvas.getContext('2d');
  let data=null,enabled=false,owned=false,prepared=false,picking=false,view='fit',scale=38,pan=[0,0],drag=null;
  let revision=0,request=null,sequence=0,lastError='',online=false,receivedAt=0,currentTask=null;
  const tiles=new Map();
  const caption=scene.querySelector('h1'),captionType=scene.querySelector('.scene-caption .eyebrow');
  const defaultCaption=caption.textContent,defaultType=captionType.textContent;
  let surface='map';
  function showSurface(name){
    surface=name;scene.dataset.surface=name;
    const grid=name==='grid';
    scene.classList.toggle('maze-active',grid);panel.hidden=!grid;
    caption.textContent=enabled?'迷宫探索':name==='map'?'累计点云':defaultCaption;
    captionType.textContent=enabled?'':name==='map'?'SLAM POINT CLOUD':defaultType;
    scene.setAttribute('aria-label',grid?'实车迷宫通行图':'三维点云与定位轨迹');
    el('cloudMapFit').hidden=name!=='map';
    el('mapEnhance').hidden=name!=='map';
    for(const b of el('sceneMapTabs').querySelectorAll('[data-surface]')){
      b.hidden=b.dataset.surface==='grid'&&!enabled;
      b.classList.toggle('active',b.dataset.surface===name);b.setAttribute('aria-pressed',String(b.dataset.surface===name));
    }
    if(!grid){picking=false;window.RobotCloudView?.(name);}
    draw();
  }
  el('sceneMapTabs').addEventListener('click',event=>{const name=event.target.closest('[data-surface]')?.dataset.surface;if(name)showSurface(name);});
  function select(){
    const active=experiment.value==='maze';
    if(enabled===active&&scene.dataset.surface)return;
    enabled=active;
    workspace.classList.toggle('maze-workspace',enabled);
    document.querySelector('.task-panel').classList.toggle('maze-task',enabled);
    goals.hidden=signal.hidden=!enabled;
    showSurface(enabled?'map':'live');
    if(!enabled){request?.abort();picking=false;sequence++;return;}
    readMap();
  }
  experiment.addEventListener('change',select);
  function hintValues(){
    const x=el('mazeForward').value,y=el('mazeLeft').value;
    if(!x.trim()&&!y.trim())return null;
    if(!x.trim()||!y.trim()||![Number(x),Number(y)].every(Number.isFinite))throw Error('请填写前方和左侧两个坐标');
    return [Number(x),Number(y)];
  }
  async function apply(kind,x,y){
    if(!owned||!prepared||!data?.id)throw Error('请先取得控制权并准备迷宫');
    await post('/api/maze/goal',{kind,x,y,id:data.id});
    picking=false;lastError='';await readMap();
  }
  el('mazeHint').onclick=async()=>{try{const h=hintValues();if(!h)throw Error('请填写搜索区相对起点的位置');await apply('hint',...h);}catch(e){lastError=e.message;draw();}};
  el('mazePick').onclick=()=>{if(surface!=='grid')showSurface('grid');picking=!picking;lastError='';draw();};
  el('mazeModes').addEventListener('change',()=>{
    el('mazeModeHelp').textContent=el('mazeModes').querySelector('[value="roam"]').checked?
      '优先沿当前方向通行，遇到阻挡时尝试旁路。':'优先去累计地图里还没探索过的通道。';
  });
  document.addEventListener('keydown',e=>{if(e.key==='Escape'&&picking){picking=false;draw();}});
  el('mazeFit').onclick=()=>{view='fit';draw();};el('mazeFollow').onclick=()=>{view='follow';scale=38;draw();};
  canvas.onwheel=e=>{e.preventDefault();view='pan';scale=Math.min(300,Math.max(.25,scale*Math.exp(-Math.sign(e.deltaY)*.19)));draw();};
  canvas.onpointerdown=e=>{if(!enabled)return;drag=[e.clientX,e.clientY,...pan];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointermove=e=>{if(!drag||picking)return;view='pan';pan=[drag[2]-(e.clientX-drag[0])/scale,drag[3]+(e.clientY-drag[1])/scale];draw();};
  canvas.onpointercancel=()=>{drag=null;};
  canvas.onpointerup=async e=>{
    if(!drag)return;const click=Math.hypot(e.clientX-drag[0],e.clientY-drag[1])<5;drag=null;
    if(!click||!picking)return;
    const r=canvas.getBoundingClientRect();
    try{await apply('exit',pan[0]+(e.clientX-r.left-r.width/2)/scale,pan[1]-(e.clientY-r.top-r.height/2)/scale);}
    catch(error){lastError=error.message;draw();}
  };
  function tileImage(encoded,n){
    const binary=atob(encoded),t=document.createElement('canvas');t.width=t.height=n;
    const c=t.getContext('2d'),im=c.createImageData(n,n);
    const colors=[[0,0,0,0],[73,100,105,255],[248,173,97,255],[132,93,67,255]];
    for(let y=0;y<n;y++)for(let x=0;x<n;x++){
      const i=y*n+x,v=(binary.charCodeAt(i>>2)>>((i&3)*2))&3;
      im.data.set(colors[v],((n-1-y)*n+x)*4);
    }
    c.putImageData(im,0,0);return t;
  }
  async function readMap(){
    if(!enabled||request)return;
    const seq=sequence,controller=request=new AbortController();
    try{
      const q=new URLSearchParams({id:data?.id||'',since:String(revision)});
      const res=await fetch('/api/maze?'+q,{signal:AbortSignal.any([controller.signal,AbortSignal.timeout(3000)])});
      if(!res.ok)throw Error('累计地图读取失败');
      const next=await res.json();if(!enabled||seq!==sequence)return;
      if(data?.id!==next.id){tiles.clear();view='fit';picking=false;lastError='';}
      for(const [tx,ty,bytes] of next.tiles)tiles.set(tx+','+ty,{tx,ty,img:tileImage(bytes,next.tile)});
      if(lastError==='地图连接中断，等待更新')lastError='';
      data=next;receivedAt=Date.now();revision=next.revision;draw();
    }catch(e){if(enabled&&seq===sequence){lastError='地图连接中断，等待更新';draw();}}
    finally{if(request===controller)request=null;}
  }
  function freshMap(){
    return online&&Boolean(data?.pose)&&data.ageMs!=null&&data.ageMs+Date.now()-receivedAt<1500;
  }
  function draw(redraw=true){
    if(!enabled)return;
    const has=Boolean(data?.pose),empty=el('mazeEmpty');
    empty.hidden=has;
    empty.querySelector('span').textContent=currentTask?.phase==='preparing'?'正在等待雷达与定位':
      online?owned?'点「准备迷宫」开始建图':'取得控制权后即可准备': '连接真机后，这里会显示通行图';
    const fresh=freshMap();
    const healthy=fresh&&data.healthy;
    if(!healthy)picking=false;
    el('mazePick').disabled=el('mazeHint').disabled=!owned||!prepared||!healthy;
    el('mazePick').querySelector('span').textContent=picking?'取消选择':data?.exit?'更改出口':'在图上选出口';
    el('mazePick').setAttribute('aria-pressed',String(picking));
    canvas.style.cursor=picking?'crosshair':drag?'grabbing':'grab';
    el('mazeGoalState').textContent=lastError||(picking?'点击出口内侧已观测的空地':data?.exit?'出口已标记，可在地图上查看':data?.hint?'搜索区已标记，车辆仍会探索':prepared?'出口可稍后选，也可以直接开始探索':'');
    const waiting=currentTask?.phase==='running'&&!healthy;
    const signalText=waiting?(data?.healthReason||'观测间断 · 等待恢复'):healthy?'观测正常':has?'显示上次地图':online?'等待观测':'真机离线';
    if(signal.textContent!==signalText)signal.textContent=signalText;
    signal.title=data?.streamError||'';
    signal.dataset.state=healthy?'ready':waiting?'hold':'idle';
    el('mazeCalibration').textContent=data?'底座单侧外扩 '+Math.round(data.baseExtension*100)+' cm · 请按现场测量校准'+(data.coverage==='front'?' · 当前仅前向建图':''):'';
    if(!redraw||surface!=='grid'||!ctx)return;
    for(const [id,mode] of [['mazeFit','fit'],['mazeFollow','follow']]){
      const button=el(id);button.classList.toggle('active',view===mode);button.setAttribute('aria-pressed',String(view===mode));
    }
    const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
    const dpr=Math.min(devicePixelRatio||1,2);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.imageSmoothingEnabled=false;
    if(!has)return;
    const span=data.cell*data.tile;
    if(view==='fit'){
      let xmin=data.pose[0]-1,xmax=xmin+2,ymin=data.pose[1]-1,ymax=ymin+2;
      for(const {tx,ty} of tiles.values()){xmin=Math.min(xmin,tx*span);ymin=Math.min(ymin,ty*span);xmax=Math.max(xmax,(tx+1)*span);ymax=Math.max(ymax,(ty+1)*span);}
      for(const p of [data.hint,data.exit])if(p){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1]);}
      pan=[(xmin+xmax)/2,(ymin+ymax)/2];scale=Math.min(w/(xmax-xmin+2),(h*.69)/(ymax-ymin+2));
    }else if(view==='follow')pan=data.pose.slice(0,2);
    const xy=([x,y])=>[w/2+(x-pan[0])*scale,h/2-(y-pan[1])*scale];
    for(const {tx,ty,img} of tiles.values()){
      const [x,y]=xy([tx*span,(ty+1)*span]);if(x>w||y>h||x+span*scale<0||y+span*scale<0)continue;
      ctx.drawImage(img,x,y,span*scale+.2,span*scale+.2);
    }
    function line(points,color,dash){
      ctx.beginPath();ctx.setLineDash(dash);ctx.strokeStyle=color;ctx.lineWidth=2;
      points.forEach((p,i)=>{const [x,y]=xy(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();ctx.setLineDash([]);
    }
    line(data.trail,'#9de2df',[]);line(data.route,'#ffe094',[5,4]);
    function pin(p,label,color,dashed){
      if(!p)return;const [x,y]=xy(p);ctx.strokeStyle=ctx.fillStyle=color;ctx.lineWidth=2;ctx.beginPath();
      ctx.setLineDash(dashed?[4,4]:[]);ctx.arc(x,y,dashed?14:6,0,2*Math.PI);ctx.stroke();ctx.setLineDash([]);
      ctx.font='12px sans-serif';ctx.fillText(label,x+9,y-8);
    }
    pin(data.origin,'起点','#b6dad9',false);pin(data.hint,'搜索区','#f0cb77',true);pin(data.exit,'出口','#a4f3a8',false);
    const [x,y]=xy(data.pose);ctx.save();ctx.translate(x,y);ctx.rotate(-data.pose[2]);ctx.fillStyle='#f3ffff';
    ctx.beginPath();ctx.moveTo(10,0);ctx.lineTo(-7,-5);ctx.lineTo(-4,0);ctx.lineTo(-7,5);ctx.closePath();ctx.fill();ctx.restore();
    el('mazeScale').textContent=Math.max(.1,Math.round(1000/scale)/10)+' m / 100 px';
  }
  new ResizeObserver(draw).observe(canvas);
  select();
  setInterval(readMap,1000);
  return {
    data:()=>enabled?data:null,
    freshReturns:()=>enabled&&freshMap(),
    showSurface,
    options(){return {mapId:data?.id||'',hint:hintValues(),mode:goals.querySelector('input[name="mazeMode"]:checked').value};},
    status(state,isOnline,isOwner){
      online=isOnline;owned=isOwner;prepared=state.preparedExperiment==='maze';
      currentTask=state.task?.experiment==='maze'?state.task:null;
      if(!enabled)return;
      const running=currentTask?.phase==='running';
      el('mazeModes').disabled=running;
      if(running){
        const activeMode=currentTask.options?.mode==='roam'?'roam':'explore';
        goals.querySelector('input[value="'+activeMode+'"]').checked=true;
      }
      if(running){
        el('reason').textContent=currentTask.log?.trim().split('\n').at(-1)||'车辆将持续检查可用通道';
        el('stage').textContent=data?.paused?'已暂停':!data?.healthy?'等待观测恢复':'探索中';
        for(const name of ['manual','auto'])document.querySelector('[data-command="'+name+'"]').disabled=true;
      }
      const start=document.querySelector('[data-command="start"]');
      start.textContent=currentTask?.phase==='preparing'?'准备中…':prepared?'开始探索':'准备迷宫';
      if(prepared&&(!freshMap()||!data.healthy))start.disabled=true;
      for(const [name,allowed] of [['pause',running&&!data?.paused],['resume',running&&!!data?.paused]]){
        const button=document.querySelector('[data-command="'+name+'"]');
        button.disabled=button.disabled||!allowed;
      }
      if(currentTask?.phase==='preparing')el('reason').textContent='正在检查雷达、定位与底盘反馈';
      if(currentTask?.phase==='ready'){
        el('stage').textContent=data?.healthy?'准备完成':'等待地图';
        el('reason').textContent=data?.healthy?'可以开始探索，或先在通行图上选出口':data?.healthReason||'等待当前地图和底盘反馈';
      }
      if(currentTask?.phase==='completed'){el('stage').textContent='已到达出口';el('reason').textContent='已到达指定出口区并确认停稳';}
      if(currentTask?.phase==='failed'){
        el('stage').textContent='本轮已中止';el('reason').textContent=currentTask.log?.trim().split('\n').at(-1)||'请检查运行日志';
      }
      draw(false);
    }
  };
})();
