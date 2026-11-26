'use strict';
window.MazeView=(()=>{
  const el=id=>document.getElementById(id);
  const scene=document.querySelector('.scene'), experiment=el('experiment');
  const panel=document.createElement('section');
  panel.id='mazeAtlas';panel.hidden=true;
  panel.setAttribute('aria-label','实车迷宫累计地图');
  panel.innerHTML=`<canvas id="mazeCanvas" aria-label="累计俯视地图，滚轮缩放，拖动平移"></canvas>
    <div id="mazeEmpty" class="scene-empty"><div class="empty-symbol"><svg aria-hidden="true"><use href="#i-map"/></svg></div><h2>等待本轮地图</h2><p>取得控制权后，点「准备漫游」检查传感器</p></div>`;
  el('cloudCanvas').after(panel);
  const gridLegend=document.createElement('span');
  gridLegend.id='mazeGridLegend';gridLegend.hidden=true;
  gridLegend.innerHTML='<span><i class="dot seen"></i>已观测</span><span><i class="dot mark"></i>障碍</span><span><i class="dot pad"></i>估计底座</span><span><i class="line trail"></i>实走轨迹</span><span><i class="line route"></i>规划路线</span>';
  scene.querySelector('.scene-legend').append(gridLegend);
  const canvas=el('mazeCanvas'),ctx=canvas.getContext('2d');
  let data=null,enabled=false,owned=false,prepared=false,view='fit',scale=38,pan=[0,0],drag=null;
  let revision=0,request=null,sequence=0,lastError='',online=false,receivedAt=0,currentTask=null;
  const tiles=new Map();
  let surface=scene.dataset.surface;
  function syncSurface(name){
    surface=name;
    const grid=name==='grid';
    gridLegend.hidden=panel.hidden=!grid;
    if(grid)readMap();
    draw();
  }
  document.addEventListener('cloud-view-change',event=>syncSurface(event.detail));
  function select(){
    const active=experiment.value==='maze';
    if(enabled===active&&scene.dataset.surface)return;
    enabled=active;
    if(!enabled)el('taskProgress').removeAttribute('aria-valuetext');
    if(!enabled){request?.abort();sequence++;return;}
    readMap();
  }
  experiment.addEventListener('change',select);
  function gridView(value){view=value;if(view==='follow')scale=38;draw();}
  function visibleWidth(rect){
    const side=document.querySelector('.inspector').getBoundingClientRect();
    return side.top<rect.bottom&&side.left>rect.left?Math.min(rect.width,side.left-rect.left):rect.width;
  }
  canvas.onwheel=e=>{e.preventDefault();view='pan';scale=Math.min(300,Math.max(.25,scale*Math.exp(-Math.sign(e.deltaY)*.19)));draw();};
  canvas.onpointerdown=e=>{if(!enabled)return;drag=[e.clientX,e.clientY,...pan];canvas.setPointerCapture(e.pointerId);};
  canvas.onpointermove=e=>{if(!drag)return;view='pan';pan=[drag[2]-(e.clientX-drag[0])/scale,drag[3]+(e.clientY-drag[1])/scale];draw();};
  canvas.onpointercancel=()=>{drag=null;};
  canvas.onpointerup=()=>{drag=null;};
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
    if(!enabled||surface!=='grid'||request)return;
    const seq=sequence,controller=request=new AbortController();
    try{
      const q=new URLSearchParams({id:data?.id||'',since:String(revision)});
      const res=await fetch('/api/maze?'+q,{signal:AbortSignal.any([controller.signal,AbortSignal.timeout(3000)])});
      if(!res.ok)throw Error('累计地图读取失败');
      const next=await res.json();if(!enabled||seq!==sequence)return;
      if(data?.id!==next.id){tiles.clear();view='fit';lastError='';}
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
    empty.querySelector('p').textContent=currentTask?.phase==='preparing'?'正在等待雷达与定位':
      online?owned?'点「准备漫游」检查传感器':'取得控制权后即可准备': '连接真机后，这里会显示通行图';
    if(!redraw||surface!=='grid'||!ctx)return;
    window.syncCloudControls?.();
    const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
    const dpr=Math.min(devicePixelRatio||1,2);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.imageSmoothingEnabled=false;
    if(!has)return;
    const span=data.cell*data.tile,visible=visibleWidth(canvas.getBoundingClientRect());
    if(view==='fit'){
      let xmin=data.pose[0]-1,xmax=xmin+2,ymin=data.pose[1]-1,ymax=ymin+2;
      for(const {tx,ty} of tiles.values()){xmin=Math.min(xmin,tx*span);ymin=Math.min(ymin,ty*span);xmax=Math.max(xmax,(tx+1)*span);ymax=Math.max(ymax,(ty+1)*span);}
      for(const p of [data.hint,data.exit])if(p){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1]);}
      pan=[(xmin+xmax)/2,(ymin+ymax)/2];scale=Math.min(visible/(xmax-xmin+2),(h*.69)/(ymax-ymin+2));
    }else if(view==='follow')pan=data.pose.slice(0,2);
    const xy=([x,y])=>[visible/2+(x-pan[0])*scale,h/2-(y-pan[1])*scale];
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
    el('gridScale').textContent=Math.max(.1,Math.round(1000/scale)/10)+' m / 100 px';
  }
  new ResizeObserver(draw).observe(canvas);
  select();
  setInterval(readMap,1000);
  return {
    data:()=>enabled?data:null,
    freshReturns:()=>enabled&&freshMap(),
    gridView,gridFollowing:()=>view==='follow',
    options(){return {mode:'roam'};},
    status(state,isOnline,isOwner){
      online=isOnline;owned=isOwner;prepared=state.preparedExperiment==='maze';
      currentTask=state.task?.experiment==='maze'?state.task:null;
      if(!enabled)return;
      const running=currentTask?.phase==='running';
      const feedback=state.status||{},paused=feedback.paused===true;
      const actions={roam_forward:'沿通道前进',roam_turn:'沿弯道转向',pivot_to_path:'原地转向',
        roam_uturn:'死路掉头',roam_backtrack:'沿来路返回岔口',roam_return_detour:'沿雷达可见通路绕行',route_retreat:'沿已观测通道倒退',
        roam_slow_for_turn:'减速入弯',roam_explored:'已回到起点，已知支路均已走过',
        roam_pivot_obstacle:'转动范围内有障碍',roam_braking_obstacle:'制动范围内有障碍',
        roam_pivot_unobserved:'等待读取转动范围内的雷达回波',roam_return_blocked:'正在重新选路，当前前后制动空间不足',
        roam_return_forward:'向前腾出回溯空间',roam_return_reverse:'沿后方通路倒车腾位',roam_return_scan:'等候新的全圈雷达数据',
        roam_return_rescan:'转动空间变化，正在重新选路',
        roam_rear_blocked:'后方无法安全倒车',roam_settle_pivot:'减速准备转向',
        roam_forward_unobserved:'前方观测不足',roam_no_passage:'未发现可通行方向',roam_wait:'等待可通行方向',
        roam_short_advance:'沿已确认的短通道直行',roam_lane_unobserved:'前方车宽范围内的观测不足',
        roam_short_clearance:'短通道制动余量不足',roam_near_memory:'记忆中的近障限制前进，等待可用侧路',
        roam_near_return:'前方有近距回波，等待可用侧路'};
      el('progressLabel').textContent='行驶状态';
      el('progress').textContent=running?(paused?'已暂停':actions[feedback.avoidance_state]||'检查通道'):'待命';
      el('total').textContent='自主漫游';
      el('progressPercent').textContent='';el('bar').style.width='0%';
      el('taskProgress').removeAttribute('aria-valuenow');
      const start=document.querySelector('[data-command="start"]');
      start.textContent=currentTask?.phase==='preparing'?'准备中…':currentTask?.phase==='stopping'?'停止中…':running?'漫游中':prepared?'开始漫游':'准备漫游';
      if(running){
        el('stage').textContent=isOnline?el('progress').textContent:'监控链路中断';
        el('reason').textContent=isOnline?'持续探路，确认驶出后点击停止运动':'最后报告为运行中，等待车端更新；立即停车请用现场遥控器或急停';
        for(const name of ['manual','auto'])document.querySelector('[data-command="'+name+'"]').disabled=true;
      }
      if(prepared&&state.startBlockReason)el('reason').textContent=state.startBlockReason;
      for(const [name,allowed] of [['pause',running&&!paused],['resume',running&&paused]]){
        const button=document.querySelector('[data-command="'+name+'"]');button.disabled=button.disabled||!allowed;
      }
      draw(false);
    }
  };
})();
