'use strict';
window.MazeView=(()=>{
  const el=id=>document.getElementById(id);
  const scene=document.querySelector('.scene'), experiment=el('experiment');
  const panel=document.createElement('section');
  panel.id='mazeAtlas';panel.hidden=true;
  panel.setAttribute('aria-label','实车迷宫累计地图');
  panel.innerHTML=`<div class="maze-key"><span><i class="seen"></i>已观测</span><span><i class="mark"></i>障碍回波</span><span><i class="pad"></i>估计底座</span><span><i class="unknown"></i>未知</span><span><i class="trail"></i>实走轨迹</span><span><i class="route"></i>规划路线</span></div>
    <canvas id="mazeCanvas" aria-label="累计俯视地图，滚轮缩放，拖动平移"></canvas>
    <div id="mazeEmpty" role="status">等待实车定位与雷达</div>
    <div class="maze-map-controls"><button id="mazeFit" type="button">查看全图</button><button id="mazeFollow" type="button">跟随车辆</button><output id="mazeScale"></output></div>`;
  scene.append(panel);
  const goals=document.createElement('section');goals.id='mazeGoals';goals.hidden=true;
  goals.innerHTML=`<fieldset id="mazeModes"><legend>行驶策略</legend><label><input type="radio" name="mazeMode" value="explore" checked>地图探索</label><label><input type="radio" name="mazeMode" value="roam">自主漫游</label></fieldset>
    <label>搜索区域 · 相对起点（米）</label>
    <div class="maze-inputs"><label>前方<input id="mazeForward" type="number" step="0.1" placeholder="米"></label><label>左侧<input id="mazeLeft" type="number" step="0.1" placeholder="米"></label></div>
    <div class="maze-goal-buttons"><button id="mazeHint" type="button" class="secondary">标记搜索区</button><button id="mazePick" type="button" class="secondary">在图上选出口</button></div>
    <div id="mazeGoalState" role="status">准备后可指定出口，也可先探索</div><small id="mazeCalibration"></small>`;
  el('taskParameters').after(goals);
  const canvas=el('mazeCanvas'),ctx=canvas.getContext('2d');
  let data=null,enabled=false,owned=false,prepared=false,picking=false,view='fit',scale=38,pan=[0,0],drag=null;
  let revision=0,request=null,sequence=0,lastError='',online=false,receivedAt=0;
  const tiles=new Map();
  const caption=scene.querySelector('h1'),captionType=scene.querySelector('.scene-caption .eyebrow');
  const defaultCaption=caption.textContent,defaultType=captionType.textContent;
  function select(){
    enabled=experiment.value==='maze';
    scene.classList.toggle('maze-active',enabled);
    document.querySelector('.task-panel').classList.toggle('maze-task',enabled);
    panel.hidden=goals.hidden=!enabled;
    caption.textContent=enabled?'迷宫地图':defaultCaption;captionType.textContent=enabled?'MAZE MAP':defaultType;
    scene.setAttribute('aria-label',enabled?'实车迷宫地图与定位轨迹':'三维点云与定位轨迹');
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
  el('mazePick').onclick=()=>{picking=!picking;lastError='';draw();};
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
  function draw(){
    if(!enabled||!ctx)return;
    const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
    const dpr=Math.min(devicePixelRatio||1,2);
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
    ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);ctx.imageSmoothingEnabled=false;
    const has=Boolean(data?.pose),empty=el('mazeEmpty');
    empty.hidden=has;empty.textContent=data?.streamError||'等待实车定位与雷达';
    const fresh=online&&has&&data.ageMs!==null&&data.ageMs+Date.now()-receivedAt<1500;
    el('mazePick').disabled=el('mazeHint').disabled=!owned||!prepared||!fresh||!data?.healthy;
    el('mazePick').textContent=picking?'取消选择':'在图上选出口';
    canvas.style.cursor=picking?'crosshair':drag?'grabbing':'grab';
    el('mazeGoalState').textContent=lastError||(picking?'点击出口内侧已观测的空地':!fresh&&has?'当前显示最近地图，等待新鲜观测':data?.departureClear===false?'车身旁有近距回波，当前无法确认起步净距':data?.exit?'最终出口已标记':data?.hint?'搜索区已标记，仍会继续探索':'准备后可指定出口，也可先探索');
    el('mazeCalibration').textContent=data?'底座单侧外扩 '+Math.round(data.baseExtension*100)+' cm · 请按现场测量校准'+(data.coverage==='front'?' · 当前仅前向建图':''):'';
    if(!has)return;
    const span=data.cell*data.tile;
    if(view==='fit'){
      let xmin=data.pose[0]-1,xmax=xmin+2,ymin=data.pose[1]-1,ymax=ymin+2;
      for(const {tx,ty} of tiles.values()){xmin=Math.min(xmin,tx*span);ymin=Math.min(ymin,ty*span);xmax=Math.max(xmax,(tx+1)*span);ymax=Math.max(ymax,(ty+1)*span);}
      for(const p of [data.hint,data.exit])if(p){xmin=Math.min(xmin,p[0]);xmax=Math.max(xmax,p[0]);ymin=Math.min(ymin,p[1]);ymax=Math.max(ymax,p[1]);}
      pan=[(xmin+xmax)/2,(ymin+ymax)/2];scale=Math.min(w/(xmax-xmin+2),h/(ymax-ymin+2));
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
  setInterval(readMap,550);
  return {
    options(){return {mapId:data?.id||'',hint:hintValues(),mode:goals.querySelector('input[name="mazeMode"]:checked').value};},
    status(state,isOnline,isOwner){
      online=isOnline;owned=isOwner;prepared=state.preparedExperiment==='maze';
      if(!enabled)return;
      const running=state.task?.experiment==='maze'&&state.task.phase==='running';
      el('mazeModes').disabled=running;
      if(running){
        const activeMode=state.task.options?.mode==='roam'?'roam':'explore';
        goals.querySelector('input[value="'+activeMode+'"]').checked=true;
      }
      if(state.task?.experiment==='maze'&&state.task.phase==='running'){
        el('reason').textContent=state.task.log;el('stage').textContent=data?.paused?'已暂停':state.task.options?.mode==='roam'?'自主漫游':'地图探索';
        for(const name of ['manual','auto'])document.querySelector('[data-command="'+name+'"]').disabled=true;
      }
      const start=document.querySelector('[data-command="start"]');
      if(prepared){start.textContent=goals.querySelector('input[value="roam"]').checked?'开始自主漫游':'开始地图探索';if(!data?.healthy)start.disabled=true;}
      if(state.task?.experiment==='maze'&&state.task.phase==='completed')el('reason').textContent='已到达指定出口区并确认停稳';
      if(state.task?.experiment==='maze'&&state.task.phase==='failed'){
        el('stage').textContent='迷宫任务中止';el('reason').textContent=state.task.log.trim().split('\n').at(-1);
      }
      draw();
    }
  };
})();
