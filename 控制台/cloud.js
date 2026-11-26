import * as THREE from '/vendor/three.module.min.js';

const $=id=>document.getElementById(id),canvas=$('cloudCanvas');
let renderer;
try {renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true,powerPreference:'high-performance'});}
catch(error){$('sceneEmpty').querySelector('h2').textContent='三维渲染不可用';$('sceneEmpty').querySelector('p').textContent='请在浏览器中启用图形加速后刷新';$('goalHint').textContent='请在浏览器中启用图形加速后再选点';throw error;}
renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.setClearColor(0xe1eaec,0);
const scene=new THREE.Scene();scene.fog=new THREE.FogExp2(0xe1eaec,.018);
const camera=new THREE.PerspectiveCamera(46,1,.05,500);camera.up.set(0,0,1);
const overhead=new THREE.OrthographicCamera(-10,10,10,-10,.05,500);overhead.up.set(0,1,0);
let topView=false,visibleWidth=1,viewportWidth=1,viewportHeight=1;
const target=new THREE.Vector3();let azimuth=-2.3,elevation=.72,distance=30,follow=true,drag=null;
let cloud={points:[],frame:'',age:null},trajectory={positions:[],pose:null,age:null},cloudAt=0,poseAt=0;
let colorMode='distance',lastTrailEnd=null,cloudStamp=null,poseEpoch=null,cloudSource='live';
let mapId='',mapCount=0,removedCursor=0,groundCount=0,noiseCount=0,groundClassAt=null,sourceRevision=0;
const usesMap=(source=cloudSource)=>source==='map'||source==='overlay';
const pointCount=()=>usesMap()?Math.min(mapCount,cloud.active??mapCount):cloud.points.length;
let geometry=new THREE.BufferGeometry(),pointBuffer=new THREE.Float32BufferAttribute(new Float32Array(32768*3),3);
// The point object spans the entire mapped space and is never frustum-culled.
// A fixed sorting center keeps erased (NaN) slots out of Three's bounds calculation.
geometry.boundingSphere=new THREE.Sphere(new THREE.Vector3(),0);
let surfaceBuffer=new THREE.Uint8BufferAttribute(new Uint8Array(pointBuffer.count),1),enhanced=false,lastGroundPose=null,needsGround=true;
try{enhanced=localStorage.getItem('robot-obstacle-enhancement')==='on';}catch{}
surfaceBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('surfaceKind',surfaceBuffer);
pointBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('position',pointBuffer);geometry.setDrawRange(0,0);
const material=new THREE.ShaderMaterial({transparent:true,depthWrite:false,uniforms:{pointSize:{value:2},pixelRatio:{value:renderer.getPixelRatio()},opacity:{value:1},radius:{value:15},center:{value:new THREE.Vector3()},heightBase:{value:0},singleColor:{value:false},lightTheme:{value:true}},
  vertexShader:`
    uniform float pointSize, pixelRatio;
    uniform vec3 center;
    uniform bool enhance, scanLayer;
    attribute float surfaceKind;
    varying float kind, radial, range;
    void main(){
      if(!(position.x>=0.0||position.x<0.0)){gl_Position=vec4(2.0,2.0,2.0,1.0);gl_PointSize=1.0;return;}
      kind=surfaceKind; range=length(position-center); radial=length(position.xy-center.xy);
      vec4 eye=modelViewMatrix*vec4(position,1.0); gl_Position=projectionMatrix*eye;
      float scale=clamp(18.0/max(4.0,-eye.z),0.65,2.2);
      if(enhance)scale*=kind==2.0?2.8:kind==3.0?1.55:kind==1.0||kind==4.0?0.7:1.2;
      gl_PointSize=max(1.0,pointSize*pixelRatio*scale*(scanLayer?1.2:1.0));
    }`,
  fragmentShader:`
    uniform bool clipRange, singleColor, lightTheme, enhance, overheadView, hideGround, scanLayer;
    uniform float opacity, radius;
    varying float kind, radial, range;
    void main(){
      float d=length(gl_PointCoord-0.5);
      if(d>0.5||(clipRange&&radial>radius)||(hideGround&&(kind==1.0||kind==5.0)))discard;
      vec3 color; float alpha=opacity;
      if(scanLayer){
        color=lightTheme?vec3(.36,.19,.69):vec3(.72,.59,1.0);
        if(enhance&&(kind==1.0||kind==4.0))alpha*=.28;
      }else if(enhance){
        color=lightTheme?vec3(.25,.38,.43):vec3(.48,.66,.71);
        if(kind==1.0){alpha*=.12;}
        else if(kind==2.0){color=lightTheme?vec3(.79,.26,.035):vec3(1.0,.64,.20);}
        else if(kind==3.0){color=lightTheme?vec3(.10,.49,.55):vec3(.27,.78,.82);alpha*=.76;}
        else if(kind==4.0){alpha*=overheadView?.025:.09;}
        else{alpha*=.65;}
      }else{
        float t=clamp(range/max(radius,1.0),0.0,1.0);
        vec3 low=vec3(1.0,.15,.05),mid=vec3(.15,1.0,.25),high=vec3(.05,.25,1.0);
        color=singleColor?vec3(.32,.73,.79):(t<.55?mix(low,mid,t/.55):mix(mid,high,(t-.55)/.45));
        if(lightTheme)color*=.57;
      }
      gl_FragColor=vec4(color,alpha*(1.0-smoothstep(.27,.50,d)));
    }`});
const points=new THREE.Points(geometry,material);points.frustumCulled=false;scene.add(points);
material.uniforms.clipRange={value:true};
material.uniforms.enhance={value:false};material.uniforms.overheadView={value:false};material.uniforms.hideGround={value:false};
material.uniforms.scanLayer={value:false};
const scanGeometry=new THREE.BufferGeometry(),scanPositions=new THREE.Float32BufferAttribute(new Float32Array(8000*3),3);
const scanKinds=new THREE.Uint8BufferAttribute(new Uint8Array(scanPositions.count),1);
scanGeometry.setAttribute('position',scanPositions.setUsage(THREE.DynamicDrawUsage));
scanGeometry.setAttribute('surfaceKind',scanKinds.setUsage(THREE.DynamicDrawUsage));scanGeometry.setDrawRange(0,0);
scanGeometry.boundingSphere=new THREE.Sphere(new THREE.Vector3(),0);
const scanMaterial=material.clone();
for(const key of ['pointSize','pixelRatio','radius','center','lightTheme','enhance','overheadView','hideGround'])scanMaterial.uniforms[key]=material.uniforms[key];
scanMaterial.uniforms.scanLayer.value=true;
const scanPoints=new THREE.Points(scanGeometry,scanMaterial);
scanPoints.frustumCulled=false;scanPoints.renderOrder=1;scanPoints.visible=false;scene.add(scanPoints);
let scan=null,scanAt=0,scanCount=0;
function updateScanClasses(){
  if(scan&&groundClassAt&&sameMap(scan,cloud)){
    for(let i=0;i<scanCount;i++)scanKinds.array[i]=groundClassAt(scanPositions.getX(i),scanPositions.getY(i),scanPositions.getZ(i));
  }else scanKinds.array.fill(0,0,scanCount);
  scanKinds.needsUpdate=true;
}
function resizeCloudBuffer(count){
  if(count<=pointBuffer.count)return;
  const size=2**Math.ceil(Math.log2(count)),old=pointBuffer.array;
  geometry.dispose();geometry=new THREE.BufferGeometry();geometry.boundingSphere=new THREE.Sphere(new THREE.Vector3(),0);
  pointBuffer=new THREE.Float32BufferAttribute(new Float32Array(size*3),3);
  pointBuffer.array.set(old);pointBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('position',pointBuffer);
  surfaceBuffer=new THREE.Uint8BufferAttribute(new Uint8Array(size),1);
  surfaceBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('surfaceKind',surfaceBuffer);points.geometry=geometry;
}
function updateSurfaceClasses(){
  const count=usesMap()?mapCount:cloud.points.length;
  if(!count||!trajectory.position||!alignedCloud()||typeof window.classifyGround!=='function')return;
  groundClassAt=window.classifyGround(pointBuffer.array,count,trajectory.position,surfaceBuffer.array);
  groundCount=noiseCount=0;
  for(let i=0;i<count;i++){
    if(surfaceBuffer.array[i]===1)groundCount++;
    else if(surfaceBuffer.array[i]===5)noiseCount++;
  }
  overlayData=null;
  surfaceBuffer.needsUpdate=true;lastGroundPose=trajectory.position.slice();needsGround=false;
  updateScanClasses();
}
function styleCloud(){
  const filter=cloudSource!=='raw'&&!gridOpen();
  const registered=filter&&cloud.registered===true;
  const on=registered&&enhanced;
  material.uniforms.enhance.value=on;
  material.uniforms.hideGround.value=registered&&!$('showGround').checked;
  $('groundSetting').hidden=$('enhanceSetting').hidden=!filter;$('mapEnhance').checked=enhanced;
  $('scanLegend').hidden=cloudSource!=='overlay';
  $('distanceLegend').lastChild.textContent=usesMap()?'累计点云':'激光点云';
  $('surfaceLegend').hidden=!on;$('distanceLegend').hidden=on;
}
function setEnhancement(on){
  enhanced=on;
  try{localStorage.setItem('robot-obstacle-enhancement',on?'on':'off');}catch{}
  styleCloud();
}
$('mapEnhance').onchange=()=>setEnhancement($('mapEnhance').checked);
$('showGround').onchange=()=>{overlayData=null;styleCloud();};
const grid=new THREE.GridHelper(200,200,0x68868b,0x92a8aa);grid.rotation.x=Math.PI/2;grid.material.transparent=true;grid.material.opacity=.25;scene.add(grid);
function applyTheme(){const dark=document.documentElement.dataset.theme==='dark';scene.fog.color.setHex(dark?0x10151c:0xe1eaec);grid.material.opacity=dark?.18:.22;material.uniforms.lightTheme.value=!dark;}
document.addEventListener('themechange',applyTheme);applyTheme();
const axes=new THREE.AxesHelper(1.5);axes.material.transparent=true;axes.material.opacity=.38;scene.add(axes);
const trailMaterial=new THREE.MeshBasicMaterial({color:0xecc48c,transparent:true,opacity:.9,side:THREE.DoubleSide,depthTest:false,depthWrite:false});
const trail=new THREE.Mesh(new THREE.BufferGeometry(),trailMaterial);trail.frustumCulled=false;trail.renderOrder=3;scene.add(trail);
const robot=new THREE.Group();scene.add(robot);robot.visible=false;
const arrowShape=new THREE.Shape();arrowShape.moveTo(.55,0);arrowShape.lineTo(-.3,.27);arrowShape.lineTo(-.16,0);arrowShape.lineTo(-.3,-.27);arrowShape.closePath();
const arrow=new THREE.Mesh(new THREE.ShapeGeometry(arrowShape),new THREE.MeshBasicMaterial({color:0xf5d5a5,side:THREE.DoubleSide,depthTest:false}));arrow.position.z=.08;arrow.renderOrder=5;robot.add(arrow);
const ring=new THREE.Mesh(new THREE.RingGeometry(.53,.55,64),new THREE.MeshBasicMaterial({color:0xecc48c,transparent:true,opacity:.45,side:THREE.DoubleSide,depthTest:false}));ring.position.z=.04;robot.add(ring);
// Select on the horizontal plane in the same world frame as /Odometry.
const goalMarker=new THREE.Mesh(new THREE.RingGeometry(.18,.25,48),new THREE.MeshBasicMaterial({color:0xe75430,side:THREE.DoubleSide,depthTest:false}));
goalMarker.visible=false;goalMarker.renderOrder=8;scene.add(goalMarker);
const mazeLayer=new THREE.Group();scene.add(mazeLayer);
function mazeLine(color){
  const line=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color,depthTest:false}));
  line.renderOrder=6;mazeLayer.add(line);return line;
}
const planned=mazeLine(0xffd778),driven=mazeLine(0x7cd8d0);
const nearbyGeometry=new THREE.BufferGeometry(),nearbyPositions=new THREE.Float32BufferAttribute(new Float32Array(768*3),3);
nearbyPositions.setUsage(THREE.DynamicDrawUsage);nearbyGeometry.setAttribute('position',nearbyPositions);nearbyGeometry.setDrawRange(0,0);
const nearby=new THREE.Points(nearbyGeometry,new THREE.PointsMaterial({color:0xff825d,size:2.5,sizeAttenuation:false,transparent:true,opacity:.88,depthWrite:false}));
nearby.frustumCulled=false;nearby.renderOrder=4;nearby.visible=false;scene.add(nearby);
const exitMarker=goalMarker.clone(),hintMarker=goalMarker.clone();
exitMarker.material=goalMarker.material.clone();exitMarker.material.color.setHex(0x81efab);
hintMarker.material=goalMarker.material.clone();hintMarker.material.color.setHex(0xeec173);
mazeLayer.add(exitMarker,hintMarker);
let overlayData=null;
function overlay(data,aligned){
  mazeLayer.visible=!!data&&aligned&&data.generation===mapGeneration;
  nearby.visible=mazeLayer.visible&&usesMap()&&enhanced&&window.MazeView?.freshReturns();
  if(!mazeLayer.visible)return;
  mazeLayer.position.z=(trajectory.position?.[2]||0)+.04;
  if(data===overlayData)return;overlayData=data;
  let count=0;
  for(const p of data.nearReturns||[]){
    if(count>=nearbyPositions.count)break;
    if(material.uniforms.hideGround.value&&groundClassAt){
      const kind=groundClassAt(...p);if(kind===1||kind===5)continue;
    }
    nearbyPositions.setXYZ(count++,...p);
  }
  nearbyPositions.needsUpdate=true;nearbyGeometry.setDrawRange(0,count);
  for(const [line,xy] of [[planned,data.route],[driven,data.trail]]){
    line.geometry.dispose();line.geometry=new THREE.BufferGeometry().setFromPoints(xy.map(([x,y])=>new THREE.Vector3(x,y,0)));
  }
  for(const [marker,p] of [[exitMarker,data.exit],[hintMarker,data.hint]]){
    marker.visible=!!p;if(p)marker.position.set(p[0],p[1],.03);
  }
}
const goalButtons=[$('pickGoal'),$('mapPickGoal')];
const navigationSelected=()=>$('experiment').value==='lab3-xy';
let picking=false,goalFrame='',goalEpoch=null;
let mapGeneration='',preparingMap=false,goalRound=null,canPickGoal=false;
function setPicking(active){
  picking=active;canvas.style.cursor=active?'crosshair':'';
  for(const button of goalButtons){
    button.textContent=active?'取消选点':'在地图上选点';
    button.classList.toggle('active',active);button.setAttribute('aria-pressed',String(active));
  }
}
function goalHint(text,showOnMap=false){
  $('goalHint').textContent=$('mapGoalHint').textContent=text;
  $('mapGoalHint').hidden=!showOnMap;
}
function clearGoal(text){
  goalFrame='';goalEpoch=null;goalMarker.visible=false;setPicking(false);
  $('goalX').value=$('goalY').value='';goalHint(text);
}
window.NavigationMap=(generation,preparing,round,ready)=>{
  generation=generation||'';
  round=round||null;
  if(generation!==mapGeneration || (preparing && !preparingMap) || round!==goalRound)clearGoal('请在本轮准备完成后的地图上选取终点');
  else if(picking&&!ready)setPicking(false);
  mapGeneration=generation;goalRound=round;preparingMap=preparing;canPickGoal=ready;
};
document.addEventListener('control-mode-reset',()=>{
  canPickGoal=false;clearGoal('已切换控制目标，进入真机任务后请重新选点');
});
$('experiment').addEventListener('change',()=>{
  if(!navigationSelected()){setPicking(false);$('mapGoalHint').hidden=true;}
});
const sameMap=(a,b)=>!!a.generation&&a.generation===b.generation&&a.epoch===b.epoch&&!!a.frame&&a.frame===b.frame;
const currentStamp=stamp=>!/^\d{18,19}$/.test(mapGeneration)||stamp>=Number(mapGeneration)*1e-9;
const currentCloud=sample=>!preparingMap&&currentStamp(sample.stamp)&&(!sample.generation||sameMap(sample,trajectory));
const alignedCloud=()=>cloud.registered===true&&!!trajectory.frame&&cloud.frame===trajectory.frame&&currentStamp(trajectory.stamp)&&currentCloud(cloud);
const cloudLimit=()=>cloudSource==='live'?2:3;
const scanReady=()=>cloudSource==='overlay'&&!gridOpen()&&!!scan&&scanCount>0&&scan.connected&&age(scan.age,scanAt)<.5&&
  trajectory.connected&&age(trajectory.age,poseAt)<.5&&Math.abs(scan.stamp-trajectory.stamp)<=.5&&
  scan.registered&&alignedCloud()&&sameMap(scan,cloud)&&currentStamp(scan.stamp);
const goalReady=()=>canPickGoal&&trajectory.connected&&age(trajectory.age,poseAt)<.5&&trajectory.position&&cloud.connected&&age(cloud.age,cloudAt)<cloudLimit()&&alignedCloud();
function setGoal(x,y){
  if(!canPickGoal)throw Error('请先完成实验三扩展的准备');
  if(!goalReady())throw Error('请等待新鲜的定位与已配准点云，不可在原始雷达坐标中选点');
  if(!Number.isFinite(x)||!Number.isFinite(y)||Math.hypot(x-trajectory.position[0],y-trajectory.position[1])>30)throw Error('终点须在当前位置 30 m 内');
  goalFrame=trajectory.frame;goalEpoch=trajectory.epoch;
  $('goalX').value=x.toFixed(2);$('goalY').value=y.toFixed(2);
  goalMarker.position.set(x,y,trajectory.position[2]);goalMarker.visible=true;
  goalHint(`已选终点 (${x.toFixed(2)}, ${y.toFixed(2)}) m · 点击开始实验执行`,true);
}
function togglePicking(){
  if(picking){setPicking(false);goalHint('已取消本次地图选点',true);return;}
  if(!canPickGoal){goalHint('请先连接真机并完成「准备实验」',true);return;}
  if(cloudSource==='raw'||gridOpen())window.PointCloudView.select('live');
  view(true);setPicking(true);goalHint('请点击地图中的空闲地面',true);
}
for(const button of goalButtons){button.disabled=false;button.onclick=togglePicking;}
document.addEventListener('keydown',event=>{if(picking&&event.key==='Escape')togglePicking();});
for(const id of ['goalX','goalY'])$(id).addEventListener('change',()=>{
  goalFrame='';goalEpoch=null;goalMarker.visible=false;
  try{if($('goalX').value!==''&&$('goalY').value!==''){setGoal(Number($('goalX').value),Number($('goalY').value));setPicking(false);}}
  catch(error){goalHint(error.message);}
});
window.hasNavigationGoal=()=>Boolean(goalFrame&&goalEpoch===trajectory.epoch&&goalFrame===trajectory.frame);
window.NavigationGoal=()=>{
  if(!goalFrame)return {};
  if($('goalX').value===''||$('goalY').value==='')throw Error('请输入完整的 X、Y 坐标');
  if(!goalReady()||goalFrame!==trajectory.frame||goalEpoch!==trajectory.epoch)throw Error('定位已更新或失效，请重新选取终点');
  return {goalX:Number($('goalX').value),goalY:Number($('goalY').value),goalFrame};
};
canvas.addEventListener('pointerdown',e=>{
  if(!picking||e.button!==0)return;e.stopImmediatePropagation();e.preventDefault();
  try{
    if(!goalReady())throw Error('请等待已配准点云与定位');
    const rect=canvas.getBoundingClientRect(),ray=new THREE.Raycaster(),hit=new THREE.Vector3();
    ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,1-(e.clientY-rect.top)/rect.height*2),topView?overhead:camera);
    if(!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,0,1),-trajectory.position[2]),hit))throw Error('无法选取地面，请使用俯视图');
    setGoal(hit.x,hit.y);setPicking(false);
  }catch(error){goalHint(error.message,true);}
},true);
function setActive(id,value){const b=$(id);b.classList.toggle('active',value);if(b.getAttribute('aria-pressed')!==String(value))b.setAttribute('aria-pressed',String(value));}
function setFollow(value){follow=value;setActive('followRobot',value);}
function view(top=false){topView=top;material.uniforms.overheadView.value=top;azimuth=top?-Math.PI/2:-2.3;elevation=top?Math.PI/2-.001:.72;setActive('viewTop',top);setActive('view3d',!top);}
function resize(){
  const w=canvas.clientWidth,h=canvas.clientHeight;if(w<1||h<1)return;
  viewportWidth=w;viewportHeight=h;const rect=canvas.getBoundingClientRect(),side=document.querySelector('.inspector').getBoundingClientRect();
  visibleWidth=side.top<rect.bottom&&side.left>rect.left?Math.min(w,side.left-rect.left):w;
  renderer.setSize(w,h,false);camera.setViewOffset(w,h,(w-visibleWidth)/2,0,w,h);
}
new ResizeObserver(resize).observe(canvas);resize();
const gridOpen=()=>document.querySelector('.scene').dataset.surface==='grid';
window.syncCloudControls=()=>{
  const grid=gridOpen();
  setActive('viewTop',grid||topView);setActive('view3d',!grid&&!topView);
  setActive('followRobot',grid?!!window.MazeView?.gridFollowing():follow);
};
function chooseView(top){
  if(gridOpen())window.PointCloudView.select('map');
  view(top);
}
$('view3d').onclick=()=>chooseView(false);$('viewTop').onclick=()=>chooseView(true);
$('viewReset').onclick=()=>{
  if(gridOpen()){window.MazeView.gridView('fit');return;}
  target.set(0,0,0);distance=Number($('cloudRange').value)*2;setFollow(true);view();
};
$('followRobot').onclick=()=>{
  if(gridOpen())window.MazeView.gridView(window.MazeView.gridFollowing()?'pan':'follow');
  else setFollow(!follow);
};
$('cloudMapFit').onclick=()=>{
  if(gridOpen()){window.MazeView.gridView('fit');return;}
  if(!mapCount||!usesMap()||!currentCloud(cloud))return;
  const box=new THREE.Box3(),point=new THREE.Vector3();
  for(let i=0;i<mapCount;i++)if(Number.isFinite(pointBuffer.getX(i)))box.expandByPoint(point.fromBufferAttribute(pointBuffer,i));
  if(box.isEmpty())return;
  box.getCenter(target);const span=box.getSize(point).length();
  distance=Math.max(3,span*1.35*Math.max(1,viewportHeight/visibleWidth));setFollow(false);
};
$('cloudRange').addEventListener('input',()=>{const value=Number($('cloudRange').value);if(value>=5&&value<=50){material.uniforms.radius.value=value;distance=value*2;}});
$('pointSize').addEventListener('input',()=>{const n=Number($('pointSize').value);if(n>=1&&n<=6)material.uniforms.pointSize.value=n;});
document.querySelectorAll('[data-color]').forEach(b=>b.onclick=()=>{
  colorMode=b.dataset.color;material.uniforms.singleColor.value=colorMode==='mono';
  if(material.uniforms.enhance.value)setEnhancement(false);
  document.querySelectorAll('[data-color]').forEach(other=>{other.classList.toggle('active',other===b);other.setAttribute('aria-pressed',String(other===b));});
});
function setSource(name){
  if(name!=='grid'&&name!==cloudSource){
    if(!usesMap(name)||!usesMap()){
      sourceRevision++;cloudStamp=null;mapId='';mapCount=removedCursor=groundCount=noiseCount=0;
      cloud={points:[],frame:'',age:null};geometry.setDrawRange(0,0);
      lastGroundPose=null;groundClassAt=null;needsGround=true;
    }
    scan=null;scanCount=0;scanGeometry.setDrawRange(0,0);
    cloudSource=name;
  }
  material.uniforms.clipRange.value=!usesMap();
  styleCloud();window.syncCloudControls();labels();
}
document.addEventListener('cloud-view-change',event=>setSource(event.detail));
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.onpointerdown=e=>{drag={id:e.pointerId,x:e.clientX,y:e.clientY,pan:e.button===2||e.shiftKey};canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
  if(topView){setFollow(false);const k=distance*.85/canvas.clientHeight;target.x-=dx*k;target.y+=dy*k;}
  else if(drag.pan){setFollow(false);const k=distance/canvas.clientHeight;target.x+=(-dx*Math.sin(azimuth)+dy*Math.cos(azimuth))*k;target.y+=(dx*Math.cos(azimuth)+dy*Math.sin(azimuth))*k;}
  else{azimuth-=dx*.006;elevation=THREE.MathUtils.clamp(elevation+dy*.006,.08,Math.PI/2-.001);setActive('viewTop',false);setActive('view3d',true);}
};canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};
canvas.addEventListener('wheel',e=>{e.preventDefault();distance=THREE.MathUtils.clamp(distance*Math.exp(e.deltaY*.001),2,cloudSource==='map'?4000:180);},{passive:false});
function updateTrail(){
  const source=trajectory.positions||[];const coords=[];
  // A narrow world-space ribbon keeps the line readable at oblique angles.
  for(let i=1;i<source.length;i++){const a=source[i-1],b=source[i],dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);if(len<1e-6)continue;const x=-dy/len*.025,y=dx/len*.025;
    const a1=[a[0]+x,a[1]+y,a[2]+.025],a2=[a[0]-x,a[1]-y,a[2]+.025],b1=[b[0]+x,b[1]+y,b[2]+.025],b2=[b[0]-x,b[1]-y,b[2]+.025];coords.push(...a1,...a2,...b1,...b1,...a2,...b2);}
  trail.geometry.dispose();trail.geometry=new THREE.BufferGeometry();trail.geometry.setAttribute('position',new THREE.Float32BufferAttribute(coords,3));
}
const age=(value,at)=>Number.isFinite(value)?value+(performance.now()-at)/1000:Infinity;
function render(){
  requestAnimationFrame(render);if(document.hidden||gridOpen())return;
  const cloudAge=age(cloud.age,cloudAt),poseAge=age(trajectory.age,poseAt);
  const freshPose=trajectory.connected&&poseAge<1&&trajectory.position;
  // Raw lidar points have no world transform. Never silently overlay odometry.
  const aligned=alignedCloud();
  goalMarker.visible=navigationSelected()&&aligned&&window.hasNavigationGoal();
  const poseOnly=!pointCount()||usesMap()&&!aligned;
  const mazeData=window.MazeView?.data();overlay(mazeData,aligned);
  robot.visible=!!freshPose&&(aligned||poseOnly);
  trail.visible=!mazeLayer.visible&&$('showTrail').checked&&(aligned||poseOnly)&&trajectory.positions?.length>1;
  driven.visible=$('showTrail').checked;
  trailMaterial.opacity=poseAge<1?.9:.22;
  if(robot.visible){robot.position.fromArray(trajectory.position);robot.quaternion.fromArray(trajectory.quaternion);if(follow)target.copy(robot.position);}
  if(aligned&&trajectory.position)material.uniforms.center.value.fromArray(trajectory.position);else material.uniforms.center.value.set(0,0,0);
  material.uniforms.heightBase.value=aligned&&trajectory.position?trajectory.position[2]:0;
  points.visible=!cloud.registered||currentCloud(cloud);
  scanPoints.visible=scanReady();
  const opacity=cloud.connected&&cloudAge<cloudLimit()?.95:usesMap()?.65:.16;
  material.uniforms.opacity.value=scanPoints.visible?opacity*.58:opacity;
  if(!aligned&&!poseOnly&&follow)target.set(0,0,0);
  grid.visible=$('showGrid').checked;grid.position.set(Math.round(target.x/10)*10,Math.round(target.y/10)*10,-.03);
  if(topView){
    const k=distance*.85/viewportHeight;
    overhead.left=-visibleWidth*k/2;overhead.right=(viewportWidth-visibleWidth/2)*k;
    overhead.top=viewportHeight*k/2;overhead.bottom=-overhead.top;overhead.far=Math.max(500,distance*3);
    overhead.position.set(target.x,target.y,target.z+distance);overhead.lookAt(target);overhead.updateProjectionMatrix();renderer.render(scene,overhead);
  }else{
    const far=Math.max(500,distance*3);if(far!==camera.far){camera.far=far;camera.updateProjectionMatrix();}
    camera.position.set(target.x+distance*Math.cos(elevation)*Math.cos(azimuth),target.y+distance*Math.cos(elevation)*Math.sin(azimuth),target.z+distance*Math.sin(elevation));camera.lookAt(target);renderer.render(scene,camera);
  }
}
function labels(){
  if(goalFrame&&(goalFrame!==trajectory.frame||goalEpoch!==trajectory.epoch))clearGoal('定位坐标已重置，请重新选终点');
  const ca=age(cloud.age,cloudAt),pa=age(trajectory.age,poseAt),aligned=alignedCloud();
  const count=cloud.registered&&!currentCloud(cloud)?0:pointCount();
  const hasScan=scanReady(),hidden=material.uniforms.hideGround.value?Math.min(groundCount+noiseCount,count):0;
  $('sceneEmpty').hidden=count>0||hasScan||(!usesMap()&&!!trajectory.position);
  $('nearLegend').hidden=!nearby.visible||nearbyGeometry.drawRange.count===0;
  $('cloudCount').textContent=hidden?(count-hidden).toLocaleString()+' 可见 · '+hidden.toLocaleString()+' 已过滤':count.toLocaleString()+' POINTS';
  $('cloudCount').title=hidden?'地面 '+groundCount+' 点，孤立地下回波 '+noiseCount+' 点':'';
  const source=cloud.source==='map'?`累计点云 · ${cloud.scans} 帧`:cloud.source==='aggregate'?`一秒聚合 · ${cloud.frames} 帧`:cloud.source==='preview'?'模拟预览':cloud.registered?'配准单帧':'原始单帧';
  const baseStatus=!Number.isFinite(ca)?(cloud.error||'点云 · 等待数据'):`${source} · ${Math.round(ca*1000)} ms${ca>=cloudLimit()||!cloud.connected?' · 已过期':''}`;
  $('cloudStatus').textContent=cloudSource==='overlay'?`${baseStatus} · ${hasScan?'实时扫描已叠加':'等待同地图扫描'}`:baseStatus;
  $('poseStatus').textContent=!trajectory.connected?'定位 · 离线':!Number.isFinite(pa)?'定位 · 等待数据':`定位 · 收到后 ${Math.round(pa*1000)} ms${pa>=1?' · 已过期':''}`;
  $('cloudStatus').classList.toggle('stale',ca>=cloudLimit()&&pointCount()>0);$('poseStatus').classList.toggle('stale',pa>=1&&!!trajectory.pose);
  $('frameLabel').textContent=cloud.frame||trajectory.frame||'等待数据';
  $('alignmentStatus').textContent=aligned?`${cloud.frame} · 位姿与点云帧末相差 ${Math.round(1000*Math.abs(trajectory.stamp-cloud.stamp))} ms`:cloud.registered===false?'雷达自身坐标系，轨迹暂不叠加':'等待同一地图的点云与定位';
}
async function refreshCloud(){
  const started=performance.now(),requested=usesMap()?'map':cloudSource,revision=sourceRevision;
  const query=new URLSearchParams({brief:'1',seen:cloud.tag||''});
  if(requested!=='live')query.set('source',requested);
  const url=requested==='map'?'/api/cloud-map?'+new URLSearchParams({id:mapId,from:String(mapCount),removed:String(removedCursor)}):'/api/cloud?'+query;
  try{const r=await fetch(url,{signal:AbortSignal.timeout(requested==='live'?800:2500),cache:'no-store'});if(!r.ok)throw Error();const next=await r.json();
    if(revision===sourceRevision&&requested==='map'){
      const first=next.id!==mapId;
      if(next.from!==(first?0:mapCount)||next.end-next.from>(first?65536:4096)||next.end<next.from)throw Error('累计点云分段已过期');
      const binary=atob(next.xyz),bytes=Uint8Array.from(binary,c=>c.charCodeAt(0));
      if(bytes.length!==(next.end-next.from)*12)throw Error('累计点云分段不完整');
      resizeCloudBuffer(next.end);
      pointBuffer.array.set(new Float32Array(bytes.buffer),next.from*3);
      if(first){mapId=next.id;groundCount=noiseCount=0;groundClassAt=null;lastGroundPose=null;surfaceBuffer.array.fill(0,0,next.end);surfaceBuffer.needsUpdate=true;}
      mapCount=next.end;
      for(const i of next.removed||[])if(i<mapCount)pointBuffer.setXYZ(i,NaN,NaN,NaN);
      removedCursor=next.removedEnd||0;
      if(bytes.length||next.removed?.length)pointBuffer.needsUpdate=true;
      geometry.setDrawRange(0,mapCount);cloudAt=started;cloud={...next,points:[]};
      if(first||bytes.length||next.removed?.length||!lastGroundPose||trajectory.position?.some((n,i)=>Math.abs(n-lastGroundPose[i])>.35))needsGround=true;
      styleCloud();
      if(needsGround)updateSurfaceClasses();
    }else if(revision===sourceRevision&&next.unchanged){
      cloudAt=started;cloud.age=next.age;cloud.connected=next.connected;
    }else if(revision===sourceRevision){
      if(!Array.isArray(next.points)||next.points.length>1000000)throw Error('点云数据超出显示容量');
      if(Number.isFinite(next.age)&&Number.isFinite(next.trajectory?.stamp)&&Number.isFinite(next.stamp)&&Number.isFinite(next.trajectory.age)){
        next.age=Math.max(next.age,next.trajectory.stamp-next.stamp+next.trajectory.age);
      }
      cloudAt=started;const changed=next.stamp==null||next.stamp!==cloudStamp||next.frame!==cloud.frame||next.source!==cloud.source||next.generation!==cloud.generation||next.epoch!==cloud.epoch;cloud=next;cloudStamp=cloud.stamp;
      if(changed){const count=cloud.points.length;
        resizeCloudBuffer(count);
        for(let i=0;i<count;i++)pointBuffer.setXYZ(i,...cloud.points[i]);pointBuffer.needsUpdate=true;geometry.setDrawRange(0,count);
        surfaceBuffer.array.fill(0,0,count);surfaceBuffer.needsUpdate=true;
        groundCount=noiseCount=0;groundClassAt=null;needsGround=requested!=='raw'&&cloud.registered;
        styleCloud();
        if(needsGround)updateSurfaceClasses();
      }
    }
  }catch(error){if(revision===sourceRevision){cloud.connected=false;cloud.error=error.message||'点云连接中断';}}
  labels();
  // A returning tab can read a large existing map without waiting a second
  // between every 4096-point chunk. Idle maps retain the usual polling rate.
  const catchingUp=usesMap()&&cloud.connected&&(mapCount<cloud.count||removedCursor<cloud.removedCount);
  const period=catchingUp?60:1000;
  setTimeout(refreshCloud,Math.max(0,period-(performance.now()-started)));
}
async function refreshScan(){
  const started=performance.now();
  if(cloudSource==='overlay'&&!gridOpen()){
    try{
      const response=await fetch('/api/cloud?'+new URLSearchParams({brief:'1',seen:scan?.tag||''}),{signal:AbortSignal.timeout(800),cache:'no-store'});
      if(!response.ok)throw Error();
      const next=await response.json();
      if(cloudSource==='overlay'&&!gridOpen()&&next.unchanged&&scan){
        scanAt=started;scan.age=next.age;scan.connected=next.connected;
      }else if(cloudSource==='overlay'&&!gridOpen()&&!next.unchanged){
        const changed=!scan||next.stamp!==scan.stamp||!sameMap(next,scan);
        scan=next;scanAt=started;
        if(changed){
          scanCount=next.registered&&Number.isFinite(next.stamp)&&Array.isArray(next.points)?Math.min(next.points.length,scanPositions.count):0;
          for(let i=0;i<scanCount;i++)scanPositions.setXYZ(i,...next.points[i]);
          scanPositions.needsUpdate=true;scanGeometry.setDrawRange(0,scanCount);updateScanClasses();
        }
      }
    }catch{scan=null;}
    labels();
  }
  setTimeout(refreshScan,Math.max(0,1000-(performance.now()-started)));
}
async function refreshPose(){
  const started=performance.now();
  try{const q=new URLSearchParams({id:trajectory.id||'',from:String(trajectory.end||0)});
    const r=await fetch('/api/trajectory?'+q,{signal:AbortSignal.timeout(800),cache:'no-store'});if(!r.ok)throw Error();
    const next=await r.json();
    if(next.reset===false){for(const key of ['points','positions'])next[key]=next[key].length?trajectory[key].concat(next[key]).slice(-2000):trajectory[key];}
    trajectory=next;poseAt=started;
    if(next.reset||next.end!==lastTrailEnd||trajectory.epoch!==poseEpoch){lastTrailEnd=next.end;poseEpoch=trajectory.epoch;updateTrail();}
    if(needsGround)updateSurfaceClasses();
  }catch{trajectory.connected=false;}labels();setTimeout(refreshPose,Math.max(0,100-(performance.now()-started)));
}
setSource(document.querySelector('.scene').dataset.surface||'live');
render();refreshCloud();refreshPose();refreshScan();
