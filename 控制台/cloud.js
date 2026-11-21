import * as THREE from '/vendor/three.module.min.js';

const $=id=>document.getElementById(id),canvas=$('cloudCanvas');
let renderer;
try {renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true,powerPreference:'high-performance'});}
catch(error){$('sceneEmpty').querySelector('h2').textContent='三维渲染不可用';$('sceneEmpty').querySelector('p').textContent='请在浏览器中启用图形加速后刷新';throw error;}
renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.setClearColor(0xe1eaec,0);
const scene=new THREE.Scene();scene.fog=new THREE.FogExp2(0xe1eaec,.018);
const camera=new THREE.PerspectiveCamera(46,1,.05,500);camera.up.set(0,0,1);
const target=new THREE.Vector3();let azimuth=-2.3,elevation=.72,distance=30,follow=true,drag=null;
let cloud={points:[],frame:'',age:null},trajectory={positions:[],pose:null,age:null},cloudAt=0,poseAt=0;
let colorMode='distance',poseStamp=null,cloudStamp=null,poseEpoch=null,cloudSource='live';
let geometry=new THREE.BufferGeometry(),pointBuffer=new THREE.Float32BufferAttribute(new Float32Array(32768*3),3);
pointBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('position',pointBuffer);geometry.setDrawRange(0,0);
const material=new THREE.ShaderMaterial({transparent:true,depthWrite:false,uniforms:{pointSize:{value:2},pixelRatio:{value:renderer.getPixelRatio()},opacity:{value:1},radius:{value:15},center:{value:new THREE.Vector3()},heightBase:{value:0},singleColor:{value:false},lightTheme:{value:true}},
  vertexShader:`uniform float pointSize;uniform float pixelRatio;uniform vec3 center;uniform float heightBase;varying float height;varying float radial;void main(){height=length(position-center);radial=length(position.xy-center.xy);vec4 p=modelViewMatrix*vec4(position,1.);gl_Position=projectionMatrix*p;gl_PointSize=pointSize*pixelRatio*clamp(18./max(4.,-p.z),.65,2.2);}`,
  fragmentShader:`uniform float opacity;uniform float radius;uniform bool singleColor;uniform bool lightTheme;varying float height;varying float radial;void main(){vec2 p=gl_PointCoord-.5;float d=length(p);if(d>.5||radial>radius)discard;vec3 low=vec3(1.,.15,.05),mid=vec3(.15,1.,.25),high=vec3(.05,.25,1.);float t=clamp(height/max(radius,1.),0.,1.);vec3 c=singleColor?vec3(.32,.73,.79):(t<.55?mix(low,mid,t/.55):mix(mid,high,(t-.55)/.45));if(lightTheme)c*=.57;gl_FragColor=vec4(c,opacity*(1.-smoothstep(.3,.5,d)));}`});
const points=new THREE.Points(geometry,material);points.frustumCulled=false;scene.add(points);
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
const goalPanel=document.createElement('div');goalPanel.className='field';
goalPanel.innerHTML='<label>导航终点 · SLAM 坐标（m）</label><div class="number-control"><input id="goalX" type="number" step="0.01" placeholder="X" aria-label="终点 X"><input id="goalY" type="number" step="0.01" placeholder="Y" aria-label="终点 Y"></div><button id="pickGoal" class="secondary" type="button">在左侧点云选终点</button><p id="goalHint" class="subtle" role="status">准备实验后选点，再点击开始实验</p>';
$('taskParameters').after(goalPanel);
let picking=false,goalFrame='',goalEpoch=null;
let mapGeneration='',preparingMap=false;
window.NavigationMap=(generation,preparing)=>{
  if((generation && generation!==mapGeneration) || (preparing && !preparingMap)){
    goalFrame='';goalEpoch=null;picking=false;goalMarker.visible=false;canvas.style.cursor='';
    $('goalX').value='';$('goalY').value='';$('pickGoal').textContent='在左侧点云选终点';
    $('goalHint').textContent='请在本轮准备完成后的新地图上选取终点';
  }
  if(generation)mapGeneration=generation;
  preparingMap=preparing;
};
document.addEventListener('control-mode-reset',()=>{
  picking=false;goalFrame='';goalEpoch=null;goalMarker.visible=false;canvas.style.cursor='';
  $('goalX').value='';$('goalY').value='';$('pickGoal').textContent='在左侧点云选终点';
  $('goalHint').textContent='已切换控制目标，进入真机任务后请重新选点';
});
const currentMap=()=>!preparingMap&&(!mapGeneration||Math.min(cloud.stamp,trajectory.stamp)>=Number(mapGeneration)*1e-9);
const alignedCloud=()=>cloud.registered===true&&!!trajectory.frame&&cloud.frame===trajectory.frame&&currentMap();
const cloudLimit=()=>cloudSource==='live'?.5:3;
const goalReady=()=>trajectory.connected&&age(trajectory.age,poseAt)<.5&&trajectory.position&&cloud.connected&&age(cloud.age,cloudAt)<cloudLimit()&&alignedCloud();
function setGoal(x,y){
  if(!goalReady())throw Error('请等待新鲜的定位与已配准点云，不可在原始雷达坐标中选点');
  if(!Number.isFinite(x)||!Number.isFinite(y)||Math.hypot(x-trajectory.position[0],y-trajectory.position[1])>30)throw Error('终点须在当前位置 30 m 内');
  goalFrame=trajectory.frame;goalEpoch=trajectory.epoch;
  $('goalX').value=x.toFixed(2);$('goalY').value=y.toFixed(2);
  goalMarker.position.set(x,y,trajectory.position[2]);goalMarker.visible=true;
  $('goalHint').textContent=`终点 (${x.toFixed(2)}, ${y.toFixed(2)}) · ${goalFrame}，点击开始实验执行`;
}
$('pickGoal').onclick=()=>{picking=!picking;if(picking)view(true);$('pickGoal').textContent=picking?'请点击左侧空闲地面（再次点击取消）':'在左侧点云选终点';canvas.style.cursor=picking?'crosshair':'';};
for(const id of ['goalX','goalY'])$(id).addEventListener('change',()=>{try{if($('goalX').value!==''&&$('goalY').value!=='')setGoal(Number($('goalX').value),Number($('goalY').value));}catch(error){goalFrame='';goalMarker.visible=false;$('goalHint').textContent=error.message;}});
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
    ray.setFromCamera(new THREE.Vector2((e.clientX-rect.left)/rect.width*2-1,1-(e.clientY-rect.top)/rect.height*2),camera);
    if(!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0,0,1),-trajectory.position[2]),hit))throw Error('无法选取地面，请使用俯视图');
    setGoal(hit.x,hit.y);picking=false;canvas.style.cursor='';$('pickGoal').textContent='在左侧点云选终点';
  }catch(error){$('goalHint').textContent=error.message;}
},true);
function setActive(id,value){$(id).classList.toggle('active',value);$(id).setAttribute('aria-pressed',String(value));}
function setFollow(value){follow=value;setActive('followRobot',value);}
function view(top=false){azimuth=top?-Math.PI/2:-2.3;elevation=top?Math.PI/2-.001:.72;distance=Number($('cloudRange').value)*2;setActive('viewTop',top);setActive('view3d',!top);}
function resize(){const w=canvas.clientWidth,h=canvas.clientHeight;if(w<1||h<1)return;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
new ResizeObserver(resize).observe(canvas);resize();
$('view3d').onclick=()=>view();$('viewTop').onclick=()=>view(true);$('viewReset').onclick=()=>{target.set(0,0,0);setFollow(true);view();};$('followRobot').onclick=()=>setFollow(!follow);
$('cloudRange').addEventListener('input',()=>{const value=Number($('cloudRange').value);if(value>=5&&value<=50){material.uniforms.radius.value=value;distance=value*2;}});
$('pointSize').addEventListener('input',()=>{const n=Number($('pointSize').value);if(n>=1&&n<=6)material.uniforms.pointSize.value=n;});
document.querySelectorAll('[data-color]').forEach(b=>b.onclick=()=>{colorMode=b.dataset.color;material.uniforms.singleColor.value=colorMode==='mono';document.querySelectorAll('[data-color]').forEach(other=>{other.classList.toggle('active',other===b);other.setAttribute('aria-pressed',String(other===b));});});
document.querySelectorAll('[data-cloud-source]').forEach(b=>b.onclick=()=>{
  if(cloudSource===b.dataset.cloudSource)return;cloudSource=b.dataset.cloudSource;cloudStamp=null;cloud={points:[],frame:'',age:null};geometry.setDrawRange(0,0);
  document.querySelectorAll('[data-cloud-source]').forEach(other=>{other.classList.toggle('active',other===b);other.setAttribute('aria-pressed',String(other===b));});
  $('cloudSourceHint').textContent=cloudSource==='live'?'最新配准扫描 · 最多显示 1600 点':cloudSource==='aggregate'?'已配准点云 · 最近一秒的完整各帧':'雷达原始单帧 · 全部有效 XYZ 点';labels();
});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.onpointerdown=e=>{drag={id:e.pointerId,x:e.clientX,y:e.clientY,pan:e.button===2||e.shiftKey};canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
  if(drag.pan){setFollow(false);const k=distance/canvas.clientHeight;target.x+=(-dx*Math.sin(azimuth)+dy*Math.cos(azimuth))*k;target.y+=(dx*Math.cos(azimuth)+dy*Math.sin(azimuth))*k;}
  else{azimuth-=dx*.006;elevation=THREE.MathUtils.clamp(elevation+dy*.006,.08,Math.PI/2-.001);setActive('viewTop',false);setActive('view3d',true);}
};canvas.onpointerup=canvas.onpointercancel=()=>{drag=null;};
canvas.addEventListener('wheel',e=>{e.preventDefault();distance=THREE.MathUtils.clamp(distance*Math.exp(e.deltaY*.001),2,180);},{passive:false});
function updateTrail(){
  const source=trajectory.positions||[];const coords=[];
  // A narrow world-space ribbon keeps the line readable at oblique angles.
  for(let i=1;i<source.length;i++){const a=source[i-1],b=source[i],dx=b[0]-a[0],dy=b[1]-a[1],len=Math.hypot(dx,dy);if(len<1e-6)continue;const x=-dy/len*.025,y=dx/len*.025;
    const a1=[a[0]+x,a[1]+y,a[2]+.025],a2=[a[0]-x,a[1]-y,a[2]+.025],b1=[b[0]+x,b[1]+y,b[2]+.025],b2=[b[0]-x,b[1]-y,b[2]+.025];coords.push(...a1,...a2,...b1,...b1,...a2,...b2);}
  trail.geometry.dispose();trail.geometry=new THREE.BufferGeometry();trail.geometry.setAttribute('position',new THREE.Float32BufferAttribute(coords,3));
}
const age=(value,at)=>Number.isFinite(value)?value+(performance.now()-at)/1000:Infinity;
function render(){
  requestAnimationFrame(render);if(document.hidden)return;
  const cloudAge=age(cloud.age,cloudAt),poseAge=age(trajectory.age,poseAt);
  const freshPose=trajectory.connected&&poseAge<1&&trajectory.position;
  // Raw lidar points have no world transform. Never silently overlay odometry.
  const aligned=alignedCloud();
  const poseOnly=!cloud.points.length;
  robot.visible=!!freshPose&&(aligned||poseOnly);
  trail.visible=$('showTrail').checked&&(aligned||poseOnly)&&trajectory.positions?.length>1;
  trailMaterial.opacity=poseAge<1?.9:.22;
  if(robot.visible){robot.position.fromArray(trajectory.position);robot.quaternion.fromArray(trajectory.quaternion);if(follow)target.copy(robot.position);}
  if(aligned&&trajectory.position)material.uniforms.center.value.fromArray(trajectory.position);else material.uniforms.center.value.set(0,0,0);
  material.uniforms.heightBase.value=aligned&&trajectory.position?trajectory.position[2]:0;
  points.visible=!cloud.registered||(!preparingMap&&(!mapGeneration||cloud.stamp>=Number(mapGeneration)*1e-9));
  material.uniforms.opacity.value=cloud.connected&&cloudAge<cloudLimit()?.95:.16;
  if(!aligned&&!poseOnly&&follow)target.set(0,0,0);
  grid.visible=$('showGrid').checked;grid.position.set(Math.round(target.x/10)*10,Math.round(target.y/10)*10,-.03);
  camera.position.set(target.x+distance*Math.cos(elevation)*Math.cos(azimuth),target.y+distance*Math.cos(elevation)*Math.sin(azimuth),target.z+distance*Math.sin(elevation));camera.lookAt(target);renderer.render(scene,camera);
}
function labels(){
  goalPanel.hidden=$('experiment').value!=='lab3-xy';
  if(goalFrame&&(goalFrame!==trajectory.frame||goalEpoch!==trajectory.epoch)){goalFrame='';goalMarker.visible=false;$('goalHint').textContent='定位坐标已重置，请重新选终点';}
  const ca=age(cloud.age,cloudAt),pa=age(trajectory.age,poseAt),aligned=alignedCloud();
  $('sceneEmpty').hidden=cloud.points.length>0||!!trajectory.position;
  $('cloudCount').textContent=cloud.points.length.toLocaleString()+' POINTS';
  const source=cloud.source==='aggregate'?`一秒聚合 · ${cloud.frames} 帧`:cloud.source==='preview'?'模拟预览':cloud.registered?'配准单帧':'原始单帧';
  $('cloudStatus').textContent=!Number.isFinite(ca)?(cloud.error||'点云 · 等待数据'):`${source} · ${Math.round(ca*1000)} ms${ca>=cloudLimit()||!cloud.connected?' · 已过期':''}`;
  $('poseStatus').textContent=!trajectory.connected?'定位 · 离线':!Number.isFinite(pa)?'定位 · 等待数据':`定位 · 收到后 ${Math.round(pa*1000)} ms${pa>=1?' · 已过期':''}`;
  $('cloudStatus').classList.toggle('stale',ca>=cloudLimit()&&cloud.points.length>0);$('poseStatus').classList.toggle('stale',pa>=1&&!!trajectory.pose);
  $('frameLabel').textContent=cloud.frame||trajectory.frame||'等待数据';
  $('alignmentStatus').textContent=aligned?`${cloud.frame} · 位姿与点云帧末相差 ${Math.round(1000*Math.abs(trajectory.stamp-cloud.stamp))} ms`:cloud.registered===false?'雷达自身坐标系，轨迹暂不叠加':'等待同一地图的点云与定位';
}
async function refreshCloud(){
  const started=performance.now(),requested=cloudSource;
  try{const r=await fetch(requested==='live'?'/api/cloud':'/api/cloud?source='+requested,{signal:AbortSignal.timeout(requested==='live'?800:2500),cache:'no-store'});if(!r.ok)throw Error();const next=await r.json();
    if(requested===cloudSource){
      if(!Array.isArray(next.points)||next.points.length>1000000)throw Error('点云数据超出显示容量');
      if(Number.isFinite(next.age)&&Number.isFinite(next.trajectory?.stamp)&&Number.isFinite(next.stamp)&&Number.isFinite(next.trajectory.age)){
        next.age=Math.max(next.age,next.trajectory.stamp-next.stamp+next.trajectory.age);
      }
      cloudAt=started;const changed=next.stamp===null||next.stamp!==cloudStamp||next.frame!==cloud.frame||next.source!==cloud.source;cloud=next;cloudStamp=cloud.stamp;
      if(changed){const count=cloud.points.length;
        if(count>pointBuffer.count){geometry.dispose();geometry=new THREE.BufferGeometry();pointBuffer=new THREE.Float32BufferAttribute(new Float32Array(2**Math.ceil(Math.log2(count))*3),3);pointBuffer.setUsage(THREE.DynamicDrawUsage);geometry.setAttribute('position',pointBuffer);points.geometry=geometry;}
        for(let i=0;i<count;i++)pointBuffer.setXYZ(i,...cloud.points[i]);pointBuffer.needsUpdate=true;geometry.setDrawRange(0,count);
      }
    }
  }catch(error){if(requested===cloudSource){cloud.connected=false;cloud.error=error.message||'点云连接中断';}}labels();setTimeout(refreshCloud,Math.max(0,(cloudSource==='live'?100:1000)-(performance.now()-started)));
}
async function refreshPose(){
  const started=performance.now();
  try{const r=await fetch('/api/trajectory',{signal:AbortSignal.timeout(800),cache:'no-store'});if(!r.ok)throw Error();trajectory=await r.json();poseAt=started;
    if(trajectory.stamp!==poseStamp||trajectory.epoch!==poseEpoch){poseStamp=trajectory.stamp;poseEpoch=trajectory.epoch;updateTrail();}
  }catch{trajectory.connected=false;}labels();setTimeout(refreshPose,Math.max(0,100-(performance.now()-started)));
}
render();refreshCloud();refreshPose();
