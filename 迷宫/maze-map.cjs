'use strict';
const {randomUUID}=require('node:crypto');

const CELL=0.10, TILE=32;
const {baseExtensionM:BASE_EXTENSION}=require('./calibration.json');
if(!Number.isFinite(BASE_EXTENSION)||BASE_EXTENSION<.02||BASE_EXTENSION>.60)
  throw Error('桩桶单侧外扩必须在 0.02–0.60 m 范围内');
const HALF_WIDTH=0.275+0.015, HALF_LENGTH=0.325+0.01;
const key=(x,y)=>x+','+y;
const index=v=>Math.floor(v/CELL);
const center=i=>(i+0.5)*CELL;
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const directions=[[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]];
const radius=Math.ceil((Math.hypot(HALF_WIDTH,HALF_LENGTH)+BASE_EXTENSION+CELL*Math.SQRT2)/CELL);
const neighbors=[];
for(let y=-radius;y<=radius;y++)for(let x=-radius;x<=radius;x++){
  const d=Math.hypot(x,y);
  if(d<=radius)neighbors.push({x,y,cm:Math.round(d*CELL*100)});
}
neighbors.sort((a,b)=>a.cm-b.cm);

class MazeMap {
  constructor(){this.reset();}
  reset(){
    this.id=randomUUID();this.generation='';this.frame='';this.tiles=new Map();
    this.version=0;this.pose=null;this.origin=null;this.stamp=0;this.at=0;this.coverage=null;
    this.trail=[];this.visits=new Map();this.lastVisit='';this.hint=null;this.exit=null;
    this.blocked=new Map();this.leg=null;this.route=[];this.note='等待配准雷达与定位';
    this.evidence=new Map();this.retired=0;this.nearReturns=[];
  }
  tile(ix,iy,create=false){
    const tx=Math.floor(ix/TILE),ty=Math.floor(iy/TILE),id=key(tx,ty);
    let t=this.tiles.get(id);
    if(!t&&create){
      t={tx,ty,revision:0,data:Buffer.alloc(TILE*TILE),distance:new Uint8Array(TILE*TILE).fill(255)};
      this.tiles.set(id,t);
    }
    return t;
  }
  offset(x,y){return (x&(TILE-1))+(y&(TILE-1))*TILE;}
  value(x,y){return this.tile(x,y)?.data[this.offset(x,y)]||0;}
  clearance(x,y){return (this.tile(x,y)?.distance[this.offset(x,y)]??255)/100;}
  mark(x,y,state){
    const tile=this.tile(x,y,true),i=this.offset(x,y);
    if(state===2){
      const id=key(x,y),seen=this.evidence.get(id);
      if(!seen)this.evidence.set(id,{hit:this.version,hits:1,first:this.stamp,last:this.stamp,miss:0,lastMiss:-1,levels:new Map()});
      else if(seen.hit!==this.version){seen.hit=this.version;seen.hits++;seen.last=this.stamp;seen.miss=0;}
    }
    if(tile.data[i]>=state)return;
    tile.data[i]=state;tile.revision=this.version;
    if(state!==2)return;
    this.spread(x,y);
  }
  spread(x,y){
    for(const {x:dx,y:dy,cm} of neighbors){
      const near=this.tile(x+dx,y+dy,true),j=this.offset(x+dx,y+dy);
      if(cm<near.distance[j]){near.distance[j]=cm;if(cm<=BASE_EXTENSION*100)near.revision=this.version;}
    }
  }
  observeFree(x,y){
    const p=this.evidence.get(key(x,y));
    if(!p||p.hit===this.version||p.lastMiss===this.version)return;
    p.lastMiss=this.version;
    p.miss++;
  }
  forget(x,y){
    if(this.retired>=4)return;
    this.retired++;this.evidence.delete(key(x,y));
    const t=this.tile(x,y);t.data[this.offset(x,y)]=1;t.revision=this.version;
    for(const d of neighbors){
      const i=x+d.x,j=y+d.y,near=this.tile(i,j),offset=this.offset(i,j);
      if(!near||near.distance[offset]!==d.cm)continue;
      const next=neighbors.find(o=>this.value(i+o.x,j+o.y)===2)?.cm??255;
      near.distance[offset]=next;
      if(d.cm<=BASE_EXTENSION*100&&next>BASE_EXTENSION*100)near.revision=this.version;
    }
  }
  heightRay(px,py,z,dx,dy,dz){
    const range=Math.hypot(dx,dy);if(range<.7)return;
    const ux=dx/range,uy=dy/range;
    for(let d=.35;d<range-.22;d+=CELL*.75){
      const x=index(px+d*ux),y=index(py+d*uy),p=this.evidence.get(key(x,y));
      if(!p||!p.levels.size||p.lastMiss!==this.version||p.hit===this.version)continue;
      const vx=center(x)-px,vy=center(y)-py,along=vx*ux+vy*uy;
      if(Math.abs(vx*uy-vy*ux)>.055||along>=range-.22)continue;
      const realHeight=z+dz*along/range;
      for(const level of p.levels.values())if(level.frame!==this.version&&Math.abs(realHeight-level.z)<=.055){
        level.frame=this.version;level.misses++;
      }
      const sustained=p.hits>=4&&p.last-p.first>=1.2,needed=p.hits===1?2:sustained?12:6;
      if(p.miss>=needed&&this.stamp-p.last>=(p.hits===1?.15:sustained?2.4:1.2)&&
         [...p.levels.values()].every(level=>level.misses>=needed))this.forget(x,y);
    }
  }
  ray(x,y,ex,ey){
    const dx=Math.abs(ex-x),dy=-Math.abs(ey-y),sx=x<ex?1:-1,sy=y<ey?1:-1;
    let err=dx+dy;
    while(true){
      if(this.value(x,y)===2){
        if(this.evidence.get(key(x,y)).hit===this.version)break;
        if(Math.hypot(ex-x,ey-y)*CELL>.18)this.observeFree(x,y);
      }else this.mark(x,y,1);
      if(x===ex&&y===ey)break;
      const twice=2*err;
      if(twice>=dy){err+=dy;x+=sx;}
      if(twice<=dx){err+=dx;y+=sy;}
    }
  }
  accept(scan,now=Date.now()){
    if(!scan||typeof scan!=='object')return false;
    const {generation,frame,pose,marks,rays,stamp}=scan;
    if(typeof generation!=='string'||!/^[A-Za-z0-9_-]{1,64}$/.test(generation)||frame!=='camera_init'||!Number.isFinite(stamp)||
       !Array.isArray(pose)||pose.length!==3||!pose.every(Number.isFinite))return false;
    const heights=scan.heightHits||[];
    const rayOrigin=scan.rayOrigin||pose.slice(0,2);
    if(!Array.isArray(rayOrigin)||rayOrigin.length!==2||!rayOrigin.every(Number.isFinite)||distance(rayOrigin,pose)>2)return false;
    const [px,py,yaw]=pose,c=Math.cos(yaw),s=Math.sin(yaw),ix=index(px),iy=index(py);
    const fromLidar=scan.raysFromLidar===true;
    const dx=rayOrigin[0]-px,dy=rayOrigin[1]-py;
    const rx=fromLidar?c*dx+s*dy:0,ry=fromLidar?-s*dx+c*dy:0;
    const valid=(points,x,y)=>Array.isArray(points)&&points.length<=15000&&points.every(p=>
      Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)&&Math.hypot(p[0]-x,p[1]-y)<=6.01);
    if(!valid(marks,0,0)||!valid(rays,rx,ry))return false;
    if(!Array.isArray(heights)||heights.length>15000||heights.length&&(!Number.isFinite(scan.sensorZ)||
       !heights.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)&&Math.hypot(p[0],p[1])<=6.01)))return false;
    if(this.generation!==generation)this.reset();
    if(stamp<=this.stamp)return false;
    this.generation=generation;this.frame=frame;this.stamp=stamp;this.at=now;this.version++;
    this.coverage=scan.coverage||null;this.retired=0;
    this.pose=pose.slice();this.origin??=pose.slice();
    const project=([x,y])=>[index(px+c*x-s*y),index(py+s*x+c*y)];
    this.nearReturns=[];
    const displayStride=Math.max(1,Math.ceil(heights.length/768));
    for(let i=this.version%displayStride;i<heights.length;i+=displayStride){
      const [x,y,z]=heights[i];this.nearReturns.push([rayOrigin[0]+c*x-s*y,rayOrigin[1]+s*x+c*y,scan.sensorZ+z]);
    }
    // Full calibrated frames supply the same obstacles in gravity-aligned
    // coordinates below. Marking raw XY too can leave a second, heightless
    // occupied cell when the chassis tilts or the sensor origin is offset.
    const calibrated=scan.coverage==='full'&&heights.length>0&&
      Array.isArray(scan.lidarOffset)&&scan.lidarOffset.length===3&&scan.lidarOffset.every(Number.isFinite);
    if(!calibrated)for(const p of marks)this.mark(...project(p),2);
    for(const p of heights){
      const x=index(rayOrigin[0]+c*p[0]-s*p[1]),y=index(rayOrigin[1]+s*p[0]+c*p[1]);this.mark(x,y,2);
      const h=scan.sensorZ+p[2],levels=this.evidence.get(key(x,y)).levels;
      levels.set(Math.floor(h/.08),{z:h,misses:0,frame:this.version});
    }
    const sx=fromLidar?index(rayOrigin[0]):ix,sy=fromLidar?index(rayOrigin[1]):iy;
    for(const p of rays)this.ray(sx,sy,...project(p));
    const stride=Math.max(1,Math.ceil(heights.length/320));
    for(let i=this.version%stride;i<heights.length;i+=stride){
      const [x,y,dz]=heights[i];this.heightRay(rayOrigin[0],rayOrigin[1],scan.sensorZ,c*x-s*y,s*x+c*y,dz);
    }
    this.mark(ix,iy,1);
    if(!this.trail.length||distance(this.trail.at(-1),pose)>.18)this.trail.push(pose.slice(0,2));
    const visit=this.visitKey(px,py);
    if(visit!==this.lastVisit){this.visits.set(visit,1+(this.visits.get(visit)||0));this.lastVisit=visit;}
    return true;
  }
  visitKey(x,y){return key(Math.floor(x/.65),Math.floor(y/.65));}
  goals({kind,x,y,id}){
    if(!this.origin||id!==this.id)throw Error('地图已更新，请在当前地图重新指定');
    if(!Number.isFinite(x)||!Number.isFinite(y)||Math.max(Math.abs(x),Math.abs(y))>10000)throw Error('请输入有效的米制坐标');
    if(kind==='hint'){
      const [px,py,a]=this.origin;
      this.hint=[px+x*Math.cos(a)-y*Math.sin(a),py+x*Math.sin(a)+y*Math.cos(a)];
    }else if(kind==='exit'){
      if(!this.walkable(index(x),index(y)))throw Error('请在已观测且有车身净距的出口内侧选点');
      this.exit=[x,y];
    }else throw Error('未知目标类型');
    this.blocked.clear();
  }
  walkable(x,y){return this.value(x,y)===1&&this.clearance(x,y)>HALF_WIDTH+BASE_EXTENSION+CELL*Math.SQRT1_2;}
  footprintClear(x,y,a){
    const c=Math.cos(a),s=Math.sin(a),pad=BASE_EXTENSION+CELL*Math.SQRT1_2;
    const range=Math.ceil((Math.hypot(HALF_WIDTH,HALF_LENGTH)+pad)/CELL);
    const ix=index(x),iy=index(y);
    if(this.clearance(ix,iy)>Math.hypot(HALF_WIDTH,HALF_LENGTH)+pad+CELL*Math.SQRT1_2)return true;
    for(let j=-range;j<=range;j++)for(let i=-range;i<=range;i++){
      if(this.value(ix+i,iy+j)!==2)continue;
      const dx=center(ix+i)-x,dy=center(iy+j)-y;
      const u=Math.max(0,Math.abs(c*dx+s*dy)-HALF_LENGTH);
      const v=Math.max(0,Math.abs(-s*dx+c*dy)-HALF_WIDTH);
      if(Math.hypot(u,v)<=pad)return false;
    }
    return true;
  }
  covered(x,y,yaw){
    const c=Math.cos(yaw),s=Math.sin(yaw),[px,py,pa]=this.pose,pc=Math.cos(pa),ps=Math.sin(pa);
    const stepsX=Math.ceil(2*HALF_LENGTH/CELL),stepsY=Math.ceil(2*HALF_WIDTH/CELL);
    for(let j=0;j<=stepsY;j++)for(let i=0;i<=stepsX;i++){
      const u=(2*i/stepsX-1)*HALF_LENGTH,v=(2*j/stepsY-1)*HALF_WIDTH;
      const wx=x+c*u-s*v,wy=y+s*u+c*v,dx=wx-px,dy=wy-py;
      if(Math.abs(pc*dx+ps*dy)<=HALF_LENGTH&&Math.abs(-ps*dx+pc*dy)<=HALF_WIDTH)continue;
      if(this.value(index(wx),index(wy))!==1)return false;
    }
    return true;
  }
  segmentClear(goal){
    const [x,y,yaw]=this.pose,dx=goal[0]-x,dy=goal[1]-y,len=Math.hypot(dx,dy);
    const a=Math.atan2(dy,dx),turn=Math.atan2(Math.sin(a-yaw),Math.cos(a-yaw));
    const steps=Math.ceil(Math.abs(turn)/.14);
    for(let i=0;i<=steps;i++){
      const angle=yaw+turn*i/Math.max(steps,1);
      if(!this.footprintClear(x,y,angle)||!this.covered(x,y,angle))return false;
    }
    const n=Math.ceil(len/(CELL*.7));
    for(let i=1;i<=n;i++){
      const px=x+dx*i/n,py=y+dy*i/n;
      if(!this.footprintClear(px,py,a)||!this.covered(px,py,a))return false;
    }
    return true;
  }
  brakingClear(v,w){
    if(!this.pose||![v,w].every(Number.isFinite))return false;
    let [x,y,a]=this.pose;
    // Include telemetry/command delay and measured deceleration limits.
    for(let t=0;t<4;t+=.08){
      if(!this.footprintClear(x,y,a)||!this.covered(x,y,a))return false;
      if(Math.abs(v)+Math.abs(w)<.001)return true;
      x+=v*Math.cos(a+w*.04)*.08;y+=v*Math.sin(a+w*.04)*.08;a+=w*.08;
      if(t>=.45){v=Math.sign(v)*Math.max(0,Math.abs(v)-.20*.08);w=Math.sign(w)*Math.max(0,Math.abs(w)-.60*.08);}
    }
    return false;
  }
  avoid(point,now){if(point)this.blocked.set(this.visitKey(...point),now+30000);}
  *search(now=Date.now()){
    this.leg=null;this.route=[];
    if(!this.pose)return null;
    if(!this.footprintClear(...this.pose)){
      this.note='车身周围的回波进入估计底座范围，正在复核净距';
      return null;
    }
    const start=[index(this.pose[0]),index(this.pose[1])];
    const target=this.exit||this.hint;
    const queue=[{x:start[0],y:start[1],parent:-1,d:0}],seen=new Set([key(...start)]);
    const frontiers=new Map(),viewpoints=new Map();let finalIndex=-1;
    const startBucket=this.visitKey(...this.pose);
    const remember=(table,bucket,item)=>{
      if(!table.has(bucket)||item.score>table.get(bucket).score)table.set(bucket,item);
    };
    for(let i=0;i<queue.length;i++){
      if(i&&i%256===0)yield;
      const {x,y,d}=queue[i],p=[center(x),center(y)],k=this.visitKey(...p);
      if(i&&this.exit&&x===index(this.exit[0])&&y===index(this.exit[1])){finalIndex=i;break;}
      if(i&&d>=.55&&(this.blocked.get(k)||0)<=now&&
          directions.some(([dx,dy])=>!this.value(x+3*dx,y+3*dy))){
        const bias=target?(distance(this.pose,target)-distance(p,target))*.75:0;
        remember(frontiers,k,{i,score:bias+Math.min(d,2.4)-.35*d-3*(this.visits.get(k)||0)});
      }else if(d>=.8&&d<=6&&!this.visits.has(k)&&(this.blocked.get(k)||0)<=now){
        remember(viewpoints,k,{i,score:-d});
      }
      for(const [dx,dy] of directions){
        const nx=x+dx,ny=y+dy,id=key(nx,ny);
        if(seen.has(id)||!this.walkable(nx,ny))continue;
        if(dx&&dy&&(!this.walkable(x+dx,y)||!this.walkable(x,y+dy)))continue;
        const bucket=this.visitKey(center(nx),center(ny));
        if(bucket!==startBucket&&(this.blocked.get(bucket)||0)>now)continue;
        seen.add(id);queue.push({x:nx,y:ny,parent:i,d:d+CELL*(dx&&dy?Math.SQRT2:1)});
      }
    }
    const best=table=>[...table.values()].sort((a,b)=>b.score-a.score);
    const choices=best(frontiers).concat(best(viewpoints));
    if(finalIndex>=0)choices.unshift({i:finalIndex});
    // Try distinct directions even when the highest-scoring frontier is obstructed nearby.
    const tried=new Set();
    for(const choice of choices){
      yield;
      const end=queue[choice.i],bucket=this.visitKey(center(end.x),center(end.y));
      if(tried.has(bucket))continue;
      tried.add(bucket);
      const path=[];
      for(let i=choice.i;i>0;i=queue[i].parent)path.push([center(queue[i].x),center(queue[i].y)]);
      path.reverse();
      if(choice.i===finalIndex)path[path.length-1]=this.exit.slice();
      for(let j=Math.min(path.length-1,20);j>=0;j--){
        const p=path[j],len=distance(p,this.pose);
        if(len>2||len<(choice.i===finalIndex?.18:.44)||!this.segmentClear(p))continue;
        this.leg=p;this.route=[this.pose.slice(0,2),...path];
        this.note=choice.i===finalIndex?'向指定出口行进':(this.visits.get(this.visitKey(...p))||0)>0?'沿已知通道返回岔口':frontiers.has(bucket)?'探索已观测区域的边缘':'前往尚未到访的位置补充观测';
        return p;
      }
    }
    this.note='等待新的通道观测，持续重选分支';
    return null;
  }
  snapshot(id,since){
    const after=id===this.id?since:-1,tiles=[];
    for(const t of this.tiles.values())if(t.revision>0&&t.revision>after){
      const bytes=Buffer.alloc(TILE*TILE/4);
      for(let j=0;j<t.data.length;j++){
        const v=t.data[j]!==2&&t.distance[j]<=BASE_EXTENSION*100?3:t.data[j];
        bytes[j>>2]|=v<<((j&3)*2);
      }
      tiles.push([t.tx,t.ty,bytes.toString('base64')]);
    }
    return {id:this.id,generation:this.generation,revision:this.version,cell:CELL,tile:TILE,baseExtension:BASE_EXTENSION,
      ageMs:this.at?Date.now()-this.at:null,pose:this.pose,origin:this.origin,hint:this.hint,exit:this.exit,
      coverage:this.coverage,departureClear:this.pose?this.footprintClear(...this.pose):null,
      trail:this.trail,route:this.route,leg:this.leg,note:this.note,nearReturns:this.nearReturns,tiles};
  }
}
module.exports={MazeMap,CELL,BASE_EXTENSION,distance};
