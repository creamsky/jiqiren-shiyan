'use strict';
const {randomUUID}=require('node:crypto');

// Confirm voxel returns across distinct scans. Remove an old return only when
// real rays pass through its former 3D position and terminate farther away.
class RegisteredMap {
  constructor(limit=65536){this.limit=limit;this.reset();}
  reset(){
    this.id=randomUUID();this.frame='';this.generation='';this.stamp=0;this.at=0;
    this.cell=.10;this.verticalCell=.04;this.points=[];this.voxels=new Map();this.pending=new Set();this.removed=[];this.scans=0;
  }
  key(x,y,z){return Math.floor(x/this.cell)+','+Math.floor(y/this.cell)+','+Math.floor(z/this.verticalCell);}
  hit(p){
    const k=this.key(...p);let v=this.voxels.get(k);
    if(!v){v={p,slot:-1,first:this.stamp,seen:0,n:0,last:0,clear:0,clearScan:0};this.voxels.set(k,v);this.pending.add(k);}
    if(v.seen===this.scans)return;
    v.seen=this.scans;v.n++;v.last=this.stamp;v.clear=0;
    if(v.slot<0&&v.n>=3&&this.stamp-v.first>=.30){
      v.slot=this.points.length;this.points.push(v.p);this.pending.delete(k);
    }
  }
  cast(origin,p){
    const dx=p[0]-origin[0],dy=p[1]-origin[1],dz=p[2]-origin[2],len=Math.hypot(dx,dy,dz);
    if(len<.7)return;
    const ux=dx/len,uy=dy/len,uz=dz/len,tolerance=Math.min(.07,this.cell*.48);
    const step=.8*Math.min(this.cell/Math.max(Math.abs(ux),Math.abs(uy)),this.verticalCell/Math.abs(uz));
    for(let d=.4;d<Math.min(10,len-.25);d+=step){
      const k=this.key(origin[0]+d*ux,origin[1]+d*uy,origin[2]+d*uz),v=this.voxels.get(k);
      if(!v||v.seen===this.scans||v.clearScan===this.scans)continue;
      const x=v.p[0]-origin[0],y=v.p[1]-origin[1],z=v.p[2]-origin[2],along=x*ux+y*uy+z*uz;
      if(along>=len-.25||Math.hypot(x-along*ux,y-along*uy,z-along*uz)>tolerance)continue;
      v.clearScan=this.scans;
      if(++v.clear<8||this.stamp-v.last<1.4)continue;
      if(v.slot>=0){this.points[v.slot]=null;this.removed.push(v.slot);}
      this.voxels.delete(k);this.pending.delete(k);
    }
  }
  compact(grow){
    if(grow)this.cell*=2;
    const old=this.points;this.points=[];this.voxels.clear();this.pending.clear();this.removed=[];this.id=randomUUID();
    for(const p of old)if(p){
      const k=this.key(...p);
      if(this.voxels.has(k))continue;
      this.voxels.set(k,{p,slot:this.points.length,n:3,last:this.stamp,seen:this.scans,clear:0,clearScan:0});
      this.points.push(p);
    }
  }
  accept(cloud,trajectory,generation,now=Date.now(),lidarOffset=null){
    if(!cloud.registered||cloud.frame!=='camera_init'||trajectory.frame!==cloud.frame||!trajectory.pose||
       typeof generation!=='string'||!generation||!Number.isFinite(cloud.stamp)||!Number.isFinite(trajectory.stamp)||
       Math.abs(cloud.stamp-trajectory.stamp)>.6||now-trajectory.at>=700||
       (/^\d{18,19}$/.test(generation)&&cloud.stamp<Number(generation)*1e-9)||!cloud.points.length)return false;
    const sensor=trajectory.lidarAt?.(cloud.stamp,lidarOffset);if(!sensor)return false;
    if(this.generation&&this.generation!==generation)this.reset();
    if(cloud.stamp<=this.stamp)return false;
    this.frame=cloud.frame;this.generation=generation;this.stamp=cloud.stamp;this.at=now;this.scans++;
    for(const p of cloud.points)if(!sensor.self(p))this.hit(p);
    const stride=Math.max(1,Math.ceil(cloud.points.length/160));
    for(let i=this.scans%stride;i<cloud.points.length;i+=stride){
      const p=cloud.points[i];if(!sensor.self(p))this.cast(sensor.origin,p);
    }
    if(this.scans%10===0)for(const k of this.pending){
      if(this.stamp-this.voxels.get(k).last>2.5){this.pending.delete(k);this.voxels.delete(k);}
    }
    const used=this.points.length-this.removed.length;
    if(used>this.limit||this.removed.length>1024&&this.removed.length>used/3)this.compact(used>this.limit);
    return true;
  }
  chunk(id,cursor,now=Date.now(),deleteCursor=0){
    const count=this.points.length,match=id===this.id;
    const from=match&&Number.isSafeInteger(cursor)&&cursor>=0&&cursor<=count?cursor:0;
    const startDelete=match&&Number.isSafeInteger(deleteCursor)&&deleteCursor>=0&&deleteCursor<=this.removed.length?deleteCursor:0;
    const removed=this.removed.slice(startDelete,startDelete+4096);
    // A new tab catches up in one bounded snapshot. Regular one-second polls use deltas.
    const end=Math.min(from+(match?4096:65536),count),bytes=Buffer.allocUnsafe((end-from)*12);
    for(let i=from;i<end;i++)for(let j=0;j<3;j++)bytes.writeFloatLE(this.points[i]?.[j]??NaN,(i-from)*12+j*4);
    return {id:this.id,from,end,count,active:count-this.removed.length,removed,removedEnd:startDelete+removed.length,removedCount:this.removed.length,
      xyz:bytes.toString('base64'),source:'map',registered:true,age:this.at?(now-this.at)/1000:null,
      stamp:this.stamp||null,frame:this.frame,generation:this.generation,cell:this.cell,verticalCell:this.verticalCell,scans:this.scans};
  }
}
module.exports={RegisteredMap};
