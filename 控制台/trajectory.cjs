// Real odometry only. Discontinuities start a new segment; no invented interpolation.
class Trajectory {
  constructor(){this.clear();}
  clear(){this.id=require('node:crypto').randomUUID();this.end=0;this.points=[];this.positions=[];this.samples=[];this.pose=null;this.position=null;this.quaternion=null;this.frame='';this.stamp=null;this.at=0;this.epoch=(this.epoch||0)+1;}
  add(value,now=Date.now()){
    const p=value.pose,s=value.stamp;
    if(!Array.isArray(p)||p.length!==3||!p.every(Number.isFinite)||!Number.isFinite(s))return;
    const frame=typeof value.frame==='string'?value.frame.replace(/^\//,''):'';
    if(this.stamp===s&&this.frame===frame)return;
    if(this.pose&&(frame!==this.frame||s<this.stamp||Math.hypot(p[0]-this.pose[0],p[1]-this.pose[1])>0.5))this.clear();
    this.pose=p.slice();this.stamp=s;this.at=now;
    this.frame=frame;
    const measuredPosition=Array.isArray(value.position)&&value.position.length===3&&value.position.every(Number.isFinite);
    this.position=measuredPosition?value.position.slice():[p[0],p[1],0];
    const q=value.quaternion;
    const measuredRotation=Array.isArray(q)&&q.length===4&&q.every(Number.isFinite)&&Math.abs(Math.hypot(...q)-1)<.05;
    this.quaternion=measuredRotation?q.slice():[0,0,Math.sin(p[2]/2),Math.cos(p[2]/2)];
    if(measuredPosition&&measuredRotation){
      this.samples.push({stamp:s,p:this.position,q:this.quaternion});
      while(this.samples.length>32||this.samples[0].stamp<s-1)this.samples.shift();
    }
    const last=this.points.at(-1);
    if(!last||Math.hypot(p[0]-last[0],p[1]-last[1])>=0.02){this.end++;this.points.push(p.slice());this.positions.push(this.position.slice());if(this.points.length>2000){this.points.shift();this.positions.shift();}}
  }
  lidarAt(stamp,offset){
    if(!offset||!this.samples.length)return null;
    const index=this.samples.findIndex(sample=>sample.stamp>=stamp);
    const b=this.samples[index],a=index<0?this.samples.at(-1):this.samples[index-1];
    let sample;
    if(b&&Math.abs(b.stamp-stamp)<.025)sample=b;
    else if(a&&Math.abs(a.stamp-stamp)<.025)sample=a;
    else if(a&&b&&b.stamp-a.stamp<=.16){
      const f=(stamp-a.stamp)/(b.stamp-a.stamp);
      const sign=a.q.reduce((sum,v,i)=>sum+v*b.q[i],0)<0?-1:1;
      const q=a.q.map((v,i)=>v+f*(sign*b.q[i]-v)),norm=Math.hypot(...q);
      sample={p:a.p.map((v,i)=>v+f*(b.p[i]-v)),q:q.map(v=>v/norm)};
    }
    if(!sample)return null;
    const [x,y,z,w]=sample.q,[px,py,pz]=sample.p;
    const xx=1-2*(y*y+z*z),xy=2*(x*y-w*z),xz=2*(x*z+w*y);
    const yx=2*(x*y+w*z),yy=1-2*(x*x+z*z),yz=2*(y*z-w*x);
    const zx=2*(x*z-w*y),zy=2*(y*z+w*x),zz=1-2*(x*x+y*y);
    const [dx,dy,dz]=offset;
    return {
      origin:[px+xx*dx+xy*dy+xz*dz,py+yx*dx+yy*dy+yz*dz,pz+zx*dx+zy*dy+zz*dz],
      self(point){
        const u=point[0]-px,v=point[1]-py,h=point[2]-pz;
        // The course SCOUT MINI is 650 x 550 mm. Mask only its physical
        // projection below the centred scanner, never a disk around the car.
        return Math.abs(xx*u+yx*v+zx*h)<=.325&&Math.abs(xy*u+yy*v+zy*h)<=.275&&xz*u+yz*v+zz*h<=dz;
      }
    };
  }
  snapshot(){return {points:this.points,positions:this.positions,pose:this.pose,position:this.position,quaternion:this.quaternion,frame:this.frame,stamp:this.stamp,epoch:this.epoch,age:this.at?(Date.now()-this.at)/1000:null};}
  delta(id,cursor){
    const first=this.end-this.positions.length;
    const reset=id!==this.id||!Number.isSafeInteger(cursor)||cursor<first||cursor>this.end;
    const offset=reset?0:cursor-first;
    return {...this.snapshot(),id:this.id,end:this.end,reset,points:this.points.slice(offset),positions:this.positions.slice(offset)};
  }
}
module.exports={Trajectory};
