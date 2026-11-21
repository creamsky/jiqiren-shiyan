// Real odometry only. Discontinuities start a new segment; no invented interpolation.
class Trajectory {
  constructor(){this.clear();}
  clear(){this.points=[];this.positions=[];this.pose=null;this.position=null;this.quaternion=null;this.frame='';this.stamp=null;this.at=0;this.epoch=(this.epoch||0)+1;}
  add(value,now=Date.now()){
    const p=value.pose,s=value.stamp;
    if(!Array.isArray(p)||p.length!==3||!p.every(Number.isFinite)||!Number.isFinite(s))return;
    const frame=typeof value.frame==='string'?value.frame.replace(/^\//,''):'';
    if(this.stamp===s&&this.frame===frame)return;
    if(this.pose&&(frame!==this.frame||s<this.stamp||Math.hypot(p[0]-this.pose[0],p[1]-this.pose[1])>0.5))this.clear();
    this.pose=p.slice();this.stamp=s;this.at=now;
    this.frame=frame;
    this.position=Array.isArray(value.position)&&value.position.length===3&&value.position.every(Number.isFinite)?value.position.slice():[p[0],p[1],0];
    const q=value.quaternion;
    this.quaternion=Array.isArray(q)&&q.length===4&&q.every(Number.isFinite)&&Math.abs(Math.hypot(...q)-1)<0.05?q.slice():[0,0,Math.sin(p[2]/2),Math.cos(p[2]/2)];
    const last=this.points.at(-1);
    if(!last||Math.hypot(p[0]-last[0],p[1]-last[1])>=0.02){this.points.push(p.slice());this.positions.push(this.position.slice());if(this.points.length>2000){this.points.shift();this.positions.shift();}}
  }
  snapshot(){return {points:this.points,positions:this.positions,pose:this.pose,position:this.position,quaternion:this.quaternion,frame:this.frame,stamp:this.stamp,epoch:this.epoch,age:this.at?(Date.now()-this.at)/1000:null};}
}
module.exports={Trajectory};
