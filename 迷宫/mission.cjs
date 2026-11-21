'use strict';
const {randomUUID}=require('node:crypto');
const {distance,BASE_EXTENSION}=require('./maze-map.cjs');
const {roam,roamCandidate,wrap}=require('./roam.cjs');
function advance(search,budget=10){
  const deadline=performance.now()+budget;
  let step;
  do{step=search.next();}while(!step.done&&performance.now()<deadline);
  return step;
}

class MazeMission {
  constructor(map,send){this.map=map;this.send=send;this.run=null;}
  start(task,ownerAlive,now=Date.now()){
    const mode=task.options?.mode||'explore';
    if(!['explore','roam'].includes(mode))throw Error('未知迷宫策略');
    this.run={task,ownerAlive,mapId:this.map.id,paused:false,leg:null,needsStop:false,
      mode,heading:null,stopAt:0,nextPlan:now};
    this.map.leg=null;this.map.route=[];
    this.say('正在选择通往出口或未知区域的通道');
  }
  say(note){this.map.note=note;if(this.run)this.run.task.log=note;}
  end(phase,note){
    if(this.run)this.run.task.phase=phase;
    this.say(note);this.run=null;this.map.leg=null;this.map.route=[];
  }
  stopLeg(now,avoid=false){
    const r=this.run;
    if(!r)return;
    if(avoid)this.map.avoid(r.leg?.goal,now);
    this.send('stop');r.needsStop=true;r.stopAt=now;r.leg=null;r.search=null;r.nextPlan=now+250;
    this.map.leg=null;this.map.route=[];
  }
  pause(now=Date.now()){
    if(!this.run||this.run.paused)return;
    this.run.paused=true;this.stopLeg(now);this.say('已暂停探索');
  }
  resume(){if(this.run){this.run.paused=false;this.say('正在复核通道');}}
  retarget(now=Date.now()){if(this.run){this.stopLeg(now);this.say('目标已更新，正在重新选路');}}
  fail(reason){this.send('stop');this.end('failed',reason);}
  tick(ctx,now=Date.now()){
    const r=this.run,m=this.map,s=ctx.status;
    if(!r)return;
    if(r.task.phase!=='running'||!r.ownerAlive()){this.send('stop');this.end('stopped','控制已结束');return;}
    if(r.mapId!==m.id || !ctx.healthy){this.fail('定位、雷达、规划器或底盘反馈已失效，请重新准备迷宫');return;}
    if(r.needsStop){
      if(ctx.statusAt>r.stopAt&&ctx.stopped&&s.stage==='FAULT'&&s.reason==='Operator stop')r.needsStop=false;
      else if(now-r.stopAt>3000)this.fail('底盘未确认停车');
      return;
    }
    if(r.paused)return;
    if(s.stage==='FAULT'&&s.reason!=='Operator stop'){this.fail('实车故障停车 · '+s.reason);return;}
    if(r.leg){
      const leg=r.leg;
      if(s.maze_leg!==leg.id || ctx.statusAt<=leg.sent){
        if(leg.previous&&s.maze_leg===leg.previous&&ctx.statusAt>leg.sent+700){
          this.stopLeg(now);this.say('前视点未续接，停稳后重新选择通道');return;
        }
        if(now-leg.sent>2500)this.fail('实车未确认当前迷宫子目标');
        return;
      }
      if(s.stage==='FAULT'){this.fail('实车结束了本段导航 · '+s.reason);return;}
      if(s.stage==='DONE'){
        if(!ctx.stopped)return;
        r.leg=null;m.leg=null;m.route=[];r.nextPlan=now;
        if(m.exit&&distance(m.pose,m.exit)<=.22){this.end('completed','已到达指定出口区并确认停稳');return;}
      }else{
        const left=distance(m.pose,leg.goal);
        if(left<leg.best-.06){leg.best=left;leg.progressAt=now;}
        if(!m.brakingClear(s.feedback_v,s.feedback_w)||
           !m.brakingClear(s.v,s.w)||now-leg.progressAt>14000){
          this.stopLeg(now,true);this.say('本段净距不足或无进展，停稳后重选分支');
        }else if(r.mode==='roam'&&!m.exit&&s.stage==='DRIVING'&&s.v>.03&&left>=.5&&left<=.95&&now>=(leg.nextExtend||0)){
          leg.extension??=roamCandidate(m,now,r.heading,true);
          const step=advance(leg.extension,6);
          if(!step.done)return;
          leg.extension=null;leg.nextExtend=now+450;
          const next=step.value,oldAngle=Math.atan2(leg.goal[1]-m.pose[1],leg.goal[0]-m.pose[0]);
          if(next&&distance(m.pose,next.goal)>left+.40&&Math.abs(wrap(next.angle-oldAngle))<=.26&&
             m.segmentClear(next.goal)&&r.ownerAlive()){
            const id=randomUUID(),[x,y]=next.goal;
            if(!this.send(['extendmaze',leg.id,id,x.toFixed(3),y.toFixed(3),m.frame,m.generation].join(' '))){
              this.fail('漫游前视点发送失败');return;
            }
            r.heading=next.angle;r.leg={id,previous:leg.id,goal:next.goal,sent:now,progressAt:now,best:distance(m.pose,next.goal)};
            m.leg=next.goal;m.route=[m.pose.slice(0,2),next.goal];this.say('漫游 · 沿已确认通道连续前进');
          }
        }
        return;
      }
    }
    if(!ctx.stopped||now<r.nextPlan)return;
    if(m.exit&&distance(m.pose,m.exit)<=.20){this.end('completed','已到达指定出口区并确认停稳');return;}
    r.search??=r.mode==='roam'?roam(m,now,r.heading):m.search(now);
    const step=advance(r.search);
    if(!step.done)return;
    r.search=null;r.nextPlan=now+1200;
    const goal=step.value;this.say(m.note);
    if(!goal||!r.ownerAlive())return;
    const id=randomUUID();
    if(!this.send(['startmaze',goal[0].toFixed(3),goal[1].toFixed(3),m.frame,m.generation,id,BASE_EXTENSION].join(' '))){
      this.fail('发送迷宫子目标失败');return;
    }
    r.heading=Math.atan2(goal[1]-m.pose[1],goal[0]-m.pose[0]);
    r.leg={id,goal,sent:now,progressAt:now,best:distance(m.pose,goal)};
  }
}
module.exports={MazeMission};
