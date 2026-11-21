'use strict';
const {randomUUID}=require('node:crypto');
const {distance,BASE_EXTENSION}=require('./maze-map.cjs');
const {roam,roamCandidate,wrap}=require('./roam.cjs');
const recoveredFaults=new Set(['Chassis fault or control-mode change','CAN feedback timeout','Lidar timeout',
  'Odometry timeout','Odometry frame changed','Odometry time reset','Odometry jump','Maze map generation changed']);
const sensorFaults=new Set(['Lidar timeout','Odometry timeout']);
const dataSources={lidar:'雷达',odometry:'定位',perception:'雷达与定位配对',planner:'路径规划'};
const recoveryMotion=new Set(['route_retreat','retreat_replan','forward_escape','following_detour','pivot_to_path']);
function advance(search,budget=10){
  const deadline=performance.now()+budget;
  let step;
  do{step=search.next();}while(!step.done&&performance.now()<deadline);
  return step;
}

class MazeMission {
  constructor(map,send){this.map=map;this.send=send;this.run=null;}
  start(task,ownerAlive,now=Date.now(),previousFault=''){
    const mode=task.options?.mode||'explore';
    if(!['explore','roam'].includes(mode))throw Error('未知迷宫策略');
    this.run={task,ownerAlive,mapId:this.map.id,paused:false,leg:null,needsStop:false,
      mode,heading:null,previousFault:recoveredFaults.has(previousFault)?previousFault:null,stopAt:0,nextPlan:now};
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
    if(r.mapId!==m.id){this.fail('地图已更新，请重新准备迷宫');return;}
    const source=dataSources[ctx.unavailable];
    const transient=Boolean(source);
    if(!ctx.healthy&&!transient){this.fail('底盘反馈或控制条件已失效，请重新准备迷宫');return;}
    const sensorStop=r.leg&&s.maze_leg===r.leg.id&&s.stage==='FAULT'&&sensorFaults.has(s.reason);
    if((transient||sensorStop)&&!r.recovery){
      this.stopLeg(now);r.recovery={since:now,stableAt:null};
      const interrupted=sensorStop?(s.reason==='Lidar timeout'?'雷达':'定位'):source;
      this.say(interrupted+'暂时中断，已请求停车，等待本轮数据恢复');
    }
    if(r.recovery){
      if(now-r.recovery.since>12000){this.fail('数据持续中断或底盘未确认停车，请重新准备迷宫');return;}
      if(s.stage==='FAULT'&&s.reason!=='Operator stop'&&!sensorFaults.has(s.reason)&&s.reason!==r.previousFault){
        this.fail('实车故障停车 · '+s.reason);return;
      }
      const terminal=s.stage==='FAULT'||s.stage==='DONE';
      if(!ctx.healthy||!ctx.stopped||ctx.statusAt<=r.stopAt||!s.paused||!terminal){r.recovery.stableAt=null;return;}
      r.recovery.stableAt??=now;
      if(now-r.recovery.stableAt<700)return;
      r.previousFault=s.stage==='FAULT'?s.reason:null;
      r.recovery=null;r.needsStop=false;r.nextPlan=now;
      this.say(r.paused?'数据已恢复，探索仍已暂停':'数据和停车反馈已恢复，重新选择通道');return;
    }
    if(r.needsStop){
      const acknowledged=s.stage==='FAULT'&&s.reason==='Operator stop'||s.stage==='DONE'&&s.paused===true;
      if(ctx.statusAt>r.stopAt&&ctx.stopped&&acknowledged)r.needsStop=false;
      else if(now-r.stopAt>3000)this.fail('底盘未确认停车');
      return;
    }
    if(r.paused)return;
    if(s.stage==='FAULT'&&s.reason!=='Operator stop'&&s.reason!==r.previousFault){this.fail('实车故障停车 · '+s.reason);return;}
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
      r.previousFault=null;
      if(s.stage==='DONE'){
        if(!ctx.stopped)return;
        r.leg=null;m.leg=null;m.route=[];r.nextPlan=now;
        if(m.exit&&distance(m.pose,m.exit)<=.22){this.end('completed','已到达指定出口区并确认停稳');return;}
      }else{
        const left=distance(m.pose,leg.goal);
        if(left<leg.best-.06){leg.best=left;leg.progressAt=now;}
        if(recoveryMotion.has(s.avoidance_state)){
          leg.recoveryPoses??=[];
          const pose=m.pose.slice();
          if((Math.abs(s.feedback_v)>.02||Math.abs(s.feedback_w)>.04)&&
              !leg.recoveryPoses.some(p=>distance(p,pose)<.08&&Math.abs(wrap(p[2]-pose[2]))<.18)){
            leg.recoveryPoses.push(pose);leg.progressAt=now;
          }
        }
        // At rest there is no braking sweep. The next moving command will
        // be checked by both the vehicle and the map before this leg continues.
        const unsafe=(s.feedback_v!==0||s.feedback_w!==0)&&!m.brakingClear(s.feedback_v,s.feedback_w)||
          (s.v!==0||s.w!==0)&&!m.brakingClear(s.v,s.w);
        if(unsafe||now-leg.progressAt>14000){
          this.stopLeg(now,true);this.say(unsafe?'制动路径净距不足，停稳后重选分支':'本段没有实测运动进展，停稳后重选分支');
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
