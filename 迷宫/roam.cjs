'use strict';
const {CELL,distance}=require('./maze-map.cjs');

const offsets=[0,15,-15,30,-30,50,-50,75,-75,100,-100,130,-130,160,-160,180];
const lengths=[1.7,1.15,.75,.48];
const wrap=a=>Math.atan2(Math.sin(a),Math.cos(a));

// One bounded straight leg at a time. The native tracker chooses the actual arc
// and checks its braking sweep against the newest scan at control frequency.
function* roamCandidate(map,now,heading,straightOnly=false){
  if(!map.pose||!map.footprintClear(...map.pose))return null;
  const [x,y,yaw]=map.pose;
  const candidates=[];
  for(const deg of straightOnly?[0,15,-15]:offsets){
    yield;
    const angle=yaw+deg*Math.PI/180,c=Math.cos(angle),s=Math.sin(angle);
    for(const length of lengths){
      if(straightOnly&&length<1.1)break;
      const goal=[x+length*c,y+length*s],bucket=map.visitKey(...goal);
      if((map.blocked.get(bucket)||0)>now||!map.segmentClear(goal))continue;
      let gap=1,visits=0,blocked=false;
      for(let d=.35;d<=length+.001;d+=.16){
        const px=x+d*c,py=y+d*s,k=map.visitKey(px,py);
        if(k!==map.lastVisit&&(map.blocked.get(k)||0)>now){blocked=true;break;}
        gap=Math.min(gap,map.clearance(Math.floor(px/CELL),Math.floor(py/CELL)));
        if(k!==map.lastVisit)visits=Math.max(visits,map.visits.get(k)||0);
      }
      if(blocked)continue;
      const hint=map.hint?(distance(map.pose,map.hint)-distance(goal,map.hint))*.3:0;
      const continuity=heading==null?0:.22*Math.cos(wrap(angle-heading));
      const score=length+.85*Math.cos(angle-yaw)+gap*.45+continuity+hint-.9*visits;
      candidates.push({goal,angle,score});break;
    }
  }
  candidates.sort((a,b)=>b.score-a.score);
  // The search yields to telemetry between directions. Recheck the winning
  // corridor against any observations received since it was first scored.
  return candidates.find(candidate=>map.segmentClear(candidate.goal))||null;
}

function* roam(map,now,heading){
  // A marked exit takes precedence, including exits many local horizons away.
  if(map.exit)return yield* map.search(now);
  const next=yield* roamCandidate(map,now,heading);
  if(next){
    map.leg=next.goal;map.route=[map.pose.slice(0,2),next.goal];
    map.note=Math.abs(wrap(next.angle-map.pose[2]))<.35?'漫游 · 沿已观测通道前进':'漫游 · 转向有净距的通道';
    return next.goal;
  }
  const detour=yield* map.search(now);
  if(detour)map.note='漫游 · 沿累计地图返回可通行的岔口';
  return detour;
}
module.exports={roamCandidate,roam,wrap};
