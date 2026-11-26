'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {MazeMap,CELL,distance}=require('./maze-map.cjs');
const {MazeMission}=require('./mission.cjs');
const {roam,roamCandidate}=require('./roam.cjs');

function scene(){
  const map=new MazeMap();
  map.accept({generation:'round-1',frame:'camera_init',stamp:1,pose:[.05,.05,0],marks:[],rays:[]});
  return map;
}
function rectangle(map,left,right,bottom,top){
  for(let x=left;x<=right;x++)for(let y=bottom;y<=top;y++)map.mark(x,y,1);
}
function plan(map,now=1000){
  const work=map.search(now);
  let result;
  do{result=work.next();}while(!result.done);
  return result.value;
}
function targets(map,kind,x,y){map.goals({kind,x,y,id:map.id});}

test('rays are transformed at their own pose, stop at returns, and tiles distinguish base from unknown',()=>{
  const map=new MazeMap();
  const scan={frame:'camera_init',generation:'a',stamp:1,pose:[.05,.05,Math.PI/2],marks:[[1,0]],rays:[[2,0]]};
  assert.ok(map.accept(scan));assert.equal(map.value(0,7),1);assert.equal(map.value(0,10),2);
  assert.equal(map.value(0,14),0);
  const snap=map.snapshot('',0);
  const tile=Buffer.from(snap.tiles.find(([x,y])=>x===0&&y===0)[2],'base64');
  const code=(x,y)=>{const i=y*32+x;return tile[i>>2]>>((i&3)*2)&3;};
  assert.equal(code(0,10),2);assert.equal(code(1,10),3);assert.equal(code(0,7),1);
  assert.deepEqual(map.snapshot(snap.id,snap.revision).tiles,[]);
  assert.equal(map.accept(scan),false);
  assert.equal(map.accept({...scan,stamp:2,marks:[[NaN,0]]}),false);
  assert.equal(map.accept({...scan,stamp:2,generation:'b\nstart'}),false);
  targets(map,'hint',3,0);assert.ok(distance(map.hint,[.05,3.05])<1e-8);
  assert.ok(map.accept({...scan,generation:'b',stamp:2}));assert.notEqual(map.id,snap.id);
  assert.equal(map.hint,null);assert.throws(()=>map.goals({kind:'exit',x:0,y:0,id:snap.id}),/地图已更新/);
});

test('horizontal-only observations never clear an obstacle of unknown height, even a single return',()=>{
  const map=scene();
  const sample={frame:'camera_init',generation:map.generation,pose:map.pose,marks:[],rays:Array(19).fill([1.8,0])};
  map.accept({...sample,stamp:2,marks:[[1,0],[1,.4]]});
  for(let i=3;i<=9;i++)map.accept({...sample,stamp:i,marks:[[1,.4]]});
  assert.equal(map.value(10,0),2);
  map.accept({...sample,stamp:10,marks:[[1,.4]]});
  assert.equal(map.value(10,0),2);
  assert.equal(map.clearance(10,0),0);
  for(let i=11;i<22;i++)map.accept({...sample,stamp:i,rays:[[1.8,.72]]});
  assert.equal(map.value(10,4),2);
});

test('unknown exits, unobserved footprint corners and passages narrower than the padded vehicle are rejected',()=>{
  const map=scene();
  assert.throws(()=>targets(map,'exit',40,0),/已观测/);
  rectangle(map,-10,30,-10,10);assert.ok(map.segmentClear([1.5,.05]));
  const t=map.tile(9,3);t.data[map.offset(9,3)]=0;
  assert.equal(map.segmentClear([1.5,.05]),false);map.mark(9,3,1);
  map.mark(9,4,2);map.mark(9,-4,2);
  assert.equal(map.segmentClear([1.5,.05]),false);
  const wide=scene();rectangle(wide,-10,30,-10,10);wide.mark(9,7,2);wide.mark(9,-7,2);
  assert.ok(wide.segmentClear([1.5,.05]));
});

test('confirmed obstacle columns clear only with repeated free observations through every measured height',()=>{
  const map=scene();let n=2;
  function observe(hits,rays){
    map.accept({frame:'camera_init',generation:map.generation,stamp:n++*.4+2,pose:map.pose,
      sensorZ:.5,marks:hits.map(([x,y])=>[x,y]),rays,heightHits:hits});
  }
  for(let i=0;i<5;i++)observe([[1,0,0],[1,0,-.2]],[]);
  for(let i=0;i<15;i++)observe([[2,0,0]],[[2,0]]);
  assert.equal(map.value(10,0),2,'an upper beam cannot clear the lower part of a cone');
  for(let i=0;i<16;i++)observe([[2,0,-.4]],[[2,0]]);
  assert.equal(map.value(10,0),1,'both old surfaces were traversed after the object left');
});

test('calibrated full scans mark each return at its corrected location only',()=>{
  const map=scene();
  map.accept({frame:'camera_init',generation:map.generation,stamp:2,pose:map.pose,
    coverage:'full',lidarOffset:[0,0,.28],rayOrigin:map.pose.slice(0,2),sensorZ:.28,
    marks:[[1,0]],heightHits:[[1.12,0,-.2]],rays:[]});
  assert.equal(map.value(11,0),2);
  assert.notEqual(map.value(10,0),2,'the raw projection must not leave a duplicate obstacle without height evidence');
});

test('long designated exit uses bounded legs and an occupied shortcut cannot cross a wall',()=>{
  const map=scene();rectangle(map,-8,455,-9,9);targets(map,'exit',44,.05);
  let goal=plan(map);assert.ok(goal);assert.ok(distance(map.pose,goal)<=2.001);
  assert.ok(goal[0]>1);assert.deepEqual(map.route.at(-1),map.exit);
  const wall=scene();rectangle(wall,-15,45,-20,20);
  for(let y=-20;y<=5;y++)wall.mark(15,y,2);
  targets(wall,'exit',3.5,.05);goal=plan(wall);
  assert.ok(goal&&wall.segmentClear(goal));assert.ok(wall.route.some(p=>p[1]>1));
});

test('a failed branch yields another reachable approach before its retry time',()=>{
  const map=scene();rectangle(map,-32,32,-10,10);
  for(let x=-37;x<=37;x++){map.mark(x,10,2);map.mark(x,-10,2);}
  targets(map,'hint',40,0);
  const first=plan(map);assert.ok(first&&first[0]>0);
  // Record all blocked gates along the east branch, leaving the west side reachable.
  for(let x=.75;x<3.4;x+=.3)for(let y=-.8;y<.81;y+=.25)map.avoid([x,y],2000);
  const alternate=plan(map,2001);assert.ok(alternate&&alternate[0]<0);
  assert.ok(map.segmentClear(alternate));assert.equal(map.exit,null);
});

function supervisor(mode='explore'){
  const map=scene();rectangle(map,-13,35,-12,12);
  const sent=[],mission=new MazeMission(map,cmd=>(sent.push(cmd),true));
  const task={phase:'running',options:{mode}};let owns=true;
  const status={stage:'FAULT',reason:'Operator stop',feedback_v:0,feedback_w:0,v:0,w:0,maze_leg:''};
  const ctx={status,statusAt:1000,stopped:true,healthy:true};
  mission.start(task,()=>owns,1000);
  return {map,mission,task,sent,status,ctx,release(){owns=false;}};
}
function firstLeg(s){
  for(let n=0;n<80&&!s.mission.run?.leg;n++)s.mission.tick(s.ctx,1000+n*10);
  assert.ok(s.mission.run?.leg);return s.mission.run.leg;
}

test('only a designated exit plus stopped feedback completes the mission',()=>{
  const s=supervisor();targets(s.map,'hint',0,0);
  const leg=firstLeg(s);assert.match(s.sent[0],/^startmaze /);
  assert.equal(s.task.phase,'running');
  s.map.pose=[...leg.goal,0];Object.assign(s.status,{stage:'DONE',maze_leg:leg.id});
  s.ctx.statusAt=2000;s.ctx.stopped=false;s.mission.tick(s.ctx,2000);
  assert.equal(s.task.phase,'running');
  targets(s.map,'exit',...leg.goal);
  s.ctx.stopped=true;s.mission.tick(s.ctx,2010);
  assert.equal(s.task.phase,'completed');assert.equal(s.mission.run,null);
});

test('stalled leg parks and fences its replacement until a fresh zero-speed stop acknowledgment',()=>{
  const s=supervisor(),leg=firstLeg(s);
  Object.assign(s.status,{stage:'DRIVING',reason:'',maze_leg:leg.id});s.ctx.stopped=false;s.ctx.statusAt=15500;
  s.mission.tick(s.ctx,15501);assert.equal(s.sent.at(-1),'stop');assert.ok(s.map.blocked.size);
  const count=s.sent.length;s.mission.tick(s.ctx,15760);assert.equal(s.sent.length,count);
  s.status.stage='FAULT';s.status.reason='Operator stop';s.ctx.stopped=true;s.ctx.statusAt=15500;
  s.mission.tick(s.ctx,15800);assert.ok(s.mission.run.needsStop);
  s.ctx.statusAt=15802;s.mission.tick(s.ctx,15803);assert.ok(!s.mission.run.needsStop);
});

test('the maze supervisor leaves a stopped start to the controller and still aborts an unsafe moving sweep',()=>{
  const s=supervisor(),leg=firstLeg(s);
  Object.assign(s.status,{stage:'WAITING',reason:'',maze_leg:leg.id});
  s.ctx.stopped=false;s.ctx.statusAt=2100;
  s.map.mark(4,3,2);
  assert.equal(s.map.brakingClear(0,0),false);
  s.mission.tick(s.ctx,2101);
  assert.equal(s.mission.run.leg,leg);
  assert.ok(!s.sent.includes('stop'));
  s.status.stage='DRIVING';s.status.v=.12;s.ctx.statusAt=2200;
  s.mission.tick(s.ctx,2201);
  assert.equal(s.sent.at(-1),'stop');
  assert.ok(s.map.blocked.size);
});

test('measured retracing can keep a maze leg alive while distance to the goal increases',()=>{
  const s=supervisor(),leg=firstLeg(s);
  Object.assign(s.status,{stage:'DRIVING',maze_leg:leg.id,avoidance_state:'route_retreat',v:-.08,feedback_v:-.08});
  s.ctx.stopped=false;
  for(let i=1;i<=18;i++){
    s.map.pose=[.05-i*.042,.05,0];s.ctx.statusAt=1000+i*1000;
    s.mission.tick(s.ctx,s.ctx.statusAt+1);
  }
  assert.equal(s.mission.run.leg,leg);
  assert.ok(!s.sent.includes('stop'));
  s.ctx.statusAt=34000;s.mission.tick(s.ctx,34001);
  assert.equal(s.sent.at(-1),'stop','A retreat that no longer moves must release the leg');
});

test('map resets, stale perception, lease loss and unconfirmed commands all terminate motion',()=>{
  for(const cause of ['map','sensor','lease','ack']){
    const s=supervisor();firstLeg(s);
    if(cause==='map')s.map.reset();
    if(cause==='sensor')s.ctx.healthy=false;
    if(cause==='lease')s.release();
    s.mission.tick(s.ctx,5020);
    assert.equal(s.mission.run,null,cause);assert.equal(s.sent.at(-1),'stop',cause);
    assert.ok(['failed','stopped'].includes(s.task.phase));
  }
});

test('an explicit restart can clear a recovered old takeover while a new takeover always ends the leg',()=>{
  const s=supervisor(),old='Chassis fault or control-mode change';
  s.status.reason=old;s.mission.start(s.task,()=>true,1000,old);
  const leg=firstLeg(s);s.status.maze_leg=leg.id;s.status.stage='DRIVING';s.status.reason='';
  s.ctx.stopped=false;s.ctx.statusAt=2100;s.mission.tick(s.ctx,2101);
  s.status.stage='FAULT';s.status.reason=old;s.ctx.statusAt=2200;s.mission.tick(s.ctx,2201);
  assert.equal(s.task.phase,'failed');assert.equal(s.sent.at(-1),'stop');
});

test('retargeting an already finished leg accepts the native latched DONE only when paused and stationary',()=>{
  const s=supervisor();firstLeg(s);s.mission.retarget(2000);
  s.status.stage='DONE';s.ctx.statusAt=2150;s.status.paused=false;s.mission.tick(s.ctx,2150);
  assert.ok(s.mission.run.needsStop);
  s.status.paused=true;s.ctx.stopped=false;s.mission.tick(s.ctx,2200);
  assert.ok(s.mission.run.needsStop);
  s.ctx.stopped=true;s.mission.tick(s.ctx,2300);assert.ok(!s.mission.run.needsStop);
});

test('roaming favors a clear forward corridor, chooses a side when blocked, and respects an explicit exit',()=>{
  const map=scene();rectangle(map,-23,32,-28,28);
  function choose(){
    const search=roam(map,1000,0);let step;
    do{step=search.next();}while(!step.done);
    return step.value;
  }
  const ahead=choose();assert.ok(ahead[0]>1.6&&Math.abs(ahead[1]-.05)<.001);
  for(let y=-5;y<=5;y++)map.mark(11,y,2);
  const side=choose();assert.ok(side&&Math.abs(side[1])>.65);assert.ok(map.segmentClear(side));
  targets(map,'exit',-.9,-1.2);choose();assert.deepEqual(map.route.at(-1),map.exit);
});

test('a close side return from the parked robot blocks departure without declaring the maze finished',()=>{
  const s=supervisor('roam');
  s.map.accept({frame:'camera_init',generation:s.map.generation,stamp:2,pose:s.map.pose,marks:[[-.133,.293]],rays:[]});
  s.mission.tick(s.ctx,1100);s.mission.tick(s.ctx,2600);
  assert.equal(s.map.snapshot('',0).departureClear,false);
  assert.ok(!s.sent.some(c=>c.startsWith('startmaze ')));
  assert.equal(s.task.phase,'running');assert.match(s.task.log,/净距/);
});

test('a fresh obstacle arriving midway through a roaming search vetoes a previously clear forward candidate',()=>{
  const map=scene();rectangle(map,-25,30,-26,26);
  const work=roamCandidate(map,1000,0);
  work.next();work.next();
  map.mark(9,0,2);
  let step;do{step=work.next();}while(!step.done);
  assert.ok(step.value);assert.ok(map.segmentClear(step.value.goal));assert.ok(Math.abs(step.value.angle)>.4);
});

test('brief planner loss stops immediately, then waits for stable data and stopped feedback before replanning',()=>{
  const s=supervisor();const leg=firstLeg(s);
  Object.assign(s.status,{stage:'DRIVING',maze_leg:leg.id});s.ctx.healthy=false;s.ctx.unavailable='planner';s.ctx.stopped=false;
  s.mission.tick(s.ctx,2100);assert.equal(s.sent.at(-1),'stop');assert.equal(s.task.phase,'running');
  s.ctx.healthy=true;s.ctx.unavailable='';s.ctx.statusAt=2300;s.ctx.stopped=true;
  Object.assign(s.status,{stage:'FAULT',reason:'Operator stop',paused:true});s.mission.tick(s.ctx,2300);
  s.mission.tick(s.ctx,2900);assert.ok(s.mission.run.recovery);
  s.ctx.statusAt=3100;s.mission.tick(s.ctx,3100);assert.ok(!s.mission.run.recovery);
  assert.equal(s.mission.run.leg,null);
  s.ctx.healthy=false;s.ctx.unavailable='control';s.mission.tick(s.ctx,3120);assert.equal(s.task.phase,'failed');
});

test('a lidar gap keeps the exit, but never overrides an operator pause',()=>{
  const s=supervisor(),leg=firstLeg(s);
  targets(s.map,'exit',2.2,.05);
  const mapId=s.map.id;
  Object.assign(s.status,{stage:'FAULT',reason:'Lidar timeout',maze_leg:leg.id,paused:false});
  s.ctx.stopped=false;s.ctx.statusAt=2100;
  s.mission.tick(s.ctx,2101);
  assert.match(s.task.log,/雷达暂时中断/);
  assert.equal(s.sent.at(-1),'stop');
  s.mission.pause(2250);
  s.status.paused=true;s.ctx.stopped=true;s.ctx.statusAt=2400;
  s.mission.tick(s.ctx,2401);s.ctx.statusAt=3200;s.mission.tick(s.ctx,3201);
  assert.ok(s.mission.run.paused);
  assert.equal(s.mission.run.recovery,null);
  assert.equal(s.map.id,mapId);
  assert.deepEqual(s.map.exit,[2.2,.05]);
  const starts=s.sent.filter(c=>c.startsWith('startmaze ')).length;
  s.mission.tick(s.ctx,3300);
  assert.equal(s.sent.filter(c=>c.startsWith('startmaze ')).length,starts);
  s.mission.resume();
  for(let now=3400;now<=3500&&!s.mission.run.leg;now+=10)s.mission.tick(s.ctx,now);
  assert.ok(s.mission.run.leg);
});

test('roaming renews a straight leg while moving, and rejects renewal across an unobserved turn',()=>{
  const s=supervisor('roam'),leg=firstLeg(s);
  s.map.pose=[leg.goal[0]-.78,.05,0];s.ctx.stopped=false;
  Object.assign(s.status,{stage:'DRIVING',maze_leg:leg.id,v:.1,feedback_v:.1,reason:''});
  for(let i=0;i<12&&!s.sent.some(c=>c.startsWith('extendmaze '));i++){
    s.ctx.statusAt=2100+i*20;s.mission.tick(s.ctx,s.ctx.statusAt+1);
  }
  assert.ok(s.sent.at(-1).startsWith('extendmaze '+leg.id+' '));
  assert.ok(!s.sent.includes('stop'));
  // This particular renewal has no matching controller acknowledgment.
  s.ctx.statusAt=3100;s.mission.tick(s.ctx,3101);
  assert.equal(s.sent.at(-1),'stop');assert.equal(s.mission.run.needsStop,true);
  const corner=supervisor('roam'),old=firstLeg(corner);
  corner.map.pose=[old.goal[0]-.78,.05,Math.PI/2];corner.ctx.stopped=false;corner.ctx.statusAt=2300;
  Object.assign(corner.status,{stage:'DRIVING',maze_leg:old.id,v:.1,feedback_v:.1,reason:''});
  corner.mission.tick(corner.ctx,2310);
  assert.ok(!corner.sent.some(c=>c.startsWith('extendmaze ')));
});
