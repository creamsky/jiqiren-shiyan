'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const {RegisteredMap}=require('./registered-map.cjs');
function frame(map,n,points,generation='round1'){
  const stamp=n*.2,now=n*200;
  return map.accept({registered:true,stamp,frame:'camera_init',points},
    {frame:'camera_init',pose:[0,0,0],position:[0,0,.05],stamp,at:now,
      lidarAt:(_stamp,offset)=>({origin:offset,self:()=>false})},generation,now,[0,0,.05]);
}
const point=[1.05,.01,.05],far=[2.1,.02,.05],high=[2.1,.02,.55];
test('the same world-space XYZ is retained once across frames, new tabs can catch up and wrong frames are rejected',()=>{
  const map=new RegisteredMap();frame(map,1,[point,point]);assert.equal(map.points.length,0);
  frame(map,2,[point]);frame(map,3,[point]);assert.equal(map.points.length,1);
  const chunk=map.chunk('',0);assert.equal(chunk.count,1);const binary=Buffer.from(chunk.xyz,'base64');
  assert.ok(Math.abs(binary.readFloatLE(0)-1.05)<1e-6);assert.equal(map.chunk(chunk.id,1).xyz,'');
  assert.equal(map.accept({registered:false},{},'x'),false);
  frame(map,4,[far],'round2');assert.notEqual(map.id,chunk.id);assert.equal(map.points.length,0);
});
test('overhead rays retain previous lower obstacles, repeated rays at their measured height remove passing objects',()=>{
  const map=new RegisteredMap();
  for(let i=1;i<=5;i++)frame(map,i,[point]);
  for(let i=6;i<=18;i++)frame(map,i,[high]);
  assert.ok(map.points[0]);const id=map.id;
  for(let i=19;i<=32;i++)frame(map,i,[far,far]);
  assert.equal(map.points[0],null);assert.ok(map.chunk(id,0).removed.includes(0));
  assert.ok(Number.isNaN(Buffer.from(map.chunk('',0).xyz,'base64').readFloatLE(0)));
  assert.equal(map.chunk(id,0,10000,map.removed.length).removed.length,0);
});
test('capacity coarsening changes the cursor generation and preserves remote stationary structure',()=>{
  const map=new RegisteredMap(3);
  const pts=[[.05,.05,.05],[.21,.05,.05],[1,.05,.05],[3,.05,.05]];
  const id=map.id;
  for(let i=1;i<=3;i++)frame(map,i,pts);
  assert.notEqual(map.id,id);assert.ok(map.points.some(p=>p[0]===3));assert.equal(map.chunk(id,3).from,0);
});
test('a low curb and its supporting floor keep separate height bins even after horizontal coarsening',()=>{
  const m=new RegisteredMap(5);
  const xyz=[[1.05,.02,.011],[1.05,.02,.085],[4,.1,.01],[2,.1,.01],[3,.1,.01],[2,.5,.01]];
  for(let i=1;i<=3;i++)frame(m,i,xyz);
  assert.ok(m.cell>.10);assert.equal(m.verticalCell,.04);
  assert.ok(m.points.some(p=>p[2]===.011));assert.ok(m.points.some(p=>p[2]===.085));
});
