const {test}=require('node:test');const assert=require('node:assert/strict');
const {Trajectory}=require('./trajectory.cjs');
test('trajectory uses finite measured poses, limits memory and resets discontinuities',()=>{
 const t=new Trajectory();t.add({stamp:1,pose:[0,0,0]});t.add({stamp:1,pose:[1,1,0]});assert.equal(t.points.length,1);
 t.add({stamp:2,pose:[NaN,0,0]});assert.equal(t.points.length,1);
 for(let i=1;i<2500;i++)t.add({stamp:i+2,pose:[i*.03,0,0]});assert.equal(t.points.length,2000);
 t.add({stamp:3000,pose:[100,0,0]});assert.equal(t.points.length,1);
 t.add({stamp:0,pose:[100.1,0,0]});assert.equal(t.points.length,1);
 t.clear();assert.equal(t.snapshot().pose,null);
});
test('world frame, elevation and quaternion are retained; frame changes break the trail',()=>{
 const t=new Trajectory();t.add({stamp:1,frame:'/camera_init',pose:[1,2,.4],position:[1,2,.7],quaternion:[0,0,0,1]});
 assert.equal(t.snapshot().frame,'camera_init');assert.deepEqual(t.snapshot().positions,[[1,2,.7]]);
 const epoch=t.snapshot().epoch;t.add({stamp:2,frame:'map',pose:[1.1,2,.4],position:[1.1,2,.8]});
 assert.equal(t.snapshot().positions.length,1);assert(t.snapshot().epoch>epoch);assert.equal(t.snapshot().frame,'map');
});
test('calibrated lidar origin and body mask follow the measured scan-time attitude, not the last position',()=>{
 const t=new Trajectory(),q=[0,0,Math.SQRT1_2,Math.SQRT1_2];
 for(const [stamp,x] of [[1,3],[1.1,3.1]])t.add({stamp,frame:'camera_init',pose:[x,1,Math.PI/2],position:[x,1,.4],quaternion:q});
 const sensor=t.lidarAt(1.05,[0,0,.28]);assert.ok(Math.abs(sensor.origin[0]-3.05)<1e-6);assert.ok(Math.abs(sensor.origin[2]-.68)<1e-6);
 assert.ok(sensor.self([3.05,1.31,.3]));assert.equal(sensor.self([3.05,1.34,.3]),false);
 assert.equal(sensor.self([3.34,1,.3]),false);assert.equal(sensor.self([3.05,1,.70]),false);
 assert.equal(t.lidarAt(.5,[0,0,.28]),null);assert.equal(t.lidarAt(1.05,null),null);
});
