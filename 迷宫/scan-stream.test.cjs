'use strict';
const test=require('node:test');
const assert=require('node:assert/strict');
const {MazeScanStream}=require('./scan-stream.cjs');

test('a healthy source with an unaccepted map is distinct from a dropped lidar pair',()=>{
  let current='old';
  const stream=new MazeScanStream(scan=>scan.type==='lidar_mount'||scan.generation===current);
  assert.equal(stream.accept('{"type":"lidar_mount","lidarOffset":[0,0,0.28]}',100),false);
  assert.equal(stream.sourceAt,0);
  assert.equal(stream.lastSampleAt,0);

  const frame='{"stamp":172.1,"generation":"new"}';
  assert.equal(stream.accept(frame,120),false);
  assert.equal(stream.sourceAt,120);
  assert.equal(stream.lastSampleAt,0);
  assert.match(stream.error,/当前地图/);

  stream.accept('{"type":"scan_status","issue":"pose"}',800);
  assert.equal(stream.sourceAt,120);
  assert.match(stream.error,/相同时刻的定位/);

  current='new';
  assert.equal(stream.accept(frame,950),true);
  assert.equal(stream.lastSampleAt,950);
  assert.equal(stream.error,'');
});
