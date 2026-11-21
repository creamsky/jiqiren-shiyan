const {test}=require('node:test');
const assert=require('node:assert/strict');
const {EventEmitter}=require('node:events');
const {PassThrough}=require('node:stream');
const {CloudStream}=require('./cloud-stream.cjs');

test('SSH cloud stream preserves full frames across chunks, isolates sources and reports staleness',()=>{
  let now=10000,killed=0,calls=0;
  const child=new EventEmitter();child.stdout=new PassThrough();child.stderr=new PassThrough();child.kill=()=>{killed++;child.emit('close');};
  const stream=new CloudStream({now:()=>now,run:(file,args)=>{calls++;assert.match(file,/ssh\.exe$/);assert(args.includes('-T'));assert.match(args.at(-1),/robot-lab-viewer\/cloud-stream$/);return child;}});
  stream.start();stream.start();assert.equal(calls,1,'one shared reader for all browser clients');
  const raw={type:'cloud',source:'raw',registered:false,stamp:10,frame:'velodyne',frames:1,sourcePoints:24514,points:Array.from({length:24514},(_,i)=>[i*.01,2,3])};
  const line=JSON.stringify(raw)+'\n';child.stdout.write(line.slice(0,301));child.stdout.write(line.slice(301,50000));child.stdout.write(line.slice(50000));
  assert.equal(stream.snapshot('raw').points.length,24514);assert.deepEqual(stream.snapshot('raw').points.at(-1),raw.points.at(-1));
  assert.equal(stream.snapshot('raw').connected,true);
  const aggregate={...raw,source:'aggregate',registered:true,frame:'camera_init',frames:20,sourcePoints:25000,points:Array.from({length:25000},()=>[1,2,3])};
  assert(stream.accept(JSON.stringify(aggregate)));assert.equal(stream.snapshot('aggregate').points.length,25000);assert.equal(stream.snapshot('raw').points.length,24514);
  assert.equal(stream.accept(JSON.stringify({...raw,points:[[null,2,3]]})),false);
  assert.equal(stream.accept(JSON.stringify({...aggregate,registered:false})),false);
  assert.equal(stream.snapshot('raw').points.length,24514,'bad input must not replace a valid frame');
  now+=3100;assert.equal(stream.snapshot('raw').connected,false);assert.equal(stream.snapshot('raw').age,3.1);
  stream.stop();assert.equal(killed,1);assert.equal(stream.active,false);assert.equal(stream.child,null);
});
