const {test}=require('node:test');
const assert=require('node:assert/strict');
const {ExperimentTasks,REMOTE}=require('./experiments.cjs');
const tick=()=>new Promise(resolve=>setTimeout(resolve,0));
test('lab3 prepares without movement and starts only on the second action',async()=>{
 const sent=[];const tasks=new ExperimentTasks({send:c=>sent.push(c),stopped:()=>true,ready:()=>true,run:async()=>0});
 tasks.start('lab3',()=>true);await tasks.job;assert.equal(tasks.task.phase,'ready');assert.deepEqual(sent,['stop']);
 tasks.start('lab3',()=>true);assert.equal(sent.at(-1),'start3');
 tasks.observe({stage:'DRIVING'});tasks.observe({stage:'DONE'});assert.equal(tasks.task.phase,'completed');
});

test('eight validates options, prepares without movement, and sends selected start point',async()=>{
 const sent=[];const tasks=new ExperimentTasks({send:c=>sent.push(c),stopped:()=>true,run:async()=>0});
 for(const options of [null,{}, {radius:0.1,startPoint:'cross'},{radius:11,startPoint:'outer'},{radius:2,startPoint:'cross\nstart'}])assert.throws(()=>tasks.start('lab2-eight',()=>true,options),/半径/);
 tasks.start('lab2-eight',()=>true,{radius:2,startPoint:'cross'});await tasks.job;
 assert.equal(tasks.task.phase,'ready');assert.deepEqual(sent,['stop']);
 tasks.start('lab2-eight',()=>true,{radius:3.5,startPoint:'outer'});assert.equal(sent.at(-1),'start8 3.5 outer');
 tasks.observe({stage:'DRIVING'});tasks.observe({stage:'DONE'});assert.equal(tasks.task.phase,'completed');
});

test('lab1 progress is reported from real split stdout lines',async()=>{
 const tasks=new ExperimentTasks({send:()=>{},stopped:()=>true,run:async(cmd,onData)=>{
 onData('EXPERIMENT_PROG');onData('RESS 40 建图采集\n');assert.equal(tasks.task.progress,40);
 onData('EXPERIMENT_PROGRESS 70 保存并检查地图\n');assert.equal(tasks.task.progress,70);return 0;
 }});
 tasks.start('lab1',()=>true);await tasks.job;assert.equal(tasks.task.progress,100);
});

test('preparation requires fresh sensors and a stopped task cannot reuse readiness',async()=>{
 let ready=false;const tasks=new ExperimentTasks({send:()=>{},stopped:()=>true,ready:()=>ready,run:async()=>0,readyMs:10});
 tasks.start('lab2',()=>true);await tasks.job;assert.equal(tasks.task.phase,'failed');assert.equal(tasks.prepared,null);
 ready=true;tasks.start('lab2',()=>true);await tasks.job;assert.equal(tasks.task.phase,'ready');
 ready=false;assert.throws(()=>tasks.start('lab2',()=>true),/未就绪/);assert.equal(tasks.prepared,null);
 ready=true;tasks.start('lab2',()=>true);await tasks.job;tasks.stop();assert.equal(tasks.prepared,null);
});
test('lab1 requires stopped feedback, prevents duplicates, and waits for stop completion',async()=>{
  const sent=[],runs=[];let finish;
  const tasks=new ExperimentTasks({send:c=>sent.push(c),stopped:()=>true,run:async(command,onData)=>{
    runs.push(command);onData('mapping log\n');
    if(command===REMOTE.lab1)return new Promise(resolve=>finish=resolve);
    return 0;
  }});
  tasks.start('lab1',()=>true);await tick();
  assert.equal(sent[0],'stop');assert.equal(tasks.task.phase,'running');
  assert.throws(()=>tasks.start('lab1',()=>true),/稍候/);
  tasks.stop();assert.equal(tasks.task.phase,'stopping');
  assert.throws(()=>tasks.start('lab2',()=>true),/稍候/);
  finish(0);await tasks.job;assert.equal(tasks.task.phase,'stopped');
  assert.deepEqual(runs,[REMOTE.lab1,REMOTE.stopLab1,REMOTE.stopLab1]);
});
test('lab2 preparation never starts motion and requires another live-owner action',async()=>{
  const sent=[],runs=[];
  const tasks=new ExperimentTasks({send:c=>sent.push(c),stopped:()=>true,run:async command=>{runs.push(command);return 0;}});
  tasks.start('lab2',()=>true);await tasks.job;
  assert.equal(tasks.task.phase,'ready');assert.equal(tasks.prepared,'lab2');
  assert(!sent.includes('start'));assert.deepEqual(runs,[REMOTE.lab2]);
  assert.throws(()=>tasks.start('lab2',()=>false),/失效/);
  tasks.start('lab2',()=>true);assert.equal(sent.at(-1),'start');
  assert.throws(()=>tasks.start('lab2',()=>true),/稍候/,'cannot reset a running automatic trajectory');
  tasks.observe({stage:'FAULT'});assert.equal(tasks.task.phase,'running','old preparation fault cannot terminate a new start');
  tasks.observe({stage:'DONE'});assert.equal(tasks.task.phase,'running','old completion cannot complete a new start');
  tasks.observe({stage:'TURNING'});tasks.observe({stage:'FAULT'});
  assert.equal(tasks.task.phase,'failed','terminal state is accepted after active state');
});
test('unsafe feedback or lost ownership prevents remote preparation',async()=>{
  let runs=0;
  const blocked=new ExperimentTasks({send:()=>{},stopped:()=>false,run:async()=>{runs++;return 0;},waitMs:30});
  blocked.start('lab1',()=>true);await blocked.job;
  assert.equal(blocked.task.phase,'failed');assert.equal(runs,0);
  const lost=new ExperimentTasks({send:()=>{},stopped:()=>true,run:async()=>{runs++;return 0;}});
  lost.start('lab2',()=>false);await lost.job;
  assert.equal(lost.task.phase,'failed');assert.equal(runs,0);
});
