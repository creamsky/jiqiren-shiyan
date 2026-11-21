'use strict';
const {test}=require('node:test');
const assert=require('node:assert/strict');
const net=require('node:net');
const {createConsole}=require('../控制台/server.cjs');
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const listen=server=>new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
async function until(read,limit=3500){
  const end=Date.now()+limit;
  do{const result=await read();if(result)return result;await wait(25);}while(Date.now()<end);
  assert.fail('timed out waiting for state');
}

test('maze HTTP flow fences preparation, goals, roaming, and map resets by lease and map ID',async()=>{
  let peer,accept,stamp=10,generation='round-1',streaming=false,remotePreparations=0;
  const commands=[],pose=[.05,.05,0];
  const baseStatus={stage:'FAULT',reason:'Operator stop',mode:'auto',paused:true,v:0,w:0,feedback_v:0,feedback_w:0,
    sensors_ready:true,feedback_fresh:true,dwa_ready:true,chassis_mode:1,chassis_errors:0,remote_switch:2,
    xy_planner:'navfn_pursuit',maze_protocol:2,maze_leg:''};
  const rays=Array.from({length:360},(_,i)=>[4*Math.cos(i*Math.PI/180),4*Math.sin(i*Math.PI/180)]);
  const robot=net.createServer(socket=>{
    peer=socket;let buffer='';
    socket.on('data',chunk=>{
      buffer+=chunk;
      for(let at;(at=buffer.indexOf('\n'))>=0;){
        commands.push(buffer.slice(0,at));buffer=buffer.slice(at+1);
      }
    });
  });
  await listen(robot);
  const probe=net.createServer();await listen(probe);const port=probe.address().port;
  await new Promise(resolve=>probe.close(resolve));
  const app=createConsole({port,tcpPort:robot.address().port,leaseMs:20000,sshRunner:async()=>{remotePreparations++;return 0;},
    mazeStreamFactory:fn=>{accept=fn;return {start(){streaming=true;},stop(){streaming=false;}};}});
  const timer=setInterval(()=>{
    if(!peer||peer.destroyed)return;
    stamp+=.06;
    peer.write(JSON.stringify({...baseStatus,map_generation:generation})+'\n'+
      JSON.stringify({type:'pose',pose,position:[.05,.05,0],quaternion:[0,0,0,1],frame:'camera_init',stamp})+'\n'+
      JSON.stringify({type:'cloud',registered:true,frame:'camera_init',stamp,points:[[1.05,.05,.25],[2.05,.05,.6]]})+'\n');
    if(streaming)accept({generation,frame:'camera_init',stamp:stamp-.06,pose,rays,marks:[],lidarOffset:[0,0,.28]});
  },60);
  const base='http://127.0.0.1:'+port;
  const get=async url=>(await fetch(base+url)).json();
  await new Promise(resolve=>app.server.listen(port,'127.0.0.1',resolve));
  try{
    for(const endpoint of ['/hardware/','/maze.js','/maze.css'])assert.equal((await fetch(base+endpoint)).status,200);
    const owner=await get('/api/session'),guest=await get('/api/session');
    let seq=0,lease='';
    async function post(url,body,session=owner){
      const response=await fetch(base+url,{method:'POST',headers:{origin:base,'Content-Type':'application/json','X-Control-Token':session.token},
        body:JSON.stringify({...body,seq:++seq,lease})});
      return {code:response.status,data:await response.json()};
    }
    let initial=await until(async()=>{const m=await get('/api/maze');return m.healthy&&m;});
    assert.ok(initial.tiles.length);
    for(const [status,reason] of [
      [{dwa_ready:false},'等待路径规划'],
      [{sensors_ready:false,manual_block_reason:'lidar_unavailable'},'等待车载雷达数据'],
      [{sensors_ready:false,manual_block_reason:'odometry_unavailable'},'等待定位更新']
    ]){
      Object.assign(baseStatus,status);
      const waiting=await until(async()=>{const m=await get('/api/maze');return m.healthReason===reason&&m;});
      assert.equal(waiting.healthy,false);assert.equal(waiting.id,initial.id);
      Object.assign(baseStatus,{dwa_ready:true,sensors_ready:true,manual_block_reason:''});
    }
    await until(async()=>(await get('/api/maze')).healthy);
    await until(async()=>!(await get('/api/status')).room.stopping);
    const claim=await post('/api/command',{command:'claim'});assert.equal(claim.code,200);lease=claim.data.room.lease;
    const cloud=await until(async()=>{const v=await get('/api/cloud-map');return v.active>0&&v;});
    assert.equal(cloud.frame,'camera_init');assert.equal(cloud.registered,true);
    assert.equal((await get('/api/cloud-map?id='+cloud.id+'&from='+cloud.end)).xyz,'');
    const start=(options={})=>post('/api/command',{command:'start',experiment:'maze',options});
    assert.equal((await start()).code,200);
    await until(async()=>(await get('/api/status')).preparedExperiment==='maze');
    const prepared=await get('/api/maze');assert.notEqual(prepared.id,initial.id);initial=prepared;
    assert.notEqual((await get('/api/cloud-map')).id,cloud.id);
    assert.equal(remotePreparations,0,'healthy live sensors must be reused');
    assert.ok(!commands.some(c=>c.startsWith('startmaze ')));
    assert.equal((await post('/api/maze/goal',{kind:'exit',x:1,y:0,id:initial.id},guest)).code,409);
    assert.equal((await start({mapId:'old',mode:'roam'})).code,409);
    assert.equal((await post('/api/maze/goal',{kind:'exit',x:50,y:50,id:initial.id})).code,409);
    assert.equal((await start({mapId:initial.id,mode:'roam',hint:[8,0]})).code,200);
    await until(()=>commands.some(c=>c.startsWith('startmaze ')));
    const running=await get('/api/maze');assert.equal(running.mode,'roam');assert.ok(running.hint[0]>8);
    assert.equal((await post('/api/command',{command:'manual'})).code,409);
    generation='round-2';
    await until(async()=>(await get('/api/status')).task.phase==='failed');
    assert.equal((await get('/api/status')).preparedExperiment,null);
    const next=await until(async()=>{const m=await get('/api/maze');return m.healthy&&m;});
    assert.notEqual(next.id,initial.id);assert.equal(next.exit,null);assert.equal(next.hint,null);
    assert.equal(commands.at(-1),'stop');
  }finally{
    clearInterval(timer);app.close();peer?.destroy();await new Promise(resolve=>robot.close(resolve));
  }
});
