const {test}=require('node:test');
const assert=require('node:assert/strict');
const net=require('node:net');
const {createConsole}=require('./server.cjs');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));

test('hardware reconnect checks the remote service and waits for telemetry before reporting online',async()=>{
  const attempts=[],commands=[];
  let peer,serviceRunning=true;
  const robot=net.createServer(client=>{
    peer=client;client.on('data',data=>commands.push(...data.toString().trim().split('\n')));
  });
  await new Promise(resolve=>robot.listen(0,'127.0.0.1',resolve));
  const probe=net.createServer();await new Promise(resolve=>probe.listen(0,'127.0.0.1',resolve));
  const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
  const app=createConsole({port,tcpPort:robot.address().port,deferredHardware:true,robotLink:{
    async ensureRobotLink(signal,{recover}){
      attempts.push(recover);
      if(recover&&!serviceRunning)throw Object.assign(Error('控制服务已停止'),{manual:true});
      serviceRunning=true;
    },closeRobotLink(){},
  }});
  await new Promise(resolve=>app.server.listen(port,'127.0.0.1',resolve));
  const base=`http://127.0.0.1:${port}`;
  const connect=()=>fetch(base+'/api/hardware/connect',{method:'POST',headers:{origin:base}});
  const status=async()=>(await fetch(base+'/api/status')).json();
  const waitFor=async predicate=>{
    for(let n=0;n<150;n++){if(await predicate())return;await delay(25);}
    assert.fail('Hardware recovery timed out');
  };
  const telemetry=()=>peer.write('{"stage":"IDLE","paused":true,"feedback_fresh":true}\n');
  try{
    await connect();await waitFor(()=>commands.includes('status'));
    assert.equal((await status()).connected,false);
    telemetry();await waitFor(async()=>(await status()).connected);
    const burst=(baseSeq,seq,add,del)=>peer.write(JSON.stringify({type:'cloud_delta',base:baseSeq,seq,add,del,frame:'camera_init',stamp:seq})+'\n');
    const preview=async(tag='')=>(await fetch(base+'/api/cloud?brief=1&seen='+encodeURIComponent(tag))).json();
    burst(0,1,[[100,0,0],[200,0,0]],[]);await waitFor(async()=>(await preview()).points?.length===2);
    burst(1,2,[[300,0,0]],[[100,0,0]]);
    await waitFor(async()=>(await preview()).stamp===2);
    const snapshot=await preview();assert.deepEqual(snapshot.points,[[2,0,0],[3,0,0]]);
    assert.equal((await preview(snapshot.tag)).unchanged,true);
    burst(12,13,[],[]);await waitFor(()=>commands.filter(c=>c==='cloudkey').length>=2);
    assert.equal((await preview()).stamp,2);
    let oldPeer=peer;peer.destroy();await waitFor(()=>peer!==oldPeer);
    assert.deepEqual(attempts,[false,true]);
    assert.equal((await status()).owned,false);
    assert.equal((await status()).connected,false);
    telemetry();await waitFor(async()=>(await status()).connected);
    serviceRunning=false;peer.destroy();
    await waitFor(async()=>(await status()).hardwareLink.message==='控制服务已停止');
    oldPeer=peer;await connect();await waitFor(()=>peer!==oldPeer);
    telemetry();await waitFor(async()=>(await status()).connected);
    assert.deepEqual(attempts,[false,true,true,false]);
    assert(commands.includes('pause'));assert(!commands.includes('resume'));assert(!commands.includes('start'));
  }finally{app.close();peer?.destroy();await new Promise(resolve=>robot.close(resolve));}
});

test('loopback API enforces origin, token, exclusive lease, ordering, deadman and disconnect',async()=>{
  const received=[];let peer;let manualReply={ok:false,reason:'odometry_unavailable'};
  let state={stage:'WAITING',mode:'auto',paused:true,v:0,w:0,feedback_v:0,feedback_w:0,feedback_fresh:true,sensors_ready:true};
  const telemetry=setInterval(()=>{if(peer&&!peer.destroyed)peer.write(JSON.stringify(state)+'\n');},50);
  const robot=net.createServer(socket=>{
    peer=socket;let buffer='';socket.on('data',chunk=>{
      buffer+=chunk;let i;
      while((i=buffer.indexOf('\n'))>=0){const command=buffer.slice(0,i);received.push(command);buffer=buffer.slice(i+1);if(command==='manual')socket.write(JSON.stringify({type:'ack',command:'manual',...manualReply})+'\n');}
    });
    socket.write(JSON.stringify({stage:'WAITING',mode:'auto',paused:true,v:0,w:0,feedback_v:0,feedback_w:0,feedback_fresh:true,sensors_ready:true})+'\n');
  });
  await new Promise(resolve=>robot.listen(0,'127.0.0.1',resolve));
  const probe=net.createServer();await new Promise(resolve=>probe.listen(0,'127.0.0.1',resolve));
  const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
  const base=`http://127.0.0.1:${port}`;
  const app=createConsole({port,tcpPort:robot.address().port,ping:done=>done(true,'TTL=64'),sshRunner:async()=>0,leaseMs:1000,
    lidarReaderFactory:()=>({start(){},stop(){}})});
  await new Promise(resolve=>app.server.listen(port,'127.0.0.1',resolve));
  try{
    for(let i=0;i<20&&!peer;i++)await delay(10);
    const session=await (await fetch(base+'/api/session')).json();
    const second=await (await fetch(base+'/api/session')).json();
    await delay(550);
    const registry=await (await fetch(base+'/api/experiments')).json();
    assert.deepEqual(registry.experiments.map(item=>item.id),['lab1','lab2','lab2-eight','lab3','lab3-xy','maze']);
    assert.equal(registry.experiments[0].enabled,true);
    peer.write(JSON.stringify({type:'cloud',frame:'velodyne',points:Array.from({length:1005},()=>[1,2,3])})+'\n');
    peer.write(JSON.stringify({type:'pose',stamp:10,pose:[1,2,0.3]})+'\n');
    await delay(20);
    const cloud=await (await fetch(base+'/api/cloud')).json();
    assert.equal(cloud.points.length,1005);assert.equal(cloud.frame,'velodyne');assert(cloud.age>=0&&cloud.age<1);
    assert.deepEqual(cloud.trajectory.pose,[1,2,0.3]);
    assert.equal(cloud.trajectory.points.length,1);
    peer.write(JSON.stringify({type:'pose',frame:'camera_init',stamp:11,pose:[1.1,2,.3],position:[1.1,2,.2],quaternion:[0,0,0,1]})+'\n');
    peer.write(JSON.stringify({type:'cloud',registered:true,frame:'camera_init',stamp:11,points:Array.from({length:8005},()=>[1,2,.1])})+'\n');await delay(20);
    const world=await (await fetch(base+'/api/cloud')).json();assert.equal(world.points.length,8000);assert.equal(world.registered,true);assert.equal(world.stamp,11);
    const pose=await (await fetch(base+'/api/trajectory')).json();assert.equal(pose.stamp,11);assert.equal(pose.frame,'camera_init');assert.deepEqual(pose.position,[1.1,2,.2]);
    assert.equal((await (await fetch(base+'/api/status')).json()).status.stage,'WAITING','cloud must not replace controller status');
    const post=async(body,token=session.token,origin=base)=>{
      const snapshot=await (await fetch(base+'/api/status',{headers:{'X-Control-Token':token}})).json();
      return fetch(base+'/api/command',{method:'POST',headers:{origin,'X-Control-Token':token,'Content-Type':'application/json'},body:JSON.stringify({...body,lease:snapshot.room.lease})});
    };
    assert.equal((await post({seq:1,command:'claim'},session.token,'http://evil.example')).status,403);
    assert.equal((await post({seq:1,command:'claim'},'bad')).status,403);
    assert.equal((await post({seq:1,command:'w'})).status,409);
    assert.equal((await post({seq:2,command:'claim'})).status,200);
    assert.equal((await post({seq:1,command:'claim'},second.token)).status,409);
    assert.equal((await post({seq:2,command:'stop'},second.token)).status,200,'a non-owner authenticated tab can stop');
    for(let i=0;i<30&&(await(await fetch(base+'/api/status')).json()).room.stopping;i++)await delay(25);
    assert.equal((await post({seq:3,command:'start',experiment:'arbitrary-shell'},second.token)).status,400);
    assert.equal((await post({seq:4,command:'start'},second.token)).status,400);
    assert.equal((await post({seq:3,command:'heartbeat'})).status,200);
    assert.equal((await post({seq:4,command:'w'})).status,200);
    assert.equal((await post({seq:4,command:'d'})).status,409);
    assert.equal((await post({seq:5,command:'rm -rf'})).status,400);
    await delay(60);assert(received.includes('w'));assert(!received.includes('d'));
    const count=received.filter(x=>x==='heartbeat').length;
    await delay(1550);
    assert.equal(received.filter(x=>x==='heartbeat').length,count,'server must never fabricate heartbeat');
    assert(received.includes('zero'));assert(received.includes('pause'));
    assert.equal((await post({seq:6,command:'w'})).status,409,'expired owner cannot move');
    assert.equal((await post({seq:7,command:'claim'})).status,200);
    assert.equal((await post({seq:8,command:'heartbeat'})).status,200);
    const refused=await post({seq:9,command:'manual'});
    assert.equal(refused.status,409,'manual mode must report a controller refusal instead of false success');
    assert.match((await refused.json()).error,/定位/);
    assert.equal((await (await fetch(base+'/api/status')).json()).status.stage,'WAITING','ack must not replace telemetry');
    manualReply={ok:true,reason:''};
    assert.equal((await post({seq:10,command:'manual'})).status,200);
    assert.equal((await post({seq:11,command:'drive',linear:1,angular:1,speed:1.5,turn:.3})).status,409,'old firmware must not silently accept vectors');
    state={...state,mode:'manual',manual_vector:true};peer.write(JSON.stringify(state)+'\n');await delay(20);
    assert.equal((await post({seq:12,command:'drive',linear:1,angular:-1,speed:.8,turn:.2})).status,200);
    assert.equal((await post({seq:13,command:'drive',linear:1,angular:1,speed:1.6,turn:.3})).status,400);
    assert.equal((await post({seq:14,command:'drive',linear:1,angular:2,speed:1,turn:.3})).status,400);
    assert.equal((await post({seq:15,command:'drive',linear:0,angular:1,speed:1,turn:.6})).status,400);
    await delay(20);assert(received.includes('drive 1 -1 0.8 0.2'));
    assert.equal((await post({seq:16,command:'drive',linear:1,angular:1,speed:1.5,turn:.5})).status,200);
    assert.equal((await post({seq:17,command:'start',experiment:'lab2'})).status,200);
    peer.destroy();await delay(50);
    assert.equal((await (await fetch(base+'/api/cloud')).json()).trajectory.pose,null);
    assert.equal((await post({seq:18,command:'w'})).status,503);
  }finally{clearInterval(telemetry);app.close();peer?.destroy();await new Promise(resolve=>robot.close(resolve));}
});

test('registered mapping starts without the maze and never carries scans across map generations',async()=>{
  let peer,mode;
  const robot=net.createServer(socket=>{peer=socket;});
  await new Promise(resolve=>robot.listen(0,'127.0.0.1',resolve));
  const probe=net.createServer();await new Promise(resolve=>probe.listen(0,'127.0.0.1',resolve));
  const port=probe.address().port;await new Promise(resolve=>probe.close(resolve));
  const app=createConsole({port,tcpPort:robot.address().port,lidarReaderFactory:onSample=>({
    start(next){mode=next;onSample({type:'lidar_mount',lidarOffset:[0,0,.28]});},
    stop(){mode=null;},
  })});
  await new Promise(resolve=>app.server.listen(port,'127.0.0.1',resolve));
  const base=`http://127.0.0.1:${port}`;
  const get=async url=>(await fetch(base+url)).json();
  const send=(...records)=>peer.write(records.map(record=>JSON.stringify(record)).join('\n')+'\n');
  const pose=stamp=>({type:'pose',frame:'camera_init',stamp,pose:[0,0,0],position:[0,0,0],quaternion:[0,0,0,1]});
  const cloud=stamp=>({type:'cloud',registered:true,frame:'camera_init',stamp,points:[[1,.15,.1]]});
  const waitFor=async probe=>{
    for(let i=0;i<40;i++){
      const result=await probe();if(result)return result;
      await delay(25);
    }
    assert.fail('Timed out waiting for synthetic robot telemetry');
  };
  try{
    await waitFor(()=>peer);
    send({map_generation:'first'},pose(10),cloud(10));
    await waitFor(()=>mode==='mount');
    send(pose(10.2),cloud(10.2),pose(10.4),cloud(10.4));
    const map=await waitFor(async()=>{const value=await get('/api/cloud-map');return value.active===1&&value;});
    const live=await get('/api/cloud'),odom=await get('/api/trajectory');
    assert.equal(map.generation,'first');assert.equal(live.generation,map.generation);assert.equal(odom.generation,map.generation);
    assert.equal(live.epoch,map.epoch);assert.equal(odom.epoch,map.epoch);
    // One frame per second like the onboard preview. The car turns and moves
    // past a cone that was only visible once. Later keyframes omit it.
    for(let i=1;i<=12;i++){
      const stamp=11+i,x=i*.15,yaw=Math.min(Math.PI,i*.3);
      send({...pose(stamp),pose:[x,0,yaw],position:[x,0,0],
          quaternion:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)]},
        {type:'cloud_delta',base:0,seq:i,frame:'camera_init',stamp,
          add:[[100+i*25,100,20]],del:[]});
    }
    await waitFor(async()=>(await get('/api/cloud')).stamp===23);
    const history=await get('/api/cloud-map'),bytes=Buffer.from(history.xyz,'base64');
    assert.equal(history.active,13,'Retain all distinct registered surfaces after they leave the live scan');
    assert.ok(Array.from({length:history.end},(_,i)=>bytes.readFloatLE(i*12)).includes(1.25));
    assert.deepEqual((await get('/api/cloud')).points,[[4,1,.2]]);
    send({map_generation:'second'});
    const cleared=await waitFor(async()=>{const value=await get(`/api/cloud-map?id=${map.id}&from=1`);return value.id!==map.id&&value;});
    assert.equal(cleared.active,0);assert.equal(cleared.from,0);
    assert.deepEqual((await get('/api/cloud')).points,[]);
    assert.equal((await get('/api/trajectory')).generation,'second');
  }finally{app.close();peer?.destroy();await new Promise(resolve=>robot.close(resolve));}
});
