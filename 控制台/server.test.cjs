const {test}=require('node:test');
const assert=require('node:assert/strict');
const net=require('node:net');
const {createConsole}=require('./server.cjs');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));

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
  const app=createConsole({port,tcpPort:robot.address().port,ping:done=>done(true,'TTL=64'),sshRunner:async()=>0,leaseMs:1000});
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
    peer.write(JSON.stringify({type:'cloud',registered:true,frame:'camera_init',stamp:10,points:Array.from({length:8005},()=>[1,2,.1])})+'\n');
    peer.write(JSON.stringify({type:'pose',frame:'camera_init',stamp:11,pose:[1.1,2,.3],position:[1.1,2,.2],quaternion:[0,0,0,1]})+'\n');await delay(20);
    const world=await (await fetch(base+'/api/cloud')).json();assert.equal(world.points.length,8000);assert.equal(world.registered,true);assert.equal(world.stamp,10);
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
