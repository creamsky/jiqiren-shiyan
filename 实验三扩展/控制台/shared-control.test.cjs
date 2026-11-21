const {test}=require('node:test');
const assert=require('node:assert/strict');
const net=require('node:net');
const os=require('node:os');
const http=require('node:http');
const {createConsole}=require('./server.cjs');
const delay=ms=>new Promise(r=>setTimeout(r,ms));
// Raw HTTP deliberately preserves Host for spoofing tests (fetch rewrites it).
const fetch=(url,options={})=>new Promise((resolve,reject)=>{
  const req=http.request(url,{method:options.method||'GET',headers:options.headers},res=>{let body='';res.on('data',x=>body+=x);res.on('end',()=>resolve({status:res.statusCode,headers:res.headers,json:async()=>JSON.parse(body)}));});
  req.on('error',reject);req.end(options.body);
});
async function fixture(t){
  let peer,buffer='',frozen=false;
  const received=[];
  const state={stage:'WAITING',mode:'manual',paused:true,v:0,w:0,feedback_v:0,feedback_w:0,feedback_fresh:true,sensors_ready:true,manual_vector:true,chassis_mode:1};
  const robot=net.createServer(s=>{peer=s;s.on('error',()=>{});s.on('data',d=>{buffer+=d;let i;while((i=buffer.indexOf('\n'))>=0){const c=buffer.slice(0,i);received.push(c);buffer=buffer.slice(i+1);if(c==='manual')s.write('{"type":"ack","command":"manual","ok":true}\n');}});});
  await new Promise(r=>robot.listen(0,'127.0.0.1',r));
  const probe=net.createServer();await new Promise(r=>probe.listen(0,'127.0.0.1',r));const port=probe.address().port;await new Promise(r=>probe.close(r));
  const app=createConsole({port,tcpPort:robot.address().port,sshRunner:async()=>0});
  await new Promise(r=>app.server.listen(port,'0.0.0.0',r));
  const timer=setInterval(()=>{if(peer&&!peer.destroyed&&!frozen)peer.write(JSON.stringify(state)+'\n');},30);
  t.after(()=>{clearInterval(timer);app.close();peer?.destroy();robot.close();});
  const base=`http://127.0.0.1:${port}`;
  const lan=Object.values(os.networkInterfaces()).flat().find(x=>x.family==='IPv4'&&!x.internal)?.address;
  assert(lan,'LAN interface required for permission test');
  const status=async client=>(await (await fetch(base+'/api/status',{headers:{'X-Control-Token':client?.token||''}})).json());
  async function client(name,admin=false,cookie=''){
    const host=(admin?'127.0.0.1':lan)+':'+port;
    const response=await fetch(base+'/api/session',{headers:{Host:host,Cookie:cookie}}),s=await response.json();
    const c={...s,host,seq:0,lease:null,cookie:response.headers['set-cookie']?.[0].split(';')[0]||cookie};
    c.post=async(body,url='/api/command')=>{
      const res=await fetch(base+url,{method:'POST',headers:{Host:host,Origin:'http://'+host,'X-Control-Token':c.token,'Content-Type':'application/json'},body:JSON.stringify({seq:++c.seq,lease:c.lease,...body})});
      const data=await res.json();if(data.room?.lease)c.lease=data.room.lease;return {code:res.status,...data};
    };
    c.sync=async()=>{const s=await status(c);c.lease=s.room.lease;return s;};
    if(name)await c.post({action:'join',name},'/api/room');return c;
  }
  await delay(550);
  return {app,base,port,lan,state,received,client,status,freeze:()=>{frozen=true;}};
}
test('two simultaneous claims grant exactly one lease; observer drives and forged admin are rejected',async t=>{
  const f=await fixture(t),a=await f.client('小林'),b=await f.client('小周');
  const results=await Promise.all([a.post({command:'claim'}),b.post({command:'claim'})]);
  assert.deepEqual(results.map(x=>x.code).sort(),[200,409]);
  const winner=results[0].code===200?a:b,other=winner===a?b:a;
  assert.equal((await other.post({command:'drive',linear:1,angular:0,speed:.2,turn:.2})).code,409);
  assert.equal((await other.post({action:'takeover',admin:true},'/api/room')).code,403);
  const forged=await(await fetch(`http://${f.lan}:${f.port}/api/session`,{headers:{Host:'127.0.0.1:'+f.port}})).json();
  assert.equal(forged.room.self.admin,false,'a forged Host is not local administrator authority');
  const publicState=await f.status(other);
  assert.equal(publicState.room.lease,null);assert(!JSON.stringify(publicState).includes(winner.token));
});
test('FIFO handoff waits for actual stationary telemetry and explicit confirmation; old lease is fenced',async t=>{
  const f=await fixture(t),a=await f.client('先操作'),b=await f.client('第二位'),c=await f.client('第三位');
  assert.equal((await a.post({command:'claim'})).code,200);const oldLease=a.lease;
  await b.post({action:'queue'},'/api/room');await c.post({action:'queue'},'/api/room');
  Object.assign(f.state,{feedback_v:.2,paused:false});
  await a.post({action:'release'},'/api/room');await delay(550);
  assert.equal((await b.post({command:'claim'})).code,409,'no transfer while actual wheels still move');
  assert.equal((await a.post({command:'w',lease:oldLease})).code,409);
  Object.assign(f.state,{feedback_v:0,paused:true});await delay(500);
  const waiting=await f.status(b);assert.equal(waiting.room.owner,null);assert.equal(waiting.room.offer.id,b.room.self.id);
  assert.equal((await c.post({command:'claim'})).code,409);
  assert.equal((await b.post({command:'claim'})).code,200);
  assert.equal((await a.post({command:'w',lease:oldLease})).code,409);
  assert(!f.received.includes('w'));
});
test('any visitor can stop; stop rotates the lease and blocks already in-flight movement',async t=>{
  const f=await fixture(t),a=await f.client('驾驶员'),b=await f.client('观察员');
  await a.post({command:'claim'});const oldLease=a.lease;
  assert.equal((await b.post({command:'stop'})).code,200);
  assert.equal((await a.post({command:'drive',lease:oldLease,linear:1,angular:0,speed:.2,turn:.2})).code,409);
  await a.sync();assert.equal((await a.post({command:'heartbeat'})).code,200);
  assert.equal((await a.post({command:'resume'})).code,409,'stop barrier also fences the same owner');
  await delay(500);await a.sync();
  assert.equal((await a.post({command:'heartbeat'})).code,200);
  assert.equal((await a.post({command:'manual'})).code,200);
  assert((await f.status(a)).room.events.some(x=>x.text==='观察员 点击停车'));
});
test('heartbeat loss releases only after stopping; cached telemetry is never a stop confirmation',async t=>{
  const f=await fixture(t),a=await f.client('离线用户'),b=await f.client('下一位');
  await a.post({command:'claim'});await b.post({action:'queue'},'/api/room');f.freeze();
  await delay(1550);const s=await f.status(b);
  assert.equal(s.room.owner,null);assert.equal(s.room.stopping,true);
  assert.equal((await b.post({command:'claim'})).code,409);
  assert.equal((await a.post({command:'heartbeat'})).code,409);
  assert(f.received.includes('stop'));
});
test('local admin takes over only after stopping, invalidating old owner without starting motion',async t=>{
  const f=await fixture(t),a=await f.client('同伴'),admin=await f.client('管理员',true);
  assert.equal(admin.room.self.admin,true);await a.post({command:'claim'});
  await admin.post({action:'takeover'},'/api/room');
  assert.equal((await a.post({command:'drive',linear:1,angular:0,speed:.2,turn:.2})).code,409);
  assert.equal((await f.status(admin)).room.mine,false);await delay(500);
  assert.equal((await admin.sync()).room.mine,true);
  assert(!f.received.some(c=>c.startsWith('drive ')||c==='start'));
});
test('origin and host validation cover the LAN interface as well as loopback',async t=>{
  const f=await fixture(t),a=await f.client('同伴');
  const hostile=await fetch(f.base+'/api/room',{method:'POST',headers:{Host:a.host,Origin:'http://evil.example','X-Control-Token':a.token,'Content-Type':'application/json'},body:JSON.stringify({seq:99,action:'join',name:'伪造'})});
  assert.equal(hostile.status,403);
  assert.equal((await fetch(f.base+'/api/session',{headers:{Host:'evil.example:'+f.port}})).status,403);
});
test('LAN devices can claim immediately, keep device identity across tabs, and never inherit admin',async t=>{
  const f=await fixture(t),a=await f.client(),b=await f.client();
  assert.match(a.room.self.name,/同伴 [0-9A-F]{6}/);assert.equal(a.room.self.admin,false);assert.equal(b.room.self.admin,false);
  assert.equal(a.room.self.deviceId,b.room.self.deviceId,'requests from the same test computer share one display identity');
  const tab=await f.client(undefined,false,a.cookie);
  assert.equal(tab.room.self.deviceId,a.room.self.deviceId);assert.notEqual(tab.token,a.token);assert.notEqual(tab.room.self.id,a.room.self.id);
  assert.equal((await a.post({command:'claim'})).code,200,'no nickname ceremony is required');
  assert.equal((await tab.post({command:'claim'})).code,409,'another tab on the same device cannot drive concurrently');
  await b.post({action:'queue'},'/api/room');await a.post({action:'release'},'/api/room');await delay(500);
  assert.equal((await b.post({command:'claim'})).code,200,'another LAN device can accept the handoff');
  const admin=await f.client(undefined,true);
  const cloned=await f.client(undefined,false,admin.cookie);assert.equal(cloned.room.self.admin,false,'device identity never grants server authority');
});
