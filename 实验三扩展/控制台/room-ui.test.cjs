const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {JSDOM}=require('jsdom');
const {ControlRoom}=require('./control-room.cjs');
function page(t){
  const dom=new JSDOM(fs.readFileSync(path.join(__dirname,'index.html'),'utf8'),{runScripts:'outside-only',url:'http://127.0.0.1:8765'});
  const w=dom.window;w.matchMedia=()=>({matches:false});w.HTMLDialogElement.prototype.showModal=function(){this.open=true;};w.HTMLDialogElement.prototype.close=function(){this.open=false;};
  w.eval(fs.readFileSync(path.join(__dirname,'room.js'),'utf8'));t.after(()=>w.close());return {w,d:w.document};
}
const initial=()=>({self:{id:'me',name:'小林',admin:false},mine:false,lease:null,owner:null,stopping:false,queue:[],offer:null,people:[{id:'me',name:'小林'}],events:[],serverTime:100000});
test('visitor join uses nickname only, and the dialog reports errors without closing',async t=>{
  const {w,d}=page(t);let payload;
  w.RoomUI.init({online:()=>true,message:()=>{},act:async(action,data)=>{payload={action,...data};return {room:initial()};}},{urls:[],room:{self:{name:''}}});
  assert.equal(d.getElementById('joinDialog').open,true);d.getElementById('nickname').value='小林';
  await d.getElementById('joinForm').onsubmit({preventDefault(){}});
  assert.equal(payload.action,'join');assert.equal(payload.name,'小林');assert.equal(d.getElementById('joinDialog').open,false);
});
test('occupied, queue offer, stop barrier and administrator are distinct visible states',t=>{
  const {w,d}=page(t),s=initial();w.RoomUI.render(s,true);
  assert.equal(d.getElementById('claim').disabled,false);assert.equal(d.getElementById('takeoverControl').hidden,true);
  s.owner={id:'other',name:'小周',since:40000};w.RoomUI.render(s,true);
  assert.equal(d.getElementById('claim').hidden,true);assert.equal(d.getElementById('queueControl').hidden,false);assert.match(d.getElementById('ownerName').textContent,/小周/);
  s.queue=[s.self];w.RoomUI.render(s,true);assert.match(d.getElementById('queueControl').textContent,/取消排队/);
  s.owner=null;s.offer={id:'me',until:120000};w.RoomUI.render(s,true);assert.match(d.getElementById('claim').textContent,/确认接手/);
  s.stopping=true;s.stopReason='等待停车';w.RoomUI.render(s,true);assert.equal(d.getElementById('claim').disabled,true);
  s.self.admin=true;s.stopping=false;s.owner={id:'other',name:'小周',since:40000};w.RoomUI.render(s,true);assert.equal(d.getElementById('takeoverControl').hidden,false);
});
test('nicknames and activity are rendered as text, never executable markup',t=>{
  const {w,d}=page(t),s=initial();s.people=[{id:'x',name:'<img src=x onerror=alert(1)>'}];s.events=[{id:1,at:0,text:'<script>alert(1)</script>'}];w.RoomUI.render(s,true);
  assert.equal(d.querySelector('#peopleList img'),null);assert.equal(d.querySelector('#activityList script'),null);assert.match(d.getElementById('peopleList').textContent,/<img/);
});
test('an automatically identified LAN device sees claim immediately without a blocked nickname dialog',t=>{
  const {w,d}=page(t),s=initial();s.self.name='同伴 A1B2C3';s.self.deviceId='a1b2c3abcdef';
  w.RoomUI.init({online:()=>true,message:()=>{},act:async()=>({room:s})},{room:s});
  assert.equal(d.getElementById('joinDialog').open,false);assert.equal(d.getElementById('claim').disabled,false);
  assert(!d.getElementById('identityBadge').textContent.includes('管理员'));
  d.getElementById('identityBadge').click();assert.equal(d.getElementById('joinDialog').open,true);assert.equal(d.getElementById('nickname').value,'同伴 A1B2C3');
  d.getElementById('joinCancel').click();assert.equal(d.getElementById('joinDialog').open,false);
});
test('absent queue head is skipped, and confirmation expires without automatically starting a robot',()=>{
  let now=1000,stops=0;
  const room=new ControlRoom({stop:()=>stops++,stopped:()=>true,connected:()=>true,now:()=>now});
  const a=room.session(false),b=room.session(false);room.join(a,'甲');room.join(b,'乙');room.enqueue(a);room.enqueue(b);
  now+=6600;room.touch(b);room.tick();assert.deepEqual(room.queue,[b]);assert.equal(room.offer.token,b);
  now+=30001;room.touch(b);room.tick();assert.equal(room.queue.length,0);assert.equal(room.owner,null);assert.equal(stops,0);
});
test('online list counts devices once while each page retains its own control token',()=>{
  const room=new ControlRoom({stop:()=>{},stopped:()=>true,connected:()=>true});
  const admin=room.session(true,'server-computer'),localGuest=room.session(false,'server-computer');
  const peer=room.session(false,'lan-computer:10.0.0.2'),peerTab=room.session(false,'lan-computer:10.0.0.2');
  const state=room.snapshot(peer);
  assert.equal(state.people.length,2);assert.equal(state.people.filter(p=>p.admin).length,1);
  assert.equal(room.person(peer).deviceId,room.person(peerTab).deviceId);assert.notEqual(peer,peerTab);
  assert.notEqual(room.person(localGuest).admin,room.person(admin).admin);
  room.claim(peer);assert.throws(()=>room.claim(peerTab),/正在使用/);
});
