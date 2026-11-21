const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const path=require('node:path');

function page(){
 const listeners={},windowListeners={},calls=[],elements=new Map();
 const element=id=>{if(!elements.has(id))elements.set(id,{value:({experiment:'lab2',manualSpeed:'1.5',manualTurn:'0.3'})[id]||'',textContent:'',classList:{remove(){},toggle(){}}});return elements.get(id);};
 const context=vm.createContext({
  document:{getElementById:element,querySelectorAll:()=>[],addEventListener:(name,fn)=>listeners[name]=fn,hidden:false,hasFocus:()=>true},
  window:{addEventListener:(name,fn)=>windowListeners[name]=fn,Workbench:{options:()=>({})}},setInterval(){},AbortSignal,console,
  fetch:(url,options)=>{calls.push({url,body:options?.body&&JSON.parse(options.body)});return new Promise(()=>{});}
 });
 vm.runInContext(fs.readFileSync(path.join(__dirname,'app.js'),'utf8'),context);
 vm.runInContext('mine=true;online=true;manual=true;vectorSupported=true;',context);
 return {listeners,windowListeners,calls,element,context};
}

test('armed global WASD works while the experiment select has focus',async()=>{
 const p=page();let prevented=false;
 p.listeners.keydown({key:'w',repeat:false,target:{matches:selector=>selector.includes('select')},preventDefault:()=>prevented=true});
 await new Promise(setImmediate);
 assert(prevented,'select type-ahead must not swallow driving keys');
 assert(p.calls.some(call=>call.body?.command==='drive'&&call.body.linear===1&&call.body.angular===0),'W must reach the command chain');
});

test('typing in an input and browser shortcuts never start movement',async()=>{
 const p=page();
 p.listeners.keydown({key:'w',repeat:false,target:{matches:()=>true},preventDefault(){}});
 p.listeners.keydown({key:'w',repeat:false,ctrlKey:true,target:{matches:()=>false},preventDefault(){}});
 await new Promise(setImmediate);
 assert(!p.calls.some(call=>call.body?.command==='drive'));
});

const down=(p,key)=>p.listeners.keydown({key,repeat:false,target:{matches:()=>false},preventDefault(){}});
for(const [forward,turn,v,w] of [['w','a',1,1],['w','d',1,-1],['s','a',-1,1],['s','d',-1,-1]]){
 test(`${forward.toUpperCase()}+${turn.toUpperCase()} controls both axes and releasing one preserves the other`,async()=>{
  const p=page();down(p,forward);down(p,turn);
  assert.deepEqual(p.calls.at(-1).body.linear,v);assert.equal(p.calls.at(-1).body.angular,w);
  p.listeners.keyup({key:forward});assert.equal(p.calls.at(-1).body.linear,0);assert.equal(p.calls.at(-1).body.angular,w);
  down(p,forward);p.listeners.keyup({key:turn});assert.equal(p.calls.at(-1).body.linear,v);assert.equal(p.calls.at(-1).body.angular,0);
  p.listeners.keyup({key:forward});await new Promise(setImmediate);assert.equal(p.calls.at(-1).body.command,'zero');
 });
}
test('opposing keys cancel only their own axis; focus loss clears held input',async()=>{
 const p=page();down(p,'w');down(p,'a');down(p,'s');
 assert.equal(p.calls.at(-1).body.linear,0);assert.equal(p.calls.at(-1).body.angular,1);
 p.windowListeners.blur();await new Promise(setImmediate);
 assert(p.calls.some(x=>x.body?.command==='zero'));assert.equal(vm.runInContext('held.size',p.context),0);
 assert.equal(vm.runInContext('mine',p.context),false);
});
test('opening experiment library or editing parameters suspends motion',async()=>{
 const p=page();down(p,'w');down(p,'d');p.listeners['control-suspend']();await new Promise(setImmediate);
 assert.equal(vm.runInContext('held.size',p.context),0);assert(p.calls.some(x=>x.body?.command==='zero'));
});
test('older robot never receives an unsupported diagonal command',async()=>{
 const p=page();vm.runInContext('vectorSupported=false;',p.context);down(p,'w');down(p,'a');await new Promise(setImmediate);
 assert(p.calls.some(x=>x.body?.command==='zero'));assert(!p.calls.some(x=>x.body?.command==='drive'));
 assert.match(p.element('message').textContent,/更新/);
});
