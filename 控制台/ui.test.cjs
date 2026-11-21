const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {JSDOM}=require('jsdom');
function page(){
  const dom=new JSDOM(fs.readFileSync(path.join(__dirname,'index.html'),'utf8'),{runScripts:'outside-only',url:'http://127.0.0.1:8767'});
  const {window}=dom;window.HTMLDialogElement.prototype.showModal=function(){this.open=true;};window.HTMLDialogElement.prototype.close=function(){this.open=false;};
  window.eval(fs.readFileSync(path.join(__dirname,'ui.js'),'utf8'));
  return {dom,window,document:window.document,ui:window.Workbench};
}
const registry=JSON.parse(fs.readFileSync(path.join(__dirname,'experiments.json'),'utf8'));
test('experiment library scales past three tasks, searches, filters and selects config-driven entries',()=>{
  const {dom,window,document:d,ui}=page();
  try{
    const future=Array.from({length:30},(_,n)=>({id:'future-'+n,title:'实验 '+n,name:'路径规划 '+n,description:'后续扩展',category:'规划',icon:'grid',enabled:true}));
    ui.load([...registry,...future]);d.getElementById('experimentPicker').click();assert.equal(d.querySelectorAll('.library-item').length,registry.length+future.length);
    const search=d.getElementById('experimentSearch');search.value='路径规划 29';search.dispatchEvent(new window.Event('input'));
    assert.equal(d.querySelectorAll('.library-item').length,1);d.querySelector('.library-item').click();assert.equal(ui.selected().id,'future-29');assert.equal(d.getElementById('experimentLibrary').open,false);
    d.getElementById('experimentPicker').click();Array.from(d.querySelectorAll('#experimentFilters button')).find(b=>b.textContent==='运动控制').click();assert.equal(d.querySelectorAll('.library-item').length,2);
    search.value='不存在';search.dispatchEvent(new window.Event('input'));assert.match(d.querySelector('.no-results').textContent,/没有匹配/);
  }finally{dom.window.close();}
});
test('numeric editing keeps slider, steps, bounds and experiment parameters consistent',()=>{
 const {dom,window,document:d,ui}=page();try{
  ui.load(registry);d.getElementById('experimentPicker').click();d.querySelectorAll('.library-item')[2].click();
  assert.equal(Number(d.getElementById('manualSpeed').value),1);assert.equal(Number(d.getElementById('manualSpeed').max),1.5);
  assert.equal(Number(d.getElementById('manualTurn').value),.3);assert.equal(Number(d.getElementById('manualTurn').max),.5);
  const radius=d.getElementById('param-radius');assert.equal(ui.options().radius,2);
  d.querySelector('[data-step="param-radius"][data-delta="0.1"]').click();assert.equal(ui.options().radius,2.1);
  assert.equal(d.querySelector('[data-sync="param-radius"]').value,'2.1');
  radius.value='100';radius.dispatchEvent(new window.Event('change',{bubbles:true}));assert.equal(ui.options().radius,10);
  radius.value='';radius.dispatchEvent(new window.Event('change',{bubbles:true}));assert.equal(ui.options().radius,10);
  const slider=d.querySelector('[data-sync="param-radius"]');slider.value='3.4';slider.dispatchEvent(new window.Event('input',{bubbles:true}));assert.equal(ui.options().radius,3.4);
  d.querySelectorAll('.parameter-choice button')[1].click();assert.equal(ui.options().startPoint,'outer');
  ui.lock(true);assert.equal(radius.disabled,true);assert.equal(d.getElementById('experimentPicker').disabled,true);
  ui.lock(false);d.getElementById('experimentPicker').click();d.querySelectorAll('.library-item')[0].click();d.getElementById('experimentPicker').click();d.querySelectorAll('.library-item')[2].click();assert.equal(ui.options().radius,3.4);assert.equal(ui.options().startPoint,'outer');
 }finally{dom.window.close();}
});
test('new markup and live status script cooperate without missing IDs or offline controls',async()=>{
 const {dom,window,document:d}=page();try{
  let status={connected:false,age:null,status:{},owned:false,task:null,preparedExperiment:null};
  window.AbortSignal=AbortSignal;window.setInterval=()=>0;
  window.fetch=async url=>({ok:true,json:async()=>url==='/api/experiments'?{experiments:registry}:url==='/api/session'?{token:'fixture'}:url==='/api/ping'?{ok:true,output:'来自 10.107.60.72 的回复: 时间<1ms TTL=64'}:status});
  window.eval(fs.readFileSync(path.join(__dirname,'app.js'),'utf8'));await new Promise(setImmediate);
  assert.equal(d.getElementById('connection').textContent,'真实机器人 · 离线');assert.equal(d.querySelector('[data-command="start"]').disabled,true);
  d.getElementById('ping').click();await new Promise(setImmediate);assert.match(d.getElementById('pingState').textContent,/网络可达.*<1 ms/);
  assert(d.querySelector('.scene #v'));assert(!d.querySelector('.inspector #v'));
  assert.equal(d.querySelectorAll('select').length,0,'No native select dropdowns remain');
  status={...status,connected:true,age:1,status:{mode:'manual',manual_vector:true,stage:'WAITING',v:0,w:0}};
  await window.eval('refresh()');assert.equal(d.getElementById('connection').textContent,'真实机器人 · 在线');
 }finally{dom.window.close();}
});
