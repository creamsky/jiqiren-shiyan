const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {JSDOM}=require('jsdom');
const {classifyMazeCloud}=require('../迷宫/view.js');
test('continuous sloped ground is hidden separately from low obstacles, ceilings and unsupported platforms',()=>{
  const pts=[];
  for(let x=-2;x<=2;x+=.24)for(let y=-2;y<=2;y+=.24)pts.push([x,y,-.22+.04*x]);
  const groundCount=pts.length;pts.push([.02,.02,-.14],[1,0,2.9],[7,6,-.18]);
  const xyz=Float32Array.from(pts.flat()),original=xyz.slice(),classes=new Uint8Array(pts.length);
  classifyMazeCloud(xyz,pts.length,[0,0,0],classes);
  assert.ok(classes.subarray(0,groundCount).filter(v=>v===1).length>groundCount*.85);
  assert.deepEqual([...classes.slice(-3)],[2,4,0]);assert.deepEqual(xyz,original);
});
test('maze opens its operation workspace, retains both point clouds, and restores the previous console layout on exit',()=>{
  const dom=new JSDOM(fs.readFileSync(path.join(__dirname,'index.html'),'utf8'),{runScripts:'outside-only'});
  const w=dom.window,d=w.document;
  try{
    w.ResizeObserver=class {observe(){}};
    w.HTMLCanvasElement.prototype.getContext=()=>null;
    const intervals=[];
    w.setInterval=(_,ms)=>{intervals.push(ms);return 0;};w.fetch=()=>new Promise(()=>{});
    w.eval(fs.readFileSync(path.join(__dirname,'../迷宫/view.js'),'utf8'));
    const exp=d.querySelector('#experiment'),scene=d.querySelector('.scene');
    exp.value='maze';exp.dispatchEvent(new w.Event('change'));
    assert.equal(scene.dataset.surface,'map');assert.ok(d.querySelector('#mazeAtlas').hidden);
    assert.ok(d.querySelector('.workspace').classList.contains('maze-workspace'));
    assert.ok(!d.querySelector('#mazeGoals').hidden);
    assert.equal(d.querySelector('#mazeAdvanced').open,false);
    assert.equal(d.querySelector('#mazeGuide').open,false);
    w.MazeView.status({},true,true);
    assert.equal(d.querySelector('[data-command="start"]').textContent,'准备迷宫');
    assert.equal(d.querySelector('#mazePick').disabled,true);
    assert.ok(d.querySelector('.view-dock #viewTop'));assert.ok(d.querySelector('.view-dock #view3d'));
    d.querySelector('[data-surface="map"]').click();assert.equal(scene.dataset.surface,'map');
    assert.ok(d.querySelector('#mazeAtlas').hidden);
    d.querySelector('[data-surface="grid"]').click();assert.equal(scene.dataset.surface,'grid');
    assert.ok(!d.querySelector('#mazeAtlas').hidden);
    d.querySelector('[data-surface="live"]').click();assert.equal(scene.dataset.surface,'live');
    assert.ok(d.querySelector('#mazeAtlas').hidden);assert.equal(scene.querySelector('h1').textContent,'迷宫探索');
    exp.value='lab3-xy';exp.dispatchEvent(new w.Event('change'));
    assert.ok(d.querySelector('[data-surface="grid"]').hidden);
    assert.equal(scene.dataset.surface,'live');assert.ok(d.querySelector('#mazeGoals').hidden);
    assert.ok(!d.querySelector('.workspace').classList.contains('maze-workspace'));
    assert.deepEqual(intervals,[1000]);
  }finally{w.close();}
});
