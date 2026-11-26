const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const {JSDOM}=require('jsdom');
const {classifyGround}=require('./point-cloud/ground-filter.js');
test('continuous sloped ground is hidden separately from low obstacles, ceilings and unsupported platforms',()=>{
  const pts=[];
  for(let x=-2;x<=2;x+=.24)for(let y=-2;y<=2;y+=.24)pts.push([x,y,-.22+.04*x]);
  const groundCount=pts.length;pts.push([.02,.02,-.14],[1,0,2.9],[7,6,-.18]);
  const xyz=Float32Array.from(pts.flat()),original=xyz.slice(),classes=new Uint8Array(pts.length);
  classifyGround(xyz,pts.length,[0,0,0],classes);
  assert.ok(classes.subarray(0,groundCount).filter(v=>v===1).length>groundCount*.85);
  assert.deepEqual([...classes.slice(-3)],[2,4,0]);assert.deepEqual(xyz,original);
});
test('maze adds map controls inside the original console and keeps the same scene, dock, telemetry and inspector',()=>{
  const dom=new JSDOM(fs.readFileSync(path.join(__dirname,'index.html'),'utf8'),{runScripts:'outside-only'});
  const w=dom.window,d=w.document;
  try{
    w.ResizeObserver=class {observe(){}};
    w.HTMLCanvasElement.prototype.getContext=()=>null;
    const intervals=[];
    w.setInterval=(_,ms)=>{intervals.push(ms);return 0;};w.fetch=()=>new Promise(()=>{});
    w.eval(fs.readFileSync(path.join(__dirname,'point-cloud/view-controls.js'),'utf8'));
    w.eval(fs.readFileSync(path.join(__dirname,'../迷宫/view.js'),'utf8'));
    const exp=d.querySelector('#experiment'),scene=d.querySelector('.scene'),shell=d.querySelector('.workspace');
    const shellClass=shell.className;
    const shared=['.inspector','.scene-top','.scene-telemetry','.scene-bottom','.manual-panel','#stop','#taskProgress'].map(selector=>d.querySelector(selector));
    assert.ok(d.querySelector('#sceneMapTabs [data-surface="grid"]').hidden);
    for(const id of ['lab1','lab2','lab2-eight','lab3','lab3-xy']){
      exp.value=id;exp.dispatchEvent(new w.Event('change'));
      d.querySelector('#sceneMapTabs [data-surface="map"]').click();
      assert.equal(scene.dataset.surface,'map');assert.equal(d.querySelector('#cloudMapFit').hidden,false);
      d.querySelector('#sceneSettings [data-cloud-source="overlay"]').click();
      assert.equal(scene.dataset.surface,'overlay');
      assert.equal(d.querySelector('#sceneMapTabs [data-surface="overlay"]').getAttribute('aria-pressed'),'true');
      assert.ok(d.querySelector('#sceneMapTabs [data-surface="grid"]').hidden);
      d.querySelector('#sceneMapTabs [data-surface="live"]').click();
    }
    exp.value='maze';exp.dispatchEvent(new w.Event('change'));
    assert.equal(scene.dataset.surface,'live');assert.ok(d.querySelector('#mazeAtlas').hidden);
    assert.equal(shell.className,shellClass);assert.equal(d.querySelector('.task-panel #sceneMapTabs').hidden,false);
    assert.equal(d.querySelector('#cloudCanvas').nextElementSibling.id,'mazeAtlas');
    assert.equal(d.querySelector('#mazeGoals'),null);
    w.MazeView.status({},true,true);
    assert.equal(d.querySelector('[data-command="start"]').textContent,'准备漫游');
    assert.equal(d.querySelector('#mazePick'),null);
    assert.ok(d.querySelector('.view-dock #viewTop'));assert.ok(d.querySelector('.view-dock #view3d'));
    d.querySelector('#sceneMapTabs [data-surface="map"]').click();assert.equal(scene.dataset.surface,'map');
    assert.ok(d.querySelector('#mazeAtlas').hidden);
    d.querySelector('#sceneMapTabs [data-surface="grid"]').click();assert.equal(scene.dataset.surface,'grid');
    assert.ok(!d.querySelector('#mazeAtlas').hidden);
    d.querySelector('#sceneMapTabs [data-surface="live"]').click();assert.equal(scene.dataset.surface,'live');
    assert.ok(d.querySelector('#mazeAtlas').hidden);assert.equal(scene.querySelector('h1').textContent,'实时点云');
    assert.deepEqual(Object.keys(w.MazeView.options()),['mode']);assert.equal(w.MazeView.options().mode,'roam');
    exp.value='lab3-xy';exp.dispatchEvent(new w.Event('change'));
    assert.ok(d.querySelector('#sceneMapTabs [data-surface="grid"]').hidden);
    assert.equal(scene.dataset.surface,'live');
    assert.equal(shell.className,shellClass);assert.ok(shared.every(node=>shell.contains(node)&&!node.hidden));
    assert.deepEqual(intervals,[1000]);
  }finally{w.close();}
});
