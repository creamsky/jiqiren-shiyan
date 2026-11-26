'use strict';
(() => {
  const scene=document.querySelector('.scene'),experiment=document.getElementById('experiment');
  const caption=scene.querySelector('h1'),captionType=scene.querySelector('.scene-caption .eyebrow');
  const hint=document.getElementById('cloudSourceHint'),fit=document.getElementById('cloudMapFit');
  const buttons=document.querySelectorAll('[data-surface], [data-cloud-source]');
  const gridTab=document.querySelector('button[data-surface="grid"]');
  const views={
    live:['实时点云','LIVE POINT CLOUD','最新配准扫描'],
    map:['累计点云','SLAM POINT CLOUD','当前定位地图中的累计点云'],
    overlay:['点云叠加','LIVE + SLAM','紫色点为实时扫描，底图为累计点云'],
    aggregate:['一秒聚合点云','AGGREGATED POINT CLOUD','最近一秒的配准扫描预览 · 最多 4096 点'],
    raw:['原始点云','RAW POINT CLOUD','雷达坐标系单帧预览 · 最多 4096 点'],
    grid:['迷宫通行图','MAZE MAP',''],
  };
  let maze=experiment.value==='maze';

  function select(name){
    if(name==='grid'&&!maze)return;
    const [title,type,help]=views[name];
    const changed=scene.dataset.surface!==name;
    scene.dataset.surface=name;
    caption.textContent=title;captionType.textContent=type;
    scene.setAttribute('aria-label',name==='grid'?'实车迷宫通行图':'三维点云与定位轨迹');
    if(help)hint.textContent=help;
    fit.hidden=!['map','overlay','grid'].includes(name);
    for(const button of buttons){
      const active=(button.dataset.surface||button.dataset.cloudSource)===name;
      button.classList.toggle('active',active);
      button.setAttribute('aria-pressed',String(active));
    }
    if(changed)document.dispatchEvent(new CustomEvent('cloud-view-change',{detail:name}));
  }

  for(const button of buttons)button.addEventListener('click',()=>select(button.dataset.surface||button.dataset.cloudSource));
  experiment.addEventListener('change',()=>{
    const active=experiment.value==='maze';
    gridTab.hidden=!active;
    if(active!==maze){maze=active;select('live');}
  });
  gridTab.hidden=!maze;
  select('live');
  window.PointCloudView={select};
})();
