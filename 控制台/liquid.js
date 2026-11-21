'use strict';
// Refraction, not a frosted overlay: sample the live backdrop through a convex edge.
(()=>{
  const themeButton=document.getElementById('themeToggle');
  function setTheme(theme){
    document.documentElement.dataset.theme=theme;
    themeButton.setAttribute('aria-label',theme==='dark'?'切换到浅色':'切换到深色');themeButton.title=themeButton.getAttribute('aria-label');
    document.dispatchEvent(new Event('themechange'));
    try{localStorage.setItem('robot-theme',theme);}catch{}
  }
  let theme='dark';try{theme=localStorage.getItem('robot-theme')==='light'?'light':'dark';}catch{}setTheme(theme);
  let themeTransition=null;
  themeButton.onclick=()=>{
    const change=()=>setTheme(document.documentElement.dataset.theme==='dark'?'light':'dark');
    if(document.startViewTransition&&!matchMedia('(prefers-reduced-motion: reduce)').matches){
      themeTransition?.skipTransition();
      const transition=themeTransition=document.startViewTransition(change);
      transition.ready.catch(()=>{});
      transition.finished.catch(()=>{}).finally(()=>{if(themeTransition===transition)themeTransition=null;});
    }else change();
  };
  window.addEventListener('storage',event=>{
    if(event.key==='robot-theme')setTheme(event.newValue==='light'?'light':'dark');
  });
  const reduced=matchMedia('(prefers-reduced-motion: reduce)'),opaque=matchMedia('(prefers-reduced-transparency: reduce)');
  window.GlassMotion={anchor(panel,button){
    const bar=(button.closest('.scene-tools')||button).getBoundingClientRect(),b=button.getBoundingClientRect();
    const width=Math.min(panel.classList.contains('library')?500:320,innerWidth-32);panel.style.width=width+'px';
    const left=Math.max(16,Math.min(innerWidth-width-16,bar.right-width)),top=Math.min(bar.bottom+12,innerHeight-panel.offsetHeight-16);
    panel.dataset.anchored='true';panel.style.position='fixed';panel.style.margin='0';panel.style.right='auto';
    panel.style.left=left+'px';panel.style.top=Math.max(16,top)+'px';
    panel.style.transformOrigin=(b.left+b.width/2-left)+'px '+(b.top+b.height/2-Math.max(16,top))+'px';
    if(!reduced.matches)panel.animate([{opacity:0,transform:'scale(.72)'},{opacity:1,transform:'scale(1.012)',offset:.72},{opacity:1,transform:'scale(1)'}],{duration:380,easing:'cubic-bezier(.2,.8,.25,1)'});
  },close(dialog){
    if(!dialog.open||dialog.dataset.closing)return;
    if(reduced.matches){dialog.close();return;}
    dialog.dataset.closing='true';
    dialog.animate([{opacity:1,transform:'scale(1)'},{opacity:0,transform:dialog.dataset.anchored?'scale(.8)':'translateY(5px) scale(.97)'}],{duration:170,easing:'cubic-bezier(.4,0,1,1)'})
      .finished.catch(()=>{}).finally(()=>{dialog.close();delete dialog.dataset.closing;});
  }};
  document.querySelectorAll('dialog').forEach(dialog=>dialog.addEventListener('cancel',event=>{
    event.preventDefault();window.GlassMotion.close(dialog);
  }));
  function selectionLenses(){
    document.querySelectorAll('.segmented').forEach(group=>{
      const selected=group.querySelector('button.active,a.active');if(!selected)return;
      let lens=group.querySelector('.selection-lens');
      if(!lens){lens=document.createElement('span');lens.className='selection-lens';lens.setAttribute('aria-hidden','true');group.prepend(lens);group.classList.add('has-lens');}
      lens.style.width=selected.offsetWidth+'px';lens.style.transform='translateX('+selected.offsetLeft+'px)';
    });
  }
  let selectionFrame=0;
  const updateSelection=()=>{if(!selectionFrame)selectionFrame=requestAnimationFrame(()=>{selectionFrame=0;selectionLenses();});};
  document.addEventListener('click',updateSelection);document.addEventListener('change',updateSelection);window.addEventListener('resize',updateSelection);updateSelection();
  new MutationObserver(updateSelection).observe(document.body,{subtree:true,attributes:true,attributeFilter:['aria-pressed']});
  const ns='http://www.w3.org/2000/svg';
  const svg=document.createElementNS(ns,'svg');svg.setAttribute('class','liquid-filters');svg.setAttribute('aria-hidden','true');
  const defs=document.createElementNS(ns,'defs');svg.append(defs);document.body.append(svg);
  const node=(tag,attrs)=>{const n=document.createElementNS(ns,tag);for(const [k,v] of Object.entries(attrs))n.setAttribute(k,v);return n;};
  const chromium=/Chrome|Chromium|Edg\//.test(navigator.userAgent);
  document.querySelectorAll('.glass,.library').forEach((panel,index)=>{
    let x=40,y=0,tx=40,ty=0,alpha=0,targetAlpha=0,frame=0,lastSize='';
    function light(){
      x+=(tx-x)*.18;y+=(ty-y)*.18;alpha+=(targetAlpha-alpha)*.12;
      panel.style.setProperty('--light-x',x+'px');panel.style.setProperty('--light-y',y+'px');panel.style.setProperty('--light-alpha',String(alpha));
      if(Math.abs(tx-x)+Math.abs(ty-y)+Math.abs(targetAlpha-alpha)*100>.3)frame=requestAnimationFrame(light);else frame=0;
    }
    panel.addEventListener('pointermove',e=>{if(reduced.matches||e.pointerType==='touch')return;const r=panel.getBoundingClientRect();tx=e.clientX-r.left;ty=e.clientY-r.top;targetAlpha=.25;if(!frame)frame=requestAnimationFrame(light);});
    panel.addEventListener('pointerleave',()=>{targetAlpha=0;if(!frame)frame=requestAnimationFrame(light);});
    if(!chromium||opaque.matches)return;
    const id='liquid-lens-'+index,filter=node('filter',{id,x:'0%',y:'0%',width:'100%',height:'100%','color-interpolation-filters':'sRGB'});
    const map=node('feImage',{x:'0',y:'0',width:'100%',height:'100%',preserveAspectRatio:'none',result:'lens'});
    filter.append(map,node('feDisplacementMap',{in:'SourceGraphic',in2:'lens',scale:'22',xChannelSelector:'R',yChannelSelector:'G'}));defs.append(filter);
    function resize(){
      const w=Math.round(panel.clientWidth),h=Math.round(panel.clientHeight);if(!w||!h||w+'x'+h===lastSize)return;lastSize=w+'x'+h;
      const ratio=Math.min(1,600/Math.max(w,h)),cw=Math.max(1,Math.round(w*ratio)),ch=Math.max(1,Math.round(h*ratio));
      const c=document.createElement('canvas');c.width=cw;c.height=ch;const ctx=c.getContext('2d'),pixels=ctx.createImageData(cw,ch);
      const r=Math.min(parseFloat(getComputedStyle(panel).borderRadius)||22,w/2,h/2),edge=Math.min(19,r*.85);
      const sdf=(px,py)=>{const qx=Math.abs(px-w/2)-(w/2-r),qy=Math.abs(py-h/2)-(h/2-r);return Math.hypot(Math.max(qx,0),Math.max(qy,0))+Math.min(Math.max(qx,qy),0)-r;};
      for(let py=0;py<ch;py++)for(let px=0;px<cw;px++){
        const xx=px/ratio,yy=py/ratio,d=-sdf(xx,yy),i=(py*cw+px)*4;
        let dx=0,dy=0;
        if(d>=0&&d<edge){const strength=Math.sin(Math.PI*(1-d/edge))*.92;dx=(sdf(xx+.5,yy)-sdf(xx-.5,yy))*strength;dy=(sdf(xx,yy+.5)-sdf(xx,yy-.5))*strength;}
        pixels.data[i]=128+dx*126;pixels.data[i+1]=128+dy*126;pixels.data[i+2]=128;pixels.data[i+3]=255;
      }
      ctx.putImageData(pixels,0,0);map.setAttribute('href',c.toDataURL());
      const soften=panel.matches('.inspector,.library,.room-dialog,.scene-settings');
      panel.style.backdropFilter=`url(#${id})${soften?' blur(1pt)':''} saturate(115%)`;
    }
    new ResizeObserver(resize).observe(panel);resize();
  });
})();
