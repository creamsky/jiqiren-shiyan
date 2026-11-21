'use strict';
// Registry-driven experiment picker and shared numeric controls.
window.Workbench = (() => {
  const byId=id=>document.getElementById(id), dialog=byId('experimentLibrary');
  const closeLibrary=()=>window.GlassMotion?window.GlassMotion.close(dialog):dialog.close();
  let items=[],category='全部',locked=false;
  const saved=new Map();
  function el(tag,className,text){const node=document.createElement(tag);if(className)node.className=className;if(text!==undefined)node.textContent=text;return node;}
  function icon(name){const svg=document.createElementNS('http://www.w3.org/2000/svg','svg'),use=document.createElementNS(svg.namespaceURI,'use');use.setAttribute('href','#i-'+(['map','route','loop','grid'].includes(name)?name:'grid'));svg.append(use);return svg;}
  const selected=()=>items.find(item=>item.id===byId('experiment').value);
  function options(){const values={};for(const spec of selected()?.parameters||[]){const input=byId('param-'+spec.key);values[spec.key]=spec.type==='number'?Number(input.value):input.value;}return values;}
  function normalize(input,value){
    const min=Number(input.min),max=Number(input.max),step=Number(input.step)||1;
    const parsed=Number(value), fallback=Number(input.dataset.last||input.defaultValue||min);
    const safe=String(value).trim()!==''&&Number.isFinite(parsed)?parsed:fallback;
    return Number(Math.max(min,Math.min(max,min+Math.round((safe-min)/step)*step)).toFixed(6));
  }
  function commit(input,value){
    if(input.id==='goalX'||input.id==='goalY')return;
    input.value=normalize(input,value);input.dataset.last=input.value;
    document.querySelectorAll('[data-sync]').forEach(slider=>{if(slider.dataset.sync===input.id)slider.value=input.value;});
    input.dispatchEvent(new Event('input',{bubbles:true}));
  }
  function numeric(spec,value){
    const field=el('div','field'),id='param-'+spec.key,label=el('label','',spec.label);label.htmlFor=id;label.append(el('span','',spec.unit||''));field.append(label);
    const control=el('div','number-control'),input=el('input');Object.assign(input,{id,type:'number',min:spec.min,max:spec.max,step:spec.step,value,inputMode:'decimal'});input.defaultValue=String(spec.default);
    for(const sign of [-1,1]){const button=el('button','',sign<0?'−':'+');button.dataset.step=id;button.dataset.delta=String(sign*spec.step);button.setAttribute('aria-label',(sign<0?'减小':'增大')+spec.label);control.append(button);if(sign<0)control.append(input);}
    field.append(control);const slider=el('input');Object.assign(slider,{type:'range',min:spec.min,max:spec.max,step:spec.step,value});slider.dataset.sync=id;slider.setAttribute('aria-label',spec.label+'滑杆');field.append(slider);return field;
  }
  function renderParameters(item){
    const container=byId('taskParameters');container.replaceChildren();
    for(const spec of item.parameters||[]){const value=saved.get(item.id)?.[spec.key]??spec.default;
      if(spec.type==='number'){container.append(numeric(spec,value));continue;}
      if(spec.type!=='choice')continue;
      const field=el('div','field'),label=el('span','field-label',spec.label),group=el('div','segmented parameter-choice'),input=el('input');input.type='hidden';input.id='param-'+spec.key;input.value=value;group.setAttribute('role','group');group.setAttribute('aria-label',spec.label);
      for(const choice of spec.options){const b=el('button','',choice.label);b.classList.toggle('active',choice.value===value);b.setAttribute('aria-pressed',String(choice.value===value));b.onclick=()=>{input.value=choice.value;for(const other of group.querySelectorAll('button')){other.classList.toggle('active',other===b);other.setAttribute('aria-pressed',String(other===b));}};group.append(b);}
      field.append(label,input,group);container.append(field);
    }
  }
  function select(id){
    if(locked)return;
    const old=selected();if(old)saved.set(old.id,options());
    const item=items.find(item=>item.id===id);if(!item||!item.enabled)return;
    byId('experiment').value=id;byId('selectedGroup').textContent=item.title+' · '+(item.category||'其他');byId('selectedTitle').textContent=item.name||item.title;byId('selectedDescription').textContent=item.description;byId('selectedIcon').replaceChildren(icon(item.icon).firstChild);renderParameters(item);
    byId('experiment').dispatchEvent(new Event('change',{bubbles:true}));
  }
  function renderList(){
    const query=byId('experimentSearch').value.trim().toLowerCase();
    const matches=items.filter(item=>(category==='全部'||(item.category||'其他')===category)&&[item.id,item.title,item.name,item.description].join(' ').toLowerCase().includes(query));
    byId('experimentList').replaceChildren();
    for(const item of matches){const row=el('button','library-item');row.disabled=!item.enabled;row.setAttribute('aria-pressed',String(item.id===byId('experiment').value));
      const graphic=el('span','item-icon');graphic.append(icon(item.icon));const copy=el('span','item-copy'),title=el('span','item-title',item.name||item.title);title.append(el('small','',item.title));copy.append(title,el('span','item-description',item.description));row.append(graphic,copy,el('span','item-check',item.enabled?'✓':'—'));row.onclick=()=>{select(item.id);closeLibrary();};byId('experimentList').append(row);}
    if(!matches.length)byId('experimentList').append(el('p','no-results','没有匹配的实验，试试其他关键词'));
    byId('experimentCount').textContent=matches.length+' / '+items.length+' 个实验';
  }
  function load(registry){items=registry;const groups=['全部',...new Set(items.map(x=>x.category||'其他'))];byId('experimentFilters').replaceChildren();for(const name of groups){const b=el('button',name===category?'active':'',name);b.setAttribute('aria-pressed',String(name===category));b.onclick=()=>{category=name;for(const other of byId('experimentFilters').children){other.classList.toggle('active',other===b);other.setAttribute('aria-pressed',String(other===b));}renderList();};byId('experimentFilters').append(b);}select(items.find(x=>x.enabled)?.id);}
  byId('experimentPicker').onclick=()=>{if(locked)return;document.dispatchEvent(new Event('control-suspend'));byId('experimentSearch').value='';renderList();dialog.showModal();byId('experimentSearch').focus();};
  byId('libraryClose').onclick=closeLibrary;dialog.addEventListener('click',event=>{if(event.target===dialog){const r=dialog.getBoundingClientRect();if(event.clientX<r.left||event.clientX>r.right||event.clientY<r.top||event.clientY>r.bottom)closeLibrary();}});byId('experimentSearch').oninput=renderList;
  const closeSettings=()=>{const panel=byId('sceneSettings');byId('settingsToggle').setAttribute('aria-expanded','false');if(panel.animate&&!matchMedia('(prefers-reduced-motion: reduce)').matches){panel.animate([{opacity:1,transform:'scale(1)'},{opacity:0,transform:'scale(.8)'}],{duration:160,easing:'ease-in'}).finished.catch(()=>{}).finally(()=>{panel.hidden=true;});}else panel.hidden=true;};
  byId('settingsToggle').onclick=()=>{const panel=byId('sceneSettings');if(!panel.hidden){closeSettings();return;}panel.hidden=false;byId('settingsToggle').setAttribute('aria-expanded','true');window.GlassMotion?.anchor(panel,byId('settingsToggle'));};byId('settingsClose').onclick=()=>{closeSettings();byId('settingsToggle').focus();};
  document.addEventListener('keydown',event=>{if(event.key==='Escape'&&!byId('sceneSettings').hidden){byId('settingsClose').click();}});
  document.addEventListener('click',event=>{const b=event.target.closest('[data-step]');if(!b||b.disabled)return;const input=byId(b.dataset.step);if(!input||input.disabled)return;commit(input,Number(input.value)+Number(b.dataset.delta));});
  document.addEventListener('input',event=>{if(event.target.matches('input[data-sync]')){const input=byId(event.target.dataset.sync);if(!input.disabled)commit(input,event.target.value);}});
  document.addEventListener('change',event=>{if(event.target.matches('input[type=number]'))commit(event.target,event.target.value);});
  document.addEventListener('keydown',event=>{if(event.target.matches('input[type=number]')&&event.key==='Enter'){commit(event.target,event.target.value);event.target.blur();}});
  document.addEventListener('focusin',event=>{if(event.target.matches('input:not([type=hidden]),[contenteditable=true]'))document.dispatchEvent(new Event('control-suspend'));});
  return {load,selected,options,items:()=>items,syncTask(task){
    if(!task||!items.some(x=>x.id===task.experiment))return;
    if(selected()?.id!==task.experiment){const wasLocked=locked;locked=false;select(task.experiment);locked=wasLocked;}
    for(const [key,value] of Object.entries(task.options||{})){const input=byId('param-'+key);if(input)input.value=value;}
  },lock(value){locked=!!value;byId('experimentPicker').disabled=locked;if(locked&&dialog.open)dialog.close();byId('taskParameters').querySelectorAll('input,button').forEach(x=>x.disabled=locked);}};
})();
