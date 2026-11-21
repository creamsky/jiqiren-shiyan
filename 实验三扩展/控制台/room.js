'use strict';
window.RoomUI=(()=>{
  const $=id=>document.getElementById(id);
  let api,state,urls=[],signature='';
  const el=(tag,cls,text)=>{const n=document.createElement(tag);n.className=cls;n.textContent=text;return n;};
  const close=dialog=>window.GlassMotion?window.GlassMotion.close(dialog):dialog.close();
  const duration=ms=>{const s=Math.max(0,Math.floor(ms/1000));return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');};
  function init(callbacks,data){
    api=callbacks;urls=data.urls||[];
    if(data.room?.self?.name){render(data.room,api.online());let saved='';try{saved=localStorage.getItem('robot-nickname')||'';}catch{}if(saved&&saved!==data.room.self.name)act('join',{name:saved});}
    else{try{$('nickname').value=localStorage.getItem('robot-nickname')||'';}catch{}$('joinDialog').showModal();}
  }
  async function act(action,extra={}){try{const result=await api.act(action,extra);render(result.room,api.online());return result;}catch(error){api.message(error.message);return null;}}
  $('joinForm').onsubmit=async event=>{event.preventDefault();$('joinError').textContent='';try{
    const name=$('nickname').value.trim(),result=await api.act('join',{name});
    try{localStorage.setItem('robot-nickname',name);}catch{}
    close($('joinDialog'));render(result.room,api.online());
  }catch(error){$('joinError').textContent=error.message;}};
  $('joinCancel').onclick=()=>close($('joinDialog'));
  $('identityBadge').onclick=()=>{$('nickname').value=state?.self?.name||'';$('joinError').textContent='';$('joinDialog').showModal();};
  $('releaseControl').onclick=()=>act('release');
  $('queueControl').onclick=()=>act(state?.queue.some(p=>p.id===state.self?.id)?'cancel':'queue');
  $('takeoverControl').onclick=()=>{$('takeoverDialog').showModal();window.GlassMotion?.anchor($('takeoverDialog'),$('takeoverControl'));};
  $('takeoverCancel').onclick=()=>close($('takeoverDialog'));
  $('takeoverConfirm').onclick=async()=>{close($('takeoverDialog'));await act('takeover');};
  $('shareOpen').onclick=()=>{
    $('shareLinks').replaceChildren();
    if(!urls.length)$('shareLinks').append(el('p','subtle','还没有可分享的局域网地址，请连接 Wi-Fi 或热点后重启服务。'));
    for(const url of urls){
      const row=el('div','share-link',''),input=document.createElement('input');input.readOnly=true;input.value=url;input.setAttribute('aria-label','同伴访问地址');
      input.onclick=()=>input.select();const button=el('button','secondary','复制');button.onclick=async()=>{
        try{await navigator.clipboard.writeText(url);button.textContent='已复制';}
        catch{input.focus();input.select();$('shareHint').textContent='地址已选中，按 Ctrl+C 复制。';}
      };row.append(input,button);$('shareLinks').append(row);
    }
    $('shareDialog').showModal();window.GlassMotion?.anchor($('shareDialog'),$('shareOpen'));
  };
  $('shareClose').onclick=()=>close($('shareDialog'));
  function render(room,online,newUrls){
    if(!room)return;state=room;if(newUrls)urls=newUrls;
    const self=room.self,owner=room.owner,mine=room.mine,position=room.queue.findIndex(p=>p.id===self?.id),next=!!self&&!!room.offer&&room.offer.id===self.id;
    document.title=next?'轮到你了 · 机器人实验':mine?'正在控制 · 机器人实验':'机器人实验';
    $('identityBadge').textContent=self?(self.name+(self.admin&&!self.name.includes('管理员')?' · 管理员':'')):'访客';
    $('identityBadge').title=self?`设备 ${self.deviceId||self.id} · 点击修改名称`:'';
    $('onlineCount').textContent=room.people.length+' 台设备在线';
    $('ownerName').textContent=room.stopping?'正在确认停车':owner?(mine?'你正在控制':owner.deviceId&&owner.deviceId===self?.deviceId?'本机其他页面正在控制':owner.name+' 正在控制'):'当前无人控制';
    $('ownerDetail').textContent=room.stopping?room.stopReason:owner?'其他同伴可以观看或排队':!online?'等待机器人连接':next?'轮到你了，点击确认接手':room.queue.length?'等待队首确认接手':'空间已就绪，随时开始';
    $('ownerTime').textContent=owner?duration(room.serverTime-owner.since):'—';
    $('ownerAvatar').textContent=room.stopping?'停':owner?owner.name.slice(0,1):'空';
    $('ownerAvatar').dataset.state=room.stopping?'stopping':owner?'active':'idle';
    $('claim').hidden=!!owner||position>0;
    $('claim').disabled=!online||room.stopping||!self?.name||!!owner||(room.queue.length>0&&!next);
    $('claim').textContent=next?'确认接手 ↗':'取得控制权 ↗';
    $('releaseControl').hidden=!mine;
    $('queueControl').hidden=mine||(!owner&&!room.stopping&&position<0&&!room.queue.length);
    $('queueControl').disabled=!self?.name;
    $('queueControl').textContent=position>=0?'取消排队 · 第 '+(position+1)+' 位':'申请排队';
    $('takeoverControl').hidden=!self?.admin||mine||(!owner&&!room.queue.length&&!room.stopping);
    $('takeoverControl').disabled=room.stopping||!online;
    $('queueArea').hidden=!room.queue.length;
    $('queueHint').textContent=room.offer?'请在 '+Math.max(0,Math.ceil((room.offer.until-room.serverTime)/1000))+' 秒内确认':room.queue.length+' 人等待';
    $('lease').textContent=room.stopping?'交接中 · 等待停车确认':mine?'控制权在你手中':owner?owner.name+' 正在控制':'观看中 · 可以申请控制';
    const key=JSON.stringify([room.queue,room.people,room.events,self?.id]);
    if(key===signature)return;signature=key;
    $('queueList').replaceChildren(...room.queue.map((p,i)=>{const row=el('li','','');row.append(el('span','queue-number',String(i+1).padStart(2,'0')),el('span','',p.name+(p.id===self?.id?'（你）':'')));return row;}));
    $('peopleList').replaceChildren(...room.people.map(p=>el('span','person-chip',p.name+(p.admin?' · 管理员':''))));
    $('activityList').replaceChildren(...room.events.slice(0,8).map(event=>{const row=el('li','','');row.append(el('time','',new Date(event.at).toLocaleTimeString('zh-CN',{hour12:false})),el('span','',event.text));return row;}));
    if(!room.events.length)$('activityList').append(el('li','subtle','还没有控制记录'));
  }
  return {init,render};
})();
