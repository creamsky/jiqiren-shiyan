const crypto=require('node:crypto');

// One room, one live lease. Tokens stay private; public IDs are display-only.
class ControlRoom {
  constructor({stop,stopped,connected,onboard=()=>false,canManageMoving=onboard,present=()=>false,now=Date.now,leaseMs=1000}){
    Object.assign(this,{stop,stopped,connected,onboard,canManageMoving,present,now,leaseMs});
    this.sessions=new Map();this.owner=null;this.queue=[];this.events=[];
    this.barrier=null;this.pendingAdmin=null;this.offer=null;this.version=0;
  }
  event(text){this.events.unshift({id:++this.version,at:this.now(),text});this.events.length=Math.min(40,this.events.length);}
  session(admin,deviceKey=crypto.randomBytes(32).toString('hex')){
    if(this.sessions.size>=200)throw Error('会话过多，请稍后再试');
    const token=crypto.randomBytes(32).toString('hex');
    const deviceId=crypto.createHash('sha256').update(deviceKey).digest('hex').slice(0,12);
    const previous=[...this.sessions.values()].find(s=>s.deviceId===deviceId&&s.admin===admin);
    this.sessions.set(token,{id:crypto.randomBytes(4).toString('hex'),deviceId,name:previous?.name||(admin?'服务器本机':'同伴 '+deviceId.slice(0,6).toUpperCase()),admin,seen:this.now(),seq:0});
    return token;
  }
  person(token){const s=this.sessions.get(token);return s?{id:s.id,deviceId:s.deviceId,name:s.name,admin:s.admin}:null;}
  touch(token){const s=this.sessions.get(token);if(s)s.seen=this.now();}
  join(token,name){
    const clean=String(name||'').replace(/[\x00-\x1f\x7f]/g,'').trim().slice(0,16);
    if(!clean)throw Error('请填写一个昵称');
    const s=this.sessions.get(token);for(const peer of this.sessions.values())if(peer.deviceId===s.deviceId&&peer.admin===s.admin)peer.name=clean;this.touch(token);return this.snapshot(token);
  }
  identified(token){if(!this.sessions.get(token)?.name)throw Error('请先填写昵称');}
  ownerPresent(){return this.present(this.owner.token)||this.now()<(this.owner.reconnectUntil||0)||this.now()-this.owner.at<this.leaseMs;}
  valid(token,lease){return !!this.owner&&this.owner.token===token&&this.owner.lease===lease&&this.ownerPresent()&&!this.barrier;}
  grant(token){
    this.owner={token,lease:crypto.randomBytes(16).toString('hex'),at:this.now(),since:this.now()};
    this.queue=this.queue.filter(t=>t!==token);this.offer=null;
    this.event(`${this.person(token).name} 取得控制权`);
  }
  beginStop(reason,{release=false,admin=null,keepReady=false}={}){
    if(release)this.owner=null;
    else if(this.owner)this.owner.lease=crypto.randomBytes(16).toString('hex');
    this.pendingAdmin=admin;this.offer=null;
    this.barrier={at:this.now(),reason,stableAt:null};
    this.stop(keepReady);
  }
  disconnected(){
    if(this.owner&&this.present(this.owner.token)){
      if(!this.onboard())this.beginStop('机器人连接中断，等待停车确认',{keepReady:true});
      return;
    }
    if(this.owner)this.event(`${this.person(this.owner.token).name} 的控制因机器人断线而结束`);
    if(this.onboard()){this.owner=null;this.offer=null;return;}
    this.beginStop('机器人连接中断，等待停车确认',{release:true});
  }
  claim(token){
    this.identified(token);this.tick();
    if(!this.connected())throw Error('机器人控制连接未建立');
    if(this.barrier)throw Error('正在确认机器人停止，请稍候');
    if(this.owner){if(this.owner.token===token)return;throw Error('机器人正在使用，请加入排队');}
    if(this.queue.length&&this.queue[0]!==token)throw Error('请按排队顺序接手');
    if(!this.canManageMoving()&&!this.stopped())throw Error('未确认机器人静止，暂不能取得控制权');
    this.grant(token);
  }
  enqueue(token){
    this.identified(token);this.touch(token);
    if(this.owner?.token===token)throw Error('你已经在控制');
    if(!this.queue.includes(token)){this.queue.push(token);this.event(`${this.person(token).name} 加入排队`);}
    this.tick();
  }
  cancel(token){this.queue=this.queue.filter(t=>t!==token);if(this.offer?.token===token)this.offer=null;}
  release(token){
    if(this.owner?.token!==token)throw Error('你当前没有控制权');
    this.event(`${this.person(token).name} 结束控制`);
    if(this.onboard()){this.owner=null;return;}
    this.beginStop('结束控制，等待停车确认',{release:true,keepReady:true});
  }
  takeover(token){
    if(!this.sessions.get(token)?.admin)throw Error('只有本机管理员可以接管');
    if(this.pendingAdmin===token)return;
    this.event(`${this.person(token).name} 发起管理员接管`);
    this.cancel(token);this.beginStop('管理员接管，等待停车确认',{release:true,admin:token,keepReady:true});
  }
  emergency(token){
    this.event(`${this.person(token)?.name||'访客'} 点击停车`);
    this.beginStop('已请求停车，等待反馈');
  }
  heartbeat(token,lease){
    // Heartbeats may keep an owner present during a stop, but never bypass it.
    if(!this.owner||this.owner.token!==token||this.owner.lease!==lease||this.now()-this.owner.at>=this.leaseMs)throw Error('控制权已失效，请重新申请');
    this.owner.at=this.now();this.touch(token);
  }
  tick(){
    const now=this.now();
    if(this.owner&&!this.ownerPresent()){
      this.event(`${this.person(this.owner.token).name} 失联，已撤销控制权`);
      if(this.onboard())this.owner=null;
      else this.beginStop('操作者失联，等待停车确认',{release:true,keepReady:true});
    }
    this.queue=this.queue.filter(t=>this.sessions.has(t)&&now-this.sessions.get(t).seen<6500);
    for(const [token,s] of this.sessions)if(!this.present(token)&&now-s.seen>12*3600e3)this.sessions.delete(token);
    if(this.barrier){
      if(this.connected()&&this.stopped()&&now-this.barrier.at>=150){
        this.barrier.stableAt??=now;
        if(now-this.barrier.stableAt>=250){
          this.barrier=null;
          const admin=this.pendingAdmin;this.pendingAdmin=null;
          if(admin&&now-(this.sessions.get(admin)?.seen??0)<6500)this.grant(admin);
        }
      }else this.barrier.stableAt=null;
    }
    if(!this.owner&&!this.barrier&&this.connected()&&this.queue.length){
      if(this.offer?.token!==this.queue[0])this.offer={token:this.queue[0],until:now+30000};
      if(now>=this.offer.until){this.event(`${this.person(this.queue[0]).name} 未确认接手，已退出排队`);this.queue.shift();this.offer=null;}
    }else this.offer=null;
  }
  snapshot(token){
    const now=this.now(),self=this.person(token),mine=!!this.owner&&this.owner.token===token;
    const people=new Map();
    for(const [t,s] of this.sessions)if(s.name&&now-s.seen<6500){
      const current=people.get(s.deviceId);
      // A computer may have several tabs. Show its active administrator identity once.
      if(!current||s.admin&&!current.admin)people.set(s.deviceId,this.person(t));
    }
    return {self,mine,lease:mine?this.owner.lease:null,
      owner:this.owner?{...this.person(this.owner.token),since:this.owner.since}:null,
      stopping:!!this.barrier,stopReason:this.barrier?.reason||'',
      queue:this.queue.map(t=>this.person(t)),offer:this.offer?{id:this.person(this.offer.token).id,until:this.offer.until}:null,
      people:[...people.values()],
      events:this.events,serverTime:now};
  }
}
module.exports={ControlRoom};
