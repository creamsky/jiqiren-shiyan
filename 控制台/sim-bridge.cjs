const {execFile} = require('node:child_process');
const path = require('node:path');

// ROS stays on the VM's loopback tunnel. Browsers can see only this experiment's
// state and commands, including when the console is opened from another computer.
class SimulationBridge {
  constructor(directory) {
    this.script = path.join(directory, 'runtime', 'Runtime.ps1');
    this.clients = new Set();
    this.phase = 'idle';
    this.message = '等待启动 Gazebo';
    this.lastAt = 0;
  }
  live() { return Date.now() - this.lastAt < 2400; }
  status() {
    if (this.live()) return {phase:'ready', message:'已连接 Gazebo'};
    return {phase:this.phase, message:this.message};
  }
  connect() {
    if (this.socket || this.retry || this.closed) return;
    const ws = this.socket = new WebSocket('ws://127.0.0.1:9091');
    ws.onopen = () => {
      ws.send(JSON.stringify({op:'advertise', topic:'/scout_lab/request', type:'std_msgs/String'}));
      ws.send(JSON.stringify({op:'subscribe', topic:'/scout_lab/state', type:'std_msgs/String', throttle_rate:140, queue_length:1}));
    };
    ws.onmessage = event => {
      try {
        const packet = JSON.parse(event.data);
        if (packet.topic !== '/scout_lab/state' || packet.op !== 'publish') return;
        const state = JSON.parse(packet.msg.data);
        if (state.engine !== 'gazebo-classic') return;
        this.phaseName=state.phase;
        this.operator=state.phase==='manual'?state.manual_owner:state.navigation_owner;
        this.lastAt = Date.now();
        this.frame = 'data: '+JSON.stringify(state)+'\n\n';
        for (const client of this.clients) {
          if (client.writableLength > 512*1024) client.destroy();
          else client.write(this.frame);
        }
      } catch { /* A malformed frame never renews freshness. */ }
    };
    ws.onerror = () => ws.close();
    ws.onclose = () => {
      this.socket = null;
      this.lastAt = 0;
      this.frame = null;
      if (!this.closed && (this.clients.size || this.launching))
        this.retry = setTimeout(() => { this.retry=null;this.connect(); }, 1000);
    };
  }
  subscribe(res) {
    res.writeHead(200, {'Content-Type':'text/event-stream', 'Cache-Control':'no-store','X-Accel-Buffering':'no'});
    res.write(': scout-lab\n\n');
    this.clients.add(res);
    if (this.frame && this.live()) res.write(this.frame);
    res.on('close', () => {
      this.clients.delete(res);
      if(!this.clients.size && !this.launching) {
        clearTimeout(this.retry);this.retry=null;
        this.socket?.close();
      }
    });
    this.connect();
  }
  start() {
    this.connect();
    if (this.live() || this.launching) return this.status();
    this.phase='starting';
    this.message='正在启动专用虚拟机和 Gazebo';
    const pwsh=path.join(process.env.LOCALAPPDATA,'Microsoft','WindowsApps','pwsh.exe');
    this.launching=execFile(pwsh,['-NoLogo','-NoProfile','-File',this.script,'-Action','Launch'],{windowsHide:true,timeout:240000},(error,stdout,stderr)=>{
      this.launching=null;
      this.phase=error?'error':'idle';
      const output=(stderr||stdout||'').replace(/\x1b\[[0-9;]*m/g,'').trim();
      this.message=error?('仿真启动失败 · '+(output.slice(-520)||error.message)):'正在等待 ROS 位姿和点云';
      this.connect();
    });
    return this.status();
  }
  command(body) {
    if (!this.live() || this.socket?.readyState !== WebSocket.OPEN) throw Error('Gazebo 数据已断开');
    if (!body || !['scene','go','stop','manual','drive','reset'].includes(body.op)) throw Error('未知仿真指令');
    if(body.op==='stop' && !body.explicit && ['manual','running'].includes(this.phaseName) &&
       (!body.owner || body.owner!==this.operator))throw Error('当前任务由其他窗口控制');
    this.socket.send(JSON.stringify({op:'publish',topic:'/scout_lab/request',msg:{data:JSON.stringify(body)}}));
  }
  close() {
    this.closed=true;
    clearTimeout(this.retry);
    for(const client of this.clients) client.end();
    this.socket?.close();
  }
}
module.exports={SimulationBridge};
