const http = require('node:http');
const path = require('node:path');
const {execFile} = require('node:child_process');

// This is the gzclient display of the project VM. Only the local workstation
// may open the interactive display. Navigation uses the separate ROS bridge.
const upstream = {host:'127.0.0.1',port:9092};
let repair=null;
function restore(directory){
  if(repair)return repair;
  const executable=path.join(process.env.LOCALAPPDATA,'Microsoft','WindowsApps','pwsh.exe');
  repair=new Promise((resolve,reject)=>execFile(executable,['-NoLogo','-NoProfile','-File',path.join(directory,'runtime','Runtime.ps1'),'-Action','Viewer'],
    {windowsHide:true,timeout:100000},(error,stdout,stderr)=>{
      if(error)reject(Error((stderr.trim()||error.message).replace(/\x1b\[[0-9;]*m/g,'').slice(-450)));else resolve();
    })).finally(()=>{repair=null;});
  return repair;
}
function serveAsset(req,res) {
  const name=new URL(req.url,'http://localhost').pathname.slice('/gazebo'.length);
  if(!/^\/(core|vendor)\/[a-zA-Z0-9./_-]+\.js$/.test(name) || name.includes('..')){
    res.writeHead(404);res.end();return;
  }
  const get=http.get({...upstream,path:name},response=>{
    res.writeHead(response.statusCode,{'Content-Type':'text/javascript; charset=utf-8','Cache-Control':response.statusCode===200?'private, max-age=3600':'no-store','X-Content-Type-Options':'nosniff'});
    response.pipe(res);
  });
  get.setTimeout(3000,()=>get.destroy());
  get.on('error',()=>{if(!res.headersSent)res.writeHead(503,{'Cache-Control':'no-store'});res.end();});
}
function status(res){
  const probe=http.request({...upstream,path:'/core/rfb.js',method:'HEAD'},response=>{response.resume();send(response.statusCode===200);});
  function send(available){if(res.writableEnded)return;res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});res.end(JSON.stringify({available}));}
  probe.setTimeout(1500,()=>probe.destroy());
  probe.on('error',()=>send(false));probe.end();
}
function upgrade(req,client,head){
  const headers={Connection:'Upgrade', Upgrade:'websocket','Sec-WebSocket-Version':'13','Sec-WebSocket-Key':req.headers['sec-websocket-key']};
  if(req.headers['sec-websocket-protocol'])headers['Sec-WebSocket-Protocol']=req.headers['sec-websocket-protocol'];
  const request=http.request({...upstream,path:'/websockify',headers});
  const fail=()=>client.destroy();
  request.setTimeout(5000,()=>request.destroy());
  client.on('error',()=>request.destroy());
  request.on('error',fail);
  request.on('response',res=>{res.resume();fail();});
  request.on('upgrade',(response,vm,first)=>{
    request.setTimeout(0);
    let handshake='HTTP/1.1 101 Switching Protocols\r\n';
    for(let i=0;i<response.rawHeaders.length;i+=2)handshake+=response.rawHeaders[i]+': '+response.rawHeaders[i+1]+'\r\n';
    client.write(handshake+'\r\n');
    if(first.length)client.write(first);
    if(head.length)vm.write(head);
    vm.on('error',fail);vm.on('close',fail);client.on('close',()=>vm.destroy());
    client.pipe(vm).pipe(client);
  });
  request.end();
}
module.exports={serveAsset,status,restore,upgrade};
