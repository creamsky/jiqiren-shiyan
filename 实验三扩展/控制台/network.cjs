const os=require('node:os');
function shareAddresses(robotIp,interfaces=os.networkInterfaces()){
  const candidates=[];
  for(const [name,entries] of Object.entries(interfaces)){
    if(/vmware|vmnet|vethernet|virtualbox|loopback|docker|wsl|tailscale|zerotier/i.test(name))continue;
    for(const item of entries||[]){
      if(item.family!=='IPv4'||item.internal||item.address.startsWith('169.254.'))continue;
      const sameSubnet=robotIp?.split('.').slice(0,3).join('.')===item.address.split('.').slice(0,3).join('.');
      candidates.push({address:item.address,score:(sameSubnet?10:0)+(/wlan|wi-fi|无线/i.test(name)?2:0)});
    }
  }
  return candidates.sort((a,b)=>b.score-a.score).map(x=>x.address).slice(0,1);
}
function deviceKey(remoteAddress,localAddresses=[]){
  const address=String(remoteAddress||'').replace(/^::ffff:/,'');
  return address==='::1'||address.startsWith('127.')||localAddresses.includes(address)?'server-computer':'lan-computer:'+address;
}
module.exports={shareAddresses,deviceKey};
