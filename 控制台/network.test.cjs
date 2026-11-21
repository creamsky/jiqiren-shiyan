const {test}=require('node:test');const assert=require('node:assert/strict');const {shareAddresses}=require('./network.cjs');
const nic=address=>[{address,family:'IPv4',internal:false}];
test('one computer has one display identity across loopback and LAN; distinct LAN peers stay distinct',()=>{
  const {deviceKey}=require('./network.cjs'),addresses=['10.194.191.87'];
  assert.equal(deviceKey('::ffff:127.0.0.1',addresses),deviceKey('10.194.191.87',addresses));
  assert.equal(deviceKey('::ffff:10.194.191.11',addresses),deviceKey('10.194.191.11',addresses));
  assert.notEqual(deviceKey('10.194.191.11',addresses),deviceKey('10.194.191.12',addresses));
});
test('invite shows one actual LAN address instead of VMware adapters',()=>{
  const interfaces={'VMware Network Adapter VMnet1':nic('192.168.62.1'),'VMware Network Adapter VMnet8':nic('192.168.100.1'),WLAN:nic('10.194.191.87'),Ethernet:nic('192.168.1.5')};
  assert.deepEqual(shareAddresses('10.194.191.72',interfaces),['10.194.191.87']);
  assert.deepEqual(shareAddresses('10.194.191.72',{'VMware Network Adapter':nic('192.168.62.1')}),[]);
});
