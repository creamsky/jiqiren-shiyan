#!/usr/bin/env python3
"""Request CAN mode while holding zero velocity. RC retains hardware priority."""
import socket, struct, time
s=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
s.bind(('can0',)); s.settimeout(0.02)
end=time.monotonic()+2
mode=None
while time.monotonic()<end:
    s.send(struct.pack('=IB3x8s',0x111,8,bytes(8)))
    s.send(struct.pack('=IB3x8s',0x421,1,bytes([1])+bytes(7)))
    until=time.monotonic()+0.02
    while time.monotonic()<until:
        try:
            cid,n,data=struct.unpack('=IB3x8s',s.recv(16))
            if cid==0x211 and n==8: mode=data[1]
        except socket.timeout: break
print('Control mode:',mode,'(1=CAN, 3=remote control)')
if mode!=1: raise SystemExit('CAN mode not enabled. Use the physical RC mode switch; no motion was commanded.')
