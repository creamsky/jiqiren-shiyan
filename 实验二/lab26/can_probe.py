#!/usr/bin/env python3
"""Read-only SocketCAN probe. Never sends a frame."""
import socket, struct, time, json, collections
s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
s.bind(('can0',))
s.settimeout(0.5)
end = time.monotonic()+3
frames = {}
counts = collections.Counter()
while time.monotonic() < end:
    try:
        cid, size, payload = struct.unpack('=IB3x8s', s.recv(16))
        counts[hex(cid)] += 1
        frames[hex(cid)] = payload[:size].hex()
    except socket.timeout:
        pass
print(json.dumps({'counts': dict(counts), 'last_frames': frames}, indent=2))
