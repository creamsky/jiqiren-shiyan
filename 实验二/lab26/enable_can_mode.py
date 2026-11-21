#!/usr/bin/env python3
"""Confirm CAN feedback before starting the hardware console."""
import socket
import struct
import time


FRAME = struct.Struct("=IB3x8s")


def ensure_can(bus, clock=time.monotonic):
    deadline = clock() + 3
    feedback = {}
    next_request = 0
    while clock() < deadline:
        try:
            cid, size, data = FRAME.unpack(bus.recv(FRAME.size))
        except socket.timeout:
            continue
        if size != 8 or cid not in (0x211, 0x221, 0x241):
            continue
        now = clock()
        feedback[cid] = (now, data)
        if len(feedback) != 3 or any(now - timestamp > 0.4 for timestamp, _ in feedback.values()):
            continue
        system = feedback[0x211][1]
        mode = system[1]
        rc = (feedback[0x241][1][0] >> 2) & 3
        if rc in (1, 3) or mode == 3:
            raise RuntimeError("遥控器正在接管，请切换到允许 CAN 控制的档位再连接")
        if system[0] or any(system[4:6]):
            raise RuntimeError("底盘报告故障，请检查实车")
        if mode == 1:
            return
        if mode != 0 or any(feedback[0x221][1][:4]):
            raise RuntimeError("底盘未在静止待机状态，已停止切换")
        if now >= next_request:
            bus.send(FRAME.pack(0x111, 8, bytes(8)))
            bus.send(FRAME.pack(0x421, 1, bytes([1]) + bytes(7)))
            next_request = now + 0.05
    raise RuntimeError("未收到新鲜的 CAN 模式确认，请检查 can0 和遥控器档位")


if __name__ == "__main__":
    try:
        with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
            bus.bind(("can0",))
            bus.settimeout(0.1)
            ensure_can(bus)
    except (RuntimeError, OSError) as error:
        raise SystemExit("CAN: " + str(error))
    print("CAN: 底盘反馈已确认 CAN 控制模式 (1)")
