import unittest

from enable_can_mode import FRAME, ensure_can


class Bus:
    def __init__(self, mode, switch=2, speed=0):
        self.mode = mode
        self.switch = switch
        self.speed = speed
        self.tick = 0
        self.sent = []

    def recv(self, size):
        frames = [(0x211, bytes([0, self.mode])), (0x241, bytes([self.switch << 2])),
                  (0x221, bytes([0, self.speed]))]
        cid, data = frames[self.tick % 3]
        self.tick += 1
        return FRAME.pack(cid, 8, data.ljust(8, b'\0'))

    def send(self, frame):
        self.sent.append(FRAME.unpack(frame))
        if self.sent[-1][0] == 0x421:
            self.mode = 1

    def check(self):
        ensure_can(self, lambda: self.tick * 0.01)


class CanStartupTest(unittest.TestCase):
    def test_standby_requires_zero_request_and_new_confirmation(self):
        bus = Bus(0)
        bus.check()
        self.assertEqual(bus.sent, [(0x111, 8, bytes(8)), (0x421, 1, bytes([1])+bytes(7))])
        self.assertGreaterEqual(bus.tick, 4)

    def test_can_already_active(self):
        bus = Bus(1)
        bus.check()
        self.assertEqual(bus.sent, [])

    def test_no_request_during_remote_control_or_motion(self):
        for bus in [Bus(3), Bus(0, switch=1), Bus(0, switch=3), Bus(0, speed=1)]:
            with self.assertRaises(RuntimeError):
                bus.check()
            self.assertEqual(bus.sent, [])

    def test_missing_mode_ack_times_out(self):
        bus = Bus(0)
        bus.send = lambda frame: None
        with self.assertRaisesRegex(RuntimeError, "确认"):
            bus.check()
