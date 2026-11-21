#!/usr/bin/python3
"""Isolated ROS/TCP tests. dry_run=True never opens a CAN socket.

Run on the robot after sourcing ROS, with /usr/bin/python3 explicitly.
"""
import json
import math
import os
import socket
import subprocess
import time

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11313'
os.environ['ROS_IP'] = '127.0.0.1'

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

ROOT = os.path.dirname(os.path.abspath(__file__))
BINARY = os.environ.get('LAB26_TEST_BINARY', '/home/nvidia/lab26-console-stage/catkin_ws/devel/lib/basic_control/basicControl')
LOGS = os.path.join(ROOT, 'logs')
os.makedirs(LOGS, exist_ok=True)


class Simulation:
    def __init__(self, process):
        self.process = process
        self.command = [0.0, 0.0]
        self.x = self.y = self.yaw = 0.0
        self.obstacle = None
        self.invalid = False
        self.status = {}
        self.ack = {}
        self.preview = {}
        self.buffer = b''
        self.odom = rospy.Publisher('/Odometry', Odometry, queue_size=1)
        self.cloud = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1)
        self.subscription = rospy.Subscriber('/lab26/command', Twist, self.on_command, queue_size=1)
        self.sock = socket.create_connection(('127.0.0.1', 18767), timeout=2)
        self.sock.setblocking(False)
        self.last_tick = time.monotonic()
        self.last_heartbeat = 0.0
        self.last_key = 0.0

    def on_command(self, msg):
        self.command[:] = [msg.linear.x, msg.angular.z]

    def send(self, value):
        self.sock.sendall((value + '\n').encode('ascii'))

    def tick(self, heartbeat=True, key=None):
        if self.process.poll() is not None:
            raise AssertionError('Controller exited unexpectedly; see test_console_controller.log')
        current = time.monotonic()
        dt = min(current - self.last_tick, 0.05)
        self.last_tick = current
        v, w = self.command
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        if heartbeat and current - self.last_heartbeat >= 0.08:
            self.send('heartbeat')
            self.last_heartbeat = current
        if key and current - self.last_key >= 0.08:
            self.send(key)
            self.last_key = current
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera_init'
        msg.pose.pose.position.x = float('nan') if self.invalid else self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2)
        self.odom.publish(msg)
        points = [(3.0 + i * 0.001, 3.0, 0.0) for i in range(120)]
        if self.obstacle is not None:
            points += [self.obstacle if isinstance(self.obstacle, tuple) else (self.obstacle, 0.0, 0.0)] * 10
        self.cloud.publish(point_cloud2.create_cloud_xyz32(
            Header(stamp=rospy.Time.now(), frame_id='velodyne'), points))
        try:
            data = self.sock.recv(16384)
            if not data:
                raise AssertionError('Controller closed TCP connection')
            self.buffer += data
        except BlockingIOError:
            pass
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            value = json.loads(line)
            if value.get('type') == 'cloud':
                self.preview = value
            elif value.get('type') == 'pose':
                assert len(value['pose']) == 3
            elif value.get('type') == 'ack':
                self.ack = value
            else:
                self.status = value
        time.sleep(0.02)

    def run(self, seconds, **kwargs):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.tick(**kwargs)

    def wait(self, predicate, message, timeout=3.0, **kwargs):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.tick(**kwargs)
            if predicate():
                return
        raise AssertionError(message + ': ' + json.dumps(self.status))

    def stopped(self):
        return abs(self.command[0]) < 1e-9 and abs(self.command[1]) < 1e-9


def record(name):
    results.append({'case': name, 'pass': True})
    print(json.dumps(results[-1]), flush=True)


master = controller = sim = None
master_log = controller_log = None
results = []
try:
    # Never attach this test to a running live controller's TCP port.
    with socket.socket() as probe:
        probe.settimeout(0.3)
        if probe.connect_ex(('127.0.0.1', 18767)) == 0:
            raise RuntimeError('Isolated test port 18767 already active')
    master_log = open(os.path.join(LOGS, 'test_console_master.log'), 'w')
    controller_log = open(os.path.join(LOGS, 'test_console_controller.log'), 'w')
    master = subprocess.Popen(['roscore', '-p', '11313'], stdout=master_log, stderr=subprocess.STDOUT)
    time.sleep(2)
    rospy.init_node('lab26_console_test', anonymous=True, disable_signals=True)
    controller = subprocess.Popen([
        BINARY, '_dry_run:=true', '_console:=true', '_console_port:=18767', '_turn_degrees:=0',
        '_run_timeout:=90', '_csv:=' + os.path.join(LOGS, 'test_console.csv')],
        stdout=controller_log, stderr=subprocess.STDOUT)
    time.sleep(0.7)
    if controller.poll() is not None:
        raise RuntimeError('Controller failed to start')
    sim = Simulation(controller)
    sim.run(1.0)
    assert sim.status.get('paused') and sim.stopped(), sim.status
    record('initial_paused_zero')
    sim.wait(lambda: bool(sim.preview.get('points')), 'Pointcloud preview absent')
    assert len(sim.preview['points']) <= 8000
    record('bounded_pointcloud_preview')

    sim.send('manual')
    sim.run(0.2, key='w')
    assert sim.ack.get('command') == 'manual' and sim.ack.get('ok') is True
    assert 0.07 < sim.command[0] < 0.16, ('Manual startup ramp must use 0.60 m/s2', sim.command)
    record('manual_acceleration_ramp')
    sim.wait(lambda: sim.command[0] > 0.95 and sim.status.get('mode') == 'manual',
             'Manual W did not reach configured limit', key='w', timeout=10)
    assert sim.status.get('mode') == 'manual'
    assert 0.95 < sim.command[0] <= 1.00 and sim.command[1] == 0
    sim.send('zero')
    sim.wait(sim.stopped, 'Key release did not stop', timeout=0.3)
    record('manual_w_and_release')

    for key, predicate in (
            ('s', lambda: sim.command[0] < -0.2 and sim.command[1] == 0),
            ('a', lambda: sim.command[0] == 0 and 0.25 < sim.command[1] <= 0.30),
            ('d', lambda: sim.command[0] == 0 and -0.30 <= sim.command[1] < -0.25)):
        sim.last_key = 0
        sim.wait(predicate, 'Manual ' + key.upper() + ' failed', key=key)
        sim.send('zero')
        sim.wait(sim.stopped, 'Manual zero failed', timeout=0.3)
    record('manual_s_a_d_and_limits')
    sim.obstacle = -0.7
    sim.wait(lambda: sim.command[0] < -0.2, 'Rear obstacle must not pause reverse with configured policy', key='s')
    sim.obstacle = None
    sim.send('zero')
    sim.wait(sim.stopped, 'Reverse release did not stop', timeout=0.3)
    record('rear_pause_disabled')
    sim.wait(lambda: sim.command[1] > 0.25, 'Manual rotation setup failed', key='a')
    sim.run(0.15, key='w')
    assert sim.command[0] > 0 and sim.command[1] == 0, ('W must not retain rotation', sim.command)
    sim.send('zero')
    sim.wait(sim.stopped, 'Transition test did not stop', timeout=0.3)
    record('manual_translation_rotation_are_exclusive')

    sim.wait(lambda: sim.command[0] > 0.2, 'Manual W did not rearm', key='w')
    sim.run(0.7, heartbeat=False, key='w')
    assert sim.status.get('paused') and sim.stopped(), sim.status
    sim.run(0.4)
    assert sim.status.get('paused') and sim.stopped(), sim.status
    record('heartbeat_timeout_latches_pause')

    sim.wait(lambda: sim.command[0] > 0.2, 'Manual W did not resume', key='w')
    sim.invalid = True
    sim.wait(lambda: sim.status.get('stage') == 'FAULT', 'Invalid manual pose did not fault')
    assert sim.stopped() and 'odometry' in sim.status.get('reason', '').lower()
    sim.run(0.6)
    sim.send('manual')
    sim.run(0.2)
    assert sim.ack.get('ok') is False and sim.ack.get('reason') == 'odometry_unavailable'
    assert sim.status['stage'] == 'FAULT' and sim.stopped()
    record('manual_rejection_ack_and_fault_retained')
    sim.invalid = False
    sim.run(0.3)
    record('invalid_manual_pose_fault')

    sim.send('start')
    sim.wait(lambda: sim.status.get('stage') == 'DRIVING' and sim.command[0] > 0,
             'New auto task did not start', timeout=4)
    sim.run(0.5)
    before = sim.status['progress']
    assert before > 0
    sim.obstacle = 0.7
    sim.wait(lambda: sim.status.get('obstacle') and sim.stopped(),
             'Obstacle did not pause automatic task', timeout=0.5)
    paused_progress = sim.status['progress']
    assert sim.status['stage'] == 'DRIVING'
    record('obstacle_pauses_without_fault')

    sim.obstacle = 1.1
    sim.run(2.4)
    assert sim.status.get('obstacle') and sim.stopped(), sim.status
    assert abs(sim.status['progress'] - paused_progress) < 0.03
    record('hysteresis_band_holds_pause')

    sim.obstacle = None
    sim.run(1.6)
    assert sim.status.get('obstacle') and sim.stopped(), sim.status
    sim.wait(lambda: not sim.status.get('obstacle', True) and sim.command[0] > 0,
             'Clear corridor did not automatically resume', timeout=1.3)
    assert sim.command[0] < 0.08, ('Automatic resume ramp missing', sim.command)
    assert sim.status['stage'] == 'DRIVING'
    sim.run(0.4)
    assert sim.status['progress'] > paused_progress
    assert sim.status['progress'] >= before
    record('clear_two_seconds_resumes_original_task')

    sim.send('stop')
    sim.wait(lambda: sim.status.get('stage') == 'FAULT' and sim.stopped(), 'Stop did not latch')
    record('operator_stop_latches')
    sim.send('start8 0.1 cross')
    sim.run(0.2)
    assert sim.status['stage'] == 'FAULT' and sim.stopped()
    record('eight_invalid_radius_cannot_start')
    for point in ['cross', 'outer']:
        sim.send('start8 0.5 ' + point)
        sim.wait(lambda: sim.status.get('stage') == 'DRIVING' and sim.command[0] > 0,
                 'Eight start failed: ' + point, timeout=4)
        assert abs(sim.status['total_distance'] - 2 * math.pi) < 0.001
        sim.run(1)
        sim.send('pause')
        sim.wait(sim.stopped, 'Eight pause failed')
        paused_progress = sim.status['progress']
        sim.run(0.3)
        assert abs(sim.status['progress'] - paused_progress) < 0.05
        sim.send('resume')
        sim.wait(lambda: sim.command[0] > 0, 'Eight resume failed')
        sim.send('stop')
        sim.wait(lambda: sim.status.get('stage') == 'FAULT' and sim.stopped(), 'Eight stop failed')
        record('eight_' + point + '_start_progress_pause_resume_stop')
    sim.send('start3')
    sim.wait(lambda: sim.command[0] > 0 and sim.status.get('avoidance_state') == 'straight', 'Lab3 straight failed')
    sim.obstacle = (-0.36, 0.0, 0.0)
    sim.run(0.5)
    assert sim.command[0] > 0 and not sim.status.get('obstacle'), 'Rear-only obstacle must not block straight forward motion'
    sim.obstacle = (0.0, 0.40, 0.0)
    sim.yaw += 0.002
    sim.run(0.6)
    assert sim.command[0] > 0 and not sim.status.get('obstacle'), 'Small heading correction beside stool must not latch circular protection'
    record('lab3_side_stool_with_heading_correction_keeps_moving')
    sim.obstacle = (0.0, 0.28, 0.0)
    sim.run(0.3)
    assert sim.command[0] > 0, 'Body points must be excluded'
    sim.obstacle = (0.1, 0.32, 0.0)
    sim.wait(lambda: sim.stopped() and sim.status.get('obstacle'), 'Side obstacle inside old radial mask must now stop motion')
    sim.obstacle = None
    sim.wait(lambda: sim.command[0] > 0, 'Side guard did not release', timeout=4)
    record('lab3_rectangle_self_mask_side_guard_rear_policy')
    sim.send('stop')
    sim.wait(sim.stopped, 'Reset before full-speed approach must stop')
    sim.send('start3')
    sim.wait(lambda: sim.command[0] > 0.79, 'Distance regression must approach at full straight speed', timeout=7)
    sim.obstacle = (1.15, -0.1, 0.0)
    sim.wait(lambda: sim.command[0] == 0 and sim.command[1] > 0 and sim.status.get('avoidance_state') == 'exploring',
             'Obstacle beyond one meter must enter exploration instead of remaining stopped', timeout=5)
    record('lab3_beyond_one_meter_enters_exploration')
    sim.obstacle = (0.7, -0.34, 0.0)
    sim.run(0.5)
    assert sim.command[0] == 0 and sim.status.get('avoidance_state') == 'exploring', 'Side-front obstacle must keep exploration active'
    sim.obstacle = (0.44, -0.34, 0.0)
    sim.run(0.5)
    assert sim.command[0] == 0 and sim.status.get('avoidance_state') == 'exploring', 'Radial self mask must not create a rectangular blind strip'
    sim.obstacle = (-0.36, -0.1, 0.0)
    sim.wait(lambda: sim.command[0] > 0 and sim.command[1] == 0,
             'Blocked return turn with clear forward path must advance without rotating')
    record('lab3_clear_side_obstacle_before_return_turn')
    sim.obstacle = (0.34, 0.0, 0.0)
    sim.wait(lambda: sim.stopped() and sim.status.get('obstacle'), 'Lab3 near obstacle must stop rotation')
    sim.obstacle = None
    sim.wait(lambda: sim.command[0] > 0, 'Lab3 did not resume after clear corridor', timeout=5)
    sim.obstacle = (0.9, 0.34, 0.0)
    sim.wait(lambda: sim.command[0] == 0 and sim.command[1] < 0, 'New obstacle during bypass must restart exploration toward the right')
    sim.obstacle = None
    sim.send('pause')
    sim.wait(sim.stopped, 'Lab3 pause failed')
    sim.send('stop')
    sim.wait(lambda: sim.status.get('stage') == 'FAULT' and sim.stopped(), 'Lab3 stop failed')
    record('lab3_decision_near_guard_clear_resume_pause_stop')
    with open(os.path.join(LOGS, 'console_integration_results.json'), 'w') as result_file:
        json.dump(results, result_file, indent=2)
finally:
    if sim is not None:
        try:
            sim.send('stop')
        except OSError:
            pass
        sim.sock.close()
    for process in (controller, master):
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
    for output in (controller_log, master_log):
        if output is not None:
            output.close()
