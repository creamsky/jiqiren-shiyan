#!/usr/bin/python3
"""ROS navigation adapter; publishes proposals only, never accesses CAN."""
import math
import threading
import time
from collections import deque
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TwistStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import Config, DoubleParameter


class Bridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.client = actionlib.SimpleActionClient('/lab3_dwa/move_base', MoveBaseAction)
        self.output = rospy.Publisher('/lab3_dwa/command', TwistStamped, queue_size=1)
        self.health = rospy.Publisher('/lab3_dwa/health', String, queue_size=1)
        self.state_pub = rospy.Publisher('/lab3_dwa/state', String, queue_size=1)
        self.token = ''
        self.goal = None
        self.started = 0.0
        self.retry_at = 0.0
        self.pose = self.progress_pose = None
        self.progress_at = 0.0
        self.retries = 0
        self.exhausted = False
        self.sent = False
        self.weak_since = self.weak_pose = None
        self.weak_at = -math.inf
        self.pose_window = deque()
        self.cloud_at = self.odom_at = 0.0
        self.server_ready = False
        self.configure = rospy.ServiceProxy('/lab3_dwa/move_base/DWAPlannerROS/set_parameters', Reconfigure)
        rospy.Subscriber('/lab3_dwa/request', String, self.request, queue_size=5)
        rospy.Subscriber('/lab3_dwa/raw_cmd', Twist, self.velocity, queue_size=1)
        rospy.Subscriber('/lab26/obstacles', PointCloud2, self.cloud, queue_size=1)
        rospy.Subscriber('/lab26/chassis_odom', Odometry, self.odom, queue_size=1)
        rospy.Timer(rospy.Duration(.1), self.tick)

    def cloud(self, _):
        self.cloud_at = time.monotonic()

    def odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if not all(map(math.isfinite, (p.x, p.y, yaw))):
            return
        with self.lock:
            self.pose = (p.x, p.y, yaw)
            self.odom_at = time.monotonic()

    def ready(self):
        now = time.monotonic()
        return self.server_ready and now-self.cloud_at < .5 and now-self.odom_at < .5

    def request(self, msg):
        with self.lock:
            fields = msg.data.split()
            if len(fields) == 2 and fields[0] in ('forward', 'reverse') and fields[1] == self.token:
                self.set_reverse(fields[0] == 'reverse')
                return
            if len(fields) == 2 and fields[0] == 'stop':
                if fields[1] == self.token:
                    self.client.cancel_goal()
                    self.goal = None
                    self.token = ''
                return
            if len(fields) != 7 or fields[0] != 'goal' or fields[6] != 'camera_init':
                return
            try:
                x, y, yaw = map(float, fields[2:5])
                if not all(map(math.isfinite, (x, y, yaw))):
                    return
            except ValueError:
                return
            if fields[1] == self.token:
                return
            self.client.cancel_goal()
            self.token = fields[1]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = fields[6]
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = math.sin(yaw/2)
            goal.target_pose.pose.orientation.w = math.cos(yaw/2)
            self.goal = goal
            self.retry_at = 0.0
            self.retries = 0
            self.exhausted = self.sent = False
            self.set_reverse(True)

    def set_reverse(self, allowed):
        config = Config()
        config.doubles = [DoubleParameter('min_vel_x', -.10 if allowed else 0.0)]
        try:
            self.configure(config)
        except rospy.ServiceException as exc:
            rospy.logwarn('DWA reverse configuration unavailable: %s', exc)

    def send_goal(self):
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.started = time.monotonic()
        self.progress_at = self.started
        self.progress_pose = self.pose
        self.sent = True
        self.weak_since = self.weak_pose = None
        self.weak_at = -math.inf
        self.pose_window.clear()
        self.client.send_goal(self.goal)

    def velocity(self, msg):
        with self.lock:
            if not self.token or not self.goal or not self.ready() or not self.sent or self.exhausted:
                return
            if self.client.get_state() != GoalStatus.ACTIVE or time.monotonic()-self.started < .25:
                return
            values = (msg.linear.x, msg.angular.z)
            if not all(map(math.isfinite, values)) or not -.1001 <= values[0] <= .5001 or abs(values[1]) > .7001:
                return
            self.observe_command(*values)
            out = TwistStamped()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.token
            out.twist = msg
            self.output.publish(out)

    def observe_command(self, v, w):
        # Detect sustained, fresh but mechanically ineffective proposals. A
        # ramp or a slow turn with measured progress must stay with DWA.
        now = time.monotonic()
        goal = self.goal.target_pose.pose.position
        if (self.pose is None or abs(v) > .075 or abs(w) > .16 or
                math.hypot(goal.x-self.pose[0], goal.y-self.pose[1]) < .60):
            self.weak_since = self.weak_pose = None
            return
        if self.weak_pose is None or now-self.weak_at > .35:
            self.weak_since, self.weak_pose = now, self.pose
        x, y, yaw = self.weak_pose
        angle = self.pose[2]-yaw
        if (math.hypot(self.pose[0]-x, self.pose[1]-y) >= .025 or
                abs(math.atan2(math.sin(angle), math.cos(angle))) >= .045):
            self.weak_since, self.weak_pose = now, self.pose
        self.weak_at = now

    def confined_motion(self, now):
        # Micro-turns and short reversals can reset instantaneous progress
        # repeatedly even though the robot remains in the same patch of floor.
        samples = self.pose_window
        if self.pose is None or (samples and now-samples[-1][0] > .4):
            samples.clear()
        if self.pose is None:
            return False
        samples.append((now, *self.pose))
        while len(samples)>1 and now-samples[1][0] >= 5.0:
            samples.popleft()
        stamp, x, y, yaw = samples[0]
        if now-stamp < 5.0:
            return False
        def yaw_change(angle):
            return abs(math.atan2(math.sin(angle-yaw), math.cos(angle-yaw)))
        px, py, heading = self.pose
        return (math.hypot(px-x, py-y)<.10 and yaw_change(heading)<.15 and
                all(math.hypot(sx-x, sy-y)<.24 and yaw_change(sa)<.35
                    for _, sx, sy, sa in samples))

    def tick(self, _):
        with self.lock:
            if not self.server_ready:
                self.server_ready = self.client.wait_for_server(rospy.Duration(.01))
            self.health.publish(String('ready' if self.ready() else 'waiting'))
            if not self.goal:
                return
            if self.exhausted:
                self.state_pub.publish(String(self.token+' STALLED'))
                return
            now = time.monotonic()
            if not self.sent:
                if self.ready() and now >= self.retry_at:
                    self.send_goal()
                else:
                    self.state_pub.publish(String(self.token+' PLANNING'))
                return
            state = self.client.get_state()
            label = 'ACTIVE' if state == GoalStatus.ACTIVE else 'PLANNING'
            confined = False
            if state == GoalStatus.ACTIVE and self.ready():
                confined = self.confined_motion(now)
            else:
                self.pose_window.clear()
            if self.pose and self.progress_pose and self.ready():
                x, y, yaw = self.progress_pose
                angle = self.pose[2]-yaw
                if (math.hypot(self.pose[0]-x, self.pose[1]-y) >= .06 or
                        abs(math.atan2(math.sin(angle), math.cos(angle))) >= .12):
                    self.progress_pose = self.pose
                    self.progress_at = now
            if state == GoalStatus.SUCCEEDED:
                label = 'DONE'
            elif confined or (state == GoalStatus.ACTIVE and self.ready() and self.weak_since is not None
                              and now-self.weak_at <= .35 and now-self.weak_since >= 1.8):
                self.client.cancel_goal()
                self.sent = False
                self.exhausted = True
                label = 'STALLED'
            elif self.ready() and (state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST)
                                   or now-self.progress_at >= 5.0):
                label = 'BLOCKED'
                self.client.cancel_goal()
                self.sent = False
                if self.retries < 1:
                    self.retries += 1
                    self.retry_at = now+1.0
                else:
                    self.exhausted = True
                    label = 'STALLED'
            self.state_pub.publish(String(self.token+' '+label))


if __name__ == '__main__':
    rospy.init_node('bridge')
    Bridge()
    rospy.spin()
