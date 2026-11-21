#!/usr/bin/env python3
"""Measure actual sensor messages using wall time, without publishing commands."""
import collections
import json
import math
import time
import rospy
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

rospy.init_node('lab26_check_topics', anonymous=True)
records = collections.defaultdict(list)
last = {}
def receive(msg, topic):
    records[topic].append(time.monotonic())
    last[topic] = msg
subs = [rospy.Subscriber(t, cls, receive, callback_args=t, queue_size=2)
        for t, cls in [('/imu', Imu), ('/rslidar_points', PointCloud2),
                       ('/velodyne_points', PointCloud2), ('/Odometry', Odometry),
                       ('/car_pose', Pose2D)]]
time.sleep(5)
result = {}
for t in ['/imu', '/rslidar_points', '/velodyne_points', '/Odometry', '/car_pose']:
    ts = records[t]
    out = {'count': len(ts), 'hz': round((len(ts)-1)/(ts[-1]-ts[0]), 2) if len(ts)>1 else 0}
    msg = last.get(t)
    if isinstance(msg, Imu):
        out['acceleration'] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        out['gyro'] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    elif isinstance(msg, PointCloud2):
        out['points'] = msg.width*msg.height
        out['fields'] = [f.name for f in msg.fields]
        if t == '/velodyne_points' and 'time' in out['fields']:
            pts = list(point_cloud2.read_points(msg, field_names=['ring', 'time'], skip_nans=True))
            out['ring_range'] = [min(p[0] for p in pts), max(p[0] for p in pts)] if pts else []
            out['time_range'] = [min(p[1] for p in pts), max(p[1] for p in pts)] if pts else []
    elif isinstance(msg, Pose2D):
        out['pose'] = [msg.x, msg.y, msg.theta]
    elif isinstance(msg, Odometry):
        out['position'] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        out['frame'] = msg.header.frame_id
        out['age'] = (rospy.Time.now()-msg.header.stamp).to_sec()
    result[t] = out
print(json.dumps(result, indent=2))
