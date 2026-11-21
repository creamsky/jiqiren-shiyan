"""Read synchronized, motion-compensated ROS observations. This node has no publishers."""
import json
import math
import time

import message_filters
import rospy
import tf2_ros
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


def planar_points(message, resolution):
    cells = {}
    for x, y in point_cloud2.read_points(message, field_names=("x", "y"), skip_nans=True):
        if not (math.isfinite(x) and math.isfinite(y)) or math.hypot(x, y) > 6:
            continue
        cells[(math.floor(x / resolution), math.floor(y / resolution))] = [round(x, 3), round(y, 3)]
    return list(cells.values())


def run():
    rospy.init_node("maze_map_reader", anonymous=True, disable_signals=True)
    transforms = tf2_ros.Buffer(rospy.Duration(6))
    listener = tf2_ros.TransformListener(transforms)
    topics = {name for name, _ in rospy.get_published_topics("/lab26")}
    full_scan = "/lab26/maze_obstacles" in topics
    obstacle_topic = "/lab26/maze_obstacles" if full_scan else "/lab26/obstacles"
    obstacles = message_filters.Subscriber(obstacle_topic, PointCloud2, queue_size=2)
    rays = message_filters.Subscriber("/lab26/clearing_rays", PointCloud2, queue_size=2)
    sync = message_filters.TimeSynchronizer([obstacles, rays], 5)
    last = 0.0

    def receive(marks, clearing):
        nonlocal last
        now = time.monotonic()
        if now - last < 0.19:
            return
        last = now
        stamp = marks.header.stamp
        frame = marks.header.frame_id.lstrip("/")
        if frame != "body" or clearing.header.frame_id.lstrip("/") != frame:
            return
        if not 0 <= (rospy.Time.now() - stamp).to_sec() < 0.6:
            return
        try:
            pose = transforms.lookup_transform("camera_init", frame, stamp, rospy.Duration(0.18)).transform
            generation = str(rospy.get_param("/lab26/map_generation"))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, KeyError, rospy.ROSException):
            return
        p, q = pose.translation, pose.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        free = planar_points(clearing, 0.035)
        if not full_scan:
            # Earlier controllers omit rear obstacle marks for Navfn. Rear
            # clearing rays alone cannot establish free space behind the car.
            free = [point for point in free if point[0] >= 0]
        sample = dict(generation=generation, frame="camera_init", stamp=stamp.to_sec(),
                      coverage="full" if full_scan else "front",
                      pose=[p.x, p.y, yaw], marks=planar_points(marks, 0.06),
                      rays=free)
        try:
            print(json.dumps(sample, allow_nan=False, separators=(",", ":")), flush=True)
        except BrokenPipeError:
            rospy.signal_shutdown("map reader disconnected")

    sync.registerCallback(receive)
    rospy.spin()


if __name__ == "__main__":
    run()
