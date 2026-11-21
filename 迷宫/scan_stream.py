"""Read synchronized, motion-compensated ROS observations. This node has no publishers."""
import json
import math
import threading
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
    rospy.init_node("maze_map_reader", anonymous=True)
    transforms = tf2_ros.Buffer(rospy.Duration(6))
    listener = tf2_ros.TransformListener(transforms)
    topics = {name for name, _ in rospy.get_published_topics("/lab26")}
    full_scan = "/lab26/maze_obstacles" in topics
    obstacle_topic = "/lab26/maze_obstacles" if full_scan else "/lab26/obstacles"
    obstacles = message_filters.Subscriber(obstacle_topic, PointCloud2, queue_size=2, buff_size=2**21)
    rays = message_filters.Subscriber("/lab26/clearing_rays", PointCloud2, queue_size=2)
    sync = message_filters.TimeSynchronizer([obstacles, rays], 5)
    mount = rospy.get_param("/mapping/extrinsic_T", [])
    axes = rospy.get_param("/mapping/extrinsic_R", [])
    calibrated = (not rospy.get_param("/mapping/extrinsic_est_en", True)
                  and isinstance(mount, list) and len(mount) == 3
                  and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) < 2 for v in mount)
                  and isinstance(axes, list) and len(axes) == 9
                  and all(isinstance(v, (int, float)) and math.isfinite(v) for v in axes))
    print(json.dumps(dict(type="lidar_mount", lidarOffset=mount if calibrated else None)), flush=True)

    pending_lock = threading.Lock()
    pending = None

    def receive(marks, clearing):
        # Keep the synchronized ROS callback short. Conversion and SSH
        # backpressure must not hold the TimeSynchronizer's lock.
        nonlocal pending
        with pending_lock:
            pending = (marks, clearing)

    def convert(marks, clearing, pose, generation):
        frame = marks.header.frame_id.lstrip("/")
        if frame != "body" or clearing.header.frame_id.lstrip("/") != frame:
            return
        p, q = pose.translation, pose.rotation
        if not all(map(math.isfinite, (p.x, p.y, p.z, q.x, q.y, q.z, q.w))):
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        heights = {}
        origin = [p.x, p.y, p.z]
        if calibrated:
            rotation = ((1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.w*q.z), 2*(q.x*q.z+q.w*q.y)),
                        (2*(q.x*q.y+q.w*q.z), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.w*q.x)),
                        (2*(q.x*q.z-q.w*q.y), 2*(q.y*q.z+q.w*q.x), 1-2*(q.x*q.x+q.y*q.y)))
            origin = [pos + sum(r*t for r, t in zip(row, mount)) for pos, row in zip(origin, rotation)]
        if full_scan and calibrated:
            # Raw beam heights come from Velodyne. Convert the sensor axes to
            # gravity-aligned world axes, then express XY in the yaw-only grid frame.
            world = [[sum(row[k]*axes[3*k+j] for k in range(3)) for j in range(3)] for row in rotation]
            c, s = math.cos(yaw), math.sin(yaw)
            local_x = [c*a+s*b for a, b in zip(world[0], world[1])]
            local_y = [-s*a+c*b for a, b in zip(world[0], world[1])]
            for x, y, z in point_cloud2.read_points(marks, field_names=("x", "y", "z"), skip_nans=True):
                if math.hypot(x, y) > 6 or not all(map(math.isfinite, (x, y, z))):
                    continue
                dx, dy, dz = (row[0]*x+row[1]*y+row[2]*z for row in (local_x, local_y, world[2]))
                if math.hypot(dx, dy) <= 6:
                    heights[(math.floor(dx/.06), math.floor(dy/.06), math.floor(dz/.08))] = [round(dx,3), round(dy,3), round(dz,3)]
        free = planar_points(clearing, 0.035)
        if not full_scan:
            # Earlier controllers omit rear obstacle marks for Navfn. Rear
            # clearing rays alone cannot establish free space behind the car.
            free = [point for point in free if point[0] >= 0]
        return dict(generation=generation, frame="camera_init", stamp=marks.header.stamp.to_sec(),
                    coverage="full" if full_scan else "front", sensorZ=origin[2], rayOrigin=origin[:2],
                    lidarOffset=mount if calibrated else None, heightHits=list(heights.values()),
                    pose=[p.x, p.y, yaw], marks=planar_points(marks, 0.06), rays=free)

    sync.registerCallback(receive)
    last_sent = last_status = time.monotonic()
    last_stamp = rospy.Time(0)
    while not rospy.is_shutdown():
        time.sleep(0.02)
        now = time.monotonic()
        if now - last_sent < 0.18:
            continue
        with pending_lock:
            pair = pending
        issue = "scan_pair"
        sample = None
        if pair is not None:
            marks, clearing = pair
            stamp = marks.header.stamp
            age = (rospy.Time.now() - stamp).to_sec()
            if stamp > last_stamp and -0.002 <= age < 0.6:
                try:
                    pose = transforms.lookup_transform("camera_init", "body", stamp).transform
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    issue = "pose"
                else:
                    try:
                        generation = str(rospy.get_param("/lab26/map_generation"))
                    except (KeyError, rospy.ROSException):
                        issue = "generation"
                    else:
                        sample = convert(marks, clearing, pose, generation)
                        last_stamp = stamp
                        if sample is None:
                            issue = "pose"
        try:
            if sample is not None:
                print(json.dumps(sample, allow_nan=False, separators=(",", ":")), flush=True)
                last_sent = now
            elif now - last_sent > 0.7 and now - last_status > 0.5:
                print(json.dumps(dict(type="scan_status", issue=issue)), flush=True)
                last_status = now
        except BrokenPipeError:
            return


if __name__ == "__main__":
    run()
