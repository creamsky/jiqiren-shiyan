"""Read the fixed FAST-LIO lidar mounting parameters from the ROS master."""
import json
import math
import time

import rospy


def read_lidar_mount():
    mount = rospy.get_param("/mapping/extrinsic_T", [])
    axes = rospy.get_param("/mapping/extrinsic_R", [])
    calibrated = (not rospy.get_param("/mapping/extrinsic_est_en", True)
                  and isinstance(mount, list) and len(mount) == 3
                  and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) < 2 for v in mount)
                  and isinstance(axes, list) and len(axes) == 9
                  and all(isinstance(v, (int, float)) and math.isfinite(v) for v in axes))
    return (mount, axes) if calibrated else (None, None)


def send_lidar_mount(mount):
    print(json.dumps(dict(type="lidar_mount", lidarOffset=mount)), flush=True)


def watch_lidar_mount():
    rospy.init_node("console_lidar_mount", anonymous=True)
    try:
        while not rospy.is_shutdown():
            mount, _ = read_lidar_mount()
            send_lidar_mount(mount)
            time.sleep(2)
    except BrokenPipeError:
        return
