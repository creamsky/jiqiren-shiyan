#!/usr/bin/env python3
"""Export FAST-LIO odometry and world clouds to the localization bag format."""

import copy
import math
import os
from pathlib import Path
import threading

import numpy as np


def normalized_quaternion(values):
    q = np.asarray(values, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("orientation must contain four finite values")
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("orientation quaternion has zero length")
    return q / norm


def pose_matrix(position, quaternion):
    """Return world-from-body transform, quaternion order x y z w."""
    p = np.asarray(position, dtype=np.float64)
    if p.shape != (3,) or not np.isfinite(p).all():
        raise ValueError("position must contain three finite values")
    x, y, z, w = normalized_quaternion(quaternion)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    result[:3, 3] = p
    return result


def world_to_body(points, world_from_body):
    """Convert Nx4 x/y/z/intensity world points to the pose's body frame."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError("point cloud must have x y z intensity columns")
    values = values[np.isfinite(values).all(axis=1)]
    if not len(values):
        raise ValueError("point cloud contains no finite points")
    result = values.copy()
    result[:, :3] = (values[:, :3] - world_from_body[:3, 3]).dot(
        world_from_body[:3, :3])
    return result.astype(np.float32)


def is_keyframe(current, previous, distance, angle_radians):
    if previous is None:
        return True
    translation = np.linalg.norm(current[:3, 3] - previous[:3, 3])
    rotation = previous[:3, :3].T.dot(current[:3, :3])
    angle = math.acos(float(np.clip((np.trace(rotation)-1) / 2, -1, 1)))
    return translation >= distance or angle >= angle_radians


class KeyframeMapRecorder:
    def __init__(self):
        import message_filters
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2

        self.ros = rospy
        self.lock = threading.Lock()
        self.count = 0
        self.closed = False
        self.failed = False
        self.last_pose = None
        self.last_stamp = None
        self.world_frame = None
        self.bag = None
        self.lock_fd = None
        self.distance = float(rospy.get_param("~keyframe_distance", 1.5))
        self.angle = math.radians(float(rospy.get_param("~keyframe_angle_deg", 15)))
        if not (math.isfinite(self.distance) and self.distance > 0
                and math.isfinite(self.angle) and 0 < self.angle <= math.pi):
            raise ValueError("keyframe distance/angle must be positive and finite")
        output = rospy.get_param("~output_bag", "")
        if not output or not Path(output).is_absolute() or not output.endswith(".bag"):
            raise ValueError("output_bag must be an absolute path ending in .bag")
        self.output = Path(output)
        self.active = Path(output + ".active")
        self.lock_path = Path(output + ".lock")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if self.output.exists() or self.active.exists():
            raise ValueError("map already exists; choose a new name: " + output)
        self.lock_fd = os.open(str(self.lock_path),
                               os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        try:
            self.bag = rosbag.Bag(str(self.active), "w")
        except Exception:
            os.close(self.lock_fd)
            self.lock_path.unlink()
            raise
        rospy.on_shutdown(self.close)
        odom = message_filters.Subscriber("/Odometry", Odometry)
        cloud = message_filters.Subscriber("/cloud_registered", PointCloud2)
        # FAST-LIO publishes both headers with the same lidar_end_time.
        self.sync = message_filters.TimeSynchronizer([odom, cloud], queue_size=20)
        self.sync.registerCallback(self.callback)
        rospy.loginfo("Recording keyframe map to %s; stop mapping with Ctrl+C.", output)

    def callback(self, odometry, cloud):
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointField
        from std_msgs.msg import Header

        try:
            with self.lock:
                if self.closed:
                    return
                if (not odometry.header.frame_id or
                        odometry.header.frame_id != cloud.header.frame_id):
                    raise ValueError("odometry and world cloud frames must match")
                if self.world_frame not in (None, odometry.header.frame_id):
                    raise ValueError("world frame changed during recording")
                stamp = odometry.header.stamp
                if stamp.to_sec() <= 0 or stamp != cloud.header.stamp:
                    raise ValueError("cloud and odometry need equal nonzero timestamps")
                if self.last_stamp is not None and stamp <= self.last_stamp:
                    raise ValueError("odometry time went backwards; do not mix sessions")
                pose = odometry.pose.pose
                quaternion = normalized_quaternion([
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w])
                transform = pose_matrix([
                    pose.position.x, pose.position.y, pose.position.z], quaternion)
                if not is_keyframe(transform, self.last_pose, self.distance, self.angle):
                    return
                names = {field.name for field in cloud.fields}
                if not {"x", "y", "z", "intensity"}.issubset(names):
                    raise ValueError("world cloud needs x y z intensity fields")
                # read_points returns fields in memory-offset order. Reorder by name
                # so arbitrary valid PointCloud2 field layouts are handled correctly.
                fields = sorted((f for f in cloud.fields
                                 if f.name in {"x", "y", "z", "intensity"}),
                                key=lambda field: field.offset)
                field_names = [f.name for f in fields]
                if any(f.count != 1 for f in fields):
                    raise ValueError("cloud x y z intensity fields must be scalars")
                values = np.asarray(list(point_cloud2.read_points(
                    cloud, field_names=field_names, skip_nans=True)), dtype=np.float64)
                if values.size == 0:
                    self.ros.logwarn_throttle(5, "Empty cloud; no keyframe written.")
                    return
                values = values[:, [field_names.index(n)
                                    for n in ("x", "y", "z", "intensity")]]
                local = world_to_body(values, transform)
                fields_out = [PointField(name, i*4, PointField.FLOAT32, 1)
                              for i, name in enumerate(("x", "y", "z", "intensity"))]
                header = Header(seq=self.count, stamp=stamp, frame_id="body")
                local_cloud = point_cloud2.create_cloud(header, fields_out, local.tolist())
                saved_pose = PoseStamped()
                saved_pose.header = Header(seq=self.count, stamp=stamp,
                                           frame_id=odometry.header.frame_id)
                saved_pose.pose = copy.deepcopy(pose)
                saved_pose.pose.orientation.x = float(quaternion[0])
                saved_pose.pose.orientation.y = float(quaternion[1])
                saved_pose.pose.orientation.z = float(quaternion[2])
                saved_pose.pose.orientation.w = float(quaternion[3])
                self.bag.write("/keyframe_pcd", local_cloud, t=stamp)
                self.bag.write("/keyframe_pose", saved_pose, t=stamp)
                self.last_pose = transform
                self.last_stamp = stamp
                self.world_frame = odometry.header.frame_id
                self.count += 1
                self.ros.loginfo("Saved keyframe %d (%d points)", self.count, len(local))
        except Exception as error:
            self.failed = True
            self.ros.logerr("Map recorder failed: %s", error)
            self.ros.signal_shutdown("map recorder failed")

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            try:
                if self.bag is not None:
                    self.bag.close()
                if self.count and not self.failed:
                    if self.output.exists():
                        raise RuntimeError("refusing to overwrite existing map")
                    # Same-directory hard-link publication is atomic and cannot
                    # overwrite an existing map. Both paths are on the same disk.
                    os.link(str(self.active), str(self.output))
                    self.active.unlink()
                    self.ros.loginfo("Map saved: %s (%d keyframes)",
                                     self.output, self.count)
                else:
                    self.failed = True
                    self.ros.logerr("Map not published (failed or no data). "
                                   "Partial file, if any: %s", self.active)
            except Exception as error:
                self.failed = True
                self.ros.logerr("Cannot finalize map: %s; retained %s", error, self.active)
            finally:
                if self.lock_fd is not None:
                    os.close(self.lock_fd)
                    self.lock_fd = None
                    self.lock_path.unlink()


def main():
    import rospy
    rospy.init_node("keyframe_map_recorder")
    try:
        recorder = KeyframeMapRecorder()
        rospy.spin()
        recorder.close()
        return 1 if recorder.failed else 0
    except Exception as error:
        rospy.logfatal("Cannot start map recorder: %s", error)
        rospy.signal_shutdown("map recorder startup failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
