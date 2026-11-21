"""Occupied columns of ROS VoxelLayer, expressed at the vehicle's current pose."""
import math
import threading

import numpy as np
import rospy
from costmap_2d.msg import VoxelGrid


class VoxelMemory:
    def __init__(self):
        self._lock = threading.Lock()
        self._empty = np.empty((0, 2), dtype=float)
        self._world = self._empty
        self._stamp = float('-inf')
        self._cutoff = float('-inf')
        self.cell_radius = 0.0
        self.subscriber = rospy.Subscriber('/move_base/local_costmap/obstacles/voxel_grid',
                                           VoxelGrid, self._receive, queue_size=1)

    def _receive(self, grid):
        if grid.header.frame_id.lstrip('/') != 'camera_init' or len(grid.data) != grid.size_x * grid.size_y:
            return
        stamp = grid.header.stamp.to_sec()
        columns = np.asarray(grid.data, dtype=np.uint32).reshape(grid.size_y, grid.size_x)
        # In voxel_grid the upper sixteen bits are marked heights. The low bits
        # can also denote unknown space, which is never an observed obstacle.
        row, col = np.nonzero(columns >> 16)
        world = np.column_stack((grid.origin.x + (col + .5) * grid.resolutions.x,
                                 grid.origin.y + (row + .5) * grid.resolutions.y))
        with self._lock:
            if stamp <= self._cutoff or stamp < self._stamp:
                return
            self._world, self._stamp = world, stamp
            self.cell_radius = .5 * math.hypot(grid.resolutions.x, grid.resolutions.y)

    def clear(self, now):
        with self._lock:
            self._cutoff = now
            self._world, self._stamp = self._empty, float('-inf')

    def body_points(self, pose, now):
        with self._lock:
            world, stamp = self._world, self._stamp
        if pose is None or not 0 <= now - stamp < .6 or not len(world):
            return self._empty
        delta = world - pose[:2]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                -s * delta[:, 0] + c * delta[:, 1]))
