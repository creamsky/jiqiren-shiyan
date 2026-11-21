import importlib.util
import math
import os
from pathlib import Path
import tempfile
import threading
import unittest

import numpy as np

SCRIPT = (Path(__file__).resolve().parents[1] /
          "FAST-LIO-Localization/FAST-LIO-Localization-QN-master/src/"
          "fast_lio_localization_qn/scripts/record_keyframe_map.py")
spec = importlib.util.spec_from_file_location("recorder", SCRIPT)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class GeometryTests(unittest.TestCase):
    def test_translation_is_removed(self):
        transform = recorder.pose_matrix([10, 20, 30], [0, 0, 0, 1])
        result = recorder.world_to_body([[11, 22, 33, 7]], transform)
        np.testing.assert_allclose(result, [[1, 2, 3, 7]])

    def test_rotation_and_translation_round_trip(self):
        transform = recorder.pose_matrix([5, -3, 1], [0, 0, math.sqrt(.5), math.sqrt(.5)])
        local = np.array([[1, 0, 0, 4], [0, 1, 2, 9]], dtype=float)
        world = local.copy()
        world[:, :3] = local[:, :3].dot(transform[:3, :3].T) + transform[:3, 3]
        np.testing.assert_allclose(recorder.world_to_body(world, transform),
                                   local, atol=1e-6)

    def test_nonunit_quaternion_normalized(self):
        np.testing.assert_allclose(recorder.pose_matrix([0, 0, 0], [0, 0, 0, 2]), np.eye(4))

    def test_invalid_pose_rejected(self):
        for position, quaternion in [
            ([0, 0, 0], [0, 0, 0, 0]),
            ([float("nan"), 0, 0], [0, 0, 0, 1]),
            ([0, 0, 0], [0, 0, float("inf"), 1]),
        ]:
            with self.assertRaises(ValueError):
                recorder.pose_matrix(position, quaternion)

    def test_nonfinite_points_filtered_without_losing_intensity(self):
        result = recorder.world_to_body(
            [[1, 2, 3, 42], [float("nan"), 1, 2, 1]], np.eye(4))
        np.testing.assert_allclose(result, [[1, 2, 3, 42]])

    def test_empty_cloud_rejected(self):
        with self.assertRaises(ValueError):
            recorder.world_to_body(np.zeros((0, 4)), np.eye(4))

    def test_keyframe_uses_translation_and_rotation(self):
        first = np.eye(4)
        moved = recorder.pose_matrix([1.5, 0, 0], [0, 0, 0, 1])
        turned = recorder.pose_matrix([0, 0, 0], [0, 0, math.sin(.2), math.cos(.2)])
        self.assertTrue(recorder.is_keyframe(first, None, 1.5, .3))
        self.assertFalse(recorder.is_keyframe(first, first, 1.5, .3))
        self.assertTrue(recorder.is_keyframe(moved, first, 1.5, .3))
        self.assertTrue(recorder.is_keyframe(turned, first, 1.5, .3))


class MapPublicationTests(unittest.TestCase):
    """Exercise real file publication, without pretending to run ROS."""
    def recorder_for(self, folder, count):
        class Logger:
            def loginfo(self, *args):
                pass
            def logerr(self, *args):
                pass
        class Bag:
            def close(self):
                pass
        node = recorder.KeyframeMapRecorder.__new__(recorder.KeyframeMapRecorder)
        node.lock = threading.Lock()
        node.closed = False
        node.failed = False
        node.count = count
        node.ros = Logger()
        node.bag = Bag()
        node.output = Path(folder) / "map.bag"
        node.active = Path(folder) / "map.bag.active"
        node.active.write_bytes(b"completed test data")
        node.lock_path = Path(folder) / "map.bag.lock"
        node.lock_fd = os.open(str(node.lock_path), os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        return node

    def test_completed_map_published_and_close_is_idempotent(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            node = self.recorder_for(folder, 1)
            node.close()
            node.close()
            self.assertEqual(node.output.read_bytes(), b"completed test data")
            self.assertFalse(node.failed)
            self.assertFalse(node.active.exists())
            self.assertFalse(node.lock_path.exists())

    def test_existing_map_not_overwritten(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            node = self.recorder_for(folder, 1)
            node.output.write_bytes(b"original map")
            node.close()
            self.assertTrue(node.failed)
            self.assertEqual(node.output.read_bytes(), b"original map")
            self.assertTrue(node.active.exists())

    def test_zero_keyframes_are_failure(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            node = self.recorder_for(folder, 0)
            node.close()
            self.assertTrue(node.failed)
            self.assertFalse(node.output.exists())
            self.assertTrue(node.active.exists())


if __name__ == "__main__":
    unittest.main()
