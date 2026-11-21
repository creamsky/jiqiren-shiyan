"""Exercise the deployed bridge callbacks without a ROS master or a chassis."""
from pathlib import Path
from collections import deque
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dwa"))
from bridge import Bridge, GoalStatus, MoveBaseGoal


class BridgeTest(unittest.TestCase):
    def setUp(self):
        b = self.bridge = Bridge.__new__(Bridge)
        b.lock = threading.RLock()
        b.server_ready = True
        b.ready = Mock(return_value=True)
        b.state_pub, b.health, b.client = Mock(), Mock(), Mock()
        b.client.get_state.return_value = GoalStatus.ACTIVE
        b.token, b.goal = "test", MoveBaseGoal()
        b.pose = b.progress_pose = (0., 0., 0.)
        b.progress_at = b.started = b.retry_at = 0.
        b.exhausted, b.sent, b.retries = False, True, 0
        b.weak_since = b.weak_pose = None
        b.weak_at = float('-inf')
        b.pose_window = deque()
        b.goal.target_pose.pose.position.x = 6.

    def tick(self, seconds):
        with patch('bridge.time.monotonic', return_value=seconds), patch('bridge.rospy.Time.now', return_value=self.bridge.goal.target_pose.header.stamp):
            self.bridge.tick(None)

    def test_short_retreat_followed_by_stationary_twiddling(self):
        b = self.bridge
        for t in range(1, 17):
            b.pose = (.002*(t%2), 0., .003*(t%2))
            self.tick(t)
        self.assertEqual(b.client.send_goal.call_count, 1)
        self.assertTrue(b.exhausted)
        self.assertEqual(b.state_pub.publish.call_args[0][0].data, 'test STALLED')

    def test_measurable_turn_is_progress(self):
        for t in range(1, 20):
            self.bridge.pose = (0., 0., .05*t)
            self.tick(t)
        self.bridge.client.cancel_goal.assert_not_called()

    def test_ineffective_pivot_hands_over_without_repeating_goal(self):
        b = self.bridge
        for i in range(11):
            with patch('bridge.time.monotonic', return_value=i*.2):
                b.observe_command(0., .07)
            self.tick(i*.2)
            if b.exhausted:
                break
        self.assertTrue(b.exhausted)
        b.client.send_goal.assert_not_called()

    def test_slow_executed_pivot_is_retained(self):
        b = self.bridge
        for i in range(22):
            b.pose = (0., 0., i*.014)
            with patch('bridge.time.monotonic', return_value=i*.2):
                b.observe_command(0., .07)
            self.tick(i*.2)
        b.client.cancel_goal.assert_not_called()

    def test_no_recovery_with_missing_input(self):
        self.bridge.ready.return_value = False
        self.tick(20)
        self.bridge.client.cancel_goal.assert_not_called()
        self.assertFalse(self.bridge.exhausted)

    def test_wobble_is_not_progress(self):
        for i in range(65):
            # Larger than instantaneous progress thresholds, but oscillatory.
            self.bridge.pose = (.075 if (i//5)%2 else 0.,0.,.14 if (i//5)%2 else 0.)
            self.tick(i*.1)
        self.assertTrue(self.bridge.exhausted)
        self.bridge.client.send_goal.assert_not_called()

    def test_productive_pivot_keeps_authority(self):
        for i in range(80):
            self.bridge.pose = (0.,0.,.012*i)
            self.tick(i*.1)
        self.bridge.client.cancel_goal.assert_not_called()


if __name__ == '__main__':
    unittest.main()
