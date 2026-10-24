"""Check that synthetic sensor inputs agree with the reference kinematics."""

import math
import unittest

from generate import DURATION, motion


class MotionTest(unittest.TestCase):
    """Physical consistency matters more than a visually plausible route."""

    def test_body_acceleration_and_yaw_rate_match_world_motion(self):
        """Independently differentiate world position and heading."""
        dt = 0.0001
        for t in (3.5, 5.0, 6.5, 10.0, 22.0, 35.0, 47.0):
            with self.subTest(t=t):
                before, state, after = motion(t - dt), motion(t), motion(t + dt)
                yaw, ax_body, ay_body = state[2:5]
                ax_world = (after[0] - 2 * state[0] + before[0]) / dt ** 2
                ay_world = (after[1] - 2 * state[1] + before[1]) / dt ** 2
                self.assertAlmostEqual(
                    ax_body, math.cos(yaw) * ax_world + math.sin(yaw) * ay_world,
                    delta=0.00002)
                self.assertAlmostEqual(
                    ay_body, -math.sin(yaw) * ax_world + math.cos(yaw) * ay_world,
                    delta=0.00002)
                dyaw = math.atan2(math.sin(after[2] - before[2]),
                                  math.cos(after[2] - before[2])) / (2 * dt)
                self.assertAlmostEqual(state[5], dyaw, delta=0.00001)
                speed = math.hypot(after[0] - before[0], after[1] - before[1]) / (2 * dt)
                self.assertAlmostEqual(state[6], speed, delta=0.00001)

    def test_stationary_start_and_closed_lap(self):
        """Initialization is stationary and the final position closes the loop."""
        self.assertEqual(motion(0), (0, 0, 0, 0, 0, 0, 0))
        self.assertEqual(motion(2), motion(0))
        self.assertAlmostEqual(motion(DURATION)[0], 0.0, places=9)
        self.assertAlmostEqual(motion(DURATION)[1], 0.0, places=9)


if __name__ == '__main__':
    unittest.main()
