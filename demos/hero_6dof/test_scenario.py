# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Independently differentiate reference motion to validate synthetic IMU data."""

import unittest

import numpy as np
from scenario import DURATION, GRAVITY, motion


class ScenarioTest(unittest.TestCase):
    def test_kinematics(self):
        """Check acceleration, body gyro, velocity and orthonormal attitude."""
        h = 1e-4
        for t in (0., 2.5, 4., 5.5, 9., 18., 24., 30., 37., 45.):
            a, s, b = motion(t-h), motion(t), motion(t+h)
            np.testing.assert_allclose(s['v'], (b['p']-a['p'])/(2*h), atol=2e-7)
            acceleration = (b['p']-2*s['p']+a['p'])/h**2
            np.testing.assert_allclose(s['r']@s['f'] - [0, 0, GRAVITY],
                                       acceleration, atol=4e-6)
            skew = s['r'].T @ ((b['r']-a['r'])/(2*h))
            np.testing.assert_allclose(s['w'], [skew[2, 1], skew[0, 2], skew[1, 0]],
                                       atol=2e-8)
            np.testing.assert_allclose(s['r'].T@s['v'], [s['speed'], 0, 0], atol=1e-10)
            np.testing.assert_allclose(s['r'].T@s['r'], np.eye(3), atol=1e-14)
            self.assertAlmostEqual(np.linalg.norm(s['q']), 1., places=14)
            x, y, z, w = s['q']
            q_matrix = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            np.testing.assert_allclose(q_matrix, s['r'], atol=1e-14)

    def test_excitation_and_closed_lap(self):
        """Ensure this is genuinely spatial and has a stationary tilted start."""
        states = [motion(t) for t in np.linspace(0, DURATION, 1000)]
        np.testing.assert_allclose(states[0]['p'], states[-1]['p'], atol=1e-12)
        np.testing.assert_allclose(motion(0)['v'], 0.)
        np.testing.assert_allclose(motion(0)['w'], 0.)
        p = np.array([s['p'] for s in states])
        rpy = np.array([s['rpy'] for s in states])
        self.assertGreater(np.ptp(p[:, 2]), 15.)
        self.assertGreater(np.max(np.abs(rpy[:, 0])), .25)
        self.assertGreater(np.max(np.abs(rpy[:, 1])), .25)


if __name__ == '__main__':
    unittest.main()
