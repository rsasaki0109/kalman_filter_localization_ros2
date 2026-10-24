#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Seeded banked, climbing 6DoF trajectory; metres, seconds and ENU/FLU axes."""

import math

import numpy as np

DURATION, START, SEED = 48.0, 100.0, 20261024
OUTAGE = (18.0, 30.0)
POSE_DELAY = 0.15
GRAVITY = 9.80665


def rotation(roll, pitch, yaw):
    """Body-to-world ZYX rotation, without a small-angle approximation."""
    sr, cr, sp, cp, sy, cy = (f(a) for a in (roll, pitch, yaw)
                             for f in (math.sin, math.cos))
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def quaternion(roll, pitch, yaw):
    """ROS xyzw quaternion for the same body-to-world rotation."""
    sr, cr, sp, cp, sy, cy = (f(a/2) for a in (roll, pitch, yaw)
                             for f in (math.sin, math.cos))
    return np.array([sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                     cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy])


def motion(t):
    """Return analytic position, velocity, orientation, IMU and forward speed."""
    ramp = 4.
    omega = 2*math.pi / (DURATION - 2. - ramp/2)
    u = min(1., max(0., (t-2.)/ramp))
    rate = omega*(3*u*u - 2*u**3)
    rate_dot = omega*6*u*(1-u)/ramp
    angle = omega*ramp*(u**3 - .5*u**4)
    if t > 6.:
        angle = omega*(t - 4.)
    s, c = math.sin(angle), math.cos(angle)
    s2, c2 = math.sin(2*angle), math.cos(2*angle)
    p = np.array([45*s, 30*(1-c), 6*s + 3*s2])
    d = np.array([45*c, 30*s, 6*c + 6*c2])
    dd = np.array([-45*s, 30*c, -6*s - 12*s2])
    h = math.hypot(d[0], d[1])
    hp = np.dot(d[:2], dd[:2])/h
    roll, pitch, yaw = .22*s - .10*s2, -math.atan2(d[2], h), math.atan2(d[1], d[0])
    rp = .22*c - .20*c2
    pp = -(h*dd[2] - d[2]*hp)/(h*h + d[2]*d[2])
    yp = (d[0]*dd[1] - d[1]*dd[0])/(h*h)
    sr, cr, sp, cp = math.sin(roll), math.cos(roll), math.sin(pitch), math.cos(pitch)
    w = rate*np.array([rp - yp*sp, pp*cr + yp*sr*cp, -pp*sr + yp*cr*cp])
    r = rotation(roll, pitch, yaw)
    acceleration = dd*rate*rate + d*rate_dot
    return dict(p=p, v=d*rate, r=r, q=quaternion(roll, pitch, yaw),
                rpy=np.array([roll, pitch, yaw]), w=w,
                f=r.T @ (acceleration + np.array([0., 0., GRAVITY])),
                speed=float(np.linalg.norm(d)*rate))
