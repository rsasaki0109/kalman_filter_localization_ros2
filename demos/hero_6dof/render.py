#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Render measured 6DoF outputs as an auditable, headless README animation."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scenario import DURATION, OUTAGE, START, motion, rotation

W, H, S, FPS, SECONDS = 1120, 700, 2, 20, 14
BG, PANEL, GRID = '#080f1c', '#0e1b2b', '#192c40'
WHITE, MUTED, MINT, AMBER, PURPLE = '#ecf5ff', '#8da4bd', '#57efcc', '#ffbd6d', '#aa96ff'
FONT = Path('/usr/share/fonts/truetype/dejavu')


def font(size, bold=False, mono=False):
    name = 'DejaVuSansMono' if mono else 'DejaVuSans'
    return ImageFont.truetype(str(FONT/(name+('-Bold' if bold else '')+'.ttf')), size*S)


class Canvas:
    def __init__(self, image):
        self.draw = ImageDraw.Draw(image)

    def __getattr__(self, name):
        def draw(xy, *args, **kwargs):
            def scale(v):
                return tuple(scale(a) for a in v) if isinstance(v, (tuple, list)) else v*S
            for key in ('width', 'radius'):
                if key in kwargs:
                    kwargs[key] *= S
            return getattr(self.draw, name)(scale(xy), *args, **kwargs)
        return draw


def read(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Empty data: '+str(path))
    data = np.array([[float(row[key]) for key in rows[0]] for row in rows])
    if not np.isfinite(data).all() or not np.all(np.diff(data[:, 0]) > 0):
        raise ValueError('Nonfinite or nonmonotonic data: '+str(path))
    return data


def assess(data):
    """Refuse incomplete or inaccurate runs before any convincing visuals exist."""
    meta = json.loads((data/'scenario.json').read_text())
    ref, est = read(data/'reference.csv'), read(data/'estimate.csv')
    expected = round(DURATION*100)+1
    if len(ref) != expected or len(est) != expected:
        raise ValueError('Incomplete 100 Hz reference/output coverage')
    if not np.allclose(ref[:, 0], est[:, 0], atol=1e-7, rtol=0):
        raise ValueError('Every estimate must match its reference sensor timestamp')
    if np.max(np.diff(est[:, 0])) > .010001:
        raise ValueError('Missing output samples')
    for key, topic in (('received_imu', '/hero/imu'), ('received_pose', '/hero/pose'),
                       ('received_twist', '/hero/speed')):
        if meta[key] != meta['input_output_counts'][topic]:
            raise ValueError('Incomplete actual ROS input receipt: '+key)
    if meta['numerical_failures'] or not meta['rewind_count']:
        raise ValueError('Replay/numerical checks failed')
    norms = np.linalg.norm(est[:, 4:8], axis=1)
    if not np.allclose(norms, 1., atol=1e-8):
        raise ValueError('Non-unit recorded output quaternion')
    error = np.linalg.norm(est[:, 1:4]-ref[:, 1:4], axis=1)
    # Geodesic SO(3) error, with the quaternion sign ambiguity removed.
    dots = np.abs(np.sum(est[:, 4:8]*ref[:, 4:8], axis=1))
    angle = np.degrees(2*np.arccos(np.clip(dots, 0., 1.)))
    time = ref[:, 0]-START
    outage = (time >= OUTAGE[0]) & (time < OUTAGE[1])
    obs = read(data/'observations.csv')
    if np.any((obs[:, 0]-START >= OUTAGE[0]) & (obs[:, 0]-START < OUTAGE[1])):
        raise ValueError('Absolute observations exist inside the declared outage')
    metrics = dict(kind=meta['kind'], position_3d_rmse_m=float(np.sqrt(np.mean(error**2))),
                   max_position_3d_error_m=float(error.max()),
                   outage_max_position_3d_error_m=float(error[outage].max()),
                   max_attitude_geodesic_error_deg=float(angle.max()),
                   attitude_geodesic_rmse_deg=float(np.sqrt(np.mean(angle**2))),
                   height_span_m=float(np.ptp(ref[:, 3])),
                   max_abs_roll_deg=float(np.degrees(abs(ref[:, 8])).max()),
                   max_abs_pitch_deg=float(np.degrees(abs(ref[:, 9])).max()),
                   output_samples=len(est), rewind_count=meta['rewind_count'],
                   source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (data/'reference.csv', data/'estimate.csv',
                                            data/'observations.csv', data/'scenario.json')},
                   render_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    if (metrics['position_3d_rmse_m'] > 1. or metrics['max_position_3d_error_m'] > 3. or
            metrics['max_attitude_geodesic_error_deg'] > 2.):
        raise ValueError('Synthetic demo readiness limits exceeded: '+str(metrics))
    return ref, est, obs, error, angle, metrics


AZ, EL = math.radians(32), math.radians(33)
VIEW = np.array([[math.cos(AZ), -math.sin(AZ), 0],
                 [math.sin(EL)*math.sin(AZ), math.sin(EL)*math.cos(AZ), -math.cos(EL)]])


def project(p):
    """Equal metric scale on all three axes; fixed orthographic camera."""
    return np.array([392., 329.]) + 6.1*(np.asarray(p)-[0, 30, 0])@VIEW.T


def line(draw, points, fill, width=1):
    if len(points) > 1:
        draw.line([tuple(p) for p in points], fill=fill, width=width, joint='curve')


def dashed(draw, points, fill, width=1, stride=3):
    for i in range(0, len(points)-1, stride*2):
        line(draw, points[i:i+stride+1], fill, width)


def vehicle(draw, position, r):
    """Draw a rigid vehicle whose full attitude comes from recorded output."""
    corners = np.array([[2.2, 1., 0], [2.2, -1., 0], [-2.2, -1., 0], [-2.2, 1., 0]])
    base = project(position+corners@r.T)
    top = project(position+(corners+[0, 0, 1.3])@r.T)
    for i in range(4):
        j = (i+1) % 4
        draw.polygon([tuple(base[i]), tuple(base[j]), tuple(top[j]), tuple(top[i])],
                     fill='#196c69', outline='#2faaa0')
    draw.polygon([tuple(p) for p in top], fill=MINT, outline='#c2fff0')
    roof = project(position+np.array([[.8, .8, 1.32], [.8, -.8, 1.32],
                                     [-.5, -.8, 1.32], [-.5, .8, 1.32]])@r.T)
    draw.polygon([tuple(p) for p in roof], fill='#173d51')
    origin = project(position+[0, 0, 1.8])
    for axis, color, name in zip(np.eye(3)*5, ('#ff8c8e', '#8dedb8', '#83baff'), 'xyz'):
        end = project(position+r@axis+[0, 0, 1.8])
        line(draw, [origin, end], color, 2)
        draw.text(tuple(end+[2, -7]), name, font=font(10, bold=True), fill=color)


def chart(draw, box, time, values, color, bounds):
    x0, y0, x1, y1 = box
    lo, hi = bounds
    return np.column_stack([x0+(x1-x0)*time/DURATION,
                            y1-(y1-y0)*(values-lo)/(hi-lo)])


def render(data, output):
    ref, est, obs, error, angle, metrics = assess(data)
    output.mkdir(parents=True, exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    frames = output/'frames'
    frames.mkdir()
    time = ref[:, 0]-START
    track_t = np.linspace(0, DURATION, 321)
    states = [motion(t) for t in track_t]
    road = np.array([s['p'] for s in states])
    left = np.array([s['p'] + 1.8*s['r'][:, 1] for s in states])
    right = np.array([s['p'] - 1.8*s['r'][:, 1] for s in states])
    road_xy, left_xy, right_xy = project(road), project(left), project(right)
    est_xy = project(est[:, 1:4])
    base = Image.new('RGB', (W*S, H*S), BG)
    d = Canvas(base)
    d.text((26, 21), 'KALMAN FILTER LOCALIZATION', font=font(12, bold=True), fill=MINT)
    d.text((24, 45), 'Six degrees of freedom. One continuous estimate.',
           font=font(32, bold=True), fill=WHITE)
    d.text((26, 96), 'ROS 2 ESKF   /   SLOPES + BANKED TURNS   /   DELAYED OBSERVATIONS',
           font=font(12), fill=MUTED)
    d.rounded_rectangle((24, 140, 764, 516), radius=16, fill=PANEL)
    d.rounded_rectangle((782, 140, 1096, 516), radius=16, fill=PANEL)
    scene = base.copy()
    d = Canvas(scene)
    # Ground and elevation pillars provide depth without exaggerating z.
    for x in range(-50, 51, 10):
        line(d, project([[x, -4, -10], [x, 64, -10]]), GRID)
    for y in range(0, 61, 10):
        line(d, project([[-50, y, -10], [50, y, -10]]), GRID)
    floor = road.copy()
    floor[:, 2] = -10
    line(d, project(floor), '#132537', 9)
    for i in range(0, len(road), 24):
        line(d, project([floor[i], road[i]]), '#243b50', 1)
        ground = project(floor[i])
        d.ellipse((ground[0]-2, ground[1]-2, ground[0]+2, ground[1]+2), fill='#304b61')
    # The ribbon banks according to the reference road frame.
    for i in range(len(road)-1):
        brightness = int(12*(road[i, 2]+8)/16)
        color = (31+brightness, 51+brightness, 68+brightness)
        d.polygon([tuple(left_xy[i]), tuple(left_xy[i+1]),
                   tuple(right_xy[i+1]), tuple(right_xy[i])], fill=color)
    line(d, left_xy, '#3c536a')
    line(d, right_xy, '#3c536a')
    dashed(d, road_xy, '#8393a3', stride=2)
    d.text((44, 159), '3D TRAJECTORY', font=font(11, bold=True), fill=MUTED)
    d.text((44, 180), 'Fixed camera · equal XYZ scale', font=font(10), fill='#6d879f')
    # Compact coordinate key on the projected floor.
    origin = np.array([-44., 3., -10.])
    o = project(origin)
    for delta, name, color in (([10, 0, 0], 'E', '#ff8c8e'),
                               ([0, 10, 0], 'N', '#8dedb8'),
                               ([0, 0, 10], 'UP', '#83baff')):
        end = project(origin+delta)
        line(d, [o, end], color, 1)
        d.text(tuple(end+[3, -4]), name, font=font(9), fill=color)
    d.text((44, 488), '10 m per grid cell', font=font(10), fill=MUTED)
    line(d, [(466, 493), (488, 493)], MINT, 3)
    d.text((495, 486), 'ESKF', font=font(10), fill=WHITE)
    dashed(d, [(548+i*3, 493) for i in range(10)], '#8393a3', stride=2)
    d.text((585, 486), 'Reference', font=font(10), fill=MUTED)
    d.ellipse((670, 490, 676, 496), fill=PURPLE)
    d.text((682, 486), 'Pose', font=font(10), fill=MUTED)
    scene_mask = Image.new('L', base.size, 0)
    ImageDraw.Draw(scene_mask).rounded_rectangle(
        (24*S, 140*S, 764*S, 516*S), radius=16*S, fill=255)
    base.paste(scene, (0, 0), scene_mask)
    d = Canvas(base)
    # Static graph backgrounds with outage region and honest labelled scales.
    err_hi = max(1., math.ceil(float(error.max())*2)/2)
    boxes = [(58, 557, 550, 620), (646, 557, 1080, 620)]
    for box, title, lo, hi in ((boxes[0], 'HEIGHT [m]', -10., 10.),
                                (boxes[1], 'POSITION ERROR · 3D [m]', 0., err_hi)):
        x0, y0, x1, y1 = box
        d.text((x0-32, 531), title, font=font(10, bold=True), fill=MUTED)
        a, b = x0+(x1-x0)*OUTAGE[0]/DURATION, x0+(x1-x0)*OUTAGE[1]/DURATION
        d.rectangle((a, y0, b, y1), fill='#29251f')
        for y, v in ((y0, hi), ((y0+y1)/2, (lo+hi)/2), (y1, lo)):
            line(d, [(x0, y), (x1, y)], GRID)
            d.text((x0-30, y-6), '{:g}'.format(v), font=font(9, mono=True), fill=MUTED)
    ref_height = chart(d, boxes[0], time[::8], ref[::8, 3], None, (-10., 10.))
    dashed(d, ref_height, '#71879c', stride=2)
    height_xy = chart(d, boxes[0], time, est[:, 3], None, (-10., 10.))
    error_xy = chart(d, boxes[1], time, error, None, (0., err_hi))
    # Timeline segments are the input schedule, independent of estimator health.
    x0, x1, ty = 26, 1094, 655
    for a, b, color in ((0, OUTAGE[0], '#1c5c56'), (*OUTAGE, '#835a2d'),
                         (OUTAGE[1], DURATION, '#1c5c56')):
        d.line((x0+(x1-x0)*a/DURATION, ty, x0+(x1-x0)*b/DURATION, ty), fill=color, width=4)
    for x, label in ((26, 'POSE + IMU + SPEED'), (435, '12 s POSE OUTAGE'), (782, 'POSE RETURNS')):
        d.text((x, 636), label, font=font(9, bold=True), fill=MUTED)
    d.text((26, 677), 'SYNTHETIC INPUTS · ACTUAL ROS ESTIMATOR OUTPUT · 48 s replay in 14 s',
           font=font(10), fill=MUTED)
    d.text((888, 677), 'No planar constraints', font=font(10), fill=MUTED)
    first = None
    frame_count = round(FPS*SECONDS)
    for frame in range(frame_count):
        # Hold the last result, then explicitly dissolve to the start for the loop.
        scenario_t = min(DURATION, DURATION*frame/(frame_count-16))
        index = min(len(time)-1, round(scenario_t*100))
        image = base.copy()
        d = Canvas(image)
        recent = (obs[:, 0] <= START+scenario_t) & (obs[:, 0] >= START+scenario_t-4.)
        for xy in project(obs[recent, 1:4]):
            d.ellipse((xy[0]-2, xy[1]-2, xy[0]+2, xy[1]+2), fill=PURPLE)
        line(d, est_xy[:index+1:3], '#15544f', 7)
        line(d, est_xy[:index+1:2], MINT, 2)
        p = est[index, 1:4]
        r = rotation(*est[index, 8:11])
        xy = est_xy[index]
        d.ellipse((xy[0]-12, xy[1]-12, xy[0]+12, xy[1]+12), outline='#217a6d', width=1)
        vehicle(d, p, r)
        d.text((968, 24), 't = {:04.1f} s'.format(scenario_t),
               font=font(13, mono=True), fill=WHITE)
        d.text((802, 162), 'POSITION ERROR · 3D', font=font(11, bold=True), fill=MUTED)
        d.text((800, 185), '{:.2f}'.format(error[index]), font=font(46, mono=True), fill=WHITE)
        d.text((970, 216), 'm', font=font(17), fill=MUTED)
        # Residual delayed packets may arrive for 150 ms after outage begins.
        outage = OUTAGE[0] <= scenario_t < OUTAGE[1]+.15
        d.rounded_rectangle((802, 254, 1076, 286), radius=8,
                            fill='#382b1e' if outage else '#143a35')
        badge = 'ABSOLUTE POSE  /  OFFLINE' if outage else 'ABSOLUTE POSE  /  ONLINE'
        d.text((812, 263), badge, font=font(11, bold=True), fill=AMBER if outage else MINT)
        d.text((802, 301), 'IMU 100 Hz   +   SPEED 20 Hz', font=font(11), fill=MUTED)
        d.text((802, 329), 'BODY ATTITUDE', font=font(10, bold=True), fill=MUTED)
        # A clipped horizon responds to the actual output roll AND pitch.
        roll, pitch = est[index, 8:10]
        tile = Image.new('RGB', (124*S, 106*S), '#132f49')
        horizon = Canvas(tile)
        shift = math.degrees(pitch)*.8
        slope = -math.tan(roll)
        horizon.polygon([(-10, 53+shift-72*slope), (134, 53+shift+72*slope),
                         (134, 120), (-10, 120)], fill='#293c3c')
        horizon.line((-10, 53+shift-72*slope, 134, 53+shift+72*slope), fill='#6c9d9e', width=2)
        mask = Image.new('L', tile.size, 0)
        ImageDraw.Draw(mask).ellipse((6*S, 0, 118*S, 106*S), fill=255)
        image.paste(tile, (878*S, 350*S), mask)
        d = Canvas(image)
        d.ellipse((884, 350, 996, 456), outline='#38566d', width=1)
        line(d, [(914, 405), (930, 405), (940, 411), (950, 405), (966, 405)], AMBER, 2)
        d.text((802, 465), 'ROLL {:+05.1f}°'.format(math.degrees(roll)), font=font(12, mono=True),
               fill=WHITE)
        d.text((947, 465), 'PITCH {:+05.1f}°'.format(math.degrees(pitch)),
               font=font(12, mono=True), fill=WHITE)
        d.text((802, 490), 'Attitude error  {:.2f}°'.format(angle[index]), font=font(10), fill=MUTED)
        line(d, height_xy[:index+1:3], MINT, 2)
        line(d, error_xy[:index+1:3], AMBER, 2)
        for points, color in ((height_xy, MINT), (error_xy, AMBER)):
            point = points[index]
            d.ellipse((point[0]-3, point[1]-3, point[0]+3, point[1]+3), fill=color)
        cursor = 26+(1094-26)*scenario_t/DURATION
        d.ellipse((cursor-4, 651, cursor+4, 659), fill=WHITE)
        image = image.resize((W, H), Image.Resampling.LANCZOS)
        if frame == 0:
            first = image.copy()
        # Last 0.4 s is a visual dissolve, never presented as filter motion.
        if frame >= frame_count-8:
            image = Image.blend(image, first, (frame-(frame_count-9))/8)
        image.save(frames/'{:04d}.png'.format(frame))
        if frame == frame_count//3:
            image.save(output/'poster.png')
    # Palette generation uses all frames; no forced dithering noise in the UI.
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-framerate', str(FPS), '-i', str(frames/'%04d.png'), '-filter_complex',
                    '[0:v]split[a][b];[a]palettegen=stats_mode=diff[p];'
                    '[b][p]paletteuse=dither=bayer:bayer_scale=4',
                    '-loop', '0', str(output/'hero.gif')], check=True)
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-framerate', str(FPS), '-i', str(frames/'%04d.png'),
                    '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
                    '-movflags', '+faststart', str(output/'hero.mp4')], check=True)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.data, args.output)
