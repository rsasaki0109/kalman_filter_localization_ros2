#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Render actual Odaiba output and unaligned reference errors, without a basemap."""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

STYLE = Path(__file__).resolve().parents[1]/'hero_6dof'
sys.path.insert(0, str(STYLE))
spec = importlib.util.spec_from_file_location('hero_style', STYLE/'render.py')
style = importlib.util.module_from_spec(spec)
spec.loader.exec_module(style)
Canvas, font = style.Canvas, style.font
line, dashed, read = style.line, style.dashed, style.read
W, H, S, FPS, SECONDS = 1120, 700, 2, 20, 16
BG, PANEL, WHITE, MUTED, MINT, AMBER, PURPLE = (
    style.BG, style.PANEL, style.WHITE, style.MUTED, style.MINT, style.AMBER, style.PURPLE)


def interpolate(reference, stamps):
    """Interpolate only bracketed timestamps; normalized shortest-sign quaternion."""
    output = np.column_stack([np.interp(stamps, reference[:, 0], reference[:, i])
                              for i in range(reference.shape[1])])
    q = reference[:, 4:8].copy()
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0:
            q[i] *= -1
    output[:, 4:8] = np.column_stack([np.interp(stamps, reference[:, 0], q[:, i])
                                     for i in range(4)])
    output[:, 4:8] /= np.linalg.norm(output[:, 4:8], axis=1)[:, None]
    output[:, 10] = np.interp(stamps, reference[:, 0], np.unwrap(reference[:, 10]))
    return output


def assess(data):
    scenario = json.loads((data/'prepared/scenario.json').read_text())
    reference = read(data/'prepared/reference.csv')
    estimate = read(data/'prepared/estimate.csv')
    observations = read(data/'prepared/observations.csv')
    expected = scenario['input_counts']['imu']
    if len(estimate) != expected or np.max(np.diff(estimate[:, 0])) > .05:
        raise ValueError('Actual output count/continuity check failed')
    if not np.allclose(np.linalg.norm(estimate[:, 4:8], axis=1), 1., atol=1e-7):
        raise ValueError('Recorded output quaternion is not normalized')
    # Do not clamp unbracketed outputs at the reference endpoints.
    matched = (estimate[:, 0] >= reference[0, 0]) & (estimate[:, 0] <= reference[-1, 0])
    if matched.mean() < .99 or np.max(np.diff(reference[:, 0])) > .15:
        raise ValueError('Insufficient independent reference coverage')
    estimate = estimate[matched]
    truth = interpolate(reference, estimate[:, 0])
    error = np.linalg.norm(estimate[:, 1:4]-truth[:, 1:4], axis=1)
    angle = np.degrees(2*np.arccos(np.clip(np.abs(np.sum(
        estimate[:, 4:8]*truth[:, 4:8], axis=1)), 0., 1.)))
    obs_mask = (observations[:, 0] >= reference[0, 0]) & (observations[:, 0] <= reference[-1, 0])
    observations = observations[obs_mask]
    gnss_truth = interpolate(reference, observations[:, 0])
    gnss_error = np.linalg.norm(observations[:, 1:4]-gnss_truth[:, 1:4], axis=1)
    gaps = [(float(a), float(b)) for a, b in zip(observations[:-1, 0], observations[1:, 0])
            if b-a > .5]
    # Freeze integrity gates on actual recorder/receipt results, not attractive errors.
    manifest = json.loads((data/'results/manifest.json').read_text())
    log = (data/'results/wheel/localization.log').read_text()
    if 'reorder_late=0 reorder_buffered=0' not in log or 'numerical_failure=0' not in log:
        raise ValueError('Actual node ordering/numerical checks failed')
    metrics = dict(dataset=scenario['dataset'], kind='real sensor data; regression/demo split',
                   position_3d_rmse_m=float(np.sqrt(np.mean(error**2))),
                   max_position_3d_error_m=float(error.max()),
                   attitude_geodesic_rmse_deg=float(np.sqrt(np.mean(angle**2))),
                   max_attitude_geodesic_error_deg=float(angle.max()),
                   gnss_position_3d_rmse_m=float(np.sqrt(np.mean(gnss_error**2))),
                   gnss_metric_sampling='GNSS solution epochs; estimator metric uses IMU output epochs',
                   output_samples=expected, matched_output_samples=len(estimate),
                   reference_match_ratio=float(matched.mean()),
                   excluded_endpoint_samples=int((~matched).sum()),
                   reference_gap_max_sec=float(np.max(np.diff(reference[:, 0]))),
                   gnss_solution_gap_max_sec=float(np.max(np.diff(observations[:, 0]))),
                   gnss_solution_gaps=[(a-scenario['start'], b-scenario['start'])
                                       for a, b in gaps],
                   start=scenario['start'], duration=scenario['duration'],
                   constraints='no flat-ground or nonholonomic constraints',
                   alignment='none; common ENU origin; reference-derived initial pose only',
                   caveat=scenario['caveat'],
                   source_sha256={str(p.relative_to(data)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in (data/'prepared/reference.csv', data/'prepared/estimate.csv',
                                            data/'prepared/observations.csv', data/'prepared/scenario.json',
                                            data/'results/manifest.json', data/'results/configs/wheel.yaml')},
                   render_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    # The runner has already checked all expected input counts. Preserve its evidence.
    metrics['runner_manifest_schema'] = manifest.get('schema_version')
    return scenario, reference, estimate, observations, error, angle, gnss_error, gaps, metrics


def render(data, output):
    scenario, ref, est, obs, error, angles, gnss_error, gaps, metrics = assess(data)
    output.mkdir(parents=True, exist_ok=False)
    (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
    frames = output/'frames'
    frames.mkdir()
    start, duration = scenario['start'], scenario['duration']
    time = est[:, 0]-start
    lo = np.minimum(ref[:, 1:3].min(axis=0), est[:, 1:3].min(axis=0))
    hi = np.maximum(ref[:, 1:3].max(axis=0), est[:, 1:3].max(axis=0))
    center = (lo+hi)/2
    scale = min(650/(hi[0]-lo[0]), 260/(hi[1]-lo[1]))

    def project(points):
        return (np.asarray(points)[..., :2]-center)*[scale, -scale]+[391, 337]

    est_xy, ref_xy, obs_xy = project(est[:, 1:3]), project(ref[:, 1:3]), project(obs[:, 1:3])
    base = Image.new('RGB', (W*S, H*S), BG)
    d = Canvas(base)
    d.text((26, 21), 'KALMAN FILTER LOCALIZATION  /  REAL DATA', font=font(12, bold=True), fill=MINT)
    d.text((24, 45), 'Real streets. Real sensors. Six degrees of freedom.',
           font=font(32, bold=True), fill=WHITE)
    d.text((26, 96), 'URBANNAV ODAIBA  /  19 DEC 2018  /  GNSS + IMU + WHEEL SPEED',
           font=font(12), fill=MUTED)
    d.rounded_rectangle((24, 140, 764, 516), radius=16, fill=PANEL)
    d.rounded_rectangle((782, 140, 1096, 516), radius=16, fill=PANEL)
    scene = base.copy()
    d = Canvas(scene)
    # Pure metric ENU grid, not fabricated streets or a map of an invented road.
    for x in np.arange(math.floor(lo[0]/100)*100, hi[0]+100, 100):
        p = project([[x, lo[1]-60], [x, hi[1]+60]])
        line(d, p, '#172b3e')
    for y in np.arange(math.floor(lo[1]/100)*100, hi[1]+100, 100):
        line(d, project([[lo[0]-60, y], [hi[0]+60, y]]), '#172b3e')
    dashed(d, ref_xy[::2], '#768b9f', stride=4)
    d.text((44, 160), 'ODAIBA · TOKYO', font=font(12, bold=True), fill=WHITE)
    d.text((44, 182), 'ENU trajectory · fixed metric scale', font=font(10), fill=MUTED)
    # Fixed north and physical scale keys.
    line(d, [(57, 459), (57, 428)], MUTED, 1)
    d.polygon([(57, 422), (53, 430), (61, 430)], fill=MUTED)
    d.text((52, 405), 'N', font=font(10), fill=MUTED)
    bar = 100*scale
    line(d, [(44, 488), (44+bar, 488)], MUTED, 2)
    d.text((44, 494), '100 m', font=font(9), fill=MUTED)
    for x, label, color in ((423, 'ESKF', MINT), (515, 'Reference', '#768b9f'), (639, 'GNSS', PURPLE)):
        line(d, [(x, 491), (x+20, 491)], color, 2)
        d.text((x+27, 484), label, font=font(10), fill=MUTED)
    scene_mask = Image.new('L', base.size, 0)
    ImageDraw.Draw(scene_mask).rounded_rectangle(
        (24*S, 140*S, 764*S, 516*S), radius=16*S, fill=255)
    base.paste(scene, (0, 0), scene_mask)
    d = Canvas(base)
    error_hi = max(2., math.ceil(max(error.max(), gnss_error.max())))
    zmin = math.floor(min(ref[:, 3].min(), est[:, 3].min()))-1
    zmax = math.ceil(max(ref[:, 3].max(), est[:, 3].max()))+1
    boxes = [(58, 557, 550, 620), (646, 557, 1080, 620)]

    def graph(box, stamp, values, bounds):
        x0, y0, x1, y1 = box
        low, high = bounds
        return np.column_stack([x0+(x1-x0)*(stamp-start)/duration,
                                y1-(y1-y0)*(values-low)/(high-low)])

    for box, label, bounds in ((boxes[0], 'HEIGHT · RELATIVE ENU [m]', (zmin, zmax)),
                               (boxes[1], 'POSITION ERROR · 3D [m]', (0, error_hi))):
        x0, y0, x1, y1 = box
        d.text((x0-32, 531), label, font=font(10, bold=True), fill=MUTED)
        for a, b in gaps:
            d.rectangle((x0+(x1-x0)*(a-start)/duration, y0,
                         x0+(x1-x0)*(b-start)/duration, y1), fill='#30271f')
        for y, v in ((y0, bounds[1]), ((y0+y1)/2, sum(bounds)/2), (y1, bounds[0])):
            line(d, [(x0, y), (x1, y)], '#1b2e42')
            d.text((x0-30, y-5), '{:g}'.format(v), font=font(9, mono=True), fill=MUTED)
    d.text((948, 531), 'ESKF', font=font(10, bold=True), fill=AMBER)
    d.text((1028, 531), 'GNSS', font=font(10, bold=True), fill=PURPLE)
    dashed(d, graph(boxes[0], ref[:, 0], ref[:, 3], (zmin, zmax))[::2], '#768b9f', stride=4)
    height_xy = graph(boxes[0], est[:, 0], est[:, 3], (zmin, zmax))
    error_xy = graph(boxes[1], est[:, 0], error, (0, error_hi))
    gnss_xy = graph(boxes[1], obs[:, 0], gnss_error, (0, error_hi))
    d.text((26, 638), '0 s', font=font(10, mono=True), fill=MUTED)
    d.text((470, 638), 'FIRST 180 s · NO ARTIFICIAL OUTAGE', font=font(10, bold=True), fill=MUTED)
    d.text((1046, 638), '180 s', font=font(10, mono=True), fill=MUTED)
    line(d, [(26, 656), (1094, 656)], '#244d48', 4)
    d.text((26, 678), 'REAL URBANNAV INPUTS · ACTUAL ROS OUTPUT · REFERENCE-SEEDED START',
           font=font(10), fill=MUTED)
    d.text((867, 678), 'UrbanNav contributors · MIT', font=font(10), fill=MUTED)
    first = None
    total = FPS*SECONDS
    for frame in range(total):
        t = min(duration, duration*frame/(total-16))
        index = min(len(time)-1, np.searchsorted(time, t))
        timestamp = est[index, 0]
        image = base.copy()
        d = Canvas(image)
        visible = (obs[:, 0] <= timestamp) & (obs[:, 0] >= timestamp-15)
        for xy in obs_xy[visible]:
            d.ellipse((xy[0]-2, xy[1]-2, xy[0]+2, xy[1]+2), fill=PURPLE)
        line(d, est_xy[:index+1:3], '#15574f', 7)
        line(d, est_xy[:index+1:2], MINT, 2)
        xy = est_xy[index]
        yaw = est[index, 10]
        # A screen-sized navigation marker (not a scale vehicle model).
        forward = np.array([math.cos(yaw), -math.sin(yaw)])
        left = np.array([-math.sin(yaw), -math.cos(yaw)])
        d.polygon([tuple(xy+forward*11), tuple(xy-forward*7+left*6),
                   tuple(xy-forward*3), tuple(xy-forward*7-left*6)], fill=MINT, outline=WHITE)
        d.ellipse((xy[0]-16, xy[1]-16, xy[0]+16, xy[1]+16), outline='#2c8d7c', width=1)
        d.text((968, 24), 't = {:05.1f} s'.format(timestamp-start),
               font=font(12, mono=True), fill=WHITE)
        d.text((802, 162), 'POSITION ERROR · 3D', font=font(11, bold=True), fill=MUTED)
        d.text((800, 185), '{:.2f}'.format(error[index]), font=font(46, mono=True), fill=WHITE)
        d.text((970, 216), 'm', font=font(17), fill=MUTED)
        last = np.searchsorted(obs[:, 0], timestamp, side='right')-1
        available = last >= 0 and timestamp-obs[last, 0] <= .5
        d.rounded_rectangle((802, 254, 1076, 286), radius=8,
                            fill='#143a35' if available else '#382b1e')
        d.text((812, 263), 'GNSS SOLUTION  /  '+('ONLINE' if available else 'WAITING' if last < 0 else 'GAP'),
               font=font(11, bold=True), fill=MINT if available else AMBER)
        d.text((802, 301), 'IMU 50 Hz   +   WHEEL 50 Hz', font=font(11), fill=MUTED)
        d.text((802, 329), 'RECORDED BODY ATTITUDE', font=font(10, bold=True), fill=MUTED)
        roll, pitch = est[index, 8:10]
        tile = Image.new('RGB', (124*S, 106*S), '#132f49')
        h = Canvas(tile)
        shift = math.degrees(pitch)*1.5
        slope = -math.tan(roll)
        h.polygon([(-10, 53+shift-72*slope), (134, 53+shift+72*slope),
                   (134, 120), (-10, 120)], fill='#293c3c')
        h.line((-10, 53+shift-72*slope, 134, 53+shift+72*slope), fill='#6c9d9e', width=2)
        mask = Image.new('L', tile.size, 0)
        ImageDraw.Draw(mask).ellipse((6*S, 0, 118*S, 106*S), fill=255)
        image.paste(tile, (878*S, 350*S), mask)
        d = Canvas(image)
        d.ellipse((884, 350, 996, 456), outline='#38566d', width=1)
        line(d, [(914, 405), (930, 405), (940, 411), (950, 405), (966, 405)], AMBER, 2)
        d.text((802, 465), 'ROLL {:+05.1f}°'.format(math.degrees(roll)),
               font=font(12, mono=True), fill=WHITE)
        d.text((947, 465), 'PITCH {:+05.1f}°'.format(math.degrees(pitch)),
               font=font(12, mono=True), fill=WHITE)
        d.text((802, 490), 'Full-window 3D RMSE  {:.2f} m'.format(metrics['position_3d_rmse_m']),
               font=font(10), fill=MUTED)
        line(d, height_xy[:index+1:3], MINT, 2)
        line(d, error_xy[:index+1:3], AMBER, 2)
        for point in gnss_xy[obs[:, 0] <= timestamp][::2]:
            d.ellipse((point[0]-1, point[1]-1, point[0]+1, point[1]+1), fill=PURPLE)
        for points, color in ((height_xy, MINT), (error_xy, AMBER)):
            p = points[index]
            d.ellipse((p[0]-3, p[1]-3, p[0]+3, p[1]+3), fill=color)
        cursor = 26+(1094-26)*t/duration
        d.ellipse((cursor-4, 652, cursor+4, 660), fill=WHITE)
        image = image.resize((W, H), Image.Resampling.LANCZOS)
        if first is None:
            first = image.copy()
        if frame >= total-8:
            image = Image.blend(image, first, (frame-(total-9))/8)
        image.save(frames/'{:04d}.png'.format(frame))
        if frame == round((95/duration)*(total-16)):
            image.save(output/'poster.png')
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(FPS),
                    '-i', str(frames/'%04d.png'), '-filter_complex',
                    '[0:v]split[a][b];[a]palettegen=stats_mode=diff[p];'
                    '[b][p]paletteuse=dither=bayer:bayer_scale=4', '-loop', '0',
                    str(output/'hero.gif')], check=True)
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(FPS),
                    '-i', str(frames/'%04d.png'), '-c:v', 'libx264', '-crf', '18',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output/'hero.mp4')], check=True)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.data, args.output)
