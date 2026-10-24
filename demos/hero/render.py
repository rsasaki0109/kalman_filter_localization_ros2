#!/usr/bin/env python3
"""Render recorded ROS estimates into a headless, reproducible README hero."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT, FPS = 960, 560, 16
SCALE = 2
BG = '#0b1220'
PANEL = '#101d2f'
WHITE, MUTED = '#edf5ff', '#8da3ba'
MINT, AMBER = '#5de4c7', '#ffbd69'
FONTS = Path('/usr/share/fonts/truetype/dejavu')


def font(size, bold=False, mono=False):
    """Use bundled system fonts, independent of an interactive display."""
    name = 'DejaVuSansMono' if mono else 'DejaVuSans'
    return ImageFont.truetype(
        str(FONTS / (name + ('-Bold' if bold else '') + '.ttf')), size * SCALE)


class Canvas:
    """Draw in logical pixels, then downsample for antialiased GIF edges."""

    def __init__(self, image):
        self.draw = ImageDraw.Draw(image)

    @staticmethod
    def coordinates(value):
        if isinstance(value, (list, tuple)):
            return tuple(Canvas.coordinates(item) for item in value)
        return value * SCALE

    def __getattr__(self, name):
        def paint(xy, *args, **kwargs):
            for key in ('width', 'radius'):
                if key in kwargs:
                    kwargs[key] *= SCALE
            if name != 'text' and 'width' not in kwargs:
                kwargs['width'] = SCALE
            return getattr(self.draw, name)(self.coordinates(xy), *args, **kwargs)
        return paint


def read_csv(path):
    """Read and validate timestamped trajectory data before interpolation."""
    with path.open(encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('empty trajectory: {}'.format(path))
    columns = {name: np.array([float(row[name]) for row in rows]) for name in rows[0]}
    if not all(np.isfinite(values).all() for values in columns.values()):
        raise ValueError('non-finite trajectory: {}'.format(path))
    if np.any(np.diff(columns['stamp']) <= 0.0):
        raise ValueError('timestamps must increase strictly: {}'.format(path))
    return columns


def point(x, y):
    """Map equal-scale ENU metres to the fixed top-down view."""
    return (340.0 + 4.0 * x, 447.0 - 4.0 * y)


def polyline(draw, xy, color, width):
    """Draw a trajectory only when it has at least two points."""
    if len(xy) > 1:
        draw.line(xy, fill=color, width=width, joint='curve')


def dashed(draw, xy, color, width=1, stride=8):
    """Draw the reference with a distinct line style as well as color."""
    for index in range(0, len(xy) - 1, stride * 2):
        polyline(draw, xy[index:index + stride + 1], color, width)


def checksum(path):
    """Record the source artifacts used for this render."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def render(data, output):
    """Encode a hero from actual recorded estimates; never substitute truth."""
    scenario = json.loads((data / 'scenario.json').read_text(encoding='utf-8'))
    reference = read_csv(data / 'reference.csv')
    estimate_path = data / 'results' / 'wheel_nhc_fixed' / 'estimate.csv'
    estimate = read_csv(estimate_path)
    gnss = read_csv(data / 'gnss.csv')
    start, duration = scenario['start'], scenario['duration']
    t = reference['stamp'] - start
    stamp = reference['stamp']
    # Check coverage before interpolating: endpoint clamping must not hide missing output.
    if estimate['stamp'][0] > stamp[0] + 0.03 or estimate['stamp'][-1] < stamp[-1] - 0.03:
        raise ValueError('recorded estimate does not cover the demonstration')
    if np.max(np.diff(estimate['stamp'])) > 0.05:
        raise ValueError('recorded estimate has a gap larger than 50 ms')
    ex = np.interp(stamp, estimate['stamp'], estimate['x'])
    ey = np.interp(stamp, estimate['stamp'], estimate['y'])
    ez = np.interp(stamp, estimate['stamp'], estimate['z'])
    yaw = np.interp(stamp, estimate['stamp'], np.unwrap(estimate['yaw']))
    error = np.hypot(ex - reference['x'], ey - reference['y'])
    outage_start, outage_end = scenario['outage']
    outage_mask = (t >= outage_start) & (t < outage_end)
    metrics = {
        'kind': scenario['kind'],
        'estimate_samples': len(estimate['stamp']),
        'horizontal_rmse_m': float(np.sqrt(np.mean(error ** 2))),
        'outage_horizontal_rmse_m': float(np.sqrt(np.mean(error[outage_mask] ** 2))),
        'outage_max_horizontal_error_m': float(np.max(error[outage_mask])),
        'max_horizontal_error_m': float(np.max(error)),
        'max_vertical_error_m': float(np.max(np.abs(ez - reference['z']))),
        'source_sha256': {str(path.relative_to(data)): checksum(path) for path in (
            data / 'scenario.json', data / 'reference.csv', data / 'gnss.csv', estimate_path)},
    }
    # A broken run should fail rather than silently produce a convincing animation.
    if metrics['outage_max_horizontal_error_m'] > 3.0:
        raise ValueError('demo exceeds the 3 m outage error readiness limit: ' + str(metrics))
    output.mkdir(parents=True, exist_ok=True)
    (output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n', encoding='utf-8')
    frames = output / 'frames'
    frames.mkdir(exist_ok=False)
    ref_xy = [point(x, y) for x, y in zip(reference['x'][::4], reference['y'][::4])]
    est_xy = [point(x, y) for x, y in zip(ex, ey)]
    base = Image.new('RGB', (WIDTH * SCALE, HEIGHT * SCALE), BG)
    draw = Canvas(base)
    # A quiet grid, with all contrast reserved for the sensor story.
    for x in range(20, 659, 40):
        draw.line((x, 146, x, 459), fill='#142237')
    for y in range(167, 460, 40):
        draw.line((20, y, 658, y), fill='#142237')
    draw.text((30, 19), 'KALMAN FILTER LOCALIZATION', font=font(12, bold=True), fill=MINT)
    draw.text((28, 44), 'Precision through the blackout.', font=font(32, bold=True), fill=WHITE)
    draw.text((30, 91), 'GNSS + IMU + wheel odometry  /  Error-state EKF  /  ROS 2',
              font=font(14), fill=MUTED)
    # Analytic road outline avoids overlapping wide joins at dense IMU samples.
    draw.ellipse((68, 179, 612, 459), outline='#18293b', width=24)
    polyline(draw, ref_xy, '#25384c', 1)
    dashed(draw, ref_xy, '#728497', width=1, stride=7)
    draw.text((244, 268), '16 s WITHOUT GNSS', font=font(15, bold=True), fill=WHITE)
    draw.text((252, 292), 'IMU + wheel keep going', font=font(12), fill=MUTED)
    # North and scale help the scene read as a physical trajectory.
    draw.line((45, 410, 45, 380), fill=MUTED, width=1)
    draw.polygon([(45, 375), (41, 383), (49, 383)], fill=MUTED)
    draw.text((40, 355), 'N', font=font(11), fill=MUTED)
    draw.line((44, 441, 124, 441), fill=MUTED, width=2)
    draw.text((64, 445), '20 m', font=font(10), fill=MUTED)
    draw.rounded_rectangle((680, 134, 930, 456), radius=14, fill=PANEL, outline='#23364d')
    draw.text((700, 154), 'SENSOR FUSION', font=font(12, bold=True), fill=MUTED)
    draw.line((700, 235, 910, 235), fill='#26374c')
    draw.text((700, 253), 'IMU', font=font(13), fill=WHITE)
    draw.text((821, 253), '100 Hz', font=font(13, mono=True), fill=MINT)
    draw.text((700, 285), 'WHEEL', font=font(13), fill=WHITE)
    draw.text((831, 285), '20 Hz', font=font(13, mono=True), fill=MINT)
    draw.line((700, 321, 910, 321), fill='#26374c')
    draw.text((700, 339), 'POSITION ERROR', font=font(11, bold=True), fill=MUTED)
    draw.text((700, 420), 'Recorded ROS 2 estimate', font=font(11), fill=MUTED)
    # Fixed chart scale for the whole clip; no changing axes that hide drift.
    chart_top, chart_bottom = 490, 526
    chart_max = max(1.0, math.ceil(float(np.max(error)) * 2.0) / 2.0)

    def chart_point(index):
        return (142 + 500 * t[index] / duration,
                chart_bottom - (chart_bottom - chart_top) * error[index] / chart_max)

    draw.text((30, 486), 'XY ERROR', font=font(11, bold=True), fill=MUTED)
    draw.text((30, 506), '0 - {:.1f} m'.format(chart_max), font=font(10), fill=MUTED)
    for y in (chart_top, chart_bottom):
        draw.line((142, y, 642, y), fill='#26374c')
    draw.rectangle((142 + 500 * outage_start / duration, chart_top,
                    142 + 500 * outage_end / duration, chart_bottom), fill='#302b26')
    draw.text((696, 491), 'SYNTHETIC DEMO', font=font(11, bold=True), fill=MUTED)
    draw.text((696, 510), 'Seeded inputs. Actual EKF output.', font=font(10), fill=MUTED)
    chart_xy = [chart_point(i) for i in range(len(t))]
    first = None
    for frame in range(FPS * 12):
        # 0.5 s opening hold, 11 s lap, 0.5 s dissolve back to the opening.
        simulation_time = 4.0 + min(1.0, max(0.0, (frame / FPS - 0.5) / 11.0)) * (duration - 4.0)
        index = min(len(t) - 1, int(np.searchsorted(t, simulation_time)))
        blackout = outage_start <= t[index] < outage_end
        image = base.copy()
        d = Canvas(image)
        status_color = AMBER if blackout else MINT
        phase = 'GNSS BLACKOUT' if blackout else ('GNSS RECOVERED' if t[index] >= outage_end else 'GNSS ONLINE')
        d.rounded_rectangle((698, 183, 911, 218), radius=7, fill='#302b26' if blackout else '#163a38')
        d.ellipse((710, 196, 717, 203), fill=status_color)
        d.text((727, 192), phase, font=font(12, bold=True), fill=status_color)
        d.text((699, 365), '{:.2f}'.format(error[index]), font=font(32, bold=True, mono=True), fill=WHITE)
        d.text((864, 384), 'm', font=font(16), fill=MUTED)
        # Only observations that have actually arrived are visible, with a 6 s tail.
        mask = (gnss['stamp'] <= stamp[index]) & (gnss['stamp'] >= stamp[index] - 6.0)
        for gx, gy in zip(gnss['x'][mask], gnss['y'][mask]):
            px, py = point(gx, gy)
            d.ellipse((px - 2.5, py - 2.5, px + 2.5, py + 2.5), fill=AMBER)
        polyline(d, est_xy[:index + 1:3], '#285b59', 3)
        polyline(d, est_xy[max(0, index - 320):index + 1], MINT, 3)
        px, py = est_xy[index]
        d.ellipse((px - 13, py - 13, px + 13, py + 13), outline='#245751', width=2)
        # A body-forward arrow, rotated using the estimator's recorded attitude.
        heading = yaw[index]
        vertices = [(11, 0), (-7, 6), (-4, 0), (-7, -6)]
        car = [(px + dx * math.cos(heading) - dy * math.sin(heading),
                py - dx * math.sin(heading) - dy * math.cos(heading)) for dx, dy in vertices]
        d.polygon(car, fill=MINT)
        polyline(d, chart_xy[:index + 1:4], MINT, 2)
        cx, cy = chart_point(index)
        d.ellipse((cx - 3, cy - 3, cx + 3, cy + 3), fill=MINT)
        # Legend stays clear of the route and uses both symbols and labels.
        d.line((33, 125, 51, 125), fill=MINT, width=3)
        d.text((59, 117), 'EKF estimate', font=font(11), fill=WHITE)
        d.ellipse((184, 122, 190, 128), fill=AMBER)
        d.text((199, 117), 'GNSS observations', font=font(11), fill=WHITE)
        d.line((355, 125, 363, 125), fill=MUTED)
        d.line((368, 125, 376, 125), fill=MUTED)
        d.text((384, 117), 'Ground truth', font=font(11), fill=MUTED)
        d.text((28, 540), 'GNSS / INS / WHEEL', font=font(10, mono=True), fill=MUTED)
        d.text((798, 540), 't = {:04.1f} s'.format(t[index]), font=font(10, mono=True), fill=MUTED)
        d.rounded_rectangle((240, 544, 745, 547), radius=1, fill='#23364d')
        d.rounded_rectangle((240, 544, 240 + 505 * t[index] / duration, 547), radius=1, fill=status_color)
        if first is None:
            first = image.copy()
        if frame >= FPS * 11.5:
            image = Image.blend(image, first, (frame - FPS * 11.5 + 1) / (FPS * 0.5))
        image = image.resize((WIDTH, HEIGHT), Image.Resampling.LANCZOS)
        image.save(frames / '{:04d}.png'.format(frame))
        if frame == round(FPS * 6):
            image.save(output / 'poster.png')
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(FPS),
        '-i', str(frames / '%04d.png'), '-c:v', 'libx264', '-crf', '18',
        '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output / 'hero.mp4')], check=True)
    subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-framerate', str(FPS),
        '-i', str(frames / '%04d.png'), '-filter_complex',
        '[0:v]split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];'
        '[b][p]paletteuse=dither=bayer:bayer_scale=3',
        '-loop', '0', str(output / 'hero.gif')], check=True)
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    render(args.data, args.output)
