#!/usr/bin/env python3
"""Create a seeded, physically consistent GNSS/IMU/wheel demonstration bag.

Synthetic data only: this is an illustration, not a dataset benchmark.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import random


DURATION = 52.0
OUTAGE = (21.0, 37.0)
START = 100.0
SEED = 20261024


def motion(t):
    """Analytic elliptical lap with a stationary start and smooth acceleration.

    Return world position, heading, body specific force, yaw rate, and speed.
    Units are metres, seconds, radians; body is forward/left/up (ENU world).
    """
    ramp = 4.0
    omega = 2.0 * math.pi / (DURATION - 3.0 - ramp / 2.0)
    u = min(1.0, max(0.0, (t - 3.0) / ramp))
    rate = omega * (3.0 * u * u - 2.0 * u ** 3)
    rate_dot = omega * 6.0 * u * (1.0 - u) / ramp
    angle = omega * ramp * (u ** 3 - 0.5 * u ** 4)
    if t > 3.0 + ramp:
        angle = omega * (t - 3.0 - ramp / 2.0)
    x, y = 65.0 * math.sin(angle), 32.0 * (1.0 - math.cos(angle))
    vx = 65.0 * math.cos(angle) * rate
    vy = 32.0 * math.sin(angle) * rate
    ax = 65.0 * (math.cos(angle) * rate_dot - math.sin(angle) * rate * rate)
    ay = 32.0 * (math.sin(angle) * rate_dot + math.cos(angle) * rate * rate)
    speed = math.hypot(vx, vy)
    yaw = math.atan2(vy, vx) if speed > 1.0e-9 else 0.0
    yaw_rate = (vx * ay - vy * ax) / (speed * speed) if speed > 1.0e-9 else 0.0
    return (x, y, yaw, math.cos(yaw) * ax + math.sin(yaw) * ay,
            -math.sin(yaw) * ax + math.cos(yaw) * ay, yaw_rate, speed)


def write_csv(path, columns, rows):
    """Write one complete CSV artifact."""
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def generate(output):
    """Generate inputs without overwriting any prior run."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu

    output.mkdir(parents=True, exist_ok=False)
    rng = random.Random(SEED)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output / 'input_bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    topics = {
        '/ekf_localization/initial_pose': 'geometry_msgs/msg/PoseStamped',
        '/gnss_pose': 'geometry_msgs/msg/PoseStamped',
        '/sensing/imu/imu_data': 'sensor_msgs/msg/Imu',
        '/wheel_speed': 'geometry_msgs/msg/TwistWithCovarianceStamped',
    }
    for index, (name, msg_type) in enumerate(topics.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=index, name=name, type=msg_type, serialization_format='cdr'))
    counts = dict.fromkeys(topics, 0)

    def emit(topic, message, t, frame):
        nanoseconds = round((START + t) * 1.0e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(
            nanoseconds, 1_000_000_000)
        message.header.frame_id = frame
        writer.write(topic, serialize_message(message), nanoseconds)
        counts[topic] += 1

    for index in range(20):
        message = PoseStamped()
        message.pose.orientation.w = 1.0
        emit('/ekf_localization/initial_pose', message, -2.0 + index * 0.1, 'map')
    reference, gnss = [], []
    for index in range(round(DURATION * 100) + 1):
        t = index / 100.0
        x, y, yaw, ax, ay, yaw_rate, speed = motion(t)
        reference.append((START + t, x, y, 0.0, yaw))
        imu = Imu()
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = yaw_rate + 0.001 + rng.gauss(0.0, 0.0015)
        imu.linear_acceleration.x = ax + 0.015 + rng.gauss(0.0, 0.025)
        imu.linear_acceleration.y = ay - 0.01 + rng.gauss(0.0, 0.025)
        imu.linear_acceleration.z = 9.80665 + rng.gauss(0.0, 0.015)
        emit('/sensing/imu/imu_data', imu, t, 'imu_link')
        if index % 5 == 0:
            wheel = TwistWithCovarianceStamped()
            wheel.twist.twist.linear.x = max(0.0, speed + rng.gauss(0.0, 0.04))
            wheel.twist.covariance[0] = 0.04 ** 2
            emit('/wheel_speed', wheel, t, 'base_link')
        if index % 20 == 0 and not OUTAGE[0] <= t < OUTAGE[1]:
            pose = PoseStamped()
            pose.pose.position.x = x + rng.gauss(0.0, 0.55)
            pose.pose.position.y = y + rng.gauss(0.0, 0.55)
            pose.pose.orientation.w = 1.0
            emit('/gnss_pose', pose, t, 'map')
            gnss.append((START + t, pose.pose.position.x, pose.pose.position.y, 0.0))
    del writer
    write_csv(output / 'reference.csv', ('stamp', 'x', 'y', 'z', 'yaw'), reference)
    write_csv(output / 'gnss.csv', ('stamp', 'x', 'y', 'z'), gnss)
    (output / 'scenario.json').write_text(json.dumps({
        'kind': 'synthetic illustration, not a real-world benchmark',
        'seed': SEED, 'start': START, 'duration': DURATION,
        'outage': list(OUTAGE), 'input_counts': counts,
        'imu_hz': 100, 'wheel_hz': 20, 'gnss_hz': 5,
        'gnss_sigma_m': 0.55,
        'gyro_bias_radps': 0.001, 'accel_bias_mps2': [0.015, -0.01, 0.0],
    }, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    generate(parser.parse_args().output)
