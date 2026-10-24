#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Convert the fixed first 180 s of Odaiba, with full reference attitude."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu

scenario_path = Path(__file__).resolve().parents[1]/'hero_6dof/scenario.py'
scenario_spec = importlib.util.spec_from_file_location('hero_scenario', scenario_path)
scenario_module = importlib.util.module_from_spec(scenario_spec)
scenario_spec.loader.exec_module(scenario_module)
quaternion = scenario_module.quaternion


def prepare(source, position, output, duration=180.):
    output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    script = repo/'kalman_filter_localization_ros2/scripts/prepare_urbannav_tokyo.py'
    spec = importlib.util.spec_from_file_location('urban_helpers', script)
    helpers = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helpers)
    imu = helpers.read_imu(source/'imu.csv')
    start = imu[0]['stamp']
    # Selection is frozen before errors are inspected: first 180 s including start/stop.
    imu = [row for row in imu if start <= row['stamp'] <= start+duration]
    raw_reference = []
    with (source/'reference.csv').open() as stream:
        for row in csv.DictReader(stream, skipinitialspace=True):
            row = helpers.normalize_fields(row)
            t = helpers.gps_stamp(row['GPS Week'], row['GPS TOW (s)'])
            if start <= t <= start+duration:
                raw_reference.append((t, row))
    first = raw_reference[0][1]
    origin = tuple(float(first[k]) for k in ('Latitude (deg)', 'Longitude (deg)',
                                             'Ellipsoid Height (m)'))
    origin_ecef = helpers.geodetic_to_ecef(*origin)
    reference = []
    for t, row in raw_reference:
        llh = tuple(float(row[k]) for k in ('Latitude (deg)', 'Longitude (deg)',
                                            'Ellipsoid Height (m)'))
        p = helpers.ecef_to_enu(helpers.geodetic_to_ecef(*llh), origin_ecef, *origin[:2])
        roll, pitch, yaw = map(math.radians, (float(row['Roll (deg)']),
                                              -float(row['Pitch (deg)']),
                                              90.-float(row['Heading (deg)'])))
        q = quaternion(roll, pitch, yaw)
        reference.append([t, *p, *q, roll, pitch, math.atan2(math.sin(yaw), math.cos(yaw))])
    gnss = helpers.read_rtk_position(position, math.floor(start/helpers.GPS_WEEK_SECONDS))
    gnss = [row for row in gnss if start <= row['stamp'] <= imu[-1]['stamp']]
    observations = []
    for row in gnss:
        row['p'] = helpers.ecef_to_enu(helpers.geodetic_to_ecef(
            row['latitude'], row['longitude'], row['height']), origin_ecef, *origin[:2])
        observations.append([row['stamp'], *row['p']])
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output/'input_bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {'imu': '/sensing/imu/imu_data', 'wheel': '/wheel_speed', 'gnss': '/gnss_pose',
              'seed': '/ekf_localization/initial_pose'}
    types = {'imu': 'sensor_msgs/msg/Imu', 'wheel': 'geometry_msgs/msg/TwistWithCovarianceStamped',
             'gnss': 'geometry_msgs/msg/PoseStamped', 'seed': 'geometry_msgs/msg/PoseStamped'}
    for i, key in enumerate(topics):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=topics[key], type=types[key],
                                                     serialization_format='cdr'))
    events = [(row['stamp'], kind, row) for row in imu for kind in ('imu', 'wheel')]
    events += [(row['stamp'], 'gnss', row) for row in gnss]
    events += [(start-2+i*.1, 'seed', None) for i in range(20)]
    counts = dict.fromkeys(topics, 0)
    for t, kind, row in sorted(events, key=lambda e: e[0]):
        if kind == 'imu':
            message = Imu()
            message.header.frame_id = 'imu_link'
            message.orientation_covariance[0] = -1.
            for name, value in zip('xyz', row['acceleration']):
                setattr(message.linear_acceleration, name, value)
            for name, value in zip('xyz', row['angular_velocity']):
                setattr(message.angular_velocity, name, value)
        elif kind == 'wheel':
            message = TwistWithCovarianceStamped()
            message.header.frame_id = 'base_link'
            message.twist.twist.linear.x = row['wheel_speed']
        else:
            message = PoseStamped()
            message.header.frame_id = 'map'
            if kind == 'seed':
                for name, value in zip('xyzw', reference[0][4:8]):
                    setattr(message.pose.orientation, name, float(value))
            else:
                for name, value in zip('xyz', row['p']):
                    setattr(message.pose.position, name, float(value))
                message.pose.orientation.w = 1.
        message.header.stamp = helpers.to_ros_time(t)
        writer.write(topics[kind], serialize_message(message), round(t*1e9))
        counts[kind] += 1
    del writer
    for name, columns, rows in (
            ('reference.csv', ['stamp', 'x', 'y', 'z', 'qx', 'qy',
             'qz', 'qw', 'roll', 'pitch', 'yaw'], reference),
            ('observations.csv', ['stamp', 'x', 'y', 'z'], observations)):
        with (output/name).open('w', newline='') as stream:
            csv_writer = csv.writer(stream)
            csv_writer.writerow(columns)
            csv_writer.writerows(rows)
    acceleration = np.array([row['acceleration'] for row in imu])
    summary = dict(dataset='UrbanNav-TK-20181219 Odaiba', kind='real sensor data',
                   start=start, end=imu[-1]['stamp'], duration=imu[-1]['stamp']-start,
                   selection='first 180 s from initial IMU, frozen before error inspection',
                   origin_llh=list(origin), input_counts=counts, reference_samples=len(reference),
                   initialization='reference first orientation; zero position in reference-origin ENU',
                   coordinate_transform='TAG264 FRD to FLU: x,-y,-z; yaw=90-heading, pitch=-pitch, roll=roll',
                   stationary_accel_norm_mps2=float(
                       np.median(np.linalg.norm(acceleration[:200], axis=1))),
                   sources={str(p.name): hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in (source/'imu.csv', source/'reference.csv', position)},
                   caveat='No published Tokyo extrinsic/lever-arm calibration; identity body mounting assumed')
    (output/'scenario.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--position', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.position, args.output)
