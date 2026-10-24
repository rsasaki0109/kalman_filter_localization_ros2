#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Export actual PoseStamped output quaternions retained in the recorder bag."""

import argparse
import csv
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message
import rosbag2_py
import yaml


def extract(bag, output):
    reader = rosbag2_py.SequentialReader()
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())
    storage = metadata['rosbag2_bagfile_information']['storage_identifier']
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    rows = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/ekf_localization/current_pose':
            continue
        message = deserialize_message(data, PoseStamped)
        q = message.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        angles = [math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                  math.asin(max(-1., min(1., 2*(w*y-z*x)))),
                  math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
        p = message.pose.position
        rows.append([message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                     p.x, p.y, p.z, x, y, z, w, *angles])
    rows.sort(key=lambda row: row[0])
    with output.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw'])
        writer.writerows(rows)
    print('Exported', len(rows), 'actual output poses')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    extract(args.bag, args.output)
