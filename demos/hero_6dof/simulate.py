#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, Ryohei Sasaki
"""Drive the installed Autoware launch, retaining inputs and actual ROS outputs."""

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from kalman_filter_localization_msgs.msg import EstimatorStatus, MeasurementQuality
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
import rosbag2_py
from scenario import DURATION, OUTAGE, POSE_DELAY, SEED, START, motion, quaternion, rotation
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
import yaml

COLUMNS = ['stamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'roll', 'pitch', 'yaw']


def stamp(message, t):
    message.sec, message.nanosec = divmod(round(t*1e9), 1_000_000_000)


def assign(target, values):
    for name, value in zip(('x', 'y', 'z', 'w'), values):
        setattr(target, name, float(value))


def rpy(q):
    x, y, z, w = q
    return [np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            np.arcsin(np.clip(2*(w*y-z*x), -1., 1.)),
            np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def write_csv(path, columns, rows):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


def simulate(output):
    output.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory('kalman_filter_localization'))
    config = yaml.safe_load((share/'param/profiles/autoware_6dof_forward.yaml').read_text())
    config['ekf_localization']['ros__parameters'].update({
        'gravity_mps2': 9.80665, 'input_qos_depth': 10000, 'output_qos_depth': 10000,
        'input_reorder_window_sec': 0., 'measurement_history_duration_sec': 2.,
        'max_future_measurement_wait_sec': .6,
        'var_imu_w': 9e-6, 'var_imu_acc': .0036,
        'var_imu_gyro_bias': 1e-8, 'var_imu_acc_bias': 1e-7,
        'initial_imu_gyro_bias_covariance': 4e-6,
        'initial_imu_acc_bias_covariance': .0004,
        'initial_velocity_variance_xy': .04, 'initial_velocity_variance_z': .04,
        'enable_stationary_initialization': False,
    })
    config_path = output/'parameters.yaml'
    config_path.write_text(yaml.safe_dump(config))
    command = ['ros2', 'launch', 'kalman_filter_localization', 'autoware.launch.py',
               'param_file:='+str(config_path.resolve()), 'use_sim_time:=true',
               'imu_topic:=/hero/imu', 'input_initial_pose_name:=/hero/seed',
               'input_pose_with_cov_name:=/hero/pose',
               'input_twist_with_cov_name:=/hero/speed']
    rclpy.init()
    node = rclpy.create_node('hero_6dof_driver')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    estimates, statuses, qualities = [], [], []
    delivery = START-.2
    bag = rosbag2_py.SequentialWriter()
    bag.open(rosbag2_py.StorageOptions(uri=str(output/'recording'), storage_id='sqlite3'),
             rosbag2_py.ConverterOptions('', ''))
    topics = {'/hero/seed': 'geometry_msgs/msg/PoseWithCovarianceStamped',
              '/hero/pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
              '/hero/speed': 'geometry_msgs/msg/TwistWithCovarianceStamped',
              '/hero/imu': 'sensor_msgs/msg/Imu', '/clock': 'rosgraph_msgs/msg/Clock',
              '/tf_static': 'tf2_msgs/msg/TFMessage', '/ekf_odom': 'nav_msgs/msg/Odometry'}
    for i, (name, kind) in enumerate(topics.items()):
        bag.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=kind,
                                                serialization_format='cdr'))
    counts = dict.fromkeys(topics, 0)

    def record(topic, message):
        bag.write(topic, serialize_message(message), round(delivery*1e9))
        counts[topic] += 1

    def receive(message):
        record('/ekf_odom', message)
        q = message.pose.pose.orientation
        p = message.pose.pose.position
        q = [q.x, q.y, q.z, q.w]
        t = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        estimates.append([t, p.x, p.y, p.z, *q, *rpy(q)])

    subscriptions = [
        node.create_subscription(Odometry, '/ekf_odom', receive, 10000),
        node.create_subscription(EstimatorStatus, '/ekf_localization/status', statuses.append, 1000),
        node.create_subscription(MeasurementQuality,
                                 '/ekf_localization/debug/measurement_quality_typed',
                                 qualities.append, 10000),
    ]
    pubs = {topic: node.create_publisher(kind, topic,
                                       qos_profile_sensor_data if topic == '/hero/imu' else 10000)
            for topic, kind in (('/hero/seed', PoseWithCovarianceStamped),
                                ('/hero/pose', PoseWithCovarianceStamped),
                                ('/hero/speed', TwistWithCovarianceStamped),
                                ('/hero/imu', Imu), ('/clock', Clock))}
    static = StaticTransformBroadcaster(node)
    client = node.create_client(SetBool, '/trigger_node')
    log_path = output/'estimator.log'

    def spin_until(predicate, timeout=15.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.002)
        if not predicate():
            raise RuntimeError('ROS readiness/receipt timeout: '+log_path.read_text()[-2500:])

    def send(topic, message):
        record(topic, message)
        pubs[topic].publish(message)

    rng = np.random.default_rng(SEED)
    reference, observations = [], []
    mounting = rotation(.12, -.08, .20)
    gyro_bias = np.array([.00025, -.00020, .00040])
    accel_bias = np.array([.006, -.004, .003])
    pending_pose = []
    started = time.monotonic()
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                if not client.wait_for_service(timeout_sec=15):
                    raise RuntimeError(log_path.read_text())
                spin_until(lambda: all(p.get_subscription_count() for p in pubs.values()))
                clock = Clock()
                stamp(clock.clock, delivery)
                send('/clock', clock)
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'base_link', 'hero_imu'
                assign(transform.transform.rotation, quaternion(.12, -.08, .20))
                static.sendTransform(transform)
                record('/tf_static', TFMessage(transforms=[transform]))
                # Wait for static calibration to be consumed before the first IMU.
                for _ in range(30):
                    executor.spin_once(timeout_sec=.005)
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id = 'map'
                stamp(seed.header.stamp, START)
                assign(seed.pose.pose.orientation, motion(0.)['q'])
                seed.pose.covariance = np.diag([.01]*3+[.0001]*3).ravel().tolist()
                send('/hero/seed', seed)
                spin_until(lambda: statuses and statuses[-1].summary == 'inactive')
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                spin_until(future.done)
                assert future.result().success
                # Wait for seed receipt, not just for the service response.
                spin_until(lambda: statuses and statuses[-1].active and
                           statuses[-1].summary == 'waiting_for_sensor_input')
                parameters = AsyncParameterClient(node, '/ekf_localization')
                assert parameters.wait_for_services(timeout_sec=5)
                listed = parameters.list_parameters([], depth=0)
                spin_until(listed.done)
                names = listed.result().result.names
                queried = parameters.get_parameters(names)
                spin_until(queried.done)
                effective = dict(zip(names, [parameter_value_to_python(v)
                                            for v in queried.result().values]))
                (output/'runtime_parameters.json').write_text(json.dumps(effective, indent=2)+'\n')
                for index in range(round(DURATION*100)+1):
                    t = index/100
                    delivery = START+t
                    state = motion(t)
                    reference.append([delivery, *state['p'], *state['q'], *state['rpy']])
                    stamp(clock.clock, delivery)
                    send('/clock', clock)
                    imu = Imu()
                    imu.header.frame_id = 'hero_imu'
                    stamp(imu.header.stamp, delivery)
                    imu.orientation_covariance[0] = -1.
                    assign(imu.angular_velocity,
                           mounting.T@(state['w']+gyro_bias+rng.normal(0, .0015, 3)))
                    assign(imu.linear_acceleration,
                           mounting.T@(state['f']+accel_bias+rng.normal(0, .025, 3)))
                    imu.angular_velocity_covariance = (np.eye(3)*.0015**2).ravel().tolist()
                    imu.linear_acceleration_covariance = (np.eye(3)*.025**2).ravel().tolist()
                    send('/hero/imu', imu)
                    if index % 5 == 0:
                        twist = TwistWithCovarianceStamped()
                        twist.header.frame_id = 'base_link'
                        stamp(twist.header.stamp, delivery)
                        twist.twist.twist.linear.x = state['speed']+rng.normal(0, .04)
                        twist.twist.twist.linear.y = float('nan')
                        twist.twist.twist.linear.z = float('nan')
                        twist.twist.covariance = [float('nan')]*36
                        twist.twist.covariance[0] = .04**2
                        send('/hero/speed', twist)
                    if index % 20 == 0 and not OUTAGE[0] <= t < OUTAGE[1]:
                        pose = PoseWithCovarianceStamped()
                        pose.header.frame_id = 'map'
                        stamp(pose.header.stamp, delivery)
                        p = state['p']+rng.normal(0, [.18, .18, .22])
                        delta = rng.normal(0, [.006, .006, .01])
                        angle = np.linalg.norm(delta)
                        dq = np.r_[delta*np.sin(angle/2)/angle, np.cos(angle/2)]
                        q = state['q']
                        noisy_q = np.r_[q[3]*dq[:3]+dq[3]*q[:3]+np.cross(q[:3], dq[:3]),
                                        q[3]*dq[3]-np.dot(q[:3], dq[:3])]
                        assign(pose.pose.pose.position, p)
                        assign(pose.pose.pose.orientation, noisy_q)
                        # Fixed-world-axis covariance of the small right/body-angle perturbation.
                        mapping = np.eye(6)
                        mapping[3:, 3:] = state['r']
                        covariance = mapping@np.diag([.18**2, .18**2, .22**2,
                                                      .006**2, .006**2, .01**2])@mapping.T
                        pose.pose.covariance = covariance.ravel().tolist()
                        pending_pose.append((delivery+POSE_DELAY, pose))
                        observations.append([delivery, *p])
                    while pending_pose and pending_pose[0][0] <= delivery+1e-8:
                        _, pose = pending_pose.pop(0)
                        send('/hero/pose', pose)
                    # Bound work in flight; DDS must deliver every actual output.
                    if index % 50 == 0:
                        spin_until(lambda: estimates and estimates[-1][0] >= delivery-1e-6)
                delivery += POSE_DELAY
                stamp(clock.clock, delivery)
                send('/clock', clock)
                for _, pose in pending_pose:
                    send('/hero/pose', pose)
                spin_until(lambda: statuses and statuses[-1].received_imu == counts['/hero/imu'] and
                           statuses[-1].received_pose_measurements == counts['/hero/pose'] and
                           statuses[-1].received_twist_measurements == counts['/hero/speed'] and
                           len(estimates) == counts['/hero/imu'] and
                           len(qualities) == counts['/hero/pose']+counts['/hero/speed'])
                assert statuses[-1].rewind_count > 0, 'Delayed observations did not exercise replay'
                assert statuses[-1].numerical_failures == 0
                assert statuses[-1].numerical_invariant_ok
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        del bag
        assert subscriptions
    write_csv(output/'reference.csv', COLUMNS, reference)
    write_csv(output/'estimate.csv', COLUMNS, estimates)
    write_csv(output/'observations.csv', ['stamp', 'x', 'y', 'z'], observations)
    write_csv(output/'measurement_quality.csv',
              ['stamp', 'source', 'accepted', 'nis', 'reason'],
              [[q.header.stamp.sec+q.header.stamp.nanosec*1e-9, q.source, int(q.accepted),
                q.raw_nis, q.reject_reason_text] for q in qualities])
    # Messages are saved in full for timing, covariance and noise auditing.
    status = statuses[-1]
    metadata = dict(kind='Synthetic illustration; not a real-data benchmark',
                    seed=SEED, duration=DURATION, start=START, outage=list(OUTAGE),
                    pose_delay_sec=POSE_DELAY, imu_hz=100, forward_speed_hz=20, pose_hz=5,
                    position_sigma_m=[.18, .18, .22],
                    attitude_body_tangent_sigma_rad=[.006, .006, .01],
                    forward_speed_sigma_mps=.04,
                    gyro_sample_sigma_radps=.0015, accelerometer_sample_sigma_mps2=.025,
                    ros_distro=os.environ.get('ROS_DISTRO', 'unknown'),
                    gyro_bias_body_radps=gyro_bias.tolist(), accel_bias_body_mps2=accel_bias.tolist(),
                    imu_mount_rpy_rad=[.12, -.08, .20], input_output_counts=counts,
                    rewind_count=status.rewind_count, numerical_failures=status.numerical_failures,
                    received_imu=status.received_imu,
                    received_pose=status.received_pose_measurements,
                    received_twist=status.received_twist_measurements,
                    accepted_measurements=status.accepted_measurements,
                    rejected_measurements=status.rejected_measurements,
                    wall_runtime_sec=time.monotonic()-started,
                    git_head=subprocess.check_output(['git', '-c', 'safe.directory=/repo', 'rev-parse', 'HEAD'], text=True).strip(),
                    git_dirty=bool(subprocess.check_output(['git', '-c', 'safe.directory=/repo', 'status', '--porcelain'], text=True)),
                    source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(Path('demos/hero_6dof').glob('*.py'))},
                    config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                    runtime_config_sha256=hashlib.sha256(
                        (output/'runtime_parameters.json').read_bytes()).hexdigest(),
                    launch_command=command,
                    estimator_source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                             for root in ('kalman_filter_localization_core',
                                                          'kalman_filter_localization_ros2',
                                                          'kalman_filter_localization_msgs')
                                             for p in sorted(Path(root).rglob('*'))
                                             if p.is_file() and (p.name == 'CMakeLists.txt' or p.suffix in (
                                                 '.cpp', '.hpp', '.yaml', '.py', '.msg', '.xml'))})
    (output/'scenario.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(json.dumps(metadata, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    simulate(parser.parse_args().output)
