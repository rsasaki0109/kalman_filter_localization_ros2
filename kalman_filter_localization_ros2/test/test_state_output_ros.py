# Copyright (c) 2026, Ryohei Sasaki
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Exercise real 6DoF messages and delayed map->odom composition over DDS."""

import math
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quaternion(axis, angle):
    return np.r_[np.asarray(axis) * math.sin(angle / 2), math.cos(angle / 2)]


def multiply(a, b):
    return np.r_[a[3] * b[:3] + b[3] * a[:3] + np.cross(a[:3], b[:3]),
                 a[3] * b[3] - np.dot(a[:3], b[:3])]


def rotation(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def assign_quaternion(message, q):
    message.x, message.y, message.z, message.w = [float(value) for value in q]


def stamp(message, nanoseconds):
    message.sec, message.nanosec = divmod(nanoseconds, 1_000_000_000)


def stamp_ns(message):
    return message.sec * 1_000_000_000 + message.nanosec


@pytest.mark.parametrize('noise_kind', ['known', 'zero', 'unavailable'])
def test_six_dof_outputs_and_delayed_nav2_tf(tmp_path, noise_kind):
    """A tilted, rotating IMU must not be flattened or mixed with stale TF."""
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('state_output_integration_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs = {name: [] for name in ('odom', 'pose', 'twist', 'twist_cov', 'tf')}
    subscriptions = []
    for name, kind, topic in (
        ('odom', Odometry, '/output_test/current_odometry'),
        ('pose', PoseWithCovarianceStamped, '/output_test/current_pose_with_covariance'),
        ('twist', TwistStamped, '/output_test/current_twist'),
        ('twist_cov', TwistWithCovarianceStamped, '/output_test/current_twist_with_covariance'),
        ('tf', TFMessage, '/tf'),
    ):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, key=name: outputs[key].append(msg), 200))
    initial_pub = node.create_publisher(PoseStamped, '/output_test/initial_pose', 10)
    imu_pub = node.create_publisher(Imu, '/output_test/imu', qos_profile_sensor_data)
    local_tf = TransformBroadcaster(node)
    log_path = tmp_path / 'node.log'
    executable = (Path(get_package_prefix('kalman_filter_localization')) /
                  'lib/kalman_filter_localization/ekf_localization_node')
    parameters = {
        'reference_frame_id': 'test_map', 'robot_frame_id': 'test_base',
        'odom_frame_id': 'test_odom', 'output_publish_mode': 'imu',
        'output_stamp_source': 'imu', 'input_qos_depth': 200, 'output_qos_depth': 200,
        'publish_tf': True, 'tf_output_mode': 'reference_to_odom',
        'use_gnss': False, 'use_odom': False, 'use_flat_ground': False,
        'use_nonholonomic_constraint': False, 'use_wheel_speed': False,
        'propagation_model': 'fast', 'initial_attitude_variance_rp': 0.02,
        'initial_attitude_variance_yaw': 0.04,
    }
    command = [str(executable), '--ros-args', '-r', '__node:=output_test']
    for key, value in parameters.items():
        command.extend(['-p', '{}:={}'.format(key, str(value).lower())])

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 12
                while initial_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    spin(0.05)
                assert initial_pub.get_subscription_count() > 0, log_path.read_text()
                # A genuinely tilted starting attitude and vertical position.
                q0 = multiply(quaternion([0, 0, 1], 0.4),
                              multiply(quaternion([0, 1, 0], -0.3),
                                       quaternion([1, 0, 0], 0.2)))
                initial = PoseStamped()
                initial.header.frame_id = 'test_map'
                stamp(initial.header.stamp, 100_000_000_000)
                initial.pose.position.z = 2.0
                assign_quaternion(initial.pose.orientation, q0)
                initial_pub.publish(initial)
                spin(0.1)
                rates = np.array([0.02, -0.015, 0.03])
                axis = rates / np.linalg.norm(rates)
                sensor_rotation = rotation(quaternion([0, 0, 1], 0.6))
                # Non-identity sensor mounting tests rotation of IMU covariance.
                mounting = TransformStamped()
                mounting.header.frame_id = 'test_base'
                mounting.child_frame_id = 'test_imu'
                assign_quaternion(mounting.transform.rotation, quaternion([0, 0, 1], 0.6))
                from tf2_ros import StaticTransformBroadcaster
                static_tf = StaticTransformBroadcaster(node)
                static_tf.sendTransform(mounting)
                spin(0.2)
                for index in range(101):
                    t = index * 0.01
                    # Local TF deliberately trails IMU output by five samples.
                    if index >= 5 and index % 5 == 0:
                        local = TransformStamped()
                        local.header.frame_id = 'test_odom'
                        local.child_frame_id = 'test_base'
                        stamp(local.header.stamp, 100_000_000_000 + (index - 5) * 10_000_000)
                        local.transform.translation.x = 1.0 + (index - 5) * 0.01
                        local.transform.translation.z = 0.5
                        local.transform.rotation.w = 1.0
                        local_tf.sendTransform(local)
                    q = multiply(q0, quaternion(axis, np.linalg.norm(rates) * t))
                    imu = Imu()
                    imu.header.frame_id = 'test_imu'
                    stamp(imu.header.stamp, 100_000_000_000 + index * 10_000_000)
                    imu.orientation_covariance[0] = -1.0
                    acceleration = sensor_rotation.T @ rotation(q).T @ np.array([0, 0, 9.80665])
                    sensor_rates = sensor_rotation.T @ rates
                    for field, values in ((imu.angular_velocity, sensor_rates),
                                          (imu.linear_acceleration, acceleration)):
                        field.x, field.y, field.z = [float(value) for value in values]
                    if noise_kind == 'known':
                        imu.angular_velocity_covariance = [
                            0.001, 0., 0., 0., 0.002, 0., 0., 0., 0.003]
                    elif noise_kind == 'unavailable':
                        imu.angular_velocity_covariance[0] = -1.0
                    imu_pub.publish(imu)
                    spin(0.015)
                spin(0.5)
                assert process.poll() is None, log_path.read_text()
                for key in ('odom', 'pose', 'twist', 'twist_cov'):
                    assert len(outputs[key]) >= 90, (key, len(outputs[key]), log_path.read_text())
                odometry = {stamp_ns(msg.header.stamp): msg for msg in outputs['odom']}
                sample = outputs['odom'][-1]
                assert sample.header.frame_id == 'test_map'
                assert sample.child_frame_id == 'test_base'
                assert abs(sample.pose.pose.position.z - 2) < 0.02
                actual_q = sample.pose.pose.orientation
                assert abs(actual_q.x) > 0.05 and abs(actual_q.y) > 0.05
                assert np.allclose([sample.twist.twist.angular.x, sample.twist.twist.angular.y,
                                    sample.twist.twist.angular.z], rates, atol=1e-8)
                for covariance in (sample.pose.covariance, sample.twist.covariance):
                    matrix = np.asarray(covariance).reshape(6, 6)
                    assert np.isfinite(matrix).all()
                    assert np.allclose(matrix, matrix.T, atol=1e-10)
                    assert np.linalg.eigvalsh(matrix).min() > -1e-9
                expected_noise = (
                    sensor_rotation @ np.diag([0.001, 0.002, 0.003]) @ sensor_rotation.T
                    if noise_kind == 'known' else 1e6 * np.eye(3))
                assert np.allclose(np.asarray(sample.twist.covariance).reshape(6, 6)[3:, 3:],
                                   expected_noise, atol=1e-8)
                latest_stamp = stamp_ns(sample.header.stamp)
                pose = next(msg for msg in outputs['pose']
                            if stamp_ns(msg.header.stamp) == latest_stamp)
                twist = next(msg for msg in outputs['twist_cov']
                             if stamp_ns(msg.header.stamp) == latest_stamp)
                assert list(pose.pose.covariance) == list(sample.pose.covariance)
                assert list(twist.twist.covariance) == list(sample.twist.covariance)
                assert twist.header.frame_id == 'test_base'
                transforms = [
                    tf for group in outputs['tf'] for tf in group.transforms
                    if tf.header.frame_id == 'test_map' and tf.child_frame_id == 'test_odom']
                assert len(transforms) >= 5, log_path.read_text()
                for transform in transforms:
                    ns = stamp_ns(transform.header.stamp)
                    assert ns in odometry
                    pose = odometry[ns].pose.pose
                    q = pose.orientation
                    tf_q = transform.transform.rotation
                    assert np.allclose(
                        rotation(np.array([tf_q.x, tf_q.y, tf_q.z, tf_q.w])),
                        rotation(np.array([q.x, q.y, q.z, q.w])), atol=1e-8)
                    # map<-odom composed with odom<-base at the identical stamp.
                    delta = rotation(np.array([q.x, q.y, q.z, q.w])) @ np.array([
                        1.0 + (ns - 100_000_000_000) / 1e9, 0, 0.5])
                    predicted = np.array([transform.transform.translation.x,
                                          transform.transform.translation.y,
                                          transform.transform.translation.z]) + delta
                    assert np.allclose(
                        predicted, [pose.position.x, pose.position.y, pose.position.z], atol=1e-7)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
