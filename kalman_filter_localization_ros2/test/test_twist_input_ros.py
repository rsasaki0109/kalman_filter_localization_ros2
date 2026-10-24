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


"""Exercise body velocity, offset sensor transport and Twist replay over DDS."""

from copy import deepcopy
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from kalman_filter_localization_msgs.msg import MeasurementQuality
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from test_state_output_ros import assign_quaternion
from test_state_output_ros import multiply
from test_state_output_ros import quaternion
from test_state_output_ros import rotation
from test_state_output_ros import stamp
from tf2_ros import StaticTransformBroadcaster


def run_twist_input(displaced, delayed, tmp_path, axes):
    """Run identical 3D motion with chronological or delayed velocity input."""
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('twist_fusion_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs, quality = [], []
    subscriptions = [
        node.create_subscription(Odometry, '/twist_input_test/current_odometry',
                                 outputs.append, 500),
        node.create_subscription(MeasurementQuality,
                                 '/twist_input_test/debug/measurement_quality_typed',
                                 quality.append, 500),
    ]
    initial_pub = node.create_publisher(PoseStamped, '/twist_input_test/initial_pose', 10)
    imu_pub = node.create_publisher(Imu, '/twist_input_test/imu', qos_profile_sensor_data)
    twist_pub = node.create_publisher(TwistWithCovarianceStamped, '/measured_twist', 100)
    broadcaster = StaticTransformBroadcaster(node)
    parameters = {
        'reference_frame_id': 'twist_map', 'robot_frame_id': 'twist_body',
        'twist_measurement_topic': '/measured_twist', 'max_twist_nis': 11.345,
        'twist_linear_axes': list(axes),
        'output_publish_mode': 'imu', 'output_stamp_source': 'imu',
        'input_qos_depth': 500, 'output_qos_depth': 500,
        'enable_measurement_replay': True, 'measurement_history_duration_sec': 2.0,
        'use_gnss': False, 'use_odom': False, 'use_flat_ground': False,
        'use_imu_orientation': False, 'propagation_model': 'fast',
        'initial_attitude_variance_rp': 0.0001, 'initial_attitude_variance_yaw': 0.0001,
    }
    executable = (Path(get_package_prefix('kalman_filter_localization')) /
                  'lib/kalman_filter_localization/ekf_localization_node')
    command = [str(executable), '--ros-args', '-r', '__node:=twist_input_test']
    for key, value in parameters.items():
        command.extend(['-p', '{}:={}'.format(key, str(value).lower())])
    log_path = tmp_path / '{}-{}.log'.format(displaced, delayed)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.005)

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + 12
                while (initial_pub.get_subscription_count() == 0 or
                       twist_pub.get_subscription_count() == 0 or
                       imu_pub.get_subscription_count() == 0) and time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    spin(0.05)
                assert twist_pub.get_subscription_count() > 0, log_path.read_text()
                q0 = multiply(quaternion([0, 0, 1], 0.3),
                              multiply(quaternion([0, 1, 0], -0.1),
                                       quaternion([1, 0, 0], 0.2)))
                sensor_q = multiply(quaternion([0, 0, 1], 0.6), quaternion([0, 1, 0], 0.4))
                sensor_r = rotation(sensor_q)
                lever_arm = np.array([0.8, -0.2, 0.3])
                if displaced:
                    transform = TransformStamped()
                    transform.header.frame_id = 'twist_body'
                    transform.child_frame_id = 'twist_sensor'
                    assign_quaternion(transform.transform.rotation, sensor_q)
                    transform.transform.translation.x = float(lever_arm[0])
                    transform.transform.translation.y = float(lever_arm[1])
                    transform.transform.translation.z = float(lever_arm[2])
                    broadcaster.sendTransform(transform)
                    spin(0.2)
                initial = PoseStamped()
                initial.header.frame_id = 'twist_map'
                stamp(initial.header.stamp, 300_000_000_000)
                assign_quaternion(initial.pose.orientation, q0)
                initial_pub.publish(initial)
                spin(0.1)
                omega = np.array([0.1, -0.2, 0.4]) if axes == 'xyz' else np.zeros(3)
                speed = np.array([1.0, -0.3, 0.5])
                covariance = np.diag([0.01, 0.015, 0.02, 0.002, 0.003, 0.004])
                covariance[0, 5] = covariance[5, 0] = 0.001
                pending = []
                for index in range(101):
                    now = 300_000_000_000 + index * 10_000_000
                    increment = (quaternion(
                        omega / np.linalg.norm(omega), np.linalg.norm(omega) * index * 0.01)
                        if np.linalg.norm(omega) > 0 else np.array([0., 0., 0., 1.]))
                    q = multiply(q0, increment)
                    imu = Imu()
                    imu.header.frame_id = 'twist_body'
                    stamp(imu.header.stamp, now)
                    imu.orientation_covariance[0] = -1.0
                    acceleration = rotation(q).T @ np.array([0, 0, 9.80665])
                    for field, values in ((imu.angular_velocity, omega),
                                          (imu.linear_acceleration, acceleration)):
                        field.x, field.y, field.z = [float(value) for value in values]
                    imu_pub.publish(imu)
                    if index == 0:
                        # Discovery across topics does not order initial pose and
                        # best-effort IMU. Observe priming before advancing time.
                        deadline = time.monotonic() + 5.0
                        while not outputs and time.monotonic() < deadline:
                            spin(0.05)
                            if not outputs:
                                imu_pub.publish(imu)
                        assert outputs, log_path.read_text()
                    else:
                        spin(0.01)
                    if index % 5 == 0 and index <= 95:
                        sample = TwistWithCovarianceStamped()
                        stamp(sample.header.stamp, now)
                        body_velocity = rotation(q).T @ speed
                        if displaced:
                            sample.header.frame_id = 'twist_sensor'
                            linear = sensor_r.T @ (body_velocity + np.cross(omega, lever_arm))
                            angular = sensor_r.T @ omega
                            noise = covariance.copy()
                        else:
                            sample.header.frame_id = 'twist_body'
                            linear = body_velocity
                            # Angular entries are genuinely unused at a coincident origin.
                            angular = np.full(3, np.nan)
                            noise = np.full((6, 6), np.nan)
                            noise[:3, :3] = covariance[:3, :3]
                        for axis, name in enumerate('xyz'):
                            if name not in axes:
                                linear[axis] = np.nan
                                noise[axis, :] = np.nan
                                noise[:, axis] = np.nan
                        if axes == 'x':
                            # Spin about source X cannot transport the source-X
                            # linear velocity, even for an offset origin.
                            angular[0] = np.nan
                            noise[3, :] = np.nan
                            noise[:, 3] = np.nan
                        sample.twist.twist.linear.x = float(linear[0])
                        sample.twist.twist.linear.y = float(linear[1])
                        sample.twist.twist.linear.z = float(linear[2])
                        sample.twist.twist.angular.x = float(angular[0])
                        sample.twist.twist.angular.y = float(angular[1])
                        sample.twist.twist.angular.z = float(angular[2])
                        sample.twist.covariance = noise.ravel().tolist()
                        pending.append((index + (2 if delayed else 0), sample))
                    while pending and pending[0][0] <= index:
                        twist_pub.publish(pending.pop(0)[1])
                    spin(0.01)
                spin(0.3)
                accepted = [msg for msg in quality
                            if msg.source == 'twist_linear' and msg.accepted]
                assert len(accepted) == 20, (len(accepted), log_path.read_text())
                assert len(outputs) >= 95, log_path.read_text()
                result = outputs[-1]
                attitude = result.pose.pose.orientation
                q_est = [attitude.x, attitude.y, attitude.z, attitude.w]
                v = result.twist.twist.linear
                world_velocity = rotation(q_est) @ np.array([v.x, v.y, v.z])
                if axes == 'xyz':
                    assert np.allclose(world_velocity, speed, atol=0.002), world_velocity
                else:
                    measurement_from_body = sensor_r.T if displaced else np.eye(3)
                    predicted = measurement_from_body @ np.array([v.x, v.y, v.z])
                    truth = measurement_from_body @ rotation(q).T @ speed
                    assert abs(predicted[0] - truth[0]) < 0.002, (predicted, truth)
                    velocity_covariance = np.asarray(result.twist.covariance).reshape(6, 6)[:3, :3]
                    # Two unobserved directions must keep substantial uncertainty.
                    assert np.count_nonzero(np.linalg.eigvalsh(velocity_covariance) > 1.0) >= 2
                w = result.twist.twist.angular
                assert np.allclose([w.x, w.y, w.z], omega, atol=1e-8)
                bad = deepcopy(sample)
                bad.twist.twist.linear.x += 100.
                stamp(bad.header.stamp, 300_960_000_000)
                twist_pub.publish(bad)
                spin(0.1)
                bad = deepcopy(sample)
                bad.twist.covariance[0] = -1.
                stamp(bad.header.stamp, 300_970_000_000)
                twist_pub.publish(bad)
                spin(0.1)
                bad = deepcopy(sample)
                bad.header.frame_id = 'missing_twist_frame'
                stamp(bad.header.stamp, 300_980_000_000)
                twist_pub.publish(bad)
                spin(0.1)
                if displaced:
                    bad = deepcopy(sample)
                    bad.twist.twist.angular.z = np.nan
                    stamp(bad.header.stamp, 300_990_000_000)
                    twist_pub.publish(bad)
                    spin(0.1)
                rejected = [msg for msg in quality
                            if msg.source == 'twist_linear' and not msg.accepted]
                assert len(rejected) == (4 if displaced else 3), log_path.read_text()
                assert any(msg.reject_reason_text == 'nis' for msg in rejected)
                assert process.poll() is None, log_path.read_text()
                assert subscriptions
                return result
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


@pytest.mark.parametrize('displaced', [False, True])
@pytest.mark.parametrize('axes', ['xyz', 'x'])
def test_twist_transport_and_delayed_replay(displaced, axes, tmp_path):
    """Compare full node results for both body-origin and offset-source messages."""
    a = run_twist_input(displaced, False, tmp_path, axes)
    b = run_twist_input(displaced, True, tmp_path, axes)
    pa, pb = a.pose.pose.position, b.pose.pose.position
    assert np.allclose([pa.x, pa.y, pa.z], [pb.x, pb.y, pb.z], atol=1e-7)
    va, vb = a.twist.twist.linear, b.twist.twist.linear
    assert np.allclose([va.x, va.y, va.z], [vb.x, vb.y, vb.z], atol=1e-7)
    assert np.allclose(a.pose.covariance, b.pose.covariance, atol=1e-7)
    assert np.allclose(a.twist.covariance, b.twist.covariance, atol=1e-7)
