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


"""Verify atomic initialization and reset across frames and clock epochs."""

from copy import deepcopy
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from kalman_filter_localization_msgs.msg import EstimatorStatus
from kalman_filter_localization_msgs.msg import ReplayTiming
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from test_state_output_ros import assign_quaternion
from test_state_output_ros import multiply
from test_state_output_ros import quaternion
from test_state_output_ros import rotation
from test_state_output_ros import stamp
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('mode', ['full', 'diagonal'])
def test_initialization_and_reset(mode, tmp_path):
    """Preserve covariance, reject invalid resets and flush queued old-epoch data."""
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('initialization_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs, statuses, timings = [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/init_test/current_odometry', outputs.append, 500),
        node.create_subscription(EstimatorStatus, '/init_test/status', statuses.append, 500),
        node.create_subscription(ReplayTiming, '/init_test/debug/replay_timing_typed',
                                 timings.append, 500),
    ]
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, '/seed_pose', 100)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/init_measurement', 100)
    imu_pub = node.create_publisher(Imu, '/init_test/imu', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    drain_client = node.create_client(Trigger, '/init_test/drain_input_buffer')
    reset_client = node.create_client(Trigger, '/init_test/reset')
    broadcaster = StaticTransformBroadcaster(node)
    dynamic_broadcaster = TransformBroadcaster(node)
    parameters = {
        'reference_frame_id': 'init_map', 'robot_frame_id': 'init_body',
        'initial_pose_covariance_topic': '/seed_pose', 'initial_pose_covariance_mode': mode,
        'pose_measurement_topic': '/init_measurement',
        'output_publish_mode': 'timer', 'output_stamp_source': 'imu',
        'input_reorder_window_sec': 1.0, 'input_qos_depth': 500, 'output_qos_depth': 500,
        'enable_measurement_replay': True, 'measurement_history_duration_sec': 2.0,
        'use_gnss': False, 'use_odom': False, 'use_flat_ground': False,
        'use_imu_orientation': False, 'use_sim_time': True,
        'initial_position_variance_xy': 0.04, 'initial_position_variance_z': 0.06,
        'initial_attitude_variance_rp': 0.001, 'initial_attitude_variance_yaw': 0.003,
    }
    executable = (Path(get_package_prefix('kalman_filter_localization')) /
                  'lib/kalman_filter_localization/ekf_localization_node')
    command = [str(executable), '--ros-args', '-r', '__node:=init_test']
    for key, value in parameters.items():
        command.extend(['-p', '{}:={}'.format(key, str(value).lower())])
    log_path = tmp_path / (mode + '.log')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def wait(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin(0.02)
        assert predicate(), log_path.read_text()

    def call(client):
        future = client.call_async(Trigger.Request())
        wait(future.done)
        assert future.result().success, future.result().message

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                assert drain_client.wait_for_service(timeout_sec=12), log_path.read_text()
                assert reset_client.wait_for_service(timeout_sec=12), log_path.read_text()
                wait(lambda: seed_pub.get_subscription_count() > 0 and
                     imu_pub.get_subscription_count() > 0 and
                     pose_pub.get_subscription_count() > 0)
                clock = Clock()
                stamp(clock.clock, 500_000_000_000)
                clock_pub.publish(clock)
                frame_q = multiply(quaternion([0, 0, 1], 0.6), quaternion([0, 1, 0], -0.2))
                frame_r = rotation(frame_q)
                frame_t = np.array([2.0, -1.0, 0.5])
                transform = TransformStamped()
                transform.header.frame_id = 'init_map'
                transform.child_frame_id = 'init_source'
                assign_quaternion(transform.transform.rotation, frame_q)
                transform.transform.translation.x = float(frame_t[0])
                transform.transform.translation.y = float(frame_t[1])
                transform.transform.translation.z = float(frame_t[2])
                broadcaster.sendTransform(transform)
                spin(0.2)
                source_q = multiply(quaternion([1, 0, 0], 0.2), quaternion([0, 1, 0], -0.1))
                world_q = multiply(frame_q, source_q)
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id = 'init_source'
                stamp(seed.header.stamp, 500_000_000_000)
                seed.pose.pose.position.x = 1.0
                seed.pose.pose.position.y = 0.5
                seed.pose.pose.position.z = 0.2
                assign_quaternion(seed.pose.pose.orientation, source_q)
                source_covariance = np.diag([0.04, 0.04, 0.06, 0.001, 0.001, 0.003])
                if mode == 'full':
                    source_covariance[0, 3] = source_covariance[3, 0] = 0.002
                    source_covariance[1, 5] = source_covariance[5, 1] = -0.001
                    seed.pose.covariance = source_covariance.ravel().tolist()
                # Diagonal mode deliberately gets zero/unknown variances and falls back.
                mapping = np.zeros((6, 6))
                mapping[:3, :3] = mapping[3:, 3:] = frame_r
                expected_covariance = mapping @ source_covariance @ mapping.T
                imu = Imu()
                imu.header.frame_id = 'init_body'
                stamp(imu.header.stamp, 500_000_000_000)
                imu.orientation_covariance[0] = -1.0
                acceleration = rotation(world_q).T @ np.array([0, 0, 9.80665])
                imu.linear_acceleration.x = float(acceleration[0])
                imu.linear_acceleration.y = float(acceleration[1])
                imu.linear_acceleration.z = float(acceleration[2])
                seed_pub.publish(seed)
                imu_pub.publish(imu)
                wait(lambda: statuses and statuses[-1].buffered_input_count >= 2)
                call(drain_client)
                wait(lambda: bool(outputs))
                assert np.allclose(outputs[-1].pose.covariance,
                                   expected_covariance.ravel(), atol=1e-11)
                # Bad requests must not clear a running state, covariance or output clock.
                invalid = []
                bad = deepcopy(seed)
                bad.pose.pose.orientation.x = bad.pose.pose.orientation.y = 0.0
                bad.pose.pose.orientation.z = bad.pose.pose.orientation.w = 0.0
                invalid.append(bad)
                bad = deepcopy(seed)
                bad.header.frame_id = 'missing_initialization_frame'
                invalid.append(bad)
                if mode == 'full':
                    bad = deepcopy(seed)
                    bad.pose.covariance[0] = -1.0
                    invalid.append(bad)
                    bad = deepcopy(seed)
                    bad.pose.covariance[3] = 1.0
                    invalid.append(bad)
                for index, bad in enumerate(invalid):
                    stamp(bad.header.stamp, 500_010_000_000 + index * 10_000_000)
                    seed_pub.publish(bad)
                    wait(lambda: statuses[-1].buffered_input_count > 0)
                    call(drain_client)
                    previous_count = len(outputs)
                    wait(lambda: len(outputs) > previous_count)
                    assert statuses[-1].initialized
                    assert np.allclose(outputs[-1].pose.covariance,
                                       expected_covariance.ravel(), atol=1e-11)
                measurement = deepcopy(seed)
                # Measurement fusion always requires a full valid covariance.
                measurement.pose.covariance = source_covariance.ravel().tolist()
                stamp(measurement.header.stamp, 500_100_000_000)
                stamp(imu.header.stamp, 500_100_000_000)
                stamp(clock.clock, 500_100_000_000)
                clock_pub.publish(clock)
                pose_pub.publish(measurement)
                imu_pub.publish(imu)
                wait(lambda: statuses[-1].buffered_input_count >= 2)
                call(drain_client)
                wait(lambda: statuses[-1].pose_available)
                dynamic = deepcopy(transform)
                dynamic.child_frame_id = 'reset_dynamic_source'
                stamp(dynamic.header.stamp, 500_100_000_000)
                dynamic_broadcaster.sendTransform(dynamic)
                # A duplicate measurement reaches replay only after reference TF
                # conversion, proving the old dynamic cache is populated.
                probe = deepcopy(measurement)
                probe.header.frame_id = 'reset_dynamic_source'
                for _ in range(5):
                    pose_pub.publish(probe)
                    wait(lambda: statuses[-1].buffered_input_count > 0)
                    call(drain_client)
                    spin(0.1)
                    if any(item.status_text == 'duplicate' for item in timings):
                        break
                assert any(item.status_text == 'duplicate' for item in timings)
                # Queue old-epoch data whose mean must never enter the restarted state.
                stale = deepcopy(measurement)
                stamp(stale.header.stamp, 500_200_000_000)
                stale.pose.pose.position.x = 1000.0
                pose_pub.publish(stale)
                stamp(imu.header.stamp, 500_200_000_000)
                imu_pub.publish(imu)
                wait(lambda: statuses[-1].buffered_input_count >= 2)
                call(reset_client)
                wait(lambda: not statuses[-1].initialized and
                     statuses[-1].buffered_input_count == 0)
                assert np.isnan(statuses[-1].pose_age_sec)
                spin(0.1)
                before = len(outputs)
                spin(0.15)
                assert len(outputs) == before
                # Old traffic while deactivated cannot establish the new reorder epoch.
                received = statuses[-1].received_pose_measurements
                stamp(stale.header.stamp, 1000_000_000_000)
                stamp(imu.header.stamp, 1000_000_000_000)
                pose_pub.publish(stale)
                imu_pub.publish(imu)
                wait(lambda: statuses[-1].received_pose_measurements > received)
                assert statuses[-1].buffered_input_count == 0
                stamp(clock.clock, 400_000_000_000)
                clock_pub.publish(clock)
                new_frame_t = np.array([-2.0, 1.0, 0.5])
                stamp(dynamic.header.stamp, 400_000_000_000)
                dynamic.transform.translation.x = float(new_frame_t[0])
                dynamic.transform.translation.y = float(new_frame_t[1])
                dynamic.transform.translation.z = float(new_frame_t[2])
                dynamic_broadcaster.sendTransform(dynamic)
                spin(0.2)
                seed.header.frame_id = 'reset_dynamic_source'
                seed.pose.pose.position.x = -1.0
                seed.pose.pose.position.y = 2.0
                stamp(seed.header.stamp, 400_000_000_000)
                seed_pub.publish(seed)
                wait(lambda: statuses[-1].summary == 'waiting_for_sensor_input')
                assert not statuses[-1].initialized
                stamp(imu.header.stamp, 400_000_000_000)
                imu_pub.publish(imu)
                wait(lambda: statuses[-1].buffered_input_count > 0)
                call(drain_client)
                wait(lambda: outputs[-1].header.stamp.sec == 400)
                actual = outputs[-1].pose.pose.position
                expected = frame_r @ np.array([-1.0, 2.0, 0.2]) + new_frame_t
                assert np.allclose([actual.x, actual.y, actual.z], expected, atol=1e-10)
                assert np.allclose(outputs[-1].pose.covariance,
                                   expected_covariance.ravel(), atol=1e-11)
                assert not statuses[-1].pose_available
                assert np.isnan(statuses[-1].pose_age_sec)
                assert subscriptions
                assert process.poll() is None, log_path.read_text()
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
