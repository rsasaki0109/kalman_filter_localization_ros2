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


"""Validate full-pose fusion, frame conversion, gating and delayed ROS input."""

from copy import deepcopy
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from kalman_filter_localization_msgs.msg import EstimatorStatus
from kalman_filter_localization_msgs.msg import MeasurementQuality
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from test_state_output_ros import assign_quaternion
from test_state_output_ros import multiply
from test_state_output_ros import quaternion
from test_state_output_ros import rotation
from test_state_output_ros import stamp
from tf2_ros import StaticTransformBroadcaster


def run_pose_input(delayed, tmp_path):
    """Run the same sensor-time sequence with two different pose arrival times."""
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('pose_fusion_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    odometry, quality, status = [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/pose_input_test/current_odometry',
                                 odometry.append, 500),
        node.create_subscription(MeasurementQuality,
                                 '/pose_input_test/debug/measurement_quality_typed',
                                 quality.append, 500),
        node.create_subscription(EstimatorStatus, '/pose_input_test/status', status.append, 500),
    ]
    initial_pub = node.create_publisher(PoseStamped, '/pose_input_test/initial_pose', 10)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/measured_pose', 100)
    imu_pub = node.create_publisher(Imu, '/pose_input_test/imu', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    broadcaster = StaticTransformBroadcaster(node)
    executable = (Path(get_package_prefix('kalman_filter_localization')) /
                  'lib/kalman_filter_localization/ekf_localization_node')
    parameters = {
        'reference_frame_id': 'pose_map', 'robot_frame_id': 'pose_body',
        'pose_measurement_topic': '/measured_pose', 'max_pose_nis': 16.812,
        'output_publish_mode': 'imu', 'output_stamp_source': 'imu',
        'input_qos_depth': 500, 'output_qos_depth': 500,
        'enable_measurement_replay': True, 'measurement_history_duration_sec': 2.0,
        'max_future_measurement_wait_sec': 0.1,
        'use_gnss': False, 'use_odom': False, 'use_flat_ground': False,
        'use_imu_orientation': False, 'propagation_model': 'fast',
        'use_sim_time': True, 'pose_measurement_timeout_sec': 1.0,
    }
    command = [str(executable), '--ros-args', '-r', '__node:=pose_input_test']
    for key, value in parameters.items():
        command.extend(['-p', '{}:={}'.format(key, str(value).lower())])
    log_path = tmp_path / ('delayed.log' if delayed else 'chronological.log')

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
                       pose_pub.get_subscription_count() == 0 or
                       imu_pub.get_subscription_count() == 0) and time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    spin(0.05)
                assert pose_pub.get_subscription_count() > 0, log_path.read_text()
                clock = Clock()
                stamp(clock.clock, 100_000_000_000)
                clock_pub.publish(clock)
                spin(0.1)
                frame_q = quaternion([0, 0, 1], 0.7)
                frame_r = rotation(frame_q)
                frame_t = np.array([3., -2., 1.])
                transform = TransformStamped()
                transform.header.frame_id = 'pose_map'
                transform.child_frame_id = 'pose_source'
                assign_quaternion(transform.transform.rotation, frame_q)
                transform.transform.translation.x = float(frame_t[0])
                transform.transform.translation.y = float(frame_t[1])
                transform.transform.translation.z = float(frame_t[2])
                broadcaster.sendTransform(transform)
                spin(0.2)
                initial = PoseStamped()
                initial.header.frame_id = 'pose_map'
                stamp(initial.header.stamp, 100_000_000_000)
                initial.pose.orientation.w = 1.0
                initial_pub.publish(initial)
                spin(0.1)
                truth_p = np.array([1., -0.5, 0.4])
                truth_q = multiply(quaternion([0, 0, 1], 0.1),
                                   multiply(quaternion([0, 1, 0], -0.06),
                                            quaternion([1, 0, 0], 0.08)))
                source_p = frame_r.T @ (truth_p - frame_t)
                source_q = multiply(np.r_[-frame_q[:3], frame_q[3]], truth_q)
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'pose_source'
                pose.pose.pose.position.x = float(source_p[0])
                pose.pose.pose.position.y = float(source_p[1])
                pose.pose.pose.position.z = float(source_p[2])
                assign_quaternion(pose.pose.pose.orientation, source_q)
                covariance = np.diag([0.001, 0.002, 0.003, 0.0001, 0.0002, 0.0003])
                covariance[0, 3] = covariance[3, 0] = 0.0001
                pose.pose.covariance = covariance.ravel().tolist()
                acceleration = rotation(truth_q).T @ np.array([0, 0, 9.80665])
                pending = []
                for index in range(151):
                    now = 100_000_000_000 + index * 10_000_000
                    stamp(clock.clock, now)
                    clock_pub.publish(clock)
                    imu = Imu()
                    imu.header.frame_id = 'pose_body'
                    stamp(imu.header.stamp, now)
                    imu.orientation_covariance[0] = -1.0
                    imu.linear_acceleration.x = float(acceleration[0])
                    imu.linear_acceleration.y = float(acceleration[1])
                    imu.linear_acceleration.z = float(acceleration[2])
                    imu_pub.publish(imu)
                    if index == 0:
                        deadline = time.monotonic() + 5.0
                        while not odometry and time.monotonic() < deadline:
                            spin(0.05)
                            if not odometry:
                                imu_pub.publish(imu)
                        assert odometry, log_path.read_text()
                    else:
                        spin(0.01)
                    if index % 10 == 0 and index <= 140:
                        sample = deepcopy(pose)
                        stamp(sample.header.stamp, now)
                        pending.append((index + (5 if delayed else 0), sample))
                    while pending and pending[0][0] <= index:
                        pose_pub.publish(pending.pop(0)[1])
                    spin(0.01)
                spin(0.3)
                accepted = [msg for msg in quality if msg.source == 'pose_6d' and msg.accepted]
                assert len(accepted) == 15, (len(accepted), log_path.read_text())
                assert len(odometry) >= 145, log_path.read_text()
                result = odometry[-1]
                actual_p = result.pose.pose.position
                actual_q = result.pose.pose.orientation
                position_error = np.array([actual_p.x, actual_p.y, actual_p.z]) - truth_p
                assert np.linalg.norm(position_error) < 0.03
                assert np.allclose(rotation([actual_q.x, actual_q.y, actual_q.z, actual_q.w]),
                                   rotation(truth_q), atol=0.002)
                result_covariance = np.array(result.pose.covariance).reshape(6, 6)
                assert np.linalg.eigvalsh(result_covariance).min() > -1e-10
                assert np.linalg.norm(result_covariance[:3, 3:]) > 1e-8
                assert status[-1].pose_input_enabled and status[-1].pose_available
                assert status[-1].received_pose_measurements == 15

                # Distinct stamps make each invalid input independently observable.
                bad_inputs = []
                outlier = deepcopy(pose)
                outlier.pose.pose.position.x += 100.
                bad_inputs.append(outlier)
                invalid_covariance = deepcopy(pose)
                invalid_covariance.pose.covariance[0] = -1.
                bad_inputs.append(invalid_covariance)
                missing_frame = deepcopy(pose)
                missing_frame.header.frame_id = 'unavailable_pose_reference'
                bad_inputs.append(missing_frame)
                invalid_orientation = deepcopy(pose)
                invalid_orientation.pose.pose.orientation.w = 0.
                invalid_orientation.pose.pose.orientation.x = 0.
                invalid_orientation.pose.pose.orientation.y = 0.
                invalid_orientation.pose.pose.orientation.z = 0.
                bad_inputs.append(invalid_orientation)
                for index, sample in enumerate(bad_inputs):
                    stamp(sample.header.stamp, 101_410_000_000 + index * 10_000_000)
                    pose_pub.publish(sample)
                    spin(0.1)
                rejected = [msg for msg in quality if msg.source == 'pose_6d' and not msg.accepted]
                assert len(rejected) == 4, (len(rejected), log_path.read_text())
                assert any(msg.reject_reason_text == 'nis' for msg in rejected)
                # Trigger a new output without another absolute-pose update.
                stamp(imu.header.stamp, 101_510_000_000)
                stamp(clock.clock, 101_510_000_000)
                clock_pub.publish(clock)
                imu_pub.publish(imu)
                spin(0.2)
                final = odometry[-1].pose.pose.position
                assert np.linalg.norm(np.array([final.x, final.y, final.z]) - truth_p) < 0.04
                # ROS time advances without IMU messages: watchdog must still expire.
                stamp(clock.clock, 102_600_000_000)
                clock_pub.publish(clock)
                spin(0.3)
                assert not status[-1].pose_available
                assert status[-1].health == EstimatorStatus.HEALTH_DEGRADED
                assert status[-1].summary == 'pose_input_stale'
                assert status[-1].pose_age_sec > 1.0
                assert status[-1].header.stamp.sec == 102
                assert status[-1].header.stamp.nanosec == 600_000_000
                # Valid but duplicate old data must not refresh sensor freshness.
                duplicate = deepcopy(pose)
                stamp(duplicate.header.stamp, 101_400_000_000)
                pose_pub.publish(duplicate)
                spin(0.2)
                assert not status[-1].pose_available
                # This hard IMU gap exceeds max_imu_dt_sec: reinitialize before
                # restarting propagation, rather than bypassing the gap limit.
                stamp(initial.header.stamp, 102_600_000_000)
                initial.pose.position.x = float(truth_p[0])
                initial.pose.position.y = float(truth_p[1])
                initial.pose.position.z = float(truth_p[2])
                assign_quaternion(initial.pose.orientation, truth_q)
                initial_pub.publish(initial)
                spin(0.2)
                assert np.isnan(status[-1].pose_age_sec)
                assert not status[-1].initialized
                assert status[-1].summary == 'waiting_for_sensor_input'
                stamp(clock.clock, 102_610_000_000)
                clock_pub.publish(clock)
                stamp(imu.header.stamp, 102_610_000_000)
                imu_pub.publish(imu)
                spin(0.1)
                fresh = deepcopy(pose)
                stamp(fresh.header.stamp, 102_610_000_000)
                pose_pub.publish(fresh)
                spin(0.3)
                assert status[-1].pose_available, log_path.read_text()
                assert status[-1].health == EstimatorStatus.HEALTH_OK
                # A Pose-only outage with continuous IMU must recover without reset.
                for index in range(1, 16):
                    current = 102_610_000_000 + index * 100_000_000
                    stamp(clock.clock, current)
                    clock_pub.publish(clock)
                    stamp(imu.header.stamp, current)
                    imu_pub.publish(imu)
                    spin(0.02)
                spin(0.2)
                assert not status[-1].pose_available
                assert status[-1].summary == 'pose_input_stale'
                stamp(fresh.header.stamp, current)
                pose_pub.publish(fresh)
                spin(0.2)
                assert status[-1].pose_available, log_path.read_text()
                assert status[-1].health == EstimatorStatus.HEALTH_OK
                assert process.poll() is None, log_path.read_text()
                assert subscriptions  # Keep subscription handles alive until process teardown.
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


def test_transformed_joint_pose_and_delayed_replay_are_equivalent(tmp_path):
    """Check the complete ROS path, including correlation and all rejection cases."""
    immediate = run_pose_input(False, tmp_path)
    delayed = run_pose_input(True, tmp_path)
    a, b = immediate.pose.pose.position, delayed.pose.pose.position
    assert np.allclose([a.x, a.y, a.z], [b.x, b.y, b.z], atol=1e-7)
    a, b = immediate.pose.pose.orientation, delayed.pose.pose.orientation
    assert np.allclose([a.x, a.y, a.z, a.w], [b.x, b.y, b.z, b.w], atol=1e-7)
    assert np.allclose(immediate.pose.covariance, delayed.pose.covariance, atol=1e-7)
    assert np.allclose(immediate.twist.covariance, delayed.twist.covariance, atol=1e-7)
