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


"""Exercise installed stack launches, activation and TF ownership over DDS."""

from copy import deepcopy
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped, TwistStamped, TwistWithCovarianceStamped
from kalman_filter_localization_msgs.msg import EstimatorStatus
from nav_msgs.msg import Odometry
import numpy as np
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger
from test_state_output_ros import assign_quaternion, multiply, quaternion, rotation
from test_state_output_ros import stamp, stamp_ns
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import yaml


@pytest.mark.parametrize('role', ['autoware', 'nav2'])
def test_stack_launch_and_activation(role, tmp_path):
    """Verify namespaced profiles, activation cycles and sole global TF edge."""
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('stack_fixture', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs = {key: [] for key in ('odom', 'pose', 'pose_cov', 'twist', 'twist_cov')}
    statuses, transforms = [], []
    subscriptions = [node.create_subscription(kind, '/vehicle/' + name, outputs[key].append, 100)
                     for key, name, kind in (
                         ('odom', 'filtered_odom', Odometry),
                         ('pose', 'ekf_pose', PoseStamped),
                         ('pose_cov', 'ekf_pose_with_covariance', PoseWithCovarianceStamped),
                         ('twist', 'ekf_twist', TwistStamped),
                         ('twist_cov', 'ekf_twist_with_covariance', TwistWithCovarianceStamped))]
    subscriptions += [
        node.create_subscription(EstimatorStatus, '/vehicle/ekf_localization/status',
                                 statuses.append, 100),
        node.create_subscription(TFMessage, '/tf',
                                 lambda message: transforms.extend(message.transforms), 100),
    ]
    seed_pub = node.create_publisher(PoseWithCovarianceStamped, '/vehicle/seed_input', 100)
    imu_pub = node.create_publisher(Imu, '/vehicle/imu_input', qos_profile_sensor_data)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/vehicle/pose_input', 100)
    twist_pub = node.create_publisher(TwistWithCovarianceStamped, '/vehicle/twist_input', 100)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    trigger = node.create_client(SetBool, '/vehicle/trigger_node')
    drain = node.create_client(Trigger, '/vehicle/ekf_localization/drain_input_buffer')
    params = AsyncParameterClient(node, '/vehicle/ekf_localization')
    broadcaster = TransformBroadcaster(node)
    # Exercise runtime profile loading in a namespace and buffered input flushing.
    profile = tmp_path / 'profile.yaml'
    share = Path(get_package_share_directory('kalman_filter_localization'))
    config = yaml.safe_load((share / 'param/profiles/autoware_6dof_forward.yaml').read_text())
    config['ekf_localization']['ros__parameters'].update({
        'input_reorder_window_sec': 1.0,
        'enable_stationary_initialization': False,
    })
    profile.write_text(yaml.safe_dump(config))
    command = ['ros2', 'launch', 'kalman_filter_localization', role + '.launch.py',
               'namespace:=vehicle', 'use_sim_time:=true', 'map_frame:=world',
               'odom_frame:=local', 'base_frame:=body', 'imu_topic:=imu_input',
               'input_initial_pose_name:=seed_input', 'input_pose_with_cov_name:=pose_input',
               'input_twist_with_cov_name:=twist_input', 'output_odom_name:=filtered_odom',
               'param_file:=' + str(profile)]
    log_path = tmp_path / (role + '.log')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    def wait(predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin(0.02)
        assert predicate(), (str({key: len(values) for key, values in outputs.items()}) +
                             '\n' + log_path.read_text())

    def call(client, request):
        future = client.call_async(request)
        wait(future.done)
        assert future.result().success, future.result().message

    def activate(value):
        request = SetBool.Request()
        request.data = value
        call(trigger, request)
        wait(lambda: statuses and statuses[-1].active == value)

    def flush():
        call(drain, Trigger.Request())

    def send_imu(nanoseconds):
        stamp(imu.header.stamp, nanoseconds)
        imu_pub.publish(imu)
        spin(0.08)
        flush()

    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                assert trigger.wait_for_service(timeout_sec=15), log_path.read_text()
                assert drain.wait_for_service(timeout_sec=5), log_path.read_text()
                assert params.wait_for_services(timeout_sec=5)
                wait(lambda: all(pub.get_subscription_count() > 0
                                 for pub in (seed_pub, imu_pub, pose_pub, twist_pub)))
                clock = Clock()
                stamp(clock.clock, 10_000_000_000)
                clock_pub.publish(clock)
                spin(0.1)
                future = params.get_parameters(['start_active', 'publish_tf', 'tf_output_mode',
                                                'twist_linear_axes', 'use_flat_ground'])
                wait(future.done)
                configured = future.result().values
                assert configured[0].bool_value == (role == 'nav2')
                assert configured[1].bool_value == (role == 'nav2')
                assert configured[2].string_value == (
                    'reference_to_odom' if role == 'nav2' else 'reference_to_base')
                assert list(configured[3].string_array_value) == ['x']
                assert not configured[4].bool_value
                q = multiply(quaternion([0, 0, 1], 0.5), quaternion([1, 0, 0], 0.2))
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id = 'world'
                stamp(seed.header.stamp, 10_000_000_000)
                seed.pose.pose.position.x = 3.0
                seed.pose.pose.position.y = -1.0
                seed.pose.pose.position.z = 0.7
                assign_quaternion(seed.pose.pose.orientation, q)
                seed.pose.covariance = np.diag([.04, .04, .06, .001, .001, .003]).ravel().tolist()
                imu = Imu()
                imu.header.frame_id = 'body'
                acceleration = rotation(q).T @ np.array([0., 0., 9.80665])
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (
                    float(v) for v in acceleration)
                imu.orientation_covariance[0] = -1.0
                seed_pub.publish(seed)
                spin(.1)
                flush()
                if role == 'autoware':
                    send_imu(10_000_000_000)
                    assert not outputs['odom']
                    assert not statuses[-1].active and statuses[-1].summary == 'inactive'
                    activate(True)
                    spin(.1)
                    assert not outputs['odom']  # ignored IMU cannot bootstrap activation
                send_imu(10_010_000_000)
                wait(lambda: all(outputs.values()))
                wait(lambda: statuses and statuses[-1].initialized)
                odom = outputs['odom'][-1]
                assert odom.header.frame_id == 'world' and odom.child_frame_id == 'body'
                np.testing.assert_allclose([odom.pose.pose.position.x, odom.pose.pose.position.y,
                                            odom.pose.pose.position.z], [3., -1., .7], atol=1e-5)
                np.testing.assert_allclose(
                    [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z, odom.pose.pose.orientation.w], q, atol=1e-6)
                assert stamp_ns(odom.header.stamp) == 10_010_000_000
                for values in outputs.values():
                    assert stamp_ns(values[-1].header.stamp) == stamp_ns(odom.header.stamp)
                if role == 'nav2':
                    # Global messages can publish before local TF, but global TF must wait.
                    assert not [tf for tf in transforms if tf.header.frame_id == 'world']
                    local = TransformStamped()
                    local.header.frame_id, local.child_frame_id = 'local', 'body'
                    local.header.stamp = odom.header.stamp
                    local.transform.translation.x = .4
                    local.transform.translation.y = -.2
                    local_q = quaternion([0, 0, 1], -.3)
                    assign_quaternion(local.transform.rotation, local_q)
                    for _ in range(3):
                        broadcaster.sendTransform(local)
                        spin(.05)
                    # Next IMU triggers another nonblocking attempt to compose queued TF.
                    send_imu(10_020_000_000)
                    wait(lambda: any(tf.header.frame_id == 'world' for tf in transforms))
                    global_tf = [tf for tf in transforms if tf.header.frame_id == 'world'][-1]
                    assert global_tf.child_frame_id == 'local'
                    assert stamp_ns(global_tf.header.stamp) == 10_010_000_000
                    world_local_q = np.array([
                        global_tf.transform.rotation.x, global_tf.transform.rotation.y,
                        global_tf.transform.rotation.z, global_tf.transform.rotation.w])
                    np.testing.assert_allclose(rotation(world_local_q) @ rotation(local_q),
                                               rotation(q), atol=1e-6)
                    world_local_t = np.array([global_tf.transform.translation.x,
                                              global_tf.transform.translation.y,
                                              global_tf.transform.translation.z])
                    np.testing.assert_allclose(
                        world_local_t + rotation(world_local_q) @ np.array([.4, -.2, 0.]),
                        [3., -1., .7], atol=1e-5)
                    assert not any(tf.header.frame_id == 'world' and tf.child_frame_id == 'body'
                                   for tf in transforms)
                else:
                    assert not [tf for tf in transforms if tf.header.frame_id == 'world']
                # Queue a sensor packet, then deactivate before draining it.
                stamp(imu.header.stamp, 10_030_000_000)
                imu_pub.publish(imu)
                wait(lambda: statuses[-1].buffered_input_count > 0)
                activate(False)
                spin(.15)
                assert statuses[-1].buffered_input_count == 0
                assert not statuses[-1].initialized
                before = len(outputs['odom'])
                before_tf = len([tf for tf in transforms if tf.header.frame_id == 'world'])
                send_imu(10_040_000_000)
                spin(.15)
                assert len(outputs['odom']) == before
                assert len([tf for tf in transforms if tf.header.frame_id == 'world']) == before_tf
                activate(True)
                send_imu(10_050_000_000)
                assert len(outputs['odom']) == before  # deactivation discarded old seed
                assert statuses[-1].summary == 'waiting_for_initial_pose'
                # Invalid new seed cannot reactivate output.
                invalid = deepcopy(seed)
                invalid.pose.pose.orientation.w = 0.
                invalid.pose.pose.orientation.x = 0.
                invalid.pose.pose.orientation.y = 0.
                invalid.pose.pose.orientation.z = 0.
                seed_pub.publish(invalid)
                spin(.1)
                send_imu(10_060_000_000)
                assert len(outputs['odom']) == before
                # New epoch seed becomes visible only after new IMU.
                stamp(seed.header.stamp, 10_070_000_000)
                seed.pose.pose.position.z = 1.2
                seed_pub.publish(seed)
                spin(.1)
                flush()
                assert len(outputs['odom']) == before
                send_imu(10_080_000_000)
                wait(lambda: len(outputs['odom']) > before)
                assert outputs['odom'][-1].pose.pose.position.z == pytest.approx(1.2, abs=1e-5)
                # Repeated activation preserves the live state and initialization.
                activate(True)
                assert statuses[-1].initialized
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        assert subscriptions
