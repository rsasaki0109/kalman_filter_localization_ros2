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


"""Launch the shared estimator with explicit Autoware or Nav2 TF ownership."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_estimator(context):
    """Resolve parameter layers independently of the runtime node namespace."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('kalman_filter_localization'))
    parameters = {}
    for path in (share / 'param' / 'ekf.yaml', Path(value('param_file'))):
        with path.open() as stream:
            parameters.update(yaml.safe_load(stream)['ekf_localization']['ros__parameters'])
    nav2 = value('integration') == 'nav2'
    parameters.update({
        'use_sim_time': value('use_sim_time') == 'true',
        'start_active': nav2 if value('start_active') == 'auto'
        else value('start_active') == 'true',
        'reference_frame_id': value('map_frame'),
        'odom_frame_id': value('odom_frame'),
        'robot_frame_id': value('base_frame'),
        'imu_topic': value('imu_topic'),
        'output_odometry_topic': 'ekf_localization/current_odometry',
        'initial_pose_covariance_topic': value('input_initial_pose_name'),
        'pose_measurement_topic': value('input_pose_with_cov_name'),
        'twist_measurement_topic': value('input_twist_with_cov_name'),
        'publish_tf': nav2 if value('publish_tf') == 'auto' else value('publish_tf') == 'true',
        'tf_output_mode': 'reference_to_odom' if nav2 else 'reference_to_base',
    })
    outputs = {
        'current_pose': 'output_pose_name',
        'current_pose_with_covariance': 'output_pose_with_covariance_name',
        'current_twist': 'output_twist_name',
        'current_twist_with_covariance': 'output_twist_with_covariance_name',
        'current_odometry': 'output_odom_name',
        'trigger_node_srv': 'input_trigger_node_service_name',
        'diagnostics': 'diagnostics_topic',
    }
    return [Node(
        package='kalman_filter_localization', executable='ekf_localization_node',
        name='ekf_localization', namespace=value('namespace'), output='screen',
        parameters=[parameters],
        remappings=[('ekf_localization/' + source, value(target))
                    for source, target in outputs.items()],
    )]


def generate_launch_description():
    """Expose topic names matching the pinned Autoware EKF launch contract."""
    share = Path(get_package_share_directory('kalman_filter_localization'))
    arguments = [
        DeclareLaunchArgument('integration', default_value='autoware',
                              choices=['autoware', 'nav2']),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('param_file', default_value=str(
            share / 'param' / 'profiles' / 'autoware_6dof_forward.yaml'),
            description='This package parameter profile; not an Autoware EKF parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_active', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='auto: Autoware inactive, Nav2 active'),
        DeclareLaunchArgument('publish_tf', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='auto: Autoware false, Nav2 map-to-odom true'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('input_initial_pose_name', default_value='initialpose3d'),
        DeclareLaunchArgument('input_trigger_node_service_name', default_value='trigger_node'),
        DeclareLaunchArgument('input_pose_with_cov_name', default_value='in_pose_with_covariance'),
        DeclareLaunchArgument('input_twist_with_cov_name',
                              default_value='in_twist_with_covariance'),
        DeclareLaunchArgument('output_pose_name', default_value='ekf_pose'),
        DeclareLaunchArgument('output_pose_with_covariance_name',
                              default_value='ekf_pose_with_covariance'),
        DeclareLaunchArgument('output_twist_name', default_value='ekf_twist'),
        DeclareLaunchArgument('output_twist_with_covariance_name',
                              default_value='ekf_twist_with_covariance'),
        DeclareLaunchArgument('output_odom_name', default_value='ekf_odom'),
        DeclareLaunchArgument('diagnostics_topic', default_value='/diagnostics'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_estimator)])
