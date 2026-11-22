# Copyright (c) 2026 Picaresque SRL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
"""
Two VIO trackers in parallel (cam1 and cam2 stereo IR + IMU, one pq_cuvslam node each) and the
odometry selector, which publishes the odometry of the healthy one in the cam2 base frame
(visual_slam/tracking/odometry_fused).

Live:    ros2 launch pq_cuvslam dual_vio.launch.py
Bag:     ros2 launch pq_cuvslam dual_vio.launch.py bag:=<bag> [rate:=1.0]   (real-time play)
Output:  run_output_dir:=<dir> -> <dir>/cam1, <dir>/cam2 (pq_cuvslam), <dir>/fused (selector)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# cam1_link in cam2_link: holder CAD translation, pitch estimated from the gyroscopes (as the
# extrinsics of the cam1_cam2_ir profile).
CAM1_IN_CAM2 = [-0.0134, 0.0, -0.0634, 0.0, 0.8604, 0.0]


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    out = arg('run_output_dir')
    enable_vis = arg('enable_vis').lower() in ('1', 'true')
    nodes = []
    for cam in ('cam1', 'cam2'):
        nodes.append(ComposableNode(
            name='visual_slam_node',
            namespace=f'{cam}_vio',
            package='pq_cuvslam',
            plugin='pq::visual_slam::VisualSlamNode',
            parameters=[{
                'profile': arg(f'{cam}_profile'),
                'processing_mode': 'realtime',
                'run_output_dir': os.path.join(out, cam) if out else '',
                'map_frame': f'{cam}_map',
                'odom_frame': f'{cam}_odom',
                'enable_slam_visualization': enable_vis,
                'enable_landmarks_view': enable_vis,
                'enable_observations_view': enable_vis,
            }],
        ))
    nodes.append(ComposableNode(
        name='odometry_selector',
        package='pq_cuvslam',
        plugin='pq::visual_slam::OdometrySelectorNode',
        parameters=[{
            'trackers': ['cam1', 'cam2'],
            'namespaces': ['/cam1_vio', '/cam2_vio'],
            'primary': arg('primary'),
            'ref_from_cam1': CAM1_IN_CAM2,
            'ref_from_cam2': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'reference_frame': 'cam2_link',
            'odom_frame': 'odom',
            'run_output_dir': os.path.join(out, 'fused') if out else '',
        }],
    ))
    actions = [ComposableNodeContainer(
        name='dual_vio_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen',
    )]
    bag = arg('bag')
    if bag:
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--rate', arg('rate'), '--delay', '5.0'],
            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam1_profile', default_value='cam1_stereo_imu'),
        DeclareLaunchArgument('cam2_profile', default_value='cam2_stereo_imu'),
        DeclareLaunchArgument('primary', default_value='cam1'),
        DeclareLaunchArgument('run_output_dir', default_value=''),
        DeclareLaunchArgument('enable_vis', default_value='false'),
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=launch_setup),
    ])
