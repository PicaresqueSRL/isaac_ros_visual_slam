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
pq_cuvslam with a camera layout profile, live or on a bag.

Live / topics:     ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu
Bag replay:        ... slam.launch.py profile:=<profile> bag:=<bag> run_output_dir:=<dir>
Simulated online:  ... slam.launch.py profile:=<profile> bag:=<bag> rate:=1.0
                   latency_profile:=<csv>

Launch arguments
----------------
profile
    Name of a profile in share/pq_cuvslam/profiles (e.g. cam2_stereo_imu, cam1_cam2_ir) or
    path of a profile YAML file. It defines cameras, topics, frames, extrinsics and mode.
bag
    Bag folder. If set, the bag is replayed by pq_cuvslam_replay instead of subscribing to
    the topics.
rate
    0 (default): deterministic replay, every frame processed, as fast as possible.
    R > 0: simulated online replay at R times the recorded speed; frames that are not
    processed in time are dropped by keep-last queues, as with live subscriptions.
processing_mode
    realtime | offline. Default: offline for the deterministic replay, realtime otherwise.
run_output_dir
    If set, the CSV files and run_info.json are written there.
max_frames
    Replay only the first N frames (-1 = all).
latency_profile
    status.csv (or Isaac ROS benchmark frames.csv) measured on the target platform: each frame
    takes at least the time measured there. Empty = off.
emulated_latency_ms
    Constant minimum time per frame in ms, when latency_profile is empty. 0 = off.
enable_vis
    Publish the visualization topics (default false).

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    bag = arg('bag')
    rate = float(arg('rate'))
    processing_mode = arg('processing_mode')
    if not processing_mode:
        processing_mode = 'offline' if bag and rate == 0.0 else 'realtime'
    enable_vis = arg('enable_vis').lower() in ('1', 'true')

    params = {
        'profile': arg('profile'),
        'processing_mode': processing_mode,
        'run_output_dir': arg('run_output_dir'),
        'latency_profile': arg('latency_profile'),
        'emulated_latency_ms': float(arg('emulated_latency_ms')),
        'enable_slam_visualization': enable_vis,
        'enable_landmarks_view': enable_vis,
        'enable_observations_view': enable_vis,
    }

    if bag:
        return [Node(
            package='pq_cuvslam',
            executable='pq_cuvslam_replay',
            name='visual_slam_node',
            arguments=['--bag', bag, '--rate', str(rate), '--max-frames', arg('max_frames')],
            parameters=[params],
            output='screen',
        )]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='pq_cuvslam',
        plugin='pq::visual_slam::VisualSlamNode',
        parameters=[params],
    )
    return [ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='cam2_stereo_imu'),
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='0.0'),
        DeclareLaunchArgument('processing_mode', default_value=''),
        DeclareLaunchArgument('run_output_dir', default_value=''),
        DeclareLaunchArgument('max_frames', default_value='-1'),
        DeclareLaunchArgument('latency_profile', default_value=''),
        DeclareLaunchArgument('emulated_latency_ms', default_value='0.0'),
        DeclareLaunchArgument('enable_vis', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
