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
cam2 IR stereo + cam2 IMU (tracking_mode 1).

Same node parameters as the Isaac ROS 5 reference runs
(cuvslam_stereo_imu_cam2_bench.launch.py), for the parity test.

Live / topics:     ros2 launch pq_cuvslam cam2_stereo_imu.launch.py
Bag replay:        ... cam2_stereo_imu.launch.py bag:=<bag> run_output_dir:=<dir>
Simulated online:  ... cam2_stereo_imu.launch.py bag:=<bag> rate:=1.0 latency_profile:=<csv>

Launch arguments
----------------
bag
    Bag folder. If set, the bag is replayed by pq_cuvslam_replay instead of subscribing to
    the topics.
rate
    0 (default): deterministic replay, every frame processed, as fast as possible.
    R > 0: simulated online replay at R times the recorded speed; frames that are not
    processed in time are dropped by keep-last queues, as with live subscriptions.
processing_mode
    realtime | offline. Default: offline for the deterministic replay, realtime otherwise.
latency_profile
    status.csv measured on the target platform (e.g. Jetson): each frame takes at least the
    time measured there. Empty = off.
emulated_latency_ms
    Constant minimum time per frame in ms, when latency_profile is empty. 0 = off.
run_output_dir
    If set, the CSV files and run_info.json are written there.
max_frames
    Replay only the first N frames (-1 = all).
enable_vis
    Publish the visualization topics (default false).

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

NUM_CAMERAS = 2

# infra1 = left (image_0), infra2 = right (image_1)
REMAPPINGS = [
    ('/visual_slam/image_0', '/cam2/infra1/image_rect_raw'),
    ('/visual_slam/camera_info_0', '/cam2/infra1/camera_info'),
    ('/visual_slam/image_1', '/cam2/infra2/image_rect_raw'),
    ('/visual_slam/camera_info_1', '/cam2/infra2/camera_info'),
    ('/visual_slam/imu', '/cam2/imu'),
]


def node_parameters(enable_vis, processing_mode, run_output_dir, latency_profile,
                    emulated_latency_ms):
    return {
        'tracking_mode': 1,
        'enable_image_denoising': False,
        'rectified_images': True,
        'base_frame': 'cam2_link',
        'image_jitter_threshold_ms': 150.0,
        'imu_frame': 'cam2_gyro_optical_frame',
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        'calibration_frequency': 200.0,
        'enable_slam_visualization': enable_vis,
        'enable_landmarks_view': enable_vis,
        'enable_observations_view': enable_vis,
        'enable_ground_constraint_in_odometry': False,
        'enable_ground_constraint_in_slam': False,
        'enable_localization_n_mapping': True,
        'enable_debug_mode': False,
        'num_cameras': NUM_CAMERAS,
        'min_num_images': NUM_CAMERAS,
        'camera_optical_frames': [
            'cam2_infra1_optical_frame',
            'cam2_infra2_optical_frame',
        ],
        'processing_mode': processing_mode,
        'run_output_dir': run_output_dir,
        'latency_profile': latency_profile,
        'emulated_latency_ms': emulated_latency_ms,
    }


def launch_setup(context):
    bag = LaunchConfiguration('bag').perform(context)
    run_output_dir = LaunchConfiguration('run_output_dir').perform(context)
    processing_mode = LaunchConfiguration('processing_mode').perform(context)
    max_frames = LaunchConfiguration('max_frames').perform(context)
    rate = float(LaunchConfiguration('rate').perform(context))
    latency_profile = LaunchConfiguration('latency_profile').perform(context)
    emulated_latency_ms = float(LaunchConfiguration('emulated_latency_ms').perform(context))
    enable_vis = LaunchConfiguration('enable_vis').perform(context).lower() in ('1', 'true')

    if not processing_mode:
        processing_mode = 'offline' if bag and rate == 0.0 else 'realtime'
    params = node_parameters(enable_vis, processing_mode, run_output_dir, latency_profile,
                             emulated_latency_ms)

    if bag:
        return [Node(
            package='pq_cuvslam',
            executable='pq_cuvslam_replay',
            name='visual_slam_node',
            arguments=['--bag', bag, '--rate', str(rate), '--max-frames', max_frames],
            parameters=[params],
            remappings=REMAPPINGS,
            output='screen',
        )]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='pq_cuvslam',
        plugin='pq::visual_slam::VisualSlamNode',
        parameters=[params],
        remappings=REMAPPINGS,
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
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('processing_mode', default_value=''),
        DeclareLaunchArgument('run_output_dir', default_value=''),
        DeclareLaunchArgument('max_frames', default_value='-1'),
        DeclareLaunchArgument('rate', default_value='0.0'),
        DeclareLaunchArgument('latency_profile', default_value=''),
        DeclareLaunchArgument('emulated_latency_ms', default_value='0.0'),
        DeclareLaunchArgument('enable_vis', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
