# SPDX-License-Identifier: Apache-2.0
"""cam2 IR stereo + cam2 IMU (tracking_mode 1).

Same parameters as the Isaac ROS 5 reference runs
(cuvslam_stereo_imu_cam2_bench.launch.py), for the parity test.
Visualization topics are off by default; enable them with enable_vis:=true.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    num_cameras = 2
    enable_vis = ParameterValue(LaunchConfiguration('enable_vis'), value_type=bool)

    # infra1 = left (image_0), infra2 = right (image_1)
    remappings = [
        ('/visual_slam/image_0', '/cam2/infra1/image_rect_raw'),
        ('/visual_slam/camera_info_0', '/cam2/infra1/camera_info'),
        ('/visual_slam/image_1', '/cam2/infra2/image_rect_raw'),
        ('/visual_slam/camera_info_1', '/cam2/infra2/camera_info'),
        ('/visual_slam/imu', '/cam2/imu'),
    ]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='pq_cuvslam',
        plugin='pq::visual_slam::VisualSlamNode',
        parameters=[{
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
            'num_cameras': num_cameras,
            'min_num_images': num_cameras,
            'camera_optical_frames': [
                'cam2_infra1_optical_frame',
                'cam2_infra2_optical_frame',
            ],
        }],
        remappings=remappings,
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_vis', default_value='false'),
        container,
    ])
