// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#include "pq_cuvslam/impl/cuvslam_compat.hpp"
#include "pq_cuvslam/impl/qos.hpp"
#include "pq_cuvslam/visual_slam_node.hpp"
#include "pq_cuvslam/impl/types.hpp"
#include "pq_cuvslam/impl/visual_slam_impl.hpp"
#include "pq_cuvslam/impl/cuvslam_ros_conversion.hpp"
#include "pq_cuvslam/impl/posegraph_vis_helper.hpp"
#include "pq_cuvslam/impl/has_subscribers.hpp"
#include "pq_cuvslam/impl/stopwatch.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace pq
{
namespace visual_slam
{

using CallbackReturn = VisualSlamNode::CallbackReturn;

VisualSlamNode::VisualSlamNode(const rclcpp::NodeOptions options)
: rclcpp_lifecycle::LifecycleNode("visual_slam", options)
{
  // Parameters are declared here, so that they are visible and settable before the node is
  // configured, and read again at every configuration.
  try {
    DeclareAndReadParameters();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Invalid parameter: %s", e.what());
  }

  // Without an external lifecycle manager the node configures and activates itself as soon as
  // it is spinning, so that it behaves like the Isaac ROS node.
  if (Param<bool>("autostart", true)) {
    autostart_timer_ = create_wall_timer(
      std::chrono::milliseconds(0), [this]() {
        autostart_timer_->cancel();
        if (configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          RCLCPP_ERROR(get_logger(), "Autostart: configuration failed, the node stays inactive");
          return;
        }
        activate();
      });
  }
  pre_shutdown_handle_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [this]() {
      active_ = false;
      WriteRunOutput();
    });
}

VisualSlamNode::~VisualSlamNode()
{
  get_node_base_interface()->get_context()->remove_pre_shutdown_callback(pre_shutdown_handle_);
  active_ = false;
  TearDown();
}

template<typename T>
T VisualSlamNode::Param(
  const std::string & name, const T & default_value,
  const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  if (!has_parameter(name)) {
    return declare_parameter<T>(name, default_value, descriptor);
  }
  return get_parameter(name).get_value<T>();
}

void VisualSlamNode::DeclareAndReadParameters()
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Camera layout profile: name of an installed profile (share/pq_cuvslam/profiles) or path "
      "of a YAML file. Empty = parameters and remappings only";
    profile_name_ = Param<std::string>("profile", "", descriptor);
  }
  // Functional Parameters:
  num_cameras_ = Param<int>("num_cameras", 2);
  num_input_masks_ = Param<int>("num_input_masks", 0);
  multicam_mode_ = Param<int>("multicam_mode", 1);
  min_num_images_ = Param<int>("min_num_images", num_cameras_);
  sync_matching_threshold_ms_ = Param<double>("sync_matching_threshold_ms", 5.0);
  border_mask_top_ = Param<int>("img_mask_top", 0);
  border_mask_bottom_ = Param<int>("img_mask_bottom", 0);
  border_mask_left_ = Param<int>("img_mask_left", 0);
  border_mask_right_ = Param<int>("img_mask_right", 0);
  enable_image_denoising_ = Param<bool>("enable_image_denoising", false);
  rectified_images_ = Param<bool>("rectified_images", true);
  enable_ground_constraint_in_odometry_ =
    Param<bool>("enable_ground_constraint_in_odometry", false);
  enable_ground_constraint_in_slam_ = Param<bool>("enable_ground_constraint_in_slam", false);
  enable_localization_n_mapping_ = Param<bool>("enable_localization_n_mapping", true);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Tracking mode: 0 = Multicamera (default), 1 = VIO (IMU fusion), 2 = RGBD, 3 = Mono, "
      "4 = Multisensor (cuVSLAM >= 17)";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = 0;
    range.to_value = 4;
    range.step = 1;
    descriptor.integer_range.push_back(range);
    tracking_mode_ = Param<int>("tracking_mode", 0, descriptor);
  }
  multisensor_use_imu_ = Param<bool>("multisensor_use_imu", true);
  enable_debug_imu_mode_ = Param<bool>("debug_imu_mode", false);
  imu_params_ = IMUParams{
    Param<double>("gyro_noise_density", 0.000244),
    Param<double>("gyro_random_walk", 0.000019393),
    Param<double>("accel_noise_density", 0.001862),
    Param<double>("accel_random_walk", 0.003),
    Param<double>("calibration_frequency", 200.0)};
  depth_scale_factor_ = Param<double>("depth_scale_factor", 1000.0);
  depth_camera_id_ = Param<int>("depth_camera_id", 0);
  // The parameters are read again after the profile is applied: start from an empty list.
  depth_camera_ids_.clear();
  for (const int64_t id :
    Param<std::vector<int64_t>>("depth_camera_ids", std::vector<int64_t>{}))
  {
    depth_camera_ids_.push_back(static_cast<int32_t>(id));
  }
  if (depth_camera_ids_.empty() && depth_camera_id_ >= 0) {
    depth_camera_ids_.push_back(depth_camera_id_);
  }
  depth_enable_stereo_tracking_ = Param<bool>("depth_enable_stereo_tracking", false);
  image_jitter_threshold_ms_ = Param<double>("image_jitter_threshold_ms", 34.0);
  imu_jitter_threshold_ms_ = Param<double>("imu_jitter_threshold_ms", 10.0);
  save_map_folder_path_ = Param<std::string>("save_map_folder_path", "");
  load_map_folder_path_ = Param<std::string>("load_map_folder_path", "");
  localize_on_startup_ = Param<bool>("localize_on_startup", false);
  localizer_horizontal_radius_ = Param<double>("localizer_horizontal_radius", 1.5);
  localizer_vertical_radius_ = Param<double>("localizer_vertical_radius", 0.5);
  localizer_horizontal_step_ = Param<double>("localizer_horizontal_step", 0.5);
  localizer_vertical_step_ = Param<double>("localizer_vertical_step", 0.25);
  localizer_angular_step_ = Param<double>("localizer_angular_step", 0.1745);
  slam_max_map_size_ = Param<int>("slam_max_map_size", 300);
  slam_throttling_time_ms_ = Param<int>("slam_throttling_time_ms", 500);
  // Frame parameters:
  map_frame_ = Param<std::string>("map_frame", "map");
  odom_frame_ = Param<std::string>("odom_frame", "odom");
  base_frame_ = Param<std::string>("base_frame", "base_link");
  camera_optical_frames_ =
    Param<std::vector<std::string>>("camera_optical_frames", std::vector<std::string>{});
  imu_frame_ = Param<std::string>("imu_frame", "");
  // Message Parameters:
  image_buffer_size_ = Param<int>("image_buffer_size", 10);
  imu_buffer_size_ = Param<int>("imu_buffer_size", 50);
  image_qos_ = ParseQosString(
    Param<std::string>("image_qos", "SENSOR_DATA"), Param<int>("image_qos_depth", 0));
  // IMU queue: 400 messages (2 s at 200 Hz). The SENSOR_DATA depth (5, 25 ms at 200 Hz) drops
  // IMU samples whenever a frame takes longer, and in real time on the Orin Nano the VIO then
  // diverges (staircase_fix_1: rotation error 7-19 deg RMS with 88-100 samples lost, 0.19 deg
  // with none).
  imu_qos_ = ParseQosString(
    Param<std::string>("imu_qos", "SENSOR_DATA"), Param<int>("imu_qos_depth", 400));
  // Output Parameters:
  override_publishing_stamp_ = Param<bool>("override_publishing_stamp", false);
  publish_map_to_odom_tf_ = Param<bool>("publish_map_to_odom_tf", true);
  publish_odom_to_base_tf_ = Param<bool>("publish_odom_to_base_tf", true);
  invert_map_to_odom_tf_ = Param<bool>("invert_map_to_odom_tf", false);
  invert_odom_to_base_tf_ = Param<bool>("invert_odom_to_base_tf", false);
  // Debug/Visualization Parameters:
  enable_slam_visualization_ = Param<bool>("enable_slam_visualization", false);
  enable_observations_view_ = Param<bool>("enable_observations_view", false);
  enable_landmarks_view_ = Param<bool>("enable_landmarks_view", false);
  path_max_size_ = Param<int>("path_max_size", 1024);
  verbosity_ = Param<int>("verbosity", 1);
  enable_debug_mode_ = Param<bool>("enable_debug_mode", false);
  debug_dump_path_ = Param<std::string>("debug_dump_path", "/tmp/cuvslam");
  enable_request_hint_ = Param<bool>("enable_request_hint", true);
  // Wrapper parameters:
  run_output_dir_ = Param<std::string>("run_output_dir", "");
  enable_frame_diagnostics_ = Param<bool>("enable_frame_diagnostics", true);
  health_max_speed_mps_ = Param<double>("health_max_speed_mps", 3.0);
  health_max_gyro_error_deg_ = Param<double>("health_max_gyro_error_deg", 5.0);
  health_min_landmarks_ = Param<int>("health_min_landmarks", 10);
  // Reset of the tracker after this many rejected or lost frames within 1 s (0: never).
  health_reset_bad_frames_ = Param<int>("health_reset_bad_frames", 8);
  diagnostics_pixel_stride_ = Param<int>("diagnostics_pixel_stride", 4);
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "realtime: bundle adjustment and SLAM in background threads (default); "
      "offline: bundle adjustment and SLAM in the tracking call, reproducible runs";
    const std::string mode = Param<std::string>("processing_mode", "realtime", descriptor);
    if (mode == "realtime") {
      processing_mode_ = ProcessingMode::REALTIME;
    } else if (mode == "offline") {
      processing_mode_ = ProcessingMode::OFFLINE;
    } else {
      throw std::invalid_argument(
              "invalid processing_mode '" + mode + "', valid values are 'realtime' and 'offline'");
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Emulated per-frame processing time: path of a status.csv measured on the target "
      "platform (columns timestamp and callback_time_ms or track_time_ms). Empty = off";
    latency_profile_ = Param<std::string>("latency_profile", "", descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description =
      "Emulated constant per-frame processing time in ms (used when latency_profile is empty). "
      "0 = off";
    emulated_latency_ms_ = Param<double>("emulated_latency_ms", 0.0, descriptor);
  }
}

void VisualSlamNode::CheckSetup()
{
  // cuVSLAM version: the library loaded at run time must be the one of the build (the
  // differences between versions are handled by impl/cuvslam_compat).
  int32_t cuvslam_major, cuvslam_minor, cuvslam_patch;
  const auto cuvslam_version = cuvslam::GetVersion(&cuvslam_major, &cuvslam_minor, &cuvslam_patch);
  const compat::Features features = compat::CompiledFeatures();
  RCLCPP_INFO(
    get_logger(), "cuVSLAM version: %s (features: mono %s, multisensor %s, cuNLS %s, "
    "SetSlamPose %s, localizer visualization %s)", std::string(cuvslam_version).c_str(),
    features.mono ? "yes" : "no", features.multisensor ? "yes" : "no",
    features.cunls ? "yes" : "no", features.set_slam_pose ? "yes" : "no",
    features.localizer_layers ? "yes" : "no");
  if (const std::string error = compat::CheckRuntimeVersion(); !error.empty()) {
    throw std::runtime_error(error);
  }

  if (tracking_mode_ < static_cast<int>(TrackingMode::MULTICAMERA) ||
    tracking_mode_ > static_cast<int>(TrackingMode::MULTISENSOR))
  {
    throw std::invalid_argument(
            "invalid tracking_mode " + std::to_string(tracking_mode_) +
            ", valid values are 0 (Multicamera), 1 (VIO), 2 (RGBD), 3 (Mono), 4 (Multisensor)");
  }
  if (tracking_mode_ == static_cast<int>(TrackingMode::MONO) && num_cameras_ != 1) {
    throw std::invalid_argument(
            "tracking_mode 3 (Mono) needs exactly one camera (num_cameras " +
            std::to_string(num_cameras_) + ")");
  }
  if (tracking_mode_ == static_cast<int>(TrackingMode::MULTISENSOR)) {
    if (!features.multisensor) {
      throw std::invalid_argument(
              "tracking_mode 4 (Multisensor) needs cuVSLAM >= 17 built with cuNLS; this build: "
              "cuVSLAM " + std::to_string(features.major) + ", cuNLS " +
              (features.cunls ? "on" : "off"));
    }
    if (depth_camera_ids_.empty() && num_cameras_ < 2) {
      throw std::invalid_argument(
              "tracking_mode 4 (Multisensor) needs a depth image (depth_camera_ids) or at "
              "least two cameras with overlapping views");
    }
    for (const int32_t id : depth_camera_ids_) {
      if (id < 0 || id >= static_cast<int32_t>(num_cameras_)) {
        throw std::invalid_argument(
                "depth_camera_ids: camera " + std::to_string(id) + " out of range (num_cameras " +
                std::to_string(num_cameras_) + ")");
      }
    }
  }
  RCLCPP_INFO(get_logger(), "Tracking mode: %s", TrackingModeToString(tracking_mode_));
  RCLCPP_INFO(
    get_logger(), "Processing mode: %s",
    processing_mode_ == ProcessingMode::OFFLINE ? "offline" : "realtime");
  if (!run_output_dir_.empty()) {
    RCLCPP_INFO(get_logger(), "Run output folder: %s", run_output_dir_.c_str());
  }

  // GPU access, with an explicit message instead of cuVSLAM's "no CUDA-capable device".
  int device_count = 0;
  const cudaError_t err = cudaGetDeviceCount(&device_count);
  if (err != cudaSuccess || device_count == 0) {
    throw std::runtime_error(
            std::string("no usable CUDA GPU (") +
            (err != cudaSuccess ? cudaGetErrorString(err) : "0 devices") + "). In a container, "
            "run it with --gpus all (PC) or with the nvidia runtime (Jetson). On Jetson also check "
            "that /dev/nvhost-ctrl-gpu and /dev/nvgpu/igpu0/* are readable and writable by the "
            "user (group video, mode g+rw)");
  }

  Stopwatch stopwatch_gpu;
  StopwatchScope ssw_gpu(stopwatch_gpu);
  cuvslam::WarmUpGPU();
  RCLCPP_INFO(get_logger(), "Time taken by cuvslam::WarmUpGPU(): %f", ssw_gpu.Stop());
}

void VisualSlamNode::CreateInterfaces()
{
  localize_in_map_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Subscribers
  rclcpp::SubscriptionOptions image_sub_options;
  image_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  image_sub_options.acceptable_buffer_backends = "any";

  // Input topics: from the profile, or visual_slam/... (to be remapped) without profile.
  const auto image_topic = [this](uint i) {
      return profile_ ? profile_->cameras.at(i).image_topic : "visual_slam/image_" +
             std::to_string(i);
    };
  const auto camera_info_topic = [this](uint i) {
      return profile_ ? profile_->cameras.at(i).camera_info_topic :
             "visual_slam/camera_info_" + std::to_string(i);
    };
  const auto depth_topic = [this](size_t k) {
      return profile_ && k < profile_->depths.size() ? profile_->depths[k].topic :
             "visual_slam/depth_" + std::to_string(k);
    };
  const std::string imu_topic = profile_ && profile_->imu ? profile_->imu->topic :
    "visual_slam/imu";

  image_subs_.clear();
  for (uint i = 0; i < num_cameras_; ++i) {
    image_subs_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        image_topic(i), image_qos_,
        [this, i](const ImageType & msg) {return CallbackImage(i, msg);},
        image_sub_options));
  }

  segmentation_masks_subs_.clear();
  if (num_input_masks_ > 0) {
    for (uint i = 0; i < num_cameras_; ++i) {
      const uint cb_index = i + num_cameras_;
      segmentation_masks_subs_.push_back(
        create_subscription<sensor_msgs::msg::Image>(
          "visual_slam/seg_mask_" + std::to_string(i), image_qos_,
          [this, cb_index](const ImageType & msg) {return CallbackImage(cb_index, msg);},
          image_sub_options));
    }
  }

  depth_image_subs_.clear();
  for (size_t k = 0; k < NumDepths(); ++k) {
    const int cb_index = DepthStreamIndex(k);
    depth_image_subs_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        depth_topic(k), image_qos_,
        [this, cb_index](const ImageType & msg) {return CallbackImage(cb_index, msg);},
        image_sub_options));
  }

  camera_info_subs_.clear();
  for (uint i = 0; i < num_cameras_; ++i) {
    camera_info_subs_.push_back(
      create_subscription<CameraInfoType>(
        camera_info_topic(i), image_qos_,
        [this, i](const CameraInfoType::ConstSharedPtr & msg) {
          return CallbackCameraInfo(i, msg);
        }));
  }

  imu_sub_.reset();
  if (UsesImu()) {
    imu_sub_ = create_subscription<ImuType>(
      imu_topic, imu_qos_,
      std::bind(&VisualSlamNode::CallbackImu, this, std::placeholders::_1));
  }

  // Gyroscopes of the synchronized devices, for the rotation compensation.
  sync_imu_subs_.clear();
  if (profile_ && profile_->sync && profile_->sync->mode == SyncMode::SOFTWARE &&
    profile_->sync->rotation_compensation)
  {
    for (size_t d = 0; d < profile_->sync->devices.size(); ++d) {
      const int device = static_cast<int>(d);
      sync_imu_subs_.emplace_back(
        device, create_subscription<ImuType>(
          profile_->sync->devices[d].imu_topic, imu_qos_,
          [this, device](const ImuType::ConstSharedPtr & msg) {
            CallbackSyncImu(device, msg);
          }));
    }
  }

  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = localize_in_map_callback_group_;
    initial_pose_sub_ = create_subscription<PoseWithCovarianceStampedType>(
      "visual_slam/initial_pose", rclcpp::QoS(rclcpp::ServicesQoS()),
      std::bind(&VisualSlamNode::CallbackInitialPose, this, std::placeholders::_1),
      options);
  }

  const rclcpp::QoS default_qos = ParseQosString("DEFAULT");

  // Publishers: Visual SLAM
  visual_slam_status_pub_ =
    create_publisher<VisualSlamStatusType>("visual_slam/status", default_qos);
  tracking_vo_pose_pub_ =
    create_publisher<PoseStampedType>("visual_slam/tracking/vo_pose", default_qos);
  tracking_vo_pose_covariance_pub_ = create_publisher<PoseWithCovarianceStampedType>(
    "visual_slam/tracking/vo_pose_covariance", default_qos);
  tracking_odometry_pub_ =
    create_publisher<OdometryType>("visual_slam/tracking/odometry", default_qos);
  tracking_odometry_guarded_pub_ =
    create_publisher<OdometryType>("visual_slam/tracking/odometry_guarded", default_qos);
  tracking_vo_path_pub_ = create_publisher<PathType>("visual_slam/tracking/vo_path", default_qos);
  tracking_slam_path_pub_ =
    create_publisher<PathType>("visual_slam/tracking/slam_path", default_qos);
  diagnostics_pub_ = create_publisher<DiagnosticArrayType>("/diagnostics", default_qos);
  frame_diagnostics_pub_ =
    create_publisher<FrameDiagnosticsType>("visual_slam/diagnostics/frame", default_qos);
  // Events: reliable and transient local, so that a late subscriber gets the recent ones.
  events_pub_ = create_publisher<SlamEventType>(
    "visual_slam/events", rclcpp::QoS(100).reliable().transient_local());

  // Publishers: sends a message to request another pose hint.
  trigger_hint_pub_.reset();
  if (enable_request_hint_) {
    trigger_hint_pub_ = create_publisher<PoseWithCovarianceStampedType>(
      "visual_slam/trigger_hint", default_qos);
  }

  // Visualizators for odometry
  vis_observations_pub_ =
    create_publisher<PointCloud2Type>("visual_slam/vis/observations_cloud", default_qos);
  vis_gravity_pub_ = create_publisher<MarkerType>("visual_slam/vis/gravity", default_qos);
  vis_vo_velocity_pub_ =
    create_publisher<MarkerArrayType>("visual_slam/vis/velocity", default_qos);
  vis_slam_odometry_pub_ =
    create_publisher<OdometryType>("visual_slam/vis/slam_odometry", default_qos);

  // Visualizators for map and "loop closure"
  vis_landmarks_pub_ =
    create_publisher<PointCloud2Type>("visual_slam/vis/landmarks_cloud", default_qos);
  vis_loop_closure_pub_ =
    create_publisher<PointCloud2Type>("visual_slam/vis/loop_closure_cloud", default_qos);
  vis_posegraph_nodes_pub_ =
    create_publisher<PoseArrayType>("visual_slam/vis/pose_graph_nodes", default_qos);
  vis_posegraph_edges_pub_ =
    create_publisher<MarkerType>("visual_slam/vis/pose_graph_edges", default_qos);
  vis_posegraph_edges2_pub_ =
    create_publisher<MarkerType>("visual_slam/vis/pose_graph_edges2", default_qos);

  // Visualizators for localization
  vis_localizer_pub_ = create_publisher<MarkerArrayType>("visual_slam/vis/localizer", default_qos);
  vis_localizer_landmarks_pub_ =
    create_publisher<PointCloud2Type>("visual_slam/vis/localizer_map_cloud", default_qos);
  vis_localizer_observations_pub_ = create_publisher<PointCloud2Type>(
    "visual_slam/vis/localizer_observations_cloud", default_qos);
  vis_localizer_loop_closure_pub_ = create_publisher<PointCloud2Type>(
    "visual_slam/vis/localizer_loop_closure_cloud", default_qos);

  // Slam Services
  reset_srv_ = create_service<SrvReset>(
    "visual_slam/reset", std::bind(
      &VisualSlamNode::CallbackReset, this, std::placeholders::_1, std::placeholders::_2));
  get_all_poses_srv_ = create_service<SrvGetAllPoses>(
    "visual_slam/get_all_poses", std::bind(
      &VisualSlamNode::CallbackGetAllPoses, this, std::placeholders::_1, std::placeholders::_2));
  set_slam_pose_srv_ = create_service<SrvSetSlamPose>(
    "visual_slam/set_slam_pose", std::bind(
      &VisualSlamNode::CallbackSetSlamPose, this, std::placeholders::_1, std::placeholders::_2));
  save_map_srv_ = create_service<SrvFilePath>(
    "visual_slam/save_map", std::bind(
      &VisualSlamNode::CallbackSaveMap, this, std::placeholders::_1, std::placeholders::_2));
  load_map_srv_ = create_service<SrvFilePath>(
    "visual_slam/load_map", std::bind(
      &VisualSlamNode::CallbackLoadMap, this, std::placeholders::_1, std::placeholders::_2));
  localize_in_map_srv_ = create_service<SrvLocalizeInMap>(
    "visual_slam/localize_in_map", std::bind(
      &VisualSlamNode::CallbackLocalizeInMap, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS(), localize_in_map_callback_group_);
}

void VisualSlamNode::DestroyInterfaces()
{
  image_subs_.clear();
  segmentation_masks_subs_.clear();
  depth_image_subs_.clear();
  camera_info_subs_.clear();
  imu_sub_.reset();
  sync_imu_subs_.clear();
  initial_pose_sub_.reset();

  visual_slam_status_pub_.reset();
  tracking_vo_pose_pub_.reset();
  tracking_vo_pose_covariance_pub_.reset();
  tracking_odometry_pub_.reset();
  tracking_odometry_guarded_pub_.reset();
  tracking_vo_path_pub_.reset();
  tracking_slam_path_pub_.reset();
  diagnostics_pub_.reset();
  frame_diagnostics_pub_.reset();
  events_pub_.reset();
  trigger_hint_pub_.reset();
  vis_observations_pub_.reset();
  vis_gravity_pub_.reset();
  vis_vo_velocity_pub_.reset();
  vis_slam_odometry_pub_.reset();
  vis_landmarks_pub_.reset();
  vis_loop_closure_pub_.reset();
  vis_posegraph_nodes_pub_.reset();
  vis_posegraph_edges_pub_.reset();
  vis_posegraph_edges2_pub_.reset();
  vis_localizer_pub_.reset();
  vis_localizer_landmarks_pub_.reset();
  vis_localizer_observations_pub_.reset();
  vis_localizer_loop_closure_pub_.reset();

  reset_srv_.reset();
  get_all_poses_srv_.reset();
  set_slam_pose_srv_.reset();
  save_map_srv_.reset();
  load_map_srv_.reset();
  localize_in_map_srv_.reset();
}

void VisualSlamNode::TearDown()
{
  // Stop any running localization thread
  localization_thread_running_ = false;
  if (localization_thread_.joinable()) {
    localization_thread_.join();
  }

  if (impl_) {
    impl_->WriteRunOutput();
    if (!save_map_folder_path_.empty() && impl_->IsInitialized()) {
      impl_->SaveMap(save_map_folder_path_);
    }
    impl_.reset();
  }
  DestroyInterfaces();
}

namespace
{

// Converts a profile value to a parameter of the type the node declared.
rclcpp::Parameter ToParameter(
  const std::string & name, const YAML::Node & value, rclcpp::ParameterType type)
{
  try {
    switch (type) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return rclcpp::Parameter(name, value.as<bool>());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return rclcpp::Parameter(name, value.as<int64_t>());
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return rclcpp::Parameter(name, value.as<double>());
      case rclcpp::ParameterType::PARAMETER_STRING:
        return rclcpp::Parameter(name, value.as<std::string>());
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        return rclcpp::Parameter(name, value.as<std::vector<std::string>>());
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        return rclcpp::Parameter(name, value.as<std::vector<double>>());
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        return rclcpp::Parameter(name, value.as<std::vector<int64_t>>());
      default:
        break;
    }
  } catch (const YAML::Exception & e) {
    throw ProfileError(
            "parameter " + name + ": value of the wrong type for the node (" + e.what() + ")");
  }
  throw ProfileError("parameter " + name + ": type not supported in profiles");
}

}  // namespace

void VisualSlamNode::ApplyProfile()
{
  profile_.reset();
  const std::string share = ament_index_cpp::get_package_share_path("pq_cuvslam").string();
  const std::string path = ResolveProfilePath(profile_name_, share);
  Profile profile = LoadProfile(path);

  const compat::Features features = compat::CompiledFeatures();
  const ProfileCheck check = CheckProfile(profile, features.major, features.multisensor);
  for (const auto & warning : check.warnings) {
    RCLCPP_WARN(get_logger(), "Profile %s: %s", profile.name.c_str(), warning.c_str());
  }
  if (!check.errors.empty()) {
    std::string message = "profile " + path + " is not valid:";
    for (const auto & error : check.errors) {
      message += "\n  - " + error;
    }
    throw ProfileError(message);
  }

  std::vector<rclcpp::Parameter> parameters;
  for (const auto & [name, value] : ProfileParameters(profile)) {
    if (!has_parameter(name)) {
      throw ProfileError(path + ": parameters." + name + ": unknown node parameter");
    }
    parameters.push_back(ToParameter(name, value, get_parameter(name).get_type()));
  }
  for (const auto & result : set_parameters(parameters)) {
    if (!result.successful) {
      throw ProfileError(path + ": cannot set the parameters: " + result.reason);
    }
  }

  RCLCPP_INFO(
    get_logger(), "Profile %s (%s): mode %s, %zu cameras%s%s, %zu extrinsics", profile.name.c_str(),
    path.c_str(), profile.mode_name.c_str(), profile.cameras.size(), profile.imu ? ", IMU" : "",
    profile.depths.empty() ? "" : (", " + std::to_string(profile.depths.size()) + " depth").c_str(),
    profile.extrinsics.size());
  for (const auto & t : profile.extrinsics) {
    RCLCPP_INFO(
      get_logger(), "  extrinsic %s -> %s: %s%s", t.parent.c_str(), t.child.c_str(),
      t.source.c_str(), t.date.empty() ? "" : (" (" + t.date + ")").c_str());
  }
  if (profile.ir_emitter) {
    RCLCPP_INFO(
      get_logger(), "  IR emitter during the recording: %s", *profile.ir_emitter ? "on" : "off");
  }
  profile_ = std::move(profile);
}

CallbackReturn VisualSlamNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  try {
    DeclareAndReadParameters();
    profile_.reset();
    if (!profile_name_.empty()) {
      ApplyProfile();
      // Read again the parameters set by the profile.
      DeclareAndReadParameters();
    }
    CheckSetup();
    CreateInterfaces();
    impl_ = std::make_unique<VisualSlamImpl>(*this);
    impl_->PublishEvent(
      SlamEventType::CONFIGURED, get_clock()->now().nanoseconds(),
      std::string("mode ") + TrackingModeToString(tracking_mode_) +
      (profile_ ? ", profile " + profile_->name : std::string()) + ", cuVSLAM " +
      std::to_string(PQ_CUVSLAM_VERSION_MAJOR) + "." + std::to_string(PQ_CUVSLAM_VERSION_MINOR) +
      "." + std::to_string(PQ_CUVSLAM_VERSION_PATCH));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
    impl_.reset();
    DestroyInterfaces();
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_activate(const rclcpp_lifecycle::State & state)
{
  // Activates the lifecycle publishers.
  LifecycleNode::on_activate(state);
  active_ = true;
  RCLCPP_INFO(get_logger(), "Active");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  active_ = false;
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "Inactive: input ignored, tracker and map kept");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  TearDown();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  active_ = false;
  TearDown();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VisualSlamNode::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Error during a transition: tearing down");
  active_ = false;
  TearDown();
  return CallbackReturn::SUCCESS;
}

void VisualSlamNode::InputImage(int index, const ImageType & msg)
{
  CallbackImage(index, msg);
}

void VisualSlamNode::InputCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg)
{
  CallbackCameraInfo(index, msg);
}

void VisualSlamNode::InputImu(const ImuType::ConstSharedPtr & msg)
{
  CallbackImu(msg);
}

void VisualSlamNode::InputStaticTransforms(const tf2_msgs::msg::TFMessage & msg)
{
  if (!impl_) {
    throw std::logic_error("InputStaticTransforms: the node is not configured");
  }
  for (const auto & transform : msg.transforms) {
    impl_->tf_buffer->setTransform(transform, "bag_replay", true);
  }
}

std::vector<std::pair<int, std::string>> VisualSlamNode::GetImageTopics() const
{
  std::vector<std::pair<int, std::string>> topics;
  for (size_t i = 0; i < image_subs_.size(); ++i) {
    topics.emplace_back(static_cast<int>(i), image_subs_[i]->get_topic_name());
  }
  for (size_t i = 0; i < segmentation_masks_subs_.size(); ++i) {
    topics.emplace_back(
      static_cast<int>(num_cameras_ + i), segmentation_masks_subs_[i]->get_topic_name());
  }
  for (size_t k = 0; k < depth_image_subs_.size(); ++k) {
    topics.emplace_back(DepthStreamIndex(k), depth_image_subs_[k]->get_topic_name());
  }
  return topics;
}

std::vector<std::pair<int, std::string>> VisualSlamNode::GetCameraInfoTopics() const
{
  std::vector<std::pair<int, std::string>> topics;
  for (size_t i = 0; i < camera_info_subs_.size(); ++i) {
    topics.emplace_back(static_cast<int>(i), camera_info_subs_[i]->get_topic_name());
  }
  return topics;
}

std::string VisualSlamNode::GetImuTopic() const
{
  return imu_sub_ ? imu_sub_->get_topic_name() : "";
}

std::vector<std::pair<int, std::string>> VisualSlamNode::GetSyncImuTopics() const
{
  std::vector<std::pair<int, std::string>> topics;
  for (const auto & [device, sub] : sync_imu_subs_) {
    topics.emplace_back(device, sub->get_topic_name());
  }
  return topics;
}

void VisualSlamNode::InputSyncImu(int device, const ImuType::ConstSharedPtr & msg)
{
  CallbackSyncImu(device, msg);
}

void VisualSlamNode::SetRunInput(const std::string & input)
{
  if (impl_) {
    impl_->run_input = input;
  }
}

void VisualSlamNode::AddRunInfo(const std::string & key, const std::string & json)
{
  if (impl_) {
    impl_->run_info_extra.emplace_back(key, json);
  }
}

void VisualSlamNode::WriteRunOutput()
{
  if (impl_) {
    impl_->WriteRunOutput();
  }
}

void VisualSlamNode::CallbackReset(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Response> res)
{
  (void)req;

  RCLCPP_INFO(this->get_logger(), "cuvslam: Reset");

  if (!impl_) {
    res->success = false;
    return;
  }
  impl_->Exit();
  res->success = true;
}

void VisualSlamNode::CallbackGetAllPoses(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "cuvslam: GetAllPoses");

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    res->success = false;
    return;
  }

  // CUVSLAM_GetAllSlamPoses
  std::vector<cuvslam::PoseStamped> cuvslam_poses;
  try {
    impl_->cuvslam_slam->GetAllSlamPoses(cuvslam_poses, req->max_count);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "GetAllSlamPoses Error: %s", e.what());
  }
  res->poses.resize(cuvslam_poses.size());

  for (uint32_t i = 0; i < cuvslam_poses.size(); i++) {
    tf2::Transform cv_map_pose_cv_base = FromcuVSLAMPose(cuvslam_poses[i].pose);
    tf2::Transform map_pose_base =
      ChangeBasis(canonical_pose_cuvslam, cv_map_pose_cv_base);
    PoseType ros_pose_msg;
    tf2::toMsg(map_pose_base, ros_pose_msg);

    res->poses[i].pose = ros_pose_msg;
    res->poses[i].header.stamp = rclcpp::Time(cuvslam_poses[i].timestamp_ns, RCL_ROS_TIME);
  }

  res->success = (cuvslam_poses.size() != 0);
}

void VisualSlamNode::CallbackSetSlamPose(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "cuvslam: CallbackSetSlamPose");

  if (impl_->IsInitialized()) {
    tf2::Transform req_map_pose_base_link;
    tf2::fromMsg(req->pose, req_map_pose_base_link);

    // Convert to cuvslam
    req_map_pose_base_link = ChangeBasis(cuvslam_pose_canonical, req_map_pose_base_link);
    cuvslam::Pose req_map_pose_cuvslam = TocuVSLAMPose(req_map_pose_base_link);

    try {
      compat::SetSlamPose(*impl_->cuvslam_slam, req_map_pose_cuvslam);
      res->success = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "SetSlamPose Error: %s", e.what());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
  }
}

void VisualSlamNode::CallbackSaveMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    return;
  }

  try {
    impl_->SaveMap(req->file_path);
    res->success = true;
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "SaveMap Error: %s", e.what());
  }
}

void VisualSlamNode::CallbackLoadMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;
  load_map_folder_path_ = req->file_path;
  res->success = true;
}

void VisualSlamNode::CallbackLocalizeInMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Response> res)
{
  res->success = false;

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    return;
  }

  if (!req->map_folder_path.empty()) {
    load_map_folder_path_ = req->map_folder_path;
  }

  // Start async localization - it will complete during normal tracking operations
  RCLCPP_INFO(this->get_logger(), "Starting async localization in map '%s'",
          load_map_folder_path_.c_str());

  // Join any existing localization thread before starting a new one
  if (localization_thread_.joinable()) {
    localization_thread_.join();
  }

  // Start the localization process in a managed thread
  localization_thread_running_ = true;
  localization_thread_ = std::thread([this, req]() {
        // Store the future in the member variable to prevent it from being destroyed
        {
          std::lock_guard<std::mutex> lock(impl_->localization_mutex_);
          impl_->localization_future_ = impl_->LocalizeInMapAsync(
            load_map_folder_path_,
            req->pose_hint,
            map_frame_);
        }

        RCLCPP_INFO(this->get_logger(),
              "Localization process initiated - will complete during normal tracking");

        // Mark thread as finished
        localization_thread_running_ = false;
      });

  // Return immediately - localization will happen during normal tracking
  res->success = true;
}

// Input is processed only in the active state; otherwise it is dropped.
void VisualSlamNode::CallbackImu(const ImuType::ConstSharedPtr & msg)
{
  if (active_) {
    impl_->CallbackImu(msg);
  }
}

void VisualSlamNode::CallbackSyncImu(int device, const ImuType::ConstSharedPtr & msg)
{
  if (active_) {
    impl_->CallbackSyncImu(device, msg);
  }
}

void VisualSlamNode::CallbackImage(int index, const ImageType & msg)
{
  if (active_) {
    impl_->CallbackImage(index, msg);
  }
}

void VisualSlamNode::CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg)
{
  if (active_) {
    impl_->CallbackCameraInfo(index, msg);
  }
}

void VisualSlamNode::CallbackInitialPose(const PoseWithCovarianceStampedType::ConstSharedPtr & msg)
{
  if (!impl_) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp);
  if (impl_->LocalizeInMap(load_map_folder_path_, msg->pose.pose, msg->header.frame_id)) {
    return;
  }

  if (enable_request_hint_) {
    trigger_hint_pub_->publish(geometry_msgs::msg::PoseWithCovarianceStamped());
    RCLCPP_INFO(this->get_logger(), "Localization failed. Requesting another localization hint.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Localization failed. Shutting down.");
    rclcpp::shutdown();
  }
}

}  // namespace visual_slam
}  // namespace pq

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pq::visual_slam::VisualSlamNode)
