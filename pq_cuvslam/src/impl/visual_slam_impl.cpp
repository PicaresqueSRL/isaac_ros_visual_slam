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
#include <cstdio>
#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <filesystem>

#if defined(USE_NVTX)
  #include <nvtx3/nvToolsExt.h>
#endif

#include "Eigen/Dense"

#include "pq_cuvslam/git_version.hpp"  // generated: PQ_CUVSLAM_GIT_COMMIT
#if defined(VSLAM_WITH_CUDA_BUFFER)
  #include "cuda_buffer/cuda_buffer_api.hpp"
#endif
#include "pq_cuvslam/impl/cuvslam_ros_conversion.hpp"
#include "pq_cuvslam/impl/has_subscribers.hpp"
#include "pq_cuvslam/impl/stopwatch.hpp"
#include "pq_cuvslam/impl/types.hpp"
#include "opencv2/imgproc.hpp"
#include "pq_cuvslam/impl/visual_slam_impl.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

constexpr uint32_t CLR_RED = 0xFFFF0000;
constexpr uint32_t CLR_YELLOW = 0xFFFFFF00;
constexpr uint32_t CLR_MAGENTA = 0xFFFF00FF;

class NvtxRangeScoped
{
public:
  NvtxRangeScoped(const char * range_title, uint32_t range_color)
  {
    #if defined(USE_NVTX)
    nvtxEventAttributes_t event_attrib = {};
    event_attrib.version = NVTX_VERSION;
    event_attrib.size = NVTX_EVENT_ATTRIB_STRUCT_SIZE;
    event_attrib.messageType = NVTX_MESSAGE_TYPE_ASCII;
    event_attrib.colorType = NVTX_COLOR_ARGB;
    event_attrib.color = range_color;
    event_attrib.message.ascii = range_title;
    nvtxRangePushEx(&event_attrib);
    #else
    (void)range_title;
    (void)range_color;
    #endif
  }
  ~NvtxRangeScoped()
  {
    #if defined(USE_NVTX)
    nvtxRangePop();
    #endif
  }
};

void PrintConfiguration(const rclcpp::Logger & logger, const cuvslam::Odometry::Config & cfg)
{
  RCLCPP_INFO(logger, "Use use_gpu: %s", cfg.use_gpu ? "true" : "false");
  RCLCPP_INFO(logger, "Enable IMU Fusion: %s",
      cfg.odometry_mode == cuvslam::Odometry::OdometryMode::Inertial ? "true" : "false");
}

void PrintImuCalibration(const rclcpp::Logger & logger, const cuvslam::ImuCalibration & imu)
{
  RCLCPP_INFO(logger, "gyroscope_noise_density: %f", imu.gyroscope_noise_density);
  RCLCPP_INFO(logger, "gyroscope_random_walk: %f", imu.gyroscope_random_walk);
  RCLCPP_INFO(logger, "accelerometer_noise_density: %f", imu.accelerometer_noise_density);
  RCLCPP_INFO(logger, "accelerometer_random_walk: %f", imu.accelerometer_random_walk);
  RCLCPP_INFO(logger, "frequency: %f", imu.frequency);
}

std::unique_ptr<cuvslam::GroundConstraint> InitGroundConstraint(const rclcpp::Logger & logger)
{
  const cuvslam::Pose identity = pq::visual_slam::TocuVSLAMPose(
    tf2::Transform::getIdentity());
  const cuvslam::Pose world_pose_ground = identity;
  const cuvslam::Pose initial_pose_on_ground = identity;
  const cuvslam::Pose initial_pose_in_space = identity;

  try {
    auto ground_constraint = std::make_unique<cuvslam::GroundConstraint>(
      world_pose_ground, initial_pose_on_ground, initial_pose_in_space);
    return ground_constraint;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to initialize cuVSLAM ground constraint: %s", e.what());
    return nullptr;
  }
}

bool PoseToGround(
  cuvslam::GroundConstraint & ground, cuvslam::Pose & pose,
  const rclcpp::Logger & logger)
{
  try {
    ground.AddNextPose(pose);
    pose = ground.GetPoseOnGround();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to add next pose to cuVSLAM ground constraint: %s", e.what());
    return false;
  }
  return true;
}

}  // namespace


namespace pq
{
namespace visual_slam
{

VisualSlamNode::VisualSlamImpl::VisualSlamImpl(VisualSlamNode & vslam_node)
: node(vslam_node),
  sync(node.num_cameras_ + node.num_input_masks_ + node.NumDepths(),
    1e6 * node.sync_matching_threshold_ms_, node.min_num_images_ +
    node.num_input_masks_ + node.NumDepths(),
    node.image_buffer_size_),
  sequencer(node.imu_buffer_size_, node.imu_jitter_threshold_ms_, node.image_buffer_size_,
    node.image_jitter_threshold_ms_),
  tf_buffer(std::make_unique<tf2_ros::Buffer>(node.get_clock())),
  tf_listener(std::make_unique<tf2_ros::TransformListener>(*tf_buffer)),
  tf_publisher(std::make_unique<tf2_ros::TransformBroadcaster>(&node)),
  tf_static_publisher(std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node)),
  vo_path(node.path_max_size_),
  slam_path(node.path_max_size_),
  // observations_vis_helper(cuvslam::Slam::DataLayer::Observations, 2048,
  //   LandmarksVisHelper::CM_RGB_MODE, 16),
  landmarks_vis_helper(cuvslam::Slam::DataLayer::Map, 1024 * 32, LandmarksVisHelper::CM_BW_MODE,
    100),
  lc_landmarks_vis_helper(cuvslam::Slam::DataLayer::LoopClosure, 2048,
    LandmarksVisHelper::CM_RED_MODE, 16),
  pose_graph_helper(cuvslam::Slam::DataLayer::PoseGraph, 2048, 100),
  localizer_helper(8 * 2048, 100),
#if PQ_CUVSLAM_HAS_LOCALIZER_LAYERS
  localizer_landmarks_vis_helper(cuvslam::Slam::DataLayer::LocalizerMap, 1024 * 32,
    LandmarksVisHelper::CM_GREEN_MODE, 16),
  localizer_observations_vis_helper(cuvslam::Slam::DataLayer::LocalizerLandmarks, 1024 * 32,
    LandmarksVisHelper::CM_WEIGHT_BW_MODE, 16),
  localizer_lc_landmarks_vis_helper(cuvslam::Slam::DataLayer::LocalizerLoopClosure, 2048,
    LandmarksVisHelper::CM_RED_MODE, 16),
#endif
  track_execution_times(100),
  last_track_ts(-1)
{
  const cudaError_t err = cudaStreamCreate(&cuda_stream_);
  if (err != cudaSuccess) {
    throw std::runtime_error(
            std::string("Failed to create CUDA stream: ") + cudaGetErrorString(err));
  }

  cuvslam::SetVerbosity(node.verbosity_);

  if (!node.run_output_dir_.empty()) {
    run_output = std::make_unique<RunOutput>(node.run_output_dir_, node.num_cameras_);
  }

  // Extrinsics of the profile (transforms missing from the TF of the data, e.g. between two
  // cameras): added to the TF buffer used to build the rig, and published on /tf_static.
  if (node.profile_ && !node.profile_->extrinsics.empty()) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & t : node.profile_->extrinsics) {
      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = node.get_clock()->now();
      msg.header.frame_id = t.parent;
      msg.child_frame_id = t.child;
      msg.transform = tf2::toMsg(t.transform);
      tf_buffer->setTransform(msg, "profile", true);
      transforms.push_back(msg);
    }
    tf_static_publisher->sendTransform(transforms);
  }

  // Names and topics of the input images, for the diagnostics.
  for (const auto & [idx, topic] : node.GetImageTopics()) {
    std::string name = "camera_" + std::to_string(idx);
    if (const int k = node.DepthOfStream(idx); k >= 0) {
      const auto & cameras = node.profile_ ? node.profile_->cameras :
        std::vector<ProfileCamera>{};
      const auto camera = static_cast<size_t>(node.DepthCamera(static_cast<size_t>(k)));
      name = node.NumDepths() == 1 ? "depth" :
        "depth_" + (camera < cameras.size() ? cameras[camera].name : std::to_string(camera));
    } else if (node.profile_ && idx < static_cast<int>(node.profile_->cameras.size())) {
      name = node.profile_->cameras[idx].name;
    }
    input_image_names[idx] = {name, topic};
  }

  HealthConfig health_config;
  health_config.max_speed_mps = node.health_max_speed_mps_;
  health_config.max_gyro_error_deg = node.health_max_gyro_error_deg_;
  health_config.min_landmarks = node.health_min_landmarks_;
  health_config.max_vio_mismatch_mps = node.health_max_vio_mismatch_mps_;
  health_config.reset_bad_frames = node.health_reset_bad_frames_;
  health = TrackingHealth(health_config);

  // Device synchronization from the profile (two or more devices, multicamera or multisensor
  // mode). A depth image goes with the device of the camera it is aligned with.
  if (node.profile_ && node.profile_->sync && node.profile_->sync->mode != SyncMode::NONE &&
    node.profile_->sync->devices.size() >= 2)
  {
    const ProfileSync & s = *node.profile_->sync;
    const bool sync_mode_ok =
      node.tracking_mode_ == static_cast<int>(TrackingMode::MULTICAMERA) ||
      node.tracking_mode_ == static_cast<int>(TrackingMode::MULTISENSOR);
    if (!sync_mode_ok || node.num_input_masks_ > 0) {
      throw std::invalid_argument(
              "profile sync between devices is supported in multicamera and multisensor mode, "
              "without masks");
    }
    DeviceSync::Config config;
    config.devices.resize(s.devices.size());
    camera_device.assign(node.num_cameras_ + node.NumDepths(), 0);
    for (size_t i = 0; i < node.profile_->cameras.size(); ++i) {
      for (size_t d = 0; d < s.devices.size(); ++d) {
        if (node.profile_->cameras[i].device == s.devices[d].name) {
          config.devices[d].push_back(static_cast<int>(i));
          camera_device[i] = static_cast<int>(d);
        }
      }
    }
    // Depth streams: the device of their camera (same capture time on the D455).
    for (size_t k = 0; k < node.NumDepths(); ++k) {
      const int stream = node.DepthStreamIndex(k);
      const int d = camera_device.at(static_cast<size_t>(node.DepthCamera(k)));
      config.devices[d].push_back(stream);
      camera_device[static_cast<size_t>(stream)] = d;
    }
    for (const auto & d : s.devices) {
      device_clock_offset_ns.push_back(static_cast<int64_t>(std::llround(d.clock_offset_ms * 1e6)));
    }
    config.clock_offset_ns = device_clock_offset_ns;
    config.max_offset_ns = static_cast<int64_t>(std::llround(s.max_offset_ms * 1e6));
    device_sync = std::make_unique<DeviceSync>(config);
    device_sync->RegisterCallback([this](const DeviceSync::RigFrame & f) {OnRigFrame(f);});
    rotation_compensation = s.mode == SyncMode::SOFTWARE && s.rotation_compensation;
    device_gyro.resize(s.devices.size());
    if (run_output) {
      run_output->EnableSyncOffsetColumn();
    }
    RCLCPP_INFO(
      node.get_logger(), "Device sync: %s, %zu devices, max offset %.1f ms, rotation "
      "compensation %s", s.mode_name.c_str(), s.devices.size(), s.max_offset_ms,
      rotation_compensation ? "on" : "off");
  }

  // Throws if the profile cannot be read: the configuration fails.
  if (!node.latency_profile_.empty()) {
    latency_emulator = LatencyEmulator::FromProfile(node.latency_profile_);
  } else if (node.emulated_latency_ms_ > 0.0) {
    latency_emulator = LatencyEmulator::Constant(node.emulated_latency_ms_);
  }
  if (latency_emulator) {
    RCLCPP_INFO(
      node.get_logger(), "Latency emulation: %s", latency_emulator->Describe().c_str());
  }

  sequencer.RegisterCallback(
    std::bind(
      &VisualSlamNode::VisualSlamImpl::UpdatePose, this,
      std::placeholders::_1, std::placeholders::_2));

  sync.RegisterCallback(
    std::bind(
      &VisualSlamNode::VisualSlamImpl::CallbackSynchronizedImages, this,
      std::placeholders::_1, std::placeholders::_2));
}

VisualSlamNode::VisualSlamImpl::~VisualSlamImpl()
{
  Exit();
  if (cuda_stream_) {
    cudaStreamDestroy(cuda_stream_);
    cuda_stream_ = nullptr;
  }
}

// Flag to check the status of initialization.
bool VisualSlamNode::VisualSlamImpl::IsInitialized() const
{
  return cuvslam_odometry != nullptr;
}

bool VisualSlamNode::VisualSlamImpl::IsReadyForInitialization() const
{
  if (configuration_rejected) {
    return false;
  }
  // We need at least one message of every sensor to initialize.
  if (node.UsesImu() && !initial_imu_message) {
    return false;
  }

  if (initial_camera_info_messages.size() == static_cast<size_t>(node.num_cameras_)) {
    return true;
  }
  return false;
}

void VisualSlamNode::VisualSlamImpl::Initialize()
{
  if (IsInitialized()) {
    RCLCPP_WARN(node.get_logger(), "VisualSlamImpl was already initialized.");
    return;
  }

  RCLCPP_INFO(node.get_logger(), "Initializing cuVSLAM.");

  if (node.enable_ground_constraint_in_odometry_) {
    ground_constraint = InitGroundConstraint(node.get_logger());
    if (!ground_constraint) {
      return;
    }
  }

  // Set the base frame.
  std::string base_frame;
  if (!node.base_frame_.empty()) {
    base_frame = node.base_frame_;
  } else {
    base_frame = initial_camera_info_messages.at(0).value()->header.frame_id;
  }

  // Set the camera frames, either from parameters or from camera info message.
  std::vector<std::string> camera_optical_frames;
  if (!node.camera_optical_frames_.empty()) {
    camera_optical_frames = node.camera_optical_frames_;
  } else {
    camera_optical_frames.reserve(node.num_cameras_);
    for (const auto & [idx, camera_info_msg] : initial_camera_info_messages) {
      camera_optical_frames.push_back(camera_info_msg.value()->header.frame_id);
    }
  }

  // Every frame needed by the rig must be in the TF tree (bag /tf_static, live TF or profile
  // extrinsics). Otherwise wait: initialization is retried with the next camera info.
  {
    std::vector<std::string> needed = camera_optical_frames;
    if (rotation_compensation) {
      for (const auto & d : node.profile_->sync->devices) {
        needed.push_back(d.imu_frame);
      }
    }
    if (node.UsesImu()) {
      needed.push_back(
        !node.imu_frame_.empty() ? node.imu_frame_ :
        initial_imu_message.value()->header.frame_id);
    }
    std::string missing;
    for (const auto & frame : needed) {
      if (!tf_buffer->canTransform(base_frame, frame, tf2::TimePointZero)) {
        missing += (missing.empty() ? "" : ", ") + frame;
      }
    }
    if (!missing.empty()) {
      RCLCPP_WARN_THROTTLE(
        node.get_logger(), *node.get_clock(), 2000,
        "Waiting for TF from %s to: %s. The frames must come from the data (/tf_static) or "
        "from the extrinsics of the profile.", base_frame.c_str(), missing.c_str());
      return;
    }
  }

  // Virtual cameras of the profile: their intrinsics replace the camera_info ones.
  virtual_cameras.clear();
  if (node.profile_) {
    for (size_t i = 0; i < node.profile_->cameras.size() && i < node.num_cameras_; ++i) {
      const auto & pc = node.profile_->cameras[i];
      if (!pc.HasVirtualCamera()) {
        continue;
      }
      const auto & info = *initial_camera_info_messages.at(static_cast<int>(i)).value();
      try {
        VirtualCamera v = MakeVirtualCamera(
          info, pc.undistort, pc.pad_top, pc.pad_bottom, pc.pad_left, pc.pad_right);
        initial_camera_info_messages[static_cast<int>(i)] = v.info;
        RCLCPP_INFO(
          node.get_logger(),
          "Camera %s: virtual camera %ux%u (undistort %s, bands %d px top, %d px bottom, %d px "
          "left, %d px right)", pc.name.c_str(), v.width, v.height, pc.undistort ? "yes" : "no",
          v.pad_top, v.pad_bottom, v.pad_left, v.pad_right);
        virtual_cameras.emplace(static_cast<int>(i), std::move(v));
      } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR(node.get_logger(), "Camera %s: %s", pc.name.c_str(), e.what());
        configuration_rejected = true;
        return;
      }
    }
  }

  // Setup camera extrinsics (and intrinsics).
  cuvslam::Rig cam_rig;
  cam_rig.cameras.resize(node.num_cameras_);

  for (const auto & [idx, camera_info_msg] : initial_camera_info_messages) {
    const rclcpp::Time stamp(camera_info_msg.value()->header.stamp);
    FillIntrinsics(camera_info_msg.value(), cam_rig.cameras[idx]);
    const tf2::Transform base_link_pose_camera_optical = GetLatestTransform(
      base_frame,
      camera_optical_frames[idx]);
    if (node.rectified_images_) {
      const tf2::Matrix3x3 rectification_matrix(
        camera_info_msg.value()->r[0], camera_info_msg.value()->r[1], camera_info_msg.value()->r[2],
        camera_info_msg.value()->r[3], camera_info_msg.value()->r[4], camera_info_msg.value()->r[5],
        camera_info_msg.value()->r[6], camera_info_msg.value()->r[7], camera_info_msg.value()->r[8]
      );
      const tf2::Transform camera_optical_pose_camera_optical_rectified(
        rectification_matrix.inverse());
      const tf2::Transform base_link_pose_camera_rectified_optical = base_link_pose_camera_optical *
        camera_optical_pose_camera_optical_rectified;
      FillExtrinsics(base_link_pose_camera_rectified_optical, cam_rig.cameras[idx]);
    } else {
      FillExtrinsics(base_link_pose_camera_optical, cam_rig.cameras[idx]);
    }
  }


  uint32_t border_mask_top = node.border_mask_top_;
  uint32_t border_mask_bottom = node.border_mask_bottom_;
  uint32_t border_mask_left = node.border_mask_left_;
  uint32_t border_mask_right = node.border_mask_right_;

  for (size_t i = 0; i < cam_rig.cameras.size(); i++) {
    auto roi = initial_camera_info_messages.at(i).value()->roi;
    uint32_t img_width = initial_camera_info_messages.at(i).value()->width;
    uint32_t img_height = initial_camera_info_messages.at(i).value()->height;

    if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
      // The default setting of roi (all values 0) is considered the same as
      // full resolution (roi.width = width, roi.height = height).
      roi.width = img_width;
      roi.height = img_height;
    }

    auto & cam = cam_rig.cameras[i];

    if (!node.rectified_images_ && (border_mask_top != 0 || border_mask_bottom != 0 ||
      border_mask_left != 0 || border_mask_right != 0))
    {
      cam.border_top = std::clamp(border_mask_top, 0U, img_height);
      cam.border_bottom = std::clamp(border_mask_bottom, 0U, img_height);
      if (i % 2 == 0) {
        cam.border_left = std::clamp(border_mask_left, 0U, img_width);
        cam.border_right = std::clamp(border_mask_right, 0U, img_width);
      } else {
        cam.border_left = std::clamp(border_mask_right, 0U, img_width);
        cam.border_right = std::clamp(border_mask_left, 0U, img_width);
      }
    } else {
      cam.border_top = std::clamp(roi.y_offset, 0U, img_height);
      cam.border_bottom = std::clamp(img_height - roi.y_offset - roi.height, 0U, img_height);
      cam.border_left = std::clamp(roi.x_offset, 0U, img_width);
      cam.border_right = std::clamp(img_width - roi.x_offset - roi.width, 0U, img_width);
    }

    // Virtual cameras: no features in the band and near the undistortion borders.
    if (const auto v = virtual_cameras.find(static_cast<int>(i)); v != virtual_cameras.end()) {
      constexpr int32_t kVirtualMargin = 8;
      const auto h = static_cast<int32_t>(img_height);
      const auto w = static_cast<int32_t>(img_width);
      cam.border_top = std::min(std::max(cam.border_top, v->second.pad_top) + kVirtualMargin, h);
      cam.border_bottom = std::min(
        std::max(cam.border_bottom, v->second.pad_bottom) + kVirtualMargin, h);
      cam.border_left = std::min(
        std::max(cam.border_left, v->second.pad_left) + kVirtualMargin, w);
      cam.border_right = std::min(
        std::max(cam.border_right, v->second.pad_right) + kVirtualMargin, w);
    }

    // Compensated images have empty (black) borders: keep features away from them.
    if (rotation_compensation && camera_device[i] != 0) {
      const uint32_t margin = static_cast<uint32_t>(
        std::max(0, node.profile_->sync->compensation_border_px));
      cam.border_top = std::min(cam.border_top + margin, img_height);
      cam.border_bottom = std::min(cam.border_bottom + margin, img_height);
      cam.border_left = std::min(cam.border_left + margin, img_width);
      cam.border_right = std::min(cam.border_right + margin, img_width);
    }
  }

  // Rotation compensation: intrinsics and IMU -> camera rotation of every camera.
  if (rotation_compensation) {
    camera_K.assign(node.num_cameras_, Eigen::Matrix3d::Identity());
    camera_R_imu.assign(node.num_cameras_, Eigen::Matrix3d::Identity());
    for (uint i = 0; i < node.num_cameras_; ++i) {
      const auto & info = *initial_camera_info_messages.at(i).value();
      // Rectified images: projection matrix P; otherwise K.
      const bool use_p = node.rectified_images_ && info.p[0] != 0.0;
      camera_K[i] << (use_p ? info.p[0] : info.k[0]), 0.0, (use_p ? info.p[2] : info.k[2]),
        0.0, (use_p ? info.p[5] : info.k[4]), (use_p ? info.p[6] : info.k[5]),
        0.0, 0.0, 1.0;
      const std::string & imu_frame = node.profile_->sync->devices[camera_device[i]].imu_frame;
      const tf2::Matrix3x3 R = GetLatestTransform(camera_optical_frames[i], imu_frame).getBasis();
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          camera_R_imu[i](r, c) = R[r][c];
        }
      }
    }
    compensation_ready = true;
  }

  // Gyroscope of the base camera for the health check of the odometry.
  {
    std::string gyro_frame;
    const bool sync_gyro = device_sync && node.profile_ && node.profile_->sync &&
      !node.profile_->sync->devices.empty();
    if (node.UsesImu()) {
      gyro_frame = !node.imu_frame_.empty() ? node.imu_frame_ :
        initial_imu_message.value()->header.frame_id;
    } else if (sync_gyro) {
      gyro_frame = node.profile_->sync->devices[0].imu_frame;
    }
    health_gyro_ready = false;
    if (!gyro_frame.empty() && tf_buffer->canTransform(base_frame, gyro_frame,
          tf2::TimePointZero))
    {
      const tf2::Matrix3x3 R = GetLatestTransform(base_frame, gyro_frame).getBasis();
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          health_R_base_gyro(r, c) = R[r][c];
        }
      }
      health_gyro_ready = true;
    }
    char vio_check[48] = "not available";
    if (node.UsesImu() && compat::CompiledFeatures().imu_state) {
      if (node.health_max_vio_mismatch_mps_ > 0.0) {
        std::snprintf(
          vio_check, sizeof(vio_check), "max %.1f m/s", node.health_max_vio_mismatch_mps_);
      } else {
        std::snprintf(vio_check, sizeof(vio_check), "off");
      }
    }
    RCLCPP_INFO(
      node.get_logger(), "Odometry health check: max %.1f m/s, max %.1f deg against the gyroscope "
      "(%s), min %d landmarks, VIO speed against the odometry: %s", node.health_max_speed_mps_,
      node.health_max_gyro_error_deg_, health_gyro_ready ? gyro_frame.c_str() : "no gyroscope",
      node.health_min_landmarks_, vio_check);
  }

  // Create the IMU configuration. If the IMU is disabled we just use the identity transform.
  tf2::Transform cv_base_link_pose_cv_imu;
  cv_base_link_pose_cv_imu.setIdentity();
  if (node.UsesImu()) {
    const std::string imu_frame =
      !node.imu_frame_.empty() ? node.imu_frame_ : initial_imu_message.value()->header.frame_id;

    // Convert the base_pose_imu from ROS to cuVSLAM frame
    const rclcpp::Time stamp(initial_imu_message.value()->header.stamp);
    cv_base_link_pose_cv_imu =
      ChangeBasis(cuvslam_pose_canonical, GetLatestTransform(base_frame, imu_frame));

    cuvslam::ImuCalibration imu_calibration;
    imu_calibration.rig_from_imu = TocuVSLAMPose(cv_base_link_pose_cv_imu);
    imu_calibration.gyroscope_noise_density = node.imu_params_.gyroscope_noise_density;
    imu_calibration.gyroscope_random_walk = node.imu_params_.gyroscope_random_walk;
    imu_calibration.accelerometer_noise_density = node.imu_params_.accelerometer_noise_density;
    imu_calibration.accelerometer_random_walk = node.imu_params_.accelerometer_random_walk;
    imu_calibration.frequency = node.imu_params_.calibration_frequency;
    cam_rig.imus.push_back(imu_calibration);
    PrintImuCalibration(node.get_logger(), imu_calibration);
  }

  tracker_rig = cam_rig;
  if (!CreateTrackers(cam_rig)) {
    return;
  }
  RCLCPP_INFO(node.get_logger(), "cuVSLAM tracker was successfully initialized.");
  PublishEvent(
    SlamEventType::TRACKER_INITIALIZED,
    rclcpp::Time(initial_camera_info_messages.at(0).value()->header.stamp).nanoseconds(),
    std::to_string(node.num_cameras_) + " cameras");

  if (node.localize_on_startup_) {
    // We don't care when this thread returns since we anyways can't do anything if the localization
    // fails. Thus we detach it.
    std::thread(
      [this]() {
        const PoseType identity;
        const auto maybe_pose = LocalizeInMap(
          node.load_map_folder_path_, identity,
          node.map_frame_);
        if (!maybe_pose) {
          RCLCPP_WARN(
            node.get_logger(),
            "Could not localize on startup. Try with a different map or different pose hint.");
        }
      }).detach();
  }
}

bool VisualSlamNode::VisualSlamImpl::CreateTrackers(const cuvslam::Rig & rig)
{
  vio_initialized = false;
  // Create cuvslam odometry tracker.
  const cuvslam::Odometry::Config configuration = CreateOdometryConfiguration();
  PrintConfiguration(node.get_logger(), configuration);

  try {
    Stopwatch stopwatch_tracker;
    StopwatchScope ssw_tracker(stopwatch_tracker);
    cuvslam_odometry = std::make_unique<cuvslam::Odometry>(rig, configuration);
    RCLCPP_INFO(node.get_logger(), "Time taken by cuvslam::Odometry::Odometry(): %f",
            ssw_tracker.Stop());
  } catch (const std::invalid_argument & e) {
    // The configuration (rig, mode) is rejected: retrying at every frame would fail the same way.
    RCLCPP_ERROR(
      node.get_logger(), "cuVSLAM rejected the configuration, the node will not track: %s",
      e.what());
    configuration_rejected = true;
    return false;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "Failed to initialize cuvslam::Odometry: %s", e.what());
    return false;
  }

  if (node.enable_localization_n_mapping_) {
    try {
      Stopwatch stopwatch_slam;
      StopwatchScope ssw_slam(stopwatch_slam);
      cuvslam_slam = std::make_shared<cuvslam::Slam>(rig, cuvslam_odometry->GetPrimaryCameras(),
              CreateSlamConfiguration());
      RCLCPP_INFO(node.get_logger(), "Time taken by cuvslam::Slam::Slam(): %f",
              ssw_slam.Stop());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node.get_logger(), "Failed to initialize cuvslam::Slam: %s", e.what());
      return false;
    }
  }

  pose_cache.Reset();
  velocity_cache.Reset();

  // Initialize visualization helpers.
  if (node.enable_slam_visualization_) {
    // observations_vis_helper.Init(
    //   node.vis_observations_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
    //   node.get_logger());
    landmarks_vis_helper.Init(
      node.vis_landmarks_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
    lc_landmarks_vis_helper.Init(
      node.vis_loop_closure_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
    pose_graph_helper.Init(
      node.vis_posegraph_nodes_pub_, node.vis_posegraph_edges_pub_,
      node.vis_posegraph_edges2_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
    localizer_helper.Init(
      node.vis_localizer_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
#if PQ_CUVSLAM_HAS_LOCALIZER_LAYERS
    localizer_landmarks_vis_helper.Init(
      node.vis_localizer_landmarks_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
    localizer_observations_vis_helper.Init(
      node.vis_localizer_observations_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
    localizer_lc_landmarks_vis_helper.Init(
      node.vis_localizer_loop_closure_pub_, cuvslam_slam, canonical_pose_cuvslam, node.map_frame_,
      node.get_logger());
#endif
  }
  return true;
}

void VisualSlamNode::VisualSlamImpl::ExitVisualizationHelpers()
{
  // observations_vis_helper.Exit();
  landmarks_vis_helper.Exit();
  lc_landmarks_vis_helper.Exit();
  pose_graph_helper.Exit();
  localizer_helper.Exit();
#if PQ_CUVSLAM_HAS_LOCALIZER_LAYERS
  localizer_landmarks_vis_helper.Exit();
  localizer_observations_vis_helper.Exit();
  localizer_lc_landmarks_vis_helper.Exit();
#endif
}

bool VisualSlamNode::VisualSlamImpl::ResetTracker(int64_t stamp_ns, const std::string & reason)
{
  if (!tracker_rig) {
    return false;
  }
  // New odometry (and SLAM) with the same rig and configuration: no wait for the camera_info,
  // no new initialization. The SLAM map is lost; the guarded odometry continues from its last
  // pose.
  ExitVisualizationHelpers();
  cuvslam_odometry.reset();
  cuvslam_slam.reset();
  const auto t0 = std::chrono::steady_clock::now();
  const bool ok = CreateTrackers(*tracker_rig);
  const double ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  health.OnTrackerReset(stamp_ns);
  ++tracker_resets;
  char text[160];
  std::snprintf(
    text, sizeof(text), "tracker reset (%s), %.0f ms%s", reason.c_str(), ms,
    ok ? "" : ", failed");
  PublishEvent(SlamEventType::TRACKER_RESET, stamp_ns, text);
  return ok;
}

void VisualSlamNode::VisualSlamImpl::Exit()
{
  ExitVisualizationHelpers();
  tracker_rig.reset();

  if (cuvslam_odometry != nullptr) {
    cuvslam_odometry.reset();
    RCLCPP_INFO(node.get_logger(), "cuVSLAM odometry was destroyed");
  }

  if (cuvslam_slam != nullptr) {
    cuvslam_slam.reset();
    RCLCPP_INFO(node.get_logger(), "cuVSLAM slam was destroyed");
  }

  if (ground_constraint != nullptr) {
    ground_constraint.reset();
    RCLCPP_INFO(node.get_logger(), "cuVSLAM ground constraint was destroyed");
  }

  pose_cache.Reset();
  velocity_cache.Reset();

  initial_imu_message.reset();
  initial_camera_info_messages.clear();

  // Try to set the promise if localization has not returned yet. We don't know if the response is
  // outstanding so we just try it and catch the exception.
  LocalizeInExistDbContext::Response response{boost::outcome_v2::failure<std::string>(
            "Cannot localize in map because Exit() was called.")};
  try {
    localize_in_exist_db_context.response_promise.set_value(response);
  } catch (const std::exception & e) {
  }
}

cuvslam::Odometry::Config VisualSlamNode::VisualSlamImpl::CreateOdometryConfiguration()
{
  cuvslam::Odometry::Config configuration = cuvslam::Odometry::GetDefaultConfig();
  configuration.multicam_mode =
    (node.multicam_mode_ == 0) ? cuvslam::Odometry::MulticameraMode::Moderate :
    (node.multicam_mode_ == 1) ? cuvslam::Odometry::MulticameraMode::Performance :
    cuvslam::Odometry::MulticameraMode::Precision;
  configuration.use_motion_model = true;
  configuration.use_denoising = node.enable_image_denoising_;
  configuration.rectified_stereo_camera = node.rectified_images_;
  configuration.enable_observations_export = node.enable_observations_view_ ||
    node.enable_localization_n_mapping_;
  if (node.enable_debug_mode_) {
    configuration.debug_dump_directory = node.debug_dump_path_.c_str();
  }
  configuration.enable_landmarks_export = node.enable_landmarks_view_ ||
    node.enable_localization_n_mapping_;
  if (node.tracking_mode_ == static_cast<int>(TrackingMode::MULTISENSOR)) {
    // Multisensor mode: IMU fusion when the rig has an IMU, depth for the listed cameras.
    std::vector<int32_t> depth_ids;
    for (size_t k = 0; k < node.NumDepths(); ++k) {
      depth_ids.push_back(node.DepthCamera(k));
    }
    compat::SetMultisensorMode(
      configuration, depth_ids, node.depth_scale_factor_, node.depth_enable_stereo_tracking_);
  } else if (node.tracking_mode_ == static_cast<int>(TrackingMode::MONO)) {
    configuration.odometry_mode = cuvslam::Odometry::OdometryMode::Mono;
  } else if (node.tracking_mode_ == static_cast<int>(TrackingMode::RGBD)) {
    // RGBD mode
    configuration.odometry_mode = cuvslam::Odometry::OdometryMode::RGBD;
    // Configure RGBD settings
    configuration.rgbd_settings.depth_scale_factor = node.depth_scale_factor_;
    configuration.rgbd_settings.depth_camera_id = node.depth_camera_id_;
    configuration.rgbd_settings.enable_depth_stereo_tracking = node.depth_enable_stereo_tracking_;
  } else if (node.tracking_mode_ == static_cast<int>(TrackingMode::VIO)) {
    // VIO mode (IMU fusion)
    configuration.odometry_mode = cuvslam::Odometry::OdometryMode::Inertial;
  } else {
    // Default multicamera mode
    configuration.odometry_mode = cuvslam::Odometry::OdometryMode::Multicamera;
  }

  configuration.debug_imu_mode = node.enable_debug_imu_mode_;
  // Offline: bundle adjustment in the tracking call instead of a background thread.
  configuration.async_sba = node.processing_mode_ != ProcessingMode::OFFLINE;
  configuration.max_frame_delta_s = node.image_jitter_threshold_ms_ / 1000.0f;

  return configuration;
}

cuvslam::Slam::Config VisualSlamNode::VisualSlamImpl::CreateSlamConfiguration()
{
  cuvslam::Slam::Config configuration = cuvslam::Slam::GetDefaultConfig();
  configuration.enable_reading_internals = node.enable_slam_visualization_;
  configuration.planar_constraints = node.enable_ground_constraint_in_slam_;
  configuration.throttling_time_ms = node.slam_throttling_time_ms_;
  configuration.max_map_size = node.slam_max_map_size_;
  // Offline: SLAM in the tracking call instead of a separate work thread.
  configuration.sync_mode = node.processing_mode_ == ProcessingMode::OFFLINE;
  return configuration;
}

// Helper function to publish source frame pose wrt target frame to the tf tree
void VisualSlamNode::VisualSlamImpl::PublishFrameTransform(
  rclcpp::Time stamp, const tf2::Transform & pose,
  const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped target_pose_source;
  target_pose_source.header.stamp = stamp;
  target_pose_source.header.frame_id = target;
  target_pose_source.child_frame_id = source;
  target_pose_source.transform = tf2::toMsg(pose);
  tf_publisher->sendTransform(target_pose_source);
}

// Helper function to get child frame pose wrt parent frame from the tf tree
tf2::Transform VisualSlamNode::VisualSlamImpl::GetLatestTransform(
  const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  tf2::Transform pose;

  // Time out duration for TF tree lookup before throwing an exception.
  constexpr int32_t kTimeOutSeconds = 10;

  try {
    if (!tf_buffer->canTransform(
        target, source, tf2::TimePointZero, tf2::durationFromSec(kTimeOutSeconds)))
    {
      RCLCPP_ERROR(
        node.get_logger(), "Transform is impossible. canTransform(%s->%s) returns false",
        target.c_str(), source.c_str());
    }
    transform_stamped = tf_buffer->lookupTransform(
      target, source, tf2::TimePointZero, tf2::durationFromSec(kTimeOutSeconds));
    tf2::fromMsg(transform_stamped.transform, pose);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      node.get_logger(), "Could not transform %s to %s: %s",
      source.c_str(), target.c_str(), ex.what());
    throw std::runtime_error("Could not find the requested transform!");
  }
  return pose;
}

void VisualSlamNode::VisualSlamImpl::PublishOdometryVelocity(
  rclcpp::Time stamp, const std::string & frame_id,
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher)
{
  geometry_msgs::msg::Twist twist;

  pose_cache.GetVelocity(
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z);

  MarkerArrayType markers;
  markers.markers.resize(2);

  float width = 0.02f;
  {
    MarkerType & m = markers.markers[0];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "linear velocity";
    m.id = 0;
    m.action = MarkerType::ADD;
    m.type = MarkerType::ARROW;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = width * 1;
    m.scale.y = width * 2;
    m.scale.z = 0.0;
    m.color.a = 0.6;
    m.color.r = 0.8;
    m.color.g = 0.2;
    m.color.b = 0.3;
    geometry_msgs::msg::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = twist.linear.x;
    p2.y = twist.linear.y;
    p2.z = twist.linear.z;
    m.points.resize(2);
    m.points[0] = p1;
    m.points[1] = p2;
  }
  {
    MarkerType & m = markers.markers[1];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "angular velocity";
    m.id = 0;
    m.action = MarkerType::ADD;
    m.type = MarkerType::ARROW;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = width * 1;
    m.scale.y = width * 2;
    m.scale.z = 0.0;
    m.color.a = 1;
    m.color.r = 0.2;
    m.color.g = 0.8;
    m.color.b = 0.3;
    geometry_msgs::msg::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = twist.angular.x;
    p2.y = twist.angular.y;
    p2.z = twist.angular.z;
    m.points.resize(2);
    m.points[0] = p1;
    m.points[1] = p2;
  }
  publisher->publish(markers);
}

void VisualSlamNode::VisualSlamImpl::PublishGravity(
  rclcpp::Time stamp, const std::string & frame_id,
  const rclcpp::Publisher<MarkerType>::SharedPtr publisher)
{
  if (!node.UsesImu()) {
    return;
  }
  auto maybe_gravity = cuvslam_odometry->GetLastGravity();
  if (!maybe_gravity) {
    // Don't publish anything if cuvslam hasn't calculated gravity yet. After the start
    // or loss of visual tracking, cuvslam needs some time to align optical and imu sensors.
    // Only after that is it able to calculate the gravity vector.
    return;
  }

  cuvslam::Odometry::Gravity gravity_in_cuvslam = *maybe_gravity;
  const tf2::Vector3 g_cuvslam(gravity_in_cuvslam[0], gravity_in_cuvslam[1], gravity_in_cuvslam[2]);
  const tf2::Vector3 g_base_link = canonical_pose_cuvslam * g_cuvslam;

  MarkerType m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "gravity";
  m.id = 0;
  m.action = MarkerType::ADD;
  m.type = MarkerType::ARROW;
  m.pose.position.x = 0;    // arrow start
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.pose.orientation.x = 0;
  m.pose.orientation.y = 0;
  m.pose.orientation.z = 0;
  m.pose.orientation.w = 1;
  m.scale.x = 0.1;    // length
  m.scale.y = 0.2;
  m.scale.z = 0.0;
  m.color.a = 0.6;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  geometry_msgs::msg::Point p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  geometry_msgs::msg::Point p2;
  p2.x = g_base_link.x();
  p2.y = g_base_link.y();
  p2.z = g_base_link.z();
  m.points.resize(2);
  m.points[0] = p1;
  m.points[1] = p2;

  publisher->publish(std::move(m));
}


void VisualSlamNode::VisualSlamImpl::CallbackImu(const ImuType::ConstSharedPtr & msg)
{
  NvtxRangeScoped trace(
    "VisualSlamNode::VisualSlamImpl::CallbackImu", CLR_RED);

  if (IsInitialized()) {
    const rclcpp::Time timestamp(msg->header.stamp);
    sequencer.CallbackStream1(timestamp.nanoseconds(), msg);
  } else {initial_imu_message = msg;}
}

void VisualSlamNode::VisualSlamImpl::CallbackImage(int index, const ImageType & image)
{
  NvtxRangeScoped trace(
    "VisualSlamNode::VisualSlamImpl::CallbackImage", CLR_YELLOW);

  if (IsInitialized()) {
    const rclcpp::Time timestamp(image->header.stamp);
    if (device_sync) {
      device_sync->Add(index, timestamp.nanoseconds(), image);
    } else {
      sync.AddMessage(index, timestamp.nanoseconds(), image);
    }
  }
}

void VisualSlamNode::VisualSlamImpl::CallbackCameraInfo(
  int index, const CameraInfoType::ConstSharedPtr & msg)
{
  NvtxRangeScoped trace(
    "VisualSlamNode::VisualSlamImpl::CallbackCameraInfo", CLR_YELLOW);

  if (!IsInitialized()) {
    initial_camera_info_messages[index] = msg;
    if (IsReadyForInitialization()) {Initialize();}
  }
}

void VisualSlamNode::VisualSlamImpl::CallbackSyncImu(
  int device, const ImuType::ConstSharedPtr & msg)
{
  if (device < 0 || device >= static_cast<int>(device_gyro.size())) {
    return;
  }
  const rclcpp::Time t(msg->header.stamp);
  const auto & w = msg->angular_velocity;
  device_gyro[device].Add(
    t.nanoseconds() - device_clock_offset_ns[device], Eigen::Vector3d(w.x, w.y, w.z));
  if (device == 0 && !node.UsesImu()) {
    health_gyro.Add(t.nanoseconds(), Eigen::Vector3d(w.x, w.y, w.z));
  }
}

void VisualSlamNode::VisualSlamImpl::OnRigFrame(const DeviceSync::RigFrame & frame)
{
  const auto prep_start = std::chrono::steady_clock::now();
  // The rig frame takes the (uncorrected) stamp of the reference device.
  const int64_t stamp = frame.reference_stamp_ns + device_clock_offset_ns[0];
  double max_offset_ms = 0.0;
  for (size_t d = 1; d < frame.device_stamps_ns.size(); ++d) {
    max_offset_ms = std::max(
      max_offset_ms, std::abs(frame.device_stamps_ns[d] - frame.reference_stamp_ns) / 1e6);
  }
  sync_stats.offsets_ms.push_back(max_offset_ms);
  frame_sync_offset_ms[stamp] = max_offset_ms;

  std::vector<std::pair<int, ImageType>> images;
  images.reserve(frame.images.size());
  for (const auto & [camera, raw] : frame.images) {
    const ImageType msg = Prepare(camera, raw);
    const int d = camera_device[camera];
    if (d == 0) {
      images.emplace_back(camera, msg);
      continue;
    }
    // Other devices: warp to the reference time (rotation compensation), or only restamp.
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
    if (rotation_compensation && compensation_ready) {
      const auto dR_imu = device_gyro[d].Integrate(
        frame.device_stamps_ns[d], frame.reference_stamp_ns);
      if (dR_imu) {
        // A depth image is warped like the image of its camera.
        const int rig_camera = node.CameraOfStream(camera);
        const Eigen::Matrix3d dR = CameraRotation(camera_R_imu[rig_camera], *dR_imu);
        const double angle = RotationAngleDeg(dR);
        sync_stats.rotations_deg.push_back(angle);
        if (angle >= node.profile_->sync->compensation_min_deg) {
          H = RotationHomography(camera_K[rig_camera], dR);
          ++sync_stats.compensated_images;
        } else {
          ++sync_stats.small_rotations;
        }
      } else {
        ++sync_stats.gyro_missing;
      }
    }
    ImageType out;
    if (!H.isIdentity()) {
      out = WarpImage(*msg, H, stamp);
    }
    if (!out) {
      auto copy = std::make_shared<sensor_msgs::msg::Image>(*msg);
      copy->header.stamp = rclcpp::Time(stamp);
      out = copy;
    }
    images.emplace_back(camera, out);
  }
  const double prep_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - prep_start).count();
  sync_stats.prep_ms.push_back(prep_ms);
  frame_prep_ms[stamp] = prep_ms;
  CallbackSynchronizedImages(stamp, images);
}

void VisualSlamNode::VisualSlamImpl::CallbackSynchronizedImages(
  int64_t latest_ts, const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs)
{
  // Without device sync the images come straight from the synchronizer: prepare them here
  // (with device sync, OnRigFrame already did it).
  if (!device_sync) {
    std::vector<std::pair<int, ImageType>> images;
    images.reserve(idx_and_image_msgs.size());
    for (const auto & [idx, msg] : idx_and_image_msgs) {
      images.emplace_back(idx, Prepare(idx, msg));
    }
    sequencer.CallbackStream2(latest_ts, images);
    return;
  }
  sequencer.CallbackStream2(latest_ts, idx_and_image_msgs);
}

ImageType VisualSlamNode::VisualSlamImpl::Prepare(int index, const ImageType & image)
{
  // cuVSLAM tracks on intensity: an RGB image converted first is three times cheaper to
  // undistort, warp and upload to the GPU.
  namespace enc = sensor_msgs::image_encodings;
  if (node.rgb_to_mono_ && index < static_cast<int>(node.num_cameras_) &&
    (image->encoding == enc::RGB8 || image->encoding == enc::BGR8) &&
    image->data.size() >= static_cast<size_t>(image->step) * image->height)
  {
    const cv::Mat src(
      static_cast<int>(image->height), static_cast<int>(image->width), CV_8UC3,
      const_cast<uint8_t *>(image->data.data()), image->step);
    auto out = std::make_shared<sensor_msgs::msg::Image>();
    out->header = image->header;
    out->height = image->height;
    out->width = image->width;
    out->encoding = enc::MONO8;
    out->is_bigendian = image->is_bigendian;
    out->step = image->width;
    out->data.resize(static_cast<size_t>(out->step) * out->height);
    cv::Mat dst(static_cast<int>(out->height), static_cast<int>(out->width), CV_8UC1,
      out->data.data(), out->step);
    cv::cvtColor(src, dst, image->encoding == enc::RGB8 ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
    return ToVirtual(index, out);
  }
  return ToVirtual(index, image);
}

ImageType VisualSlamNode::VisualSlamImpl::ToVirtual(int camera, const ImageType & image)
{
  // A depth image goes through the virtual camera of the camera it is aligned with.
  const auto v = virtual_cameras.find(node.CameraOfStream(camera));
  if (v == virtual_cameras.end()) {
    return image;
  }
  if (auto out = ApplyVirtualCamera(v->second, *image, node.DepthOfStream(camera) >= 0)) {
    return out;
  }
  RCLCPP_ERROR_THROTTLE(
    node.get_logger(), *node.get_clock(), 5000,
    "Input %d: image %ux%u %s does not match the virtual camera (calibration %ux%u)", camera,
    image->width, image->height, image->encoding.c_str(),
    v->second.width - v->second.pad_left - v->second.pad_right,
    v->second.height - v->second.pad_top - v->second.pad_bottom);
  return image;
}

void VisualSlamNode::VisualSlamImpl::UpdatePose(
  const std::vector<ImuType::ConstSharedPtr> & imu_msgs,
  const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs)
{
  NvtxRangeScoped trace(
    "VisualSlamNode::VisualSlamImpl::UpdatePose", CLR_MAGENTA);
  std::lock_guard<std::mutex> frame_lock(frame_mutex_);
  if (run_output_written) {
    return;   // shutting down: the run output is already written
  }
  Stopwatch stopwatch;
  StopwatchScope ssw(stopwatch);
  const auto callback_start = std::chrono::steady_clock::now();

  // Check for completed localization
  CheckLocalizationStatus();

  // Get the latest timestamp from images. We assume that the vector is never empty.
  const auto max_element = std::max_element(
    idx_and_image_msgs.begin(), idx_and_image_msgs.end(),
    [](const std::pair<int, ImageType> & msg1, const std::pair<int, ImageType> & msg2) {
      const rclcpp::Time ts1(msg1.second->header.stamp);
      const rclcpp::Time ts2(msg2.second->header.stamp);
      return ts1.nanoseconds() < ts2.nanoseconds();
    });

  const int64_t latest_ts = rclcpp::Time(max_element->second->header.stamp).nanoseconds();

  RCLCPP_DEBUG(node.get_logger(), "Using image msg timestamp [%ld]", latest_ts);

  const double time_delta_ms = (latest_ts - last_track_ts) / 1e6;
  // Skipping the calculation for the first frame
  if (time_delta_ms > node.image_jitter_threshold_ms_ && last_track_ts != -1) {
    RCLCPP_WARN(
      node.get_logger(),
      "Delta between current and previous frame [%f ms] is above threshold [%f ms]",
      time_delta_ms, node.image_jitter_threshold_ms_);
  }

  Stopwatch stopwatch_track;
  // First we add all imu measurements received since the last update.
  for (const auto & imu_msg : imu_msgs) {
    const rclcpp::Time timestamp(imu_msg->header.stamp);
    int64_t imu_ts = static_cast<int64_t>(timestamp.nanoseconds());
    RCLCPP_DEBUG(node.get_logger(), "Using imu msg timestamp [%ld]", imu_ts);
    const auto imu_measurement = TocuVSLAMImuMeasurement(imu_msg, imu_ts);
    const auto & w = imu_msg->angular_velocity;
    health_gyro.Add(imu_ts, Eigen::Vector3d(w.x, w.y, w.z));
    try {
      cuvslam_odometry->RegisterImuMeasurement(0, imu_measurement);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node.get_logger(), "Failed to register an IMU measurement: %s",
              e.what());
    }
  }

  // Convert images to cuvslam's format.
  std::vector<cuvslam::Image> cuvslam_images;
  cuvslam_images.reserve(node.num_cameras_);
  std::vector<cuvslam::Image> cuvslam_masks;
  std::vector<cuvslam::Image> cuvslam_depth_images;
#if defined(VSLAM_WITH_CUDA_BUFFER)
  // Zero-copy path: images are accessed in GPU memory through cuda_buffer.
  // ReadHandles must outlive the cuvslam::Odometry::Track call so that the
  // GPU memory referenced by cuvslam_images stays valid and the CUDA event
  // synchronization between producer and consumer is preserved.
  std::vector<cuda_buffer_backend::ReadHandle> read_handles;
  read_handles.reserve(idx_and_image_msgs.size());
  const auto acquire_pixels = [&](const sensor_msgs::msg::Image & msg) {
      auto handle = cuda_buffer_backend::from_input_buffer(msg.data, cuda_stream_);
      const ImagePixels pixels{handle.get_ptr(), true};
      read_handles.push_back(std::move(handle));
      return pixels;
    };
#else
  // Standard path: images are passed to cuVSLAM in host memory. The messages are
  // kept alive by idx_and_image_msgs until the end of this function.
  const auto acquire_pixels = [](const sensor_msgs::msg::Image & msg) {
      return ImagePixels{msg.data.data(), false};
    };
#endif

  // Index of the first depth image (after the cameras and the masks)
  const int depth_image_idx = node.DepthStreamIndex(0);

  // First, extract the masks from the synchronized messages
  std::unordered_map<int, const ImageType *> mask_msgs;
  for (const auto & [idx, image_msg] : idx_and_image_msgs) {
    if (idx >= static_cast<int>(node.num_cameras_) && idx < depth_image_idx) {
      mask_msgs[idx - node.num_cameras_] = &image_msg;
    }
  }

  for (const auto & [idx, image_msg] : idx_and_image_msgs) {
    if (idx < static_cast<int>(node.num_cameras_)) {
      // This is a regular RGB/MONO image modes
      cuvslam_images.push_back(
        TocuVSLAMImage(idx, *image_msg, acquire_pixels(*image_msg), latest_ts));
      auto mask_it = mask_msgs.find(idx);
      if (mask_it != mask_msgs.end() && mask_it->second != nullptr) {
        const auto & mask_msg = *mask_it->second;
        cuvslam_masks.push_back(
          TocuVSLAMImage(idx, *mask_msg, acquire_pixels(*mask_msg), latest_ts));
      }
    } else if (const int k = node.DepthOfStream(idx); k >= 0) {
      // Depth image (RGBD mode, or one of the depth cameras in Multisensor mode)
      cuvslam_depth_images.push_back(
        TocuVSLAMDepthImage(
          node.DepthCamera(static_cast<size_t>(k)), *image_msg, acquire_pixels(*image_msg),
          latest_ts));
    }
  }

#if defined(VSLAM_WITH_CUDA_BUFFER)
  // Ensure all producer writes have completed before cuvslam consumes the data.
  // cuvslam may use its own internal stream and synchronous CUDA APIs, so we cannot
  // rely on the from_input_buffer stream-wait alone.
  if (!read_handles.empty()) {
    const cudaError_t sync_err = cudaStreamSynchronize(cuda_stream_);
    if (sync_err != cudaSuccess) {
      RCLCPP_ERROR(
        node.get_logger(), "cudaStreamSynchronize failed: %s", cudaGetErrorString(sync_err));
      return;
    }
  }
#endif

  cuvslam::PoseEstimate vo_pose_estimate;
  {
    NvtxRangeScoped trace(
      "cuvslam::Odometry::Track", CLR_MAGENTA);
    StopwatchScope ssw_track(stopwatch_track);
    try {
      vo_pose_estimate = cuvslam_odometry->Track(cuvslam_images, cuvslam_masks,
              cuvslam_depth_images);
    } catch (const std::exception & e) {
      RCLCPP_WARN(node.get_logger(), "Failed to track: %s", e.what());
      return;
    }
  }
  bool vo_success = vo_pose_estimate.world_from_rig != std::nullopt;
  // Velocity and IMU biases of the visual-inertial odometry (cuVSLAM >= 17).
  std::optional<compat::ImuState> imu_state;
  if (vo_success && node.UsesImu()) {
    imu_state = compat::GetImuState(*cuvslam_odometry);
  }

  if (!vo_success) {
    pose_cache.Reset();
    RCLCPP_WARN(node.get_logger(), "Visual tracking is lost");
  }

  const rclcpp::Time timestamp_output = node.override_publishing_stamp_ ?
    node.get_clock()->now() : rclcpp::Time(latest_ts);

  // Poses of the base frame for the run output.
  std::optional<tf2::Transform> output_odom_pose;
  std::optional<tf2::Transform> output_slam_pose;

  if (vo_success) {
    if (ground_constraint &&
      !PoseToGround(*ground_constraint, vo_pose_estimate.world_from_rig.value().pose,
            node.get_logger()))
    {
      RCLCPP_WARN(node.get_logger(), "Unknown ground constraint Error");
      return;
    }

    // Get VO pose and transform to ROS conventions.
    // odom and map are using same conventions.
    const tf2::Transform cv_odom_pose_cv_base_link =
      FromcuVSLAMPose(vo_pose_estimate.world_from_rig.value().pose);
    const tf2::Transform odom_pose_base_link = ChangeBasis(
      canonical_pose_cuvslam, cv_odom_pose_cv_base_link);

    // Get SLAM pose and transform to ROS conventions. The SLAM pose uses loop closures for robust
    // tracking.
    // Note: It can result in sudden jumps of the final pose.
    // Publish Smooth pose if enable_rectified_pose_ = false
    tf2::Transform cv_map_pose_cv_base_link = cv_odom_pose_cv_base_link;
    if (node.enable_localization_n_mapping_) {
      try {
        cuvslam_odometry->GetState(odometry_state);
        cuvslam::Pose slam_pose = compat::SlamTrack(*cuvslam_slam, odometry_state);
        cv_map_pose_cv_base_link = FromcuVSLAMPose(slam_pose);
      } catch (const std::exception & e) {
        RCLCPP_WARN(node.get_logger(), "Failed to get SLAM pose: %s", e.what());
        return;
      }
    }

    const tf2::Transform map_pose_base_link =
      ChangeBasis(canonical_pose_cuvslam, cv_map_pose_cv_base_link);
    const tf2::Transform map_pose_odom = map_pose_base_link * odom_pose_base_link.inverse();

    output_odom_pose = odom_pose_base_link;
    if (node.enable_localization_n_mapping_) {
      output_slam_pose = map_pose_base_link;
    }

    // Publish transforms to the TF tree.
    if (node.publish_map_to_odom_tf_) {
      if (!node.invert_map_to_odom_tf_) {
        PublishFrameTransform(
          timestamp_output, map_pose_odom, node.map_frame_,
          node.odom_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output,
          map_pose_odom.inverse(), node.odom_frame_, node.map_frame_);
      }
    }

    if (node.publish_odom_to_base_tf_) {
      if (!node.invert_odom_to_base_tf_) {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link, node.odom_frame_, node.base_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link.inverse(), node.base_frame_, node.odom_frame_);
      }
    }

    // Calculate velocity using last position.
    pose_cache.Add(latest_ts, odom_pose_base_link);
    geometry_msgs::msg::Twist velocity;
    pose_cache.GetVelocity(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);
    velocity_cache.Add(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);

    // Prepare message parts needed for VO messages.
    PoseType vo_pose;
    tf2::toMsg(odom_pose_base_link, vo_pose);
    std_msgs::msg::Header header_odom;
    header_odom.stamp = timestamp_output;
    header_odom.frame_id = node.odom_frame_;

    // Prepare message parts needed for SLAM messages.
    PoseType slam_pose;
    tf2::toMsg(map_pose_base_link, slam_pose);
    std_msgs::msg::Header header_map;
    header_map.stamp = timestamp_output;
    header_map.frame_id = node.map_frame_;

    if (HasSubscribers(node.tracking_vo_pose_pub_)) {
      // Tracking_vo_pose_pub_
      auto pose_only = std::make_unique<PoseStampedType>();
      pose_only->header = header_odom;
      pose_only->pose = vo_pose;
      node.tracking_vo_pose_pub_->publish(std::move(pose_only));
    }
    if (HasSubscribers(node.tracking_vo_pose_covariance_pub_)) {
      // Tracking_vo_covariance_pub_
      auto pose_n_cov = std::make_unique<PoseWithCovarianceStampedType>();
      pose_n_cov->header = header_odom;
      pose_n_cov->pose.pose = vo_pose;
      auto covariance_transform =
        FromcuVSLAMCovariance(
        compat::PoseCovarianceRotationFirst(vo_pose_estimate.world_from_rig.value()));
      for (size_t i = 0; i < 6 * 6; i++) {
        pose_n_cov->pose.covariance[i] = covariance_transform(i);
      }
      node.tracking_vo_pose_covariance_pub_->publish(std::move(pose_n_cov));
    }
    if (HasSubscribers(node.tracking_odometry_pub_)) {
      OdometryType odom;
      odom.header = header_odom;
      odom.child_frame_id = node.base_frame_;

      odom.pose.pose = vo_pose;
      bool res = pose_cache.GetCovariance(odom.pose.covariance);
      if (!res) {
        // set default Identity matrix
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            odom.pose.covariance[i * 6 + j] = i == j ? 1 : 0;
          }
        }
      }

      odom.twist.twist.linear = velocity.linear;
      odom.twist.twist.angular = velocity.angular;

      res = velocity_cache.GetCovariance(odom.twist.covariance);
      if (!res) {
        // set default Identity matrix
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            odom.twist.covariance[i * 6 + j] = i == j ? 1 : 0;
          }
        }
      }
      node.tracking_odometry_pub_->publish(odom);
    }
    if (HasSubscribers(node.vis_vo_velocity_pub_)) {
      PublishOdometryVelocity(
        timestamp_output,
        node.base_frame_,
        node.vis_vo_velocity_pub_);
    }
    if (HasSubscribers(node.vis_slam_odometry_pub_)) {
      OdometryType odom;
      odom.header = header_map;
      odom.child_frame_id = node.base_frame_;

      // only populating slam_pose for viz
      odom.pose.pose = slam_pose;
      node.vis_slam_odometry_pub_->publish(odom);
    }
    if (HasSubscribers(node.tracking_vo_path_pub_)) {
      // Tracking_vo_path_pub_
      PoseStampedType pose_stamped;
      pose_stamped.header = header_odom;
      pose_stamped.pose = vo_pose;
      vo_path.add(pose_stamped);

      auto path_msg = std::make_unique<PathType>();
      path_msg->header = header_odom;
      path_msg->poses = vo_path.getData();
      node.tracking_vo_path_pub_->publish(std::move(path_msg));
    }
    if (HasSubscribers(node.tracking_slam_path_pub_)) {
      // Tracking_slam_path_pub_
      PoseStampedType pose_stamped;
      pose_stamped.header = header_map;
      pose_stamped.pose = slam_pose;
      slam_path.add(pose_stamped);

      auto path_msg = std::make_unique<PathType>();
      path_msg->header = header_map;
      path_msg->poses = slam_path.getData();
      node.tracking_slam_path_pub_->publish(std::move(path_msg));
    }

    // Draw gravity vector
    if (HasSubscribers(node.vis_gravity_pub_) &&
      node.UsesImu())
    {
      PublishGravity(
        timestamp_output,
        node.base_frame_,
        node.vis_gravity_pub_);
    }
  }

  // Calculate the tracking execution time statistics.
  ssw.Stop();
  const double track_execution_time = stopwatch_track.Seconds();
  track_execution_times.add(track_execution_time);
  double track_execution_time_max = 0;
  double track_execution_time_mean = 0;
  const auto & values = track_execution_times.getData();
  for (double t : values) {
    track_execution_time_max = std::max(track_execution_time_max, t);
    track_execution_time_mean += t;
  }
  if (values.size()) {
    track_execution_time_mean /= values.size();
  }
  // Health check of the odometry and guarded odometry.
  HealthInput health_in;
  health_in.stamp_ns = latest_ts;
  health_in.pose = output_odom_pose;
  if (vo_success && (node.enable_landmarks_view_ || node.enable_localization_n_mapping_)) {
    try {
      health_in.landmarks = cuvslam_odometry->GetLastLandmarks().size();
    } catch (const std::exception &) {
    }
  }
  if (health_gyro_ready && health_last_stamp_ns > 0) {
    if (const auto dR = health_gyro.Integrate(health_last_stamp_ns, latest_ts)) {
      const Eigen::Quaterniond q(health_R_base_gyro * *dR * health_R_base_gyro.transpose());
      health_in.gyro_rotation = tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
    }
  }
  if (imu_state && imu_state->initialized) {
    const auto & v = imu_state->velocity;
    health_in.vio_speed_mps = std::sqrt(
      static_cast<double>(v[0]) * v[0] + static_cast<double>(v[1]) * v[1] +
      static_cast<double>(v[2]) * v[2]);
  }
  health_last_stamp_ns = latest_ts;
  const HealthOutput health_out = health.Update(health_in);
  if (health_out.guarded_pose && HasSubscribers(node.tracking_odometry_guarded_pub_)) {
    OdometryType odom;
    odom.header.stamp = timestamp_output;
    odom.header.frame_id = node.odom_frame_;
    odom.child_frame_id = node.base_frame_;
    tf2::toMsg(*health_out.guarded_pose, odom.pose.pose);
    node.tracking_odometry_guarded_pub_->publish(odom);
  }

  ReportFrame(latest_ts, vo_success, output_odom_pose, output_slam_pose,
    track_execution_time * 1000.0,
    std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count(),
    idx_and_image_msgs, health_out, imu_state);

  if (health_out.reset_tracker) {
    ResetTracker(
      latest_ts, std::to_string(health_out.bad_in_window) + " rejected or lost frames in " +
      std::to_string(health.Config().reset_window_s).substr(0, 3) + " s");
    return;
  }

  // Publish status.
  std_msgs::msg::Header header;
  header.stamp = timestamp_output;
  header.frame_id = node.map_frame_;
  if (HasSubscribers(node.visual_slam_status_pub_)) {
    VisualSlamStatusType visual_slam_status_msg;
    visual_slam_status_msg.header = header;
    visual_slam_status_msg.vo_state = vo_success ? 1 : 2;
    visual_slam_status_msg.node_callback_execution_time = stopwatch.Seconds();
    visual_slam_status_msg.track_execution_time = track_execution_time;
    visual_slam_status_msg.track_execution_time_max = track_execution_time_max;
    visual_slam_status_msg.track_execution_time_mean = track_execution_time_mean;
    node.visual_slam_status_pub_->publish(visual_slam_status_msg);
  }
  // Publish diagnostics.
  if (HasSubscribers(node.diagnostics_pub_)) {
    DiagnosticArrayType diagnostics;
    diagnostics.header = header;
    DiagnosticStatusType & status = diagnostics.status.emplace_back();
    if (vo_success) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.name = "Visual Slam Diagnostics";
    status.message = "Tracking state and execution time measurements";
    status.hardware_id = "visual_slam";
    status.values.emplace_back();
    status.values.back().key = "vo_status";
    status.values.back().value = vo_success ? "OK" : "Lost";
    status.values.emplace_back();
    status.values.back().key = "track_execution_time";
    status.values.back().value = std::to_string(track_execution_time);
    status.values.emplace_back();
    status.values.back().key = "track_execution_time_max";
    status.values.back().value = std::to_string(track_execution_time_max);
    status.values.emplace_back();
    status.values.back().key = "track_execution_time_mean";
    status.values.back().value = std::to_string(track_execution_time_mean);
    status.values.emplace_back();
    status.values.back().key = "localized_in_exist_map";
    status.values.back().value = localized_in_exist_map_ ? "Yes" : "No";
    node.diagnostics_pub_->publish(diagnostics);
  }

  last_track_ts = latest_ts;

  // Latency emulation: the frame takes at least as long as on the target platform.
  if (latency_emulator) {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    const double wait_ms = latency_emulator->TargetMs(latest_ts) - elapsed_ms;
    if (wait_ms > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(wait_ms));
      ++emulated_frames_delayed;
      emulated_delay_total_ms += wait_ms;
    }
  }
}


bool VisualSlamNode::VisualSlamImpl::SaveMap(const std::string & map_folder_path)
{
  RCLCPP_INFO(node.get_logger(), "Saving map to %s", map_folder_path.c_str());

  if (!node.enable_localization_n_mapping_) {
    RCLCPP_WARN(
      node.get_logger(),
      "Cannot save map because `enable_localization_n_mapping` is set to false.");
    return false;
  }

  // Create context.
  boost::promise<bool> response_promise;
  auto response_future = response_promise.get_future();
  if (!response_future.valid()) {
    RCLCPP_ERROR(node.get_logger(), "Future is invalid.");
    return false;
  }

  // Trigger the saving asynchronously. Error handling is done in the callback.
  cuvslam_slam->SaveMap(map_folder_path, [&response_promise](bool success) {
      response_promise.set_value(success);
  });

  // Wait for the save operation to complete, so that response_promise is not destroyed
  // before the callback is called.
  const bool success = response_future.get();
  if (!success) {
    RCLCPP_ERROR(node.get_logger(), "Failed to save map");
    return false;
  }

  RCLCPP_INFO(node.get_logger(), "Finished saving map");
  return true;
}


boost::future<VisualSlamNode::VisualSlamImpl::LocalizeInExistDbContext::Response> VisualSlamNode::
VisualSlamImpl::CuvslamInternalLocalizeInMapAsync(
  const std::string & map_folder_path,
  const cuvslam::Pose & pose_hint)
{
  localize_in_exist_db_context = LocalizeInExistDbContext();
  auto response_future = localize_in_exist_db_context.response_promise.get_future();

  if (!node.enable_localization_n_mapping_) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Cannot localize in map because `enable_localization_n_mapping` is set to false.");
    localize_in_exist_db_context.response_promise.set_value(boost::outcome_v2::failure<std::string>(
            "Cannot localize in map because `enable_localization_n_mapping` is set to false."));
    return response_future;
  }

  if (!std::filesystem::is_directory(map_folder_path)) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Map folder '%s' does not exist.", map_folder_path.c_str());
    localize_in_exist_db_context.response_promise.set_value(boost::outcome_v2::failure<std::string>(
            "Map folder '" + map_folder_path + "' does not exist."));
    return response_future;
  }

  // Check if the map folder contains the expected database files
  bool has_lmdb_files = false;
  for (const auto & entry : std::filesystem::directory_iterator(map_folder_path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".mdb") {
      has_lmdb_files = true;
      break;
    }
  }

  if (!has_lmdb_files) {
    RCLCPP_ERROR(
      node.get_logger(),
      "Map folder '%s' does not contain .mdb database files.", map_folder_path.c_str());
    localize_in_exist_db_context.response_promise.set_value(boost::outcome_v2::failure<std::string>(
            "Map folder '" + map_folder_path + "' does not contain .mdb database files."));
    return response_future;
  }

  RCLCPP_INFO(
    node.get_logger(),
    "Map folder '%s' exists and contains database files. Starting async localization...",
    map_folder_path.c_str());

  // For async localization, we don't need to provide images immediately
  // The localization will use images from normal tracking operations
  std::vector<cuvslam::Image> empty_images;

  compat::LocalizationParams localization_params;
  localization_params.horizontal_search_radius = node.localizer_horizontal_radius_;
  localization_params.vertical_search_radius = node.localizer_vertical_radius_;
  localization_params.horizontal_step = node.localizer_horizontal_step_;
  localization_params.vertical_step = node.localizer_vertical_step_;
  localization_params.angular_step_rads = node.localizer_angular_step_;
  localization_params.enable_reading_internals = node.enable_slam_visualization_;

  // NOTE: Even if LocalizeInMap fails, we still expect it to call the callback.
  // We rely on the callback to set the value of the response_promise.
  compat::LocalizeInMap(
    *cuvslam_slam, map_folder_path, last_track_ts, pose_hint, empty_images, localization_params,
    [this](const cuvslam::Result<cuvslam::Pose> & result) {
      LocalizeInExistDbContext::Response response{boost::outcome_v2::failure(std::string(
              result.error_message))};
      if (result.data.has_value()) {
        response = boost::outcome_v2::success<cuvslam::Pose>(result.data.value());
      }
      localize_in_exist_db_context.response_promise.set_value(response);
  });

  // The async localization callback will be triggered during normal tracking operations
  // when CUVSLAM_Track is called with real images
  RCLCPP_INFO(node.get_logger(),
          "Async localization started, waiting for callback during normal tracking");

  return response_future;
}


boost::future<std::optional<PoseType>> VisualSlamNode::VisualSlamImpl::LocalizeInMapAsync(
  const std::string & map_folder_path,
  const PoseType & pose_hint,
  const std::string & frame_id)
{
  RCLCPP_INFO(
    node.get_logger(), "Trying to localize in map '%s' around [%f, %f, %f]",
    map_folder_path.c_str(), pose_hint.position.x, pose_hint.position.y,
    pose_hint.position.z);
  // Convert the pose hint from ROS coordinates to cuvslam coordinates.
  tf2::Transform frame_id_pose_base;
  tf2::fromMsg(pose_hint, frame_id_pose_base);
  const tf2::Transform map_pose_frame_id = GetLatestTransform(node.map_frame_, frame_id);
  const tf2::Transform map_pose_base = map_pose_frame_id * frame_id_pose_base;
  const tf2::Transform cv_map_pose_cv_base = ChangeBasis(cuvslam_pose_canonical, map_pose_base);
  const cuvslam::Pose pose_hint_cv = TocuVSLAMPose(cv_map_pose_cv_base);

  auto extract_pose_in_ros_conventions =
    [&](boost::future<LocalizeInExistDbContext::Response> response_future)
    -> std::optional<PoseType>
    {
      const auto response = response_future.get();
      if (!response.has_value()) {
        RCLCPP_ERROR(node.get_logger(), "Failed to localize in map. Error %s",
              response.error().c_str());
        localizer_helper.SetResult(false, tf2::Transform());
        localized_in_exist_map_ = false;
        return std::nullopt;
      }

      // Convert the pose back from cuvslam coordinates to ROS coordinates.
      const tf2::Transform cv_map_pose_cv_base_link = FromcuVSLAMPose(response.value());
      const tf2::Transform map_pose_base_link = ChangeBasis(
        canonical_pose_cuvslam,
        cv_map_pose_cv_base_link);
      localizer_helper.SetResult(true, map_pose_base_link);

      localized_in_exist_map_ = true;

      const auto pt = map_pose_base_link.getOrigin();
      RCLCPP_INFO(node.get_logger(), "Successfully localized at {%f, %f, %f}", pt[0], pt[1], pt[2]);

      PoseType localized_pose;
      tf2::toMsg(map_pose_base_link, localized_pose);
      return localized_pose;
    };

  return CuvslamInternalLocalizeInMapAsync(
    map_folder_path,
    pose_hint_cv).then(std::move(extract_pose_in_ros_conventions));
}

std::optional<PoseType> VisualSlamNode::VisualSlamImpl::LocalizeInMap(
  const std::string & map_folder_path,
  const PoseType & pose_hint,
  const std::string & frame_id)
{
  return LocalizeInMapAsync(map_folder_path, pose_hint, frame_id).get();
}

void VisualSlamNode::VisualSlamImpl::CheckLocalizationStatus()
{
  std::lock_guard<std::mutex> lock(localization_mutex_);
  if (localization_future_.has_value()) {
    // Check if the future is ready using standard-compliant boost::future API
    if (localization_future_->wait_for(boost::chrono::seconds(0)) == boost::future_status::ready) {
      try {
        const auto maybe_pose = localization_future_->get();
        if (maybe_pose) {
          RCLCPP_INFO(node.get_logger(), "Localization completed successfully");
        } else {
          RCLCPP_ERROR(node.get_logger(), "Localization failed");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node.get_logger(), "Localization exception: %s", e.what());
      }
      // Clear the future
      localization_future_.reset();
    }
  }
}

void VisualSlamNode::VisualSlamImpl::PublishEvent(
  uint8_t type, int64_t stamp_ns, const std::string & description)
{
  static const std::map<uint8_t, std::string> kNames = {
    {SlamEventType::TRACKING_LOST, "tracking_lost"},
    {SlamEventType::TRACKING_RECOVERED, "tracking_recovered"},
    {SlamEventType::LOOP_CLOSURE, "loop_closure"},
    {SlamEventType::CONFIGURED, "configured"},
    {SlamEventType::TRACKER_INITIALIZED, "tracker_initialized"},
    {SlamEventType::POSE_REJECTED, "pose_rejected"},
    {SlamEventType::POSE_ACCEPTED, "pose_accepted"},
    {SlamEventType::TRACKER_RESET, "tracker_reset"},
    {SlamEventType::VIO_INITIALIZED, "vio_initialized"}};
  const auto name_it = kNames.find(type);
  const std::string name = name_it != kNames.end() ? name_it->second : std::to_string(type);
  if (node.events_pub_) {
    SlamEventType msg;
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = node.map_frame_;
    msg.type = type;
    msg.description = description;
    node.events_pub_->publish(msg);
  }
  if (type == SlamEventType::TRACKING_LOST) {
    RCLCPP_WARN(node.get_logger(), "Event %s: %s", name.c_str(), description.c_str());
  } else {
    RCLCPP_INFO(node.get_logger(), "Event %s: %s", name.c_str(), description.c_str());
  }
  if (run_output) {
    run_output->AddEvent(stamp_ns, name, description);
  }
}

void VisualSlamNode::VisualSlamImpl::ReportFrame(
  int64_t timestamp_ns, bool tracked, const std::optional<tf2::Transform> & odom_pose,
  const std::optional<tf2::Transform> & slam_pose, double track_time_ms,
  double callback_time_ms, const std::vector<std::pair<int, ImageType>> & images,
  const HealthOutput & health_out, const std::optional<compat::ImuState> & imu_state)
{
  const bool publish_diagnostics = node.enable_frame_diagnostics_ &&
    HasSubscribers(node.frame_diagnostics_pub_);
  const bool details = publish_diagnostics || run_output;

  // Observations and landmarks are available only when their export is enabled
  // (see CreateOdometryConfiguration).
  std::vector<size_t> observations(node.num_cameras_, 0);
  size_t num_landmarks = 0;
  if (details) {
    const bool observations_exported = node.enable_observations_view_ ||
      node.enable_localization_n_mapping_;
    const bool landmarks_exported = node.enable_landmarks_view_ ||
      node.enable_localization_n_mapping_;
    try {
      if (observations_exported) {
        for (uint32_t i = 0; i < node.num_cameras_; ++i) {
          observations[i] = cuvslam_odometry->GetLastObservations(i).size();
        }
      }
      if (landmarks_exported) {
        num_landmarks = cuvslam_odometry->GetLastLandmarks().size();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_ONCE(node.get_logger(), "Cannot read observations: %s", e.what());
    }
  }

  std::optional<cuvslam::Slam::Metrics> metrics;
  if (cuvslam_slam) {
    try {
      cuvslam::Slam::Metrics m;
      cuvslam_slam->GetSlamMetrics(m);
      metrics = m;
    } catch (const std::exception & e) {
      RCLCPP_WARN_ONCE(node.get_logger(), "Cannot read SLAM metrics: %s", e.what());
    }
  }

  std::optional<double> sync_offset_ms;
  std::optional<double> prep_ms;
  if (device_sync) {
    if (const auto it = frame_sync_offset_ms.find(timestamp_ns); it != frame_sync_offset_ms.end()) {
      sync_offset_ms = it->second;
    }
    frame_sync_offset_ms.erase(
      frame_sync_offset_ms.begin(), frame_sync_offset_ms.upper_bound(timestamp_ns));
    if (const auto it = frame_prep_ms.find(timestamp_ns); it != frame_prep_ms.end()) {
      prep_ms = it->second;
    }
    frame_prep_ms.erase(frame_prep_ms.begin(), frame_prep_ms.upper_bound(timestamp_ns));
  }

  // Events.
  if (tracked != last_frame_tracked) {
    PublishEvent(
      tracked ? SlamEventType::TRACKING_RECOVERED : SlamEventType::TRACKING_LOST, timestamp_ns,
      tracked ? "visual tracking recovered" : "visual tracking lost");
    last_frame_tracked = tracked;
  }
  const bool loop_closure = metrics && metrics->lc_status;
  if (loop_closure && !last_loop_closure) {
    PublishEvent(
      SlamEventType::LOOP_CLOSURE, timestamp_ns,
      "loop closure, " + std::to_string(metrics->lc_good_landmarks_count) + " landmarks");
  }
  last_loop_closure = loop_closure;
  const float vio_speed = imu_state ?
    std::sqrt(
    imu_state->velocity[0] * imu_state->velocity[0] +
    imu_state->velocity[1] * imu_state->velocity[1] +
    imu_state->velocity[2] * imu_state->velocity[2]) :
    std::numeric_limits<float>::quiet_NaN();
  if (imu_state && imu_state->initialized && !vio_initialized) {
    char text[160];
    std::snprintf(
      text, sizeof(text), "visual-inertial initialization done (Track() %.0f ms, VIO speed "
      "%.2f m/s)", track_time_ms, vio_speed);
    PublishEvent(SlamEventType::VIO_INITIALIZED, timestamp_ns, text);
    vio_initialized = true;
  }
  if (health_out.state == HealthState::REJECTED) {
    if (health_last_state != HealthState::REJECTED) {
      char text[128];
      std::snprintf(
        text, sizeof(text), "odometry increment rejected (%s, %.1f m/s, %.1f deg)",
        HealthReasons(health_out.reasons).c_str(), health_out.speed_mps,
        health_out.gyro_error_deg);
      PublishEvent(SlamEventType::POSE_REJECTED, timestamp_ns, text);
    }
    ++health_rejected_run;
  } else if (health_last_state == HealthState::REJECTED && health_out.state != HealthState::LOST) {
    PublishEvent(
      SlamEventType::POSE_ACCEPTED, timestamp_ns,
      "odometry accepted again after " + std::to_string(health_rejected_run) + " rejected frames");
    health_rejected_run = 0;
  }
  if (health_out.state != HealthState::LOST) {
    health_last_state = health_out.state;
  }

  // Statistics of the input images.
  std::vector<std::pair<int, ImageStats>> stats;
  if (details) {
    for (const auto & [idx, msg] : images) {
      const bool is_camera = idx < static_cast<int>(node.num_cameras_);
      const bool is_depth = node.DepthOfStream(idx) >= 0;
      if (is_camera || is_depth) {
        stats.emplace_back(idx, ComputeImageStats(*msg, is_depth, node.diagnostics_pixel_stride_));
      }
    }
  }

  if (publish_diagnostics) {
    FrameDiagnosticsType d;
    d.header.stamp = rclcpp::Time(timestamp_ns);
    d.header.frame_id = node.map_frame_;
    d.mode = TrackingModeToString(node.tracking_mode_);
    d.profile = node.profile_ ? node.profile_->name : "";
    d.tracked = tracked;
    d.track_time_ms = static_cast<float>(track_time_ms);
    d.callback_time_ms = static_cast<float>(callback_time_ms);
    // Latency from the capture: meaningful only with live data (NaN in a bag replay).
    d.latency_ms = run_input.empty() ?
      static_cast<float>((node.get_clock()->now().nanoseconds() - timestamp_ns) / 1e6) :
      std::numeric_limits<float>::quiet_NaN();
    d.sync_offset_ms = sync_offset_ms ? static_cast<float>(*sync_offset_ms) :
      std::numeric_limits<float>::quiet_NaN();
    for (uint32_t i = 0; i < node.num_cameras_; ++i) {
      d.camera_names.push_back(input_image_names[i].first);
      d.observations.push_back(static_cast<uint32_t>(observations[i]));
    }
    d.landmarks = static_cast<uint32_t>(num_landmarks);
    d.loop_closure = loop_closure;
    d.pose_graph_optimized = metrics && metrics->pgo_status;
    d.health = static_cast<uint8_t>(health_out.state);
    d.health_reasons = HealthReasons(health_out.reasons);
    d.speed_mps = static_cast<float>(health_out.speed_mps);
    d.gyro_error_deg = static_cast<float>(health_out.gyro_error_deg);
    d.imu_state_valid = imu_state.has_value();
    d.vio_initialized = imu_state && imu_state->initialized;
    d.vio_speed_mps = vio_speed;
    d.vio_mismatch_mps = static_cast<float>(health_out.vio_mismatch_mps);
    for (size_t i = 0; i < 3; ++i) {
      d.gyro_bias[i] = imu_state ? imu_state->gyro_bias[i] :
        std::numeric_limits<float>::quiet_NaN();
      d.acc_bias[i] = imu_state ? imu_state->acc_bias[i] :
        std::numeric_limits<float>::quiet_NaN();
    }
    for (const auto & [idx, st] : stats) {
      StreamStatsType ss;
      ss.name = input_image_names[idx].first;
      ss.topic = input_image_names[idx].second;
      const auto & msg = std::find_if(
        images.begin(), images.end(), [i = idx](const auto & p) {return p.first == i;})->second;
      ss.width = msg->width;
      ss.height = msg->height;
      ss.mean_intensity = st.mean_intensity;
      ss.saturated_pct = st.saturated_pct;
      ss.dark_pct = st.dark_pct;
      ss.valid_depth_pct = st.valid_depth_pct;
      d.streams.push_back(ss);
    }
    node.frame_diagnostics_pub_->publish(d);
  }

  if (run_output) {
    run_output->AddFrame(
      timestamp_ns, tracked, odom_pose, slam_pose, track_time_ms, num_landmarks, observations,
      callback_time_ms, sync_offset_ms, prep_ms);
    if (metrics) {
      run_output->AddSlamMetrics(timestamp_ns, *metrics);
    }
    for (const auto & [idx, st] : stats) {
      run_output->AddStreamStats(timestamp_ns, input_image_names[idx].first, st);
    }
    run_output->AddHealth(timestamp_ns, health_out);
    if (imu_state) {
      run_output->AddImuState(timestamp_ns, *imu_state);
    }
  }
}

void VisualSlamNode::VisualSlamImpl::WriteRunOutput()
{
  std::lock_guard<std::mutex> frame_lock(frame_mutex_);
  if (!run_output || run_output_written) {
    return;
  }
  run_output_written = true;
  // Do not overwrite the output of a previous run with an empty one (e.g. a configure /
  // cleanup cycle without input).
  if (run_output->NumFrames() == 0) {
    RCLCPP_INFO(
      node.get_logger(), "No frame processed: run output not written to %s",
      run_output->Directory().c_str());
    return;
  }

  int32_t major = 0, minor = 0, patch = 0;
  const std::string cuvslam_version(cuvslam::GetVersion(&major, &minor, &patch));

  JsonObject params;
  params.Add("tracking_mode", node.tracking_mode_)
  .Add("num_cameras", static_cast<uint64_t>(node.num_cameras_))
  .Add("min_num_images", static_cast<uint64_t>(node.min_num_images_))
  .Add("multicam_mode", static_cast<uint64_t>(node.multicam_mode_))
  .Add("rectified_images", node.rectified_images_)
  .Add("enable_image_denoising", node.enable_image_denoising_)
  .Add("enable_localization_n_mapping", node.enable_localization_n_mapping_)
  .Add("enable_ground_constraint_in_odometry", node.enable_ground_constraint_in_odometry_)
  .Add("enable_ground_constraint_in_slam", node.enable_ground_constraint_in_slam_)
  .Add("sync_matching_threshold_ms", node.sync_matching_threshold_ms_)
  .Add("image_jitter_threshold_ms", node.image_jitter_threshold_ms_)
  .Add("imu_jitter_threshold_ms", node.imu_jitter_threshold_ms_)
  .Add("gyro_noise_density", node.imu_params_.gyroscope_noise_density)
  .Add("gyro_random_walk", node.imu_params_.gyroscope_random_walk)
  .Add("accel_noise_density", node.imu_params_.accelerometer_noise_density)
  .Add("accel_random_walk", node.imu_params_.accelerometer_random_walk)
  .Add("calibration_frequency", node.imu_params_.calibration_frequency)
  .Add("slam_max_map_size", static_cast<uint64_t>(node.slam_max_map_size_))
  .Add("slam_throttling_time_ms", static_cast<uint64_t>(node.slam_throttling_time_ms_))
  .Add("image_buffer_size", static_cast<uint64_t>(node.image_buffer_size_))
  .Add("imu_buffer_size", static_cast<uint64_t>(node.imu_buffer_size_))
  .Add("camera_optical_frames", node.camera_optical_frames_)
  .Add("imu_frame", node.imu_frame_);

  std::vector<std::string> image_topics;
  for (const auto & [idx, topic] : node.GetImageTopics()) {
    image_topics.push_back(topic);
  }

  JsonObject info;
  info.Add("tool", "pq_cuvslam")
  .Add("wrapper_commit", PQ_CUVSLAM_GIT_COMMIT)
  .Add("cuvslam_version", cuvslam_version)
  .Add("cuvslam_compiled", std::to_string(PQ_CUVSLAM_VERSION_MAJOR) + "." +
    std::to_string(PQ_CUVSLAM_VERSION_MINOR) + "." + std::to_string(PQ_CUVSLAM_VERSION_PATCH))
  .Add("cuvslam_cunls", compat::CompiledFeatures().cunls)
  .Add("input", run_input.empty() ? std::string("live topics") : run_input)
  .Add("processing_mode",
    node.processing_mode_ == ProcessingMode::OFFLINE ? "offline" : "realtime")
  .Add("mode", TrackingModeToString(node.tracking_mode_))
  .Add("slam", node.enable_localization_n_mapping_)
  .Add("imu", node.UsesImu())
  .Add("image_topics", image_topics)
  .Add("imu_topic", node.GetImuTopic())
  .Add("base_frame", node.base_frame_)
  .Add("parameters", params);

  if (latency_emulator) {
    JsonObject emulation;
    emulation.Add("description", latency_emulator->Describe())
    .Add("frames_matched_by_timestamp", latency_emulator->Matched())
    .Add("frames_drawn_from_profile", latency_emulator->Drawn())
    .Add("frames_delayed", emulated_frames_delayed)
    .Add("total_delay_s", emulated_delay_total_ms / 1000.0);
    info.Add("latency_emulation", emulation);
  }
  if (node.profile_) {
    const Profile & p = *node.profile_;
    JsonObject profile;
    profile.Add("name", p.name).Add("path", p.path).Add("mode", p.mode_name);
    std::vector<std::string> cameras;
    for (const auto & c : p.cameras) {
      cameras.push_back(c.name);
    }
    profile.Add("cameras", cameras);
    if (p.imu) {
      JsonObject imu;
      imu.Add("noise_source", p.imu->noise_source).Add("noise_date", p.imu->noise_date);
      profile.Add("imu", imu);
    }
    std::vector<std::string> extrinsics;
    for (const auto & t : p.extrinsics) {
      const std::string date = t.date.empty() ? "" : " (" + t.date + ")";
      extrinsics.push_back(t.parent + " -> " + t.child + ": " + t.source + date);
    }
    profile.Add("extrinsics", extrinsics);
    if (p.ir_emitter) {
      profile.Add("ir_emitter", *p.ir_emitter);
    }
    info.Add("profile", profile);
  }
  if (device_sync) {
    const auto & st = device_sync->GetStats();
    const auto summary = [](std::vector<double> v) {
        JsonObject o;
        if (v.empty()) {
          return o;
        }
        std::sort(v.begin(), v.end());
        o.Add("median", v[v.size() / 2]).Add("p95", v[static_cast<size_t>(0.95 * (v.size() - 1))])
        .Add("max", v.back());
        return o;
      };
    JsonObject sync;
    sync.Add("mode", node.profile_->sync->mode_name)
    .Add("max_offset_ms", node.profile_->sync->max_offset_ms)
    .Add("rotation_compensation", rotation_compensation)
    .Add("rig_frames", st.rig_frames)
    .Add("incomplete_device_frames", st.incomplete_device_frames)
    .Add("stale_device_frames", st.stale_device_frames)
    .Add("late_device_frames", st.late_device_frames)
    .Add("unmatched_reference_frames", st.unmatched_reference_frames)
    .Add("unused_device_frames", st.unused_device_frames)
    .Add("reused_device_frames", st.reused_device_frames)
    .Add("offset_ms", summary(sync_stats.offsets_ms))
    .Add("prep_ms", summary(sync_stats.prep_ms));
    if (rotation_compensation) {
      sync.Add("compensated_images", sync_stats.compensated_images)
      .Add("small_rotations_not_warped", sync_stats.small_rotations)
      .Add("compensation_min_deg", node.profile_->sync->compensation_min_deg)
      .Add("images_without_gyroscope", sync_stats.gyro_missing)
      .Add("rotation_deg", summary(sync_stats.rotations_deg));
    }
    info.Add("sync", sync);
  }
  for (const auto & [key, json] : run_info_extra) {
    info.AddRaw(key, json);
  }

  if (run_output->Write(cuvslam_slam.get(), info)) {
    RCLCPP_INFO(
      node.get_logger(), "Run output written to %s (%zu frames)",
      run_output->Directory().c_str(), run_output->NumFrames());
  } else {
    RCLCPP_ERROR(
      node.get_logger(), "Cannot write the run output to %s", run_output->Directory().c_str());
  }
}

}  // namespace visual_slam
}  // namespace pq
