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

#ifndef PQ_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
#define PQ_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_

#include <cuda_runtime.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define BOOST_THREAD_PROVIDES_FUTURE
#define BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION
#include "boost/outcome/result.hpp"
#include "boost/thread.hpp"
#include "boost/thread/future.hpp"
#include "cuvslam/cuvslam2.h"
#include "cuvslam/ground_constraint2.h"
#include "cv_bridge/cv_bridge.hpp"
#include "pq_cuvslam/third_party/isaac_common/messaging/message_stream_synchronizer.hpp"
#include "pq_cuvslam/impl/device_frame_synchronizer.hpp"
#include "pq_cuvslam/impl/landmarks_vis_helper.hpp"
#include "pq_cuvslam/impl/latency_emulator.hpp"
#include "pq_cuvslam/impl/limited_vector.hpp"
#include "pq_cuvslam/impl/localizer_vis_helper.hpp"
#include "pq_cuvslam/impl/message_stream_sequencer.hpp"
#include "pq_cuvslam/impl/motion_compensation.hpp"
#include "pq_cuvslam/impl/pose_cache.hpp"
#include "pq_cuvslam/impl/posegraph_vis_helper.hpp"
#include "pq_cuvslam/impl/run_output.hpp"
#include "pq_cuvslam/impl/types.hpp"
#include "pq_cuvslam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace pq
{
namespace visual_slam
{

constexpr int32_t kMaxNumCameraParameters = 12;

struct VisualSlamNode::VisualSlamImpl
{
  // Asynchronous response for CUVSLAM_LocalizeInExistDb()
  struct LocalizeInExistDbContext
  {
    using Response = boost::outcome_v2::result<cuvslam::Pose, std::string>;
    boost::promise<Response> response_promise;
  };


  explicit VisualSlamImpl(VisualSlamNode & vslam_node);

  ~VisualSlamImpl();

  // Helpers to initialize the cuvslam tracker.
  void Initialize();

  bool IsReadyForInitialization() const;

  bool IsInitialized() const;

  void Exit();

  // Create the configuration for the cuvslam tracker.
  cuvslam::Odometry::Config CreateOdometryConfiguration();

  // Create the configuration for the cuvslam slam.
  cuvslam::Slam::Config CreateSlamConfiguration();

  // Helper function to publish a transform to the tf tree.
  void PublishFrameTransform(
    rclcpp::Time stamp, const tf2::Transform & pose, const std::string & target,
    const std::string & source);

  // Helper to get latest transform from the tf tree.
  tf2::Transform GetLatestTransform(
    const std::string & target, const std::string & source);

  // Helper to publish the estimated velocity from odometry.
  void PublishOdometryVelocity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher);

  // Helper to publish the estimated gravity vector.
  void PublishGravity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerType>::SharedPtr publisher);

  // Callbacks for subscribers.
  void CallbackImu(const ImuType::ConstSharedPtr & msg);
  void CallbackImage(int index, const ImageType & image_view);
  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);

  // Gyroscope of a synchronized device (rotation compensation).
  void CallbackSyncImu(int device, const ImuType::ConstSharedPtr & msg);

  // Rig frame of the device synchronizer: rotation compensation, then the sequencer.
  using DeviceSync = DeviceFrameSynchronizer<ImageType>;
  void OnRigFrame(const DeviceSync::RigFrame & frame);

  // Callback for synchronizer.
  void CallbackSynchronizedImages(
    int64_t current_ts, const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Callback for sequencer
  void UpdatePose(
    const std::vector<ImuType::ConstSharedPtr> & imu_msgs,
    const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Save the current map to disk.
  bool SaveMap(const std::string & map_folder_path);

  // Core functionality to localize in a map. Expects and returns the pose in cuvslam conventions.
  boost::future<LocalizeInExistDbContext::Response> CuvslamInternalLocalizeInMapAsync(
    const std::string & map_folder_path,
    const cuvslam::Pose & pose_hint);

  // Localize in an existing map asynchronously. Expects and returns the pose in ROS conventions.
  boost::future<std::optional<PoseType>> LocalizeInMapAsync(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Synchonous wrapper for LocalizeInMapAsync.
  std::optional<PoseType> LocalizeInMap(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Check and handle the localization future status
  void CheckLocalizationStatus();

  // Add one processed frame to the run output.
  void AddRunOutputFrame(
    int64_t timestamp_ns, bool tracked, const std::optional<tf2::Transform> & odom_pose,
    const std::optional<tf2::Transform> & slam_pose, double track_time_ms,
    double callback_time_ms);

  // Write the run output (see RunOutput) if enabled and not written yet.
  void WriteRunOutput();

  // Reference to the ros node.
  VisualSlamNode & node;

  // Synchronizer used to sync all image messages.
  using Synchronizer = nvidia::isaac_common::messaging::MessageStreamSynchronizer<ImageType>;
  Synchronizer sync;

  // Device synchronizer (profile sync mode software or hardware), used instead of sync.
  std::unique_ptr<DeviceSync> device_sync;
  bool rotation_compensation = false;
  std::vector<int64_t> device_clock_offset_ns;  // per device
  std::vector<int> camera_device;               // device index per camera
  std::vector<GyroBuffer> device_gyro;          // per device, corrected stamps
  std::vector<Eigen::Matrix3d> camera_K;        // per camera, set at initialization
  std::vector<Eigen::Matrix3d> camera_R_imu;    // per camera: rotation imu -> optical frame
  bool compensation_ready = false;
  struct SyncStats
  {
    std::vector<double> offsets_ms;     // max |offset| of the devices, per rig frame
    std::vector<double> rotations_deg;  // compensated rotation, per compensated image
    uint64_t compensated_images = 0;
    uint64_t gyro_missing = 0;          // images not compensated: gyroscope not available
  } sync_stats;
  // Capture offset of the rig frames, by reference stamp, for the run output.
  std::map<int64_t, double> frame_sync_offset_ms;

  // Sequencer used to sequence imu and image messages.
  using Sequencer = MessageStreamSequencer<ImuType::ConstSharedPtr,
      std::vector<std::pair<int, ImageType>>>;
  Sequencer sequencer;

  // cuVSLAM API objects
  std::unique_ptr<cuvslam::Odometry> cuvslam_odometry;
  cuvslam::Odometry::State odometry_state;
  std::shared_ptr<cuvslam::Slam> cuvslam_slam;  // Shared pointer for async visualization helpers
  std::unique_ptr<cuvslam::GroundConstraint> ground_constraint;

  // Define cameras for cuVSLAM. 2 for stereo camera.
  std::vector<cuvslam::Camera> cuvslam_cameras;

  // Helper classes for tf listening and publishing.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher{nullptr};

  limited_vector<PoseStampedType> vo_path;
  limited_vector<PoseStampedType> slam_path;

  // Point cloud to visualize tracks
  // LandmarksVisHelper observations_vis_helper;
  // Point cloud to visualize landmarks
  LandmarksVisHelper landmarks_vis_helper;
  // Loop closure landmarks
  LandmarksVisHelper lc_landmarks_vis_helper;
  // PoseGraph visualization
  PoseGraphVisHelper pose_graph_helper;
  // Localizer visualization
  LocalizerVisHelper localizer_helper;
  // Localizer: map landmarks
  LandmarksVisHelper localizer_landmarks_vis_helper;
  // Localizer: tracked landmarks
  LandmarksVisHelper localizer_observations_vis_helper;
  // Localizer: loop closure landmarks
  LandmarksVisHelper localizer_lc_landmarks_vis_helper;

  // Pose cache. For velocity calculation.
  PoseCache pose_cache;

  // Velocity cache. For velocity covariance calculation.
  VelocityCache velocity_cache;

  // Container to calulate execution time statictics.
  limited_vector<double> track_execution_times;

  // Timestamp of the last time CUVSLAM_Track was called in nanoseconds.
  int64_t last_track_ts;

  // Initial messages required for initialization.
  std::optional<ImuType::ConstSharedPtr> initial_imu_message;
  std::map<int, std::optional<CameraInfoType::ConstSharedPtr>> initial_camera_info_messages;

  LocalizeInExistDbContext localize_in_exist_db_context;
  bool localized_in_exist_map_ = false;

  // Store the localization future to prevent it from being destroyed before callback completes
  std::optional<boost::future<std::optional<PoseType>>> localization_future_;
  mutable std::mutex localization_mutex_;

  // Run output (CSV files and run_info.json); null when run_output_dir is not set.
  std::unique_ptr<RunOutput> run_output;
  bool run_output_written = false;
  // Description of the input, for run_info.json (e.g. bag path in replay).
  std::string run_input;
  // Extra run_info.json entries (key, JSON value), e.g. replay statistics.
  std::vector<std::pair<std::string, std::string>> run_info_extra;

  // Emulation of the processing time of another platform; empty when disabled.
  std::optional<LatencyEmulator> latency_emulator;
  // Frames delayed by the emulation and total delay added.
  uint64_t emulated_frames_delayed = 0;
  double emulated_delay_total_ms = 0.0;

  // CUDA stream used for rosidl::Buffer ReadHandle synchronization between the producer
  // (upstream node) and cuVSLAM. Created in the constructor and destroyed in the destructor.
  cudaStream_t cuda_stream_{};
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
