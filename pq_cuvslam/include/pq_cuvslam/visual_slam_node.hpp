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

#ifndef PQ_CUVSLAM__VISUAL_SLAM_NODE_HPP_
#define PQ_CUVSLAM__VISUAL_SLAM_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <atomic>

#include "pq_cuvslam/impl/profile.hpp"
#include "pq_cuvslam/impl/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace pq
{
namespace visual_slam
{

struct IMUParams
{
  double gyroscope_noise_density;       // rad / (s * srqt(Hz))
  double gyroscope_random_walk;         // rad / (s ^ 2 * srqt(Hz))
  double accelerometer_noise_density;   // m / (s ^ 2 * srqt(Hz))
  double accelerometer_random_walk;     // m / (s ^ 3 * srqt(Hz))
  double calibration_frequency;
};

/// Tracking mode enumeration
/// Defines the type of tracking algorithm used by the visual SLAM system
enum class TrackingMode : int
{
  MULTICAMERA = 0,  ///< Default multicamera stereo tracking
  VIO = 1,          ///< Visual-Inertial Odometry (IMU fusion required)
  RGBD = 2,         ///< RGBD depth-based tracking
  MONO = 3,         ///< Single camera, trajectory up to scale
  MULTISENSOR = 4   ///< cuVSLAM >= 17 with cuNLS: cameras, optional depth, optional IMU
};

/// Convert TrackingMode enum to string representation
inline const char * TrackingModeToString(int mode)
{
  switch (mode) {
    case static_cast<int>(TrackingMode::MULTICAMERA):
      return "Multicamera";
    case static_cast<int>(TrackingMode::VIO):
      return "VIO (IMU fusion)";
    case static_cast<int>(TrackingMode::RGBD):
      return "RGBD";
    case static_cast<int>(TrackingMode::MONO):
      return "Mono";
    case static_cast<int>(TrackingMode::MULTISENSOR):
      return "Multisensor";
    default:
      return "Unknown";
  }
}

/// Processing mode of the cuVSLAM tracker.
enum class ProcessingMode
{
  REALTIME,  ///< Bundle adjustment and SLAM in background threads (Isaac ROS behaviour)
  OFFLINE    ///< Bundle adjustment and SLAM in the tracking call: reproducible runs
};

// Managed (lifecycle) node:
//   configure   reads the parameters, checks cuVSLAM and the GPU, creates subscriptions,
//               publishers, services and the tracker (initialized on the first camera info)
//   activate    input is processed
//   deactivate  input is dropped; tracker and map are kept
//   cleanup     writes the run output, destroys the tracker and the interfaces
// With autostart:=true (default) the node configures and activates itself when it starts
// spinning, like the Isaac ROS node.
class VisualSlamNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit VisualSlamNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  virtual ~VisualSlamNode();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  // Direct input, bypassing the subscriptions (bag replay). Messages must be given in the
  // order in which they would have been received. Not thread safe: call from one thread and
  // do not spin the node at the same time. The node must be configured and active (input is
  // dropped otherwise).
  void InputImage(int index, const ImageType & msg);
  void InputCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);
  void InputImu(const ImuType::ConstSharedPtr & msg);
  void InputStaticTransforms(const tf2_msgs::msg::TFMessage & msg);

  // Input topics after remapping, with the input index used by InputImage / InputCameraInfo.
  std::vector<std::pair<int, std::string>> GetImageTopics() const;
  std::vector<std::pair<int, std::string>> GetCameraInfoTopics() const;
  // Empty when the IMU is not used.
  std::string GetImuTopic() const;
  // Gyroscope topics of the synchronized devices (device index, topic); empty without
  // rotation compensation.
  std::vector<std::pair<int, std::string>> GetSyncImuTopics() const;
  void InputSyncImu(int device, const ImuType::ConstSharedPtr & msg);
  // History depth of the image (also camera info) and IMU subscriptions.
  size_t GetImageQueueDepth() const {return image_qos_.depth();}
  size_t GetImuQueueDepth() const {return imu_qos_.depth();}

  // Description of the input written to run_info.json (e.g. the bag path).
  void SetRunInput(const std::string & input);
  // Extra entry of run_info.json; json must be a valid JSON value. Requires a configured node.
  void AddRunInfo(const std::string & key, const std::string & json);

  // Writes the run output to run_output_dir (if set). Called automatically at destruction;
  // later calls do nothing.
  void WriteRunOutput();

private:
  // Declares the parameter on first use, then returns its current value.
  template<typename T>
  T Param(
    const std::string & name, const T & default_value,
    const rcl_interfaces::msg::ParameterDescriptor & descriptor =
    rcl_interfaces::msg::ParameterDescriptor());
  // Declares (first call) and reads all parameters into the members below.
  void DeclareAndReadParameters();
  // Loads the profile named by profile_name_, checks it and sets the node parameters it
  // defines. Throws on error.
  void ApplyProfile();
  // Checks the cuVSLAM version, the tracking mode and the GPU; throws on error.
  void CheckSetup();
  void CreateInterfaces();
  void DestroyInterfaces();
  // Writes the run output, saves the map if requested, destroys tracker and interfaces.
  void TearDown();

  // True in the active state: input is processed.
  std::atomic<bool> active_{false};
  rclcpp::TimerBase::SharedPtr autostart_timer_;
  // Writes the run output when the context shuts down (SIGINT): with several nodes in one
  // container the process may end before every node is destroyed.
  rclcpp::PreShutdownCallbackHandle pre_shutdown_handle_;

  // Camera layout profile (parameter "profile"): name of an installed profile or path of a
  // YAML file. When set, it defines the cameras, topics, frames, extrinsics and mode, and the
  // parameters it lists; empty = configuration from the parameters and remappings only.
  std::string profile_name_;
  std::optional<Profile> profile_;

  // Parameters below are read at every configuration (see DeclareAndReadParameters).

  // Functional Parameters:
  // Number of cameras used. If a single stereocamera is used this has to be set to 2.
  // This is the single source of truth, ie. if you set num_cameras=2 but pass 4
  // input_camera_optical_frames we will only use the first 2.
  uint num_cameras_;

  // Functional Parameters:
  // Number of input masks used.
  uint num_input_masks_;

  // Multicamera mode: moderate (0), performance (1) or precision (2).
  // Multicamera odometry settings will be adjusted depending on a chosen strategy.
  // Default is performance (1).
  uint multicam_mode_;

  // The minimum number of images used. This configures how many camera images can be dropped
  // without stopping tracking.
  uint min_num_images_;

  // Maximum value of acceptable timestamp deltas between the camera images to still consider their
  // timestamp identical. Should be < 0.5 / fps_value.
  double sync_matching_threshold_ms_;

  // Number of pixels on the outer edges of the frame will be masked
  // during the feature selection stage. Values are defined for the left camera
  // of the stereo pair (camera at even index in the list of cameras). For the right camera
  // (camera at odd index in the list of cameras), the left and right borders will be mirrored.
  uint border_mask_top_;
  uint border_mask_bottom_;
  uint border_mask_left_;
  uint border_mask_right_;

  // Disable this if the input images have already passed through a denoising filter.
  bool enable_image_denoising_;

  // Enable fast and robust left-to-right tracking for rectified cameras with
  // principal points on the horizontal line. Disable this if input images are
  // not already rectified.
  bool rectified_images_;

  // If enabled, visual odometry poses will be modified such that the camera moves
  // on a horizontal plane.
  bool enable_ground_constraint_in_odometry_;

  // If enabled, slam poses will be modified such that the camera moves on a horizontal plane.
  bool enable_ground_constraint_in_slam_;

  // Use localization and mapping (SLAM).
  // If it is false, it is only Visual Odometry and no map will get produced.
  bool enable_localization_n_mapping_;

  // Tracking mode: determines the type of odometry algorithm used.
  // Stored as int for ROS 2 parameter compatibility, but should be compared
  // against TrackingMode enum values: MULTICAMERA (0), VIO (1), RGBD (2), MONO (3),
  // MULTISENSOR (4).
  int tracking_mode_;

  // Multisensor mode: fuse the IMU (imu_frame, noise parameters, IMU topic). The depth is used
  // when depth_camera_id >= 0.
  bool multisensor_use_imu_;

  // The mode uses the IMU (VIO, Multisensor with IMU) / a depth image (RGBD, Multisensor with
  // depth).
  bool UsesImu() const
  {
    return tracking_mode_ == static_cast<int>(TrackingMode::VIO) ||
           (tracking_mode_ == static_cast<int>(TrackingMode::MULTISENSOR) && multisensor_use_imu_);
  }
  bool UsesDepth() const
  {
    return tracking_mode_ == static_cast<int>(TrackingMode::RGBD) ||
           (tracking_mode_ == static_cast<int>(TrackingMode::MULTISENSOR) && depth_camera_id_ >= 0);
  }

  // Enable IMU debug mode
  bool enable_debug_imu_mode_;

  // Data structure to store imu noise params
  IMUParams imu_params_;

  // Depth scale factor to convert raw depth values to meters
  float depth_scale_factor_;

  // Camera index that the depth image is aligned with
  int32_t depth_camera_id_;

  // Enable stereo tracking between depth-aligned camera and other cameras
  bool depth_enable_stereo_tracking_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value). cuVSLAM library will print out warning messages
  // if rate of incoming image pair is lower than the threshold value.
  double image_jitter_threshold_ms_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value).
  double imu_jitter_threshold_ms_;

  // If set will save the generated map when shutting down the node to the specified path. Requires
  // enable_localization_n_mapping_ to be true.
  std::string save_map_folder_path_;

  // Default map that is used to localize in. Can be changed at runtime with the "LoadMapSrv"
  // service. In order to use the map an initial pose hint also has to be provided with the
  // "LocalizeInMap" service or the "initialpose" topic. Using maps requires
  // enable_localization_n_mapping_ to be true.
  std::string load_map_folder_path_;

  // If true will try to localize in the map around [0,0,0]. Requires that load_map_folder_path_ is
  // set.
  bool localize_on_startup_;

  // Radius of the area on the horizontal plane used for localization in meters.
  float localizer_horizontal_radius_;

  // Radius along the vertical axis used for localization in meters.
  float localizer_vertical_radius_;

  // Step size along each axis on the horizontal plane for localization in meters.
  float localizer_horizontal_step_;

  // Step size along vectical axis for localization in meters.
  float localizer_vertical_step_;

  // Step size around vectical axis for localization in radians. Must be less then 2 Pi.
  float localizer_angular_step_;

  // Maximum number of poses that is stored in the SLAM map. Default is 300.
  // Adding more poses will invoke reduction of the map until it`s size matches this constant.
  uint slam_max_map_size_;

  // Minimum time interval between loop closures. Default is 500 ms.
  uint slam_throttling_time_ms_;

  // This node assumes the following frame hierarchy:
  // map - odom - base - ... - camera_i
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  // Per default the frames from the camera info messages will be used. Use this parameter to
  // override this.
  std::vector<std::string> camera_optical_frames_;

  // Per default the frame from the imu message will be used. Use this parameter to override this.
  std::string imu_frame_;

  // Message Parameters:
  // Buffer size of image buffer used in synchronizer.
  uint image_buffer_size_;

  // Buffer size of imu buffer.
  uint imu_buffer_size_;

  // The QoS used for the image and subscriptions
  rclcpp::QoS image_qos_{rclcpp::SystemDefaultsQoS()};
  rclcpp::QoS imu_qos_{rclcpp::SystemDefaultsQoS()};

  // Output Parameters:
  // Enable this to override the timestamps of all outputs to the current time.
  // This is helpful when playing back with rosbags and allows to ignore the
  // recording time.
  bool override_publishing_stamp_;

  // Publish output frames hierarchy.
  bool publish_map_to_odom_tf_;
  bool publish_odom_to_base_tf_;

  // Invert the odom -> base transform published to the tf tree.
  // This is useful for maintaining the one-parent rule in systems with multiple odometry sources.
  bool invert_map_to_odom_tf_;
  bool invert_odom_to_base_tf_;

  // Debug/Visualization Parameters:
  // Enable slam data visualization (Landmarks, Pose Graph, etc).
  // Demands additional hardware resources.
  bool enable_slam_visualization_;

  // Enable view observations.
  bool enable_observations_view_;

  // Enable view landmarks for map and loop closure events.
  bool enable_landmarks_view_;

  // Max size of the buffer for pose trail visualization.
  uint path_max_size_;

  // Verbosity for cuvslam, the larger the more verbose.
  int verbosity_;

  // Enable this flag to dump the received images, imu messages, timestamps and
  // camera infos at the location specified by debug_dump_path_.
  bool enable_debug_mode_;

  // Path to directory to store the dump data. Only used when enable_debug_mode_ is true.
  std::string debug_dump_path_;

  // Enable requesting another hint if current hint fails to initialize cuvslam localization.
  bool enable_request_hint_;

  // Processing mode: "realtime" (default) or "offline" (reproducible, see ProcessingMode).
  ProcessingMode processing_mode_;

  // If set, the run output (CSV files and run_info.json) is written to this folder.
  std::string run_output_dir_;

  // Diagnostics: per-frame topic visual_slam/diagnostics/frame (when it has subscribers) and
  // statistics of the input images computed on one pixel every diagnostics_pixel_stride_.
  bool enable_frame_diagnostics_;

  // Health check of the odometry and guarded odometry (impl/tracking_health.hpp).
  double health_max_speed_mps_;
  double health_max_gyro_error_deg_;
  int health_min_landmarks_;
  int health_reset_bad_frames_;
  int diagnostics_pixel_stride_;

  // Emulation of the processing time of another platform (e.g. Jetson on a PC): each frame
  // takes at least the time given by latency_profile_ (status.csv measured on that platform)
  // or emulated_latency_ms_. See impl/latency_emulator.hpp.
  std::string latency_profile_;
  double emulated_latency_ms_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr localize_in_map_callback_group_;

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  segmentation_masks_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  depth_image_subs_;
  std::vector<rclcpp::Subscription<CameraInfoType>::SharedPtr> camera_info_subs_;
  rclcpp::Subscription<ImuType>::SharedPtr imu_sub_;
  // Gyroscopes of the synchronized devices (software sync with rotation compensation).
  std::vector<std::pair<int, rclcpp::Subscription<ImuType>::SharedPtr>> sync_imu_subs_;
  rclcpp::Subscription<PoseWithCovarianceStampedType>::SharedPtr initial_pose_sub_;

  // Publishers: Visual SLAM
  rclcpp::Publisher<VisualSlamStatusType>::SharedPtr visual_slam_status_pub_;
  rclcpp::Publisher<PoseStampedType>::SharedPtr tracking_vo_pose_pub_;
  rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr
    tracking_vo_pose_covariance_pub_;
  rclcpp::Publisher<OdometryType>::SharedPtr tracking_odometry_pub_;
  rclcpp::Publisher<OdometryType>::SharedPtr tracking_odometry_guarded_pub_;
  rclcpp::Publisher<PathType>::SharedPtr tracking_vo_path_pub_;
  rclcpp::Publisher<PathType>::SharedPtr tracking_slam_path_pub_;
  rclcpp::Publisher<DiagnosticArrayType>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<FrameDiagnosticsType>::SharedPtr frame_diagnostics_pub_;
  rclcpp::Publisher<SlamEventType>::SharedPtr events_pub_;

  // Sends a message to global localization to generate a pose hint
  rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr trigger_hint_pub_;

  // Visualization for odometry
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_observations_pub_;
  rclcpp::Publisher<MarkerType>::SharedPtr vis_gravity_pub_;
  rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_vo_velocity_pub_;
  rclcpp::Publisher<OdometryType>::SharedPtr vis_slam_odometry_pub_;

  // Visualization for map and "loop closure"
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_landmarks_pub_;
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_loop_closure_pub_;
  rclcpp::Publisher<PoseArrayType>::SharedPtr vis_posegraph_nodes_pub_;
  rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges_pub_;
  rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges2_pub_;

  // Visualization for localization
  rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_localizer_pub_;
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_landmarks_pub_;
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_observations_pub_;
  rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_loop_closure_pub_;

  // Slam Services
  rclcpp::Service<SrvReset>::SharedPtr reset_srv_;
  rclcpp::Service<SrvGetAllPoses>::SharedPtr get_all_poses_srv_;
  rclcpp::Service<SrvSetSlamPose>::SharedPtr set_slam_pose_srv_;
  rclcpp::Service<SrvFilePath>::SharedPtr save_map_srv_;
  rclcpp::Service<SrvFilePath>::SharedPtr load_map_srv_;
  rclcpp::Service<SrvLocalizeInMap>::SharedPtr localize_in_map_srv_;

  // Callback functions for services.
  void CallbackReset(
    const std::shared_ptr<SrvReset::Request> req,
    std::shared_ptr<SrvReset::Response> res);
  void CallbackGetAllPoses(
    const std::shared_ptr<SrvGetAllPoses::Request> req,
    std::shared_ptr<SrvGetAllPoses::Response> res);
  void CallbackSetSlamPose(
    const std::shared_ptr<SrvSetSlamPose::Request> req,
    std::shared_ptr<SrvSetSlamPose::Response> res);
  void CallbackSaveMap(
    const std::shared_ptr<SrvFilePath::Request> req,
    std::shared_ptr<SrvFilePath::Response> res);
  void CallbackLoadMap(
    const std::shared_ptr<SrvFilePath::Request> req,
    std::shared_ptr<SrvFilePath::Response> res);
  void CallbackLocalizeInMap(
    const std::shared_ptr<SrvLocalizeInMap::Request> req,
    std::shared_ptr<SrvLocalizeInMap::Response> res);

  // Callback functions for subscribers.
  void CallbackImu(const ImuType::ConstSharedPtr & msg);
  void CallbackSyncImu(int device, const ImuType::ConstSharedPtr & msg);
  void CallbackImage(int index, const ImageType & msg);
  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);
  void CallbackInitialPose(const PoseWithCovarianceStampedType::ConstSharedPtr & msg);

  struct VisualSlamImpl;
  std::unique_ptr<VisualSlamImpl> impl_;

  // Thread management for localization
  std::thread localization_thread_;
  std::atomic<bool> localization_thread_running_{false};
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__VISUAL_SLAM_NODE_HPP_
