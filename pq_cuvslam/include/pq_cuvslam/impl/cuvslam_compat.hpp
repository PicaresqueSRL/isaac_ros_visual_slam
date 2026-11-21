// Copyright (c) 2026 Picaresque SRL
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
//
// Adaptation layer between the node and the cuVSLAM API of the build (versions 15 to 17).
// The rest of the node uses these functions where the API differs between versions; this is
// the only place with code that depends on the cuVSLAM version.
//
// Differences handled (public header libs/cuvslam/cuvslam2.h):
//   16: Slam::SetSlamPose removed; Slam::LocalizeInMap takes a timestamp and start/finish
//       callbacks, LocalizationSettings has no enable_reading_internals; the localizer data
//       layers (LocalizerProbes, LocalizerMap, ...) and ReadLocalizerProbes removed.
//   17: PoseWithCovariance::covariance renamed covariance_xyz_rpy, with translation first;
//       Slam::Track returns nothing (pose from Slam::GetPose); OdometryMode::Multisensor.
//
// PQ_CUVSLAM_VERSION_MAJOR/MINOR/PATCH and PQ_CUVSLAM_WITH_CUNLS are set by CMake.

#ifndef PQ_CUVSLAM__IMPL__CUVSLAM_COMPAT_HPP_
#define PQ_CUVSLAM__IMPL__CUVSLAM_COMPAT_HPP_

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

#include "cuvslam/cuvslam2.h"

#ifndef PQ_CUVSLAM_VERSION_MAJOR
#error "PQ_CUVSLAM_VERSION_MAJOR is not defined (set by the pq_cuvslam CMakeLists)"
#endif

// Localizer data layers and probes (visualization of the localization): cuVSLAM 15 only.
#define PQ_CUVSLAM_HAS_LOCALIZER_LAYERS (PQ_CUVSLAM_VERSION_MAJOR < 16)

namespace pq
{
namespace visual_slam
{
namespace compat
{

// cuVSLAM version and features of the build.
struct Features
{
  int major = PQ_CUVSLAM_VERSION_MAJOR;
  int minor = PQ_CUVSLAM_VERSION_MINOR;
  int patch = PQ_CUVSLAM_VERSION_PATCH;
  bool mono = true;                // OdometryMode::Mono (all supported versions)
  bool multisensor = false;        // OdometryMode::Multisensor (17, with cuNLS)
  bool cunls = false;              // cuVSLAM built with cuNLS
  bool set_slam_pose = false;      // Slam::SetSlamPose (15)
  bool localizer_layers = false;   // localizer visualization layers (15)
};

// Features of the cuVSLAM headers this node was compiled with.
Features CompiledFeatures();

// Checks that the cuVSLAM library loaded at run time is the version the node was compiled
// with (major and minor). Returns an empty string, or the error message.
std::string CheckRuntimeVersion();

// Covariance of a pose estimate in the cuVSLAM 15 layout: rotation about x, y, z (rad), then
// translation x, y, z (m), row-major.
cuvslam::PoseCovariance PoseCovarianceRotationFirst(const cuvslam::PoseWithCovariance & pose);

// Sets the Multisensor odometry mode (cuVSLAM >= 17): depth images for the cameras in
// depth_camera_ids (empty: no depth), IMU fusion when the rig has an IMU. Throws before 17.
void SetMultisensorMode(
  cuvslam::Odometry::Config & config, const std::vector<int32_t> & depth_camera_ids,
  float depth_scale_factor, bool enable_depth_stereo_tracking);

// Adds an odometry state to SLAM and returns the SLAM pose.
cuvslam::Pose SlamTrack(cuvslam::Slam & slam, const cuvslam::Odometry::State & state);

// Sets the SLAM pose. Throws std::runtime_error when the cuVSLAM version does not support it.
void SetSlamPose(cuvslam::Slam & slam, const cuvslam::Pose & pose);

struct LocalizationParams
{
  float horizontal_search_radius = 0.0f;
  float vertical_search_radius = 0.0f;
  float horizontal_step = 0.0f;
  float vertical_step = 0.0f;
  float angular_step_rads = 0.0f;
  bool enable_reading_internals = false;  // cuVSLAM 15 only
};

using LocalizationCallback = std::function<void (const cuvslam::Result<cuvslam::Pose> &)>;

// Starts the localization in a saved map; the callback receives the result.
void LocalizeInMap(
  cuvslam::Slam & slam, std::string_view map_folder, int64_t timestamp_ns,
  const cuvslam::Pose & guess_pose, const cuvslam::Odometry::ImageSet & images,
  const LocalizationParams & params, LocalizationCallback callback);

}  // namespace compat
}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__CUVSLAM_COMPAT_HPP_
