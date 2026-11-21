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

#include "pq_cuvslam/impl/cuvslam_compat.hpp"

#include <stdexcept>
#include <string>

namespace pq
{
namespace visual_slam
{
namespace compat
{

Features CompiledFeatures()
{
  Features f;
  f.cunls = PQ_CUVSLAM_WITH_CUNLS != 0;
  f.multisensor = PQ_CUVSLAM_VERSION_MAJOR >= 17 && f.cunls;
  f.set_slam_pose = PQ_CUVSLAM_VERSION_MAJOR < 16;
  f.localizer_layers = PQ_CUVSLAM_HAS_LOCALIZER_LAYERS;
  return f;
}

std::string CheckRuntimeVersion()
{
  int32_t major = 0, minor = 0, patch = 0;
  const std::string version(cuvslam::GetVersion(&major, &minor, &patch));
  if (major != PQ_CUVSLAM_VERSION_MAJOR || minor != PQ_CUVSLAM_VERSION_MINOR) {
    return "the cuVSLAM library loaded at run time is " + version +
           " but the node was compiled with cuVSLAM " +
           std::to_string(PQ_CUVSLAM_VERSION_MAJOR) + "." +
           std::to_string(PQ_CUVSLAM_VERSION_MINOR) + "." +
           std::to_string(PQ_CUVSLAM_VERSION_PATCH) + " (check LD_LIBRARY_PATH)";
  }
  return "";
}

cuvslam::PoseCovariance PoseCovarianceRotationFirst(const cuvslam::PoseWithCovariance & pose)
{
#if PQ_CUVSLAM_VERSION_MAJOR >= 17
  // 17: (x, y, z, rx, ry, rz) -> (rx, ry, rz, x, y, z): swap the 3x3 blocks.
  cuvslam::PoseCovariance out{};
  const auto & in = pose.covariance_xyz_rpy;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      out[r * 6 + c] = in[((r + 3) % 6) * 6 + (c + 3) % 6];
    }
  }
  return out;
#else
  return pose.covariance;
#endif
}

void SetMultisensorMode(
  cuvslam::Odometry::Config & config, const std::vector<int32_t> & depth_camera_ids,
  float depth_scale_factor, bool enable_depth_stereo_tracking)
{
#if PQ_CUVSLAM_VERSION_MAJOR >= 17
  config.odometry_mode = cuvslam::Odometry::OdometryMode::Multisensor;
  config.multisensor_settings.depth_camera_ids = depth_camera_ids;
  config.multisensor_settings.depth_scale_factor = depth_scale_factor;
  config.multisensor_settings.enable_depth_stereo_tracking = enable_depth_stereo_tracking;
#else
  (void)config;
  (void)depth_camera_ids;
  (void)depth_scale_factor;
  (void)enable_depth_stereo_tracking;
  throw std::invalid_argument(
          "Multisensor odometry needs cuVSLAM >= 17 (this build: " +
          std::to_string(PQ_CUVSLAM_VERSION_MAJOR) + ")");
#endif
}

cuvslam::Pose SlamTrack(cuvslam::Slam & slam, const cuvslam::Odometry::State & state)
{
#if PQ_CUVSLAM_VERSION_MAJOR >= 17
  slam.Track(state);
  return slam.GetPose();
#else
  return slam.Track(state);
#endif
}

void SetSlamPose(cuvslam::Slam & slam, const cuvslam::Pose & pose)
{
#if PQ_CUVSLAM_VERSION_MAJOR < 16
  slam.SetSlamPose(pose);
#else
  (void)slam;
  (void)pose;
  throw std::runtime_error(
          "SetSlamPose is not available in cuVSLAM " + std::to_string(PQ_CUVSLAM_VERSION_MAJOR) +
          " (removed in 16)");
#endif
}

void LocalizeInMap(
  cuvslam::Slam & slam, std::string_view map_folder, int64_t timestamp_ns,
  const cuvslam::Pose & guess_pose, const cuvslam::Odometry::ImageSet & images,
  const LocalizationParams & params, LocalizationCallback callback)
{
  cuvslam::Slam::LocalizationSettings settings;
  settings.horizontal_search_radius = params.horizontal_search_radius;
  settings.vertical_search_radius = params.vertical_search_radius;
  settings.horizontal_step = params.horizontal_step;
  settings.vertical_step = params.vertical_step;
  settings.angular_step_rads = params.angular_step_rads;
#if PQ_CUVSLAM_VERSION_MAJOR < 16
  (void)timestamp_ns;
  settings.enable_reading_internals = params.enable_reading_internals;
  slam.LocalizeInMap(map_folder, guess_pose, images, settings, std::move(callback));
#else
  slam.LocalizeInMap(
    map_folder, timestamp_ns, guess_pose, images, settings, []() {}, std::move(callback));
#endif
}

}  // namespace compat
}  // namespace visual_slam
}  // namespace pq
