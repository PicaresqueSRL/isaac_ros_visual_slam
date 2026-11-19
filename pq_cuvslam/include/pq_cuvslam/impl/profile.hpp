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
// Camera layout profiles: one YAML file per configuration (cameras and streams, IMU, depth,
// extrinsics with their source, cuVSLAM mode and parameters). See profiles/README.md for the
// format.

#ifndef PQ_CUVSLAM__IMPL__PROFILE_HPP_
#define PQ_CUVSLAM__IMPL__PROFILE_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Transform.hpp"
#include "yaml-cpp/yaml.h"

namespace pq
{
namespace visual_slam
{

// Error in a profile file; the message names the file and the key.
class ProfileError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

enum class ProfileMode
{
  MULTICAMERA,   // stereo pairs, no IMU (tracking_mode 0)
  INERTIAL,      // one stereo pair + IMU (tracking_mode 1)
  RGBD,          // camera + aligned depth (tracking_mode 2)
  MONO,          // single camera (cuVSLAM Mono; not implemented in the node yet)
  MULTISENSOR    // cameras + depth + IMU (cuVSLAM >= 17)
};

struct ProfileCamera
{
  std::string name;
  std::string device;  // physical camera (e.g. cam2); cameras of one device share the shutter
  std::string image_topic;
  std::string camera_info_topic;
  std::string optical_frame;
};

struct ProfileImu
{
  std::string topic;
  std::string frame;
  double gyro_noise_density = 0.0;
  double gyro_random_walk = 0.0;
  double accel_noise_density = 0.0;
  double accel_random_walk = 0.0;
  double frequency = 0.0;
  std::string noise_source;  // e.g. default, allan_variance, kalibr
  std::string noise_date;
};

struct ProfileDepth
{
  std::string topic;
  int camera_index = 0;
  double scale = 1000.0;  // raw depth units per meter
  bool stereo_tracking = false;
};

// Transform missing from the TF of the data (e.g. between two cameras), with its origin.
struct ProfileTransform
{
  std::string parent;
  std::string child;
  tf2::Transform transform;  // pose of child in parent
  std::string source;
  std::string date;
};

// Synchronization between devices (e.g. two D455 on a rigid mount).
enum class SyncMode
{
  NONE,      // images grouped by timestamp only (Isaac ROS behaviour)
  SOFTWARE,  // nearest frame of each device to the reference device, optional rotation compensation
  HARDWARE   // devices triggered together (D455 master/slave): tight threshold, no compensation
};

struct ProfileSyncDevice
{
  std::string name;
  std::string imu_topic;   // gyroscope used for the rotation compensation
  std::string imu_frame;
  double clock_offset_ms = 0.0;  // subtracted from the stamps of this device
};

struct ProfileSync
{
  SyncMode mode = SyncMode::NONE;
  std::string mode_name = "none";
  double max_offset_ms = 0.0;        // max capture time difference between paired frames
  bool rotation_compensation = false;
  int compensation_border_px = 0;    // image border excluded from tracking when compensating
  std::vector<ProfileSyncDevice> devices;  // first = reference device
};

struct Profile
{
  std::string path;
  std::string name;
  std::string description;
  ProfileMode mode = ProfileMode::MULTICAMERA;
  std::string mode_name;
  std::string base_frame;
  std::vector<ProfileCamera> cameras;
  std::optional<ProfileImu> imu;
  std::optional<ProfileDepth> depth;
  std::vector<ProfileTransform> extrinsics;
  std::optional<ProfileSync> sync;
  std::optional<bool> ir_emitter;  // D455 IR projector state during the recording
  // Other node parameters (section "parameters"), in file order.
  std::vector<std::pair<std::string, YAML::Node>> parameters;
};

// Loads and checks the structure of a profile. Throws ProfileError.
Profile LoadProfile(const std::string & path);

// Resolves a profile argument: an existing file path, or the name of a profile installed in
// <share>/profiles/<name>.yaml. Throws ProfileError if not found.
std::string ResolveProfilePath(const std::string & profile, const std::string & share_directory);

// Node parameters set by the profile (name, value), in the order in which they are applied:
// first those derived from the structured sections, then the "parameters" section.
std::vector<std::pair<std::string, YAML::Node>> ProfileParameters(const Profile & profile);

// Consistency checks for the cuVSLAM version of the build: errors prevent the configuration,
// warnings are logged.
struct ProfileCheck
{
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
};
// cuvslam_major and multisensor_available describe the cuVSLAM build (impl/cuvslam_compat).
ProfileCheck CheckProfile(
  const Profile & profile, int cuvslam_major, bool multisensor_available);

const char * ProfileModeName(ProfileMode mode);

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__PROFILE_HPP_
