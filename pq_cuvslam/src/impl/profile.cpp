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

#include "pq_cuvslam/impl/profile.hpp"

#include <cmath>
#include <filesystem>
#include <set>

namespace pq
{
namespace visual_slam
{

namespace
{

constexpr double kDegToRad = M_PI / 180.0;

// Context for error messages: "<file>: <key>: <message>".
class Reader
{
public:
  explicit Reader(std::string path)
  : path_(std::move(path)) {}

  [[noreturn]] void Fail(const std::string & key, const std::string & message) const
  {
    throw ProfileError(path_ + ": " + key + ": " + message);
  }

  YAML::Node Required(const YAML::Node & node, const std::string & key, const std::string & where)
  const
  {
    const YAML::Node value = node[key];
    if (!value) {
      Fail(where + key, "missing");
    }
    return value;
  }

  template<typename T>
  T As(const YAML::Node & node, const std::string & key) const
  {
    try {
      return node.as<T>();
    } catch (const YAML::Exception & e) {
      Fail(key, std::string("invalid value (") + e.what() + ")");
    }
  }

  std::string String(
    const YAML::Node & node, const std::string & key, const std::string & where,
    bool required = true, const std::string & default_value = "") const
  {
    const YAML::Node value = node[key];
    if (!value || value.IsNull()) {
      if (required) {
        Fail(where + key, "missing");
      }
      return default_value;
    }
    return As<std::string>(value, where + key);
  }

  double Number(const YAML::Node & node, const std::string & key, const std::string & where) const
  {
    const double value = As<double>(Required(node, key, where), where + key);
    if (!std::isfinite(value)) {
      Fail(where + key, "not a finite number");
    }
    return value;
  }

  std::vector<double> Vector(
    const YAML::Node & node, const std::string & key, const std::string & where,
    size_t size) const
  {
    const YAML::Node value = Required(node, key, where);
    if (!value.IsSequence() || value.size() != size) {
      Fail(where + key, "expected a list of " + std::to_string(size) + " numbers");
    }
    std::vector<double> out;
    for (size_t i = 0; i < size; ++i) {
      out.push_back(As<double>(value[i], where + key));
    }
    return out;
  }

private:
  std::string path_;
};

ProfileMode ParseMode(const Reader & r, const std::string & mode)
{
  if (mode == "multicamera") {return ProfileMode::MULTICAMERA;}
  if (mode == "inertial") {return ProfileMode::INERTIAL;}
  if (mode == "rgbd") {return ProfileMode::RGBD;}
  if (mode == "mono") {return ProfileMode::MONO;}
  if (mode == "multisensor") {return ProfileMode::MULTISENSOR;}
  r.Fail("mode", "'" + mode + "' is not one of multicamera, inertial, rgbd, mono, multisensor");
}

ProfileTransform ParseTransform(const Reader & r, const YAML::Node & node, const std::string & w)
{
  ProfileTransform t;
  t.parent = r.String(node, "parent", w);
  t.child = r.String(node, "child", w);
  const auto xyz = r.Vector(node, "translation", w, 3);

  tf2::Quaternion q;
  const int rotations = (node["rotation_rpy"] ? 1 : 0) + (node["rotation_rpy_deg"] ? 1 : 0) +
    (node["rotation_quaternion"] ? 1 : 0);
  if (rotations != 1) {
    r.Fail(w + "rotation", "give exactly one of rotation_rpy (rad), rotation_rpy_deg, "
      "rotation_quaternion ([x, y, z, w])");
  }
  if (node["rotation_rpy"]) {
    const auto rpy = r.Vector(node, "rotation_rpy", w, 3);
    q.setRPY(rpy[0], rpy[1], rpy[2]);
  } else if (node["rotation_rpy_deg"]) {
    const auto rpy = r.Vector(node, "rotation_rpy_deg", w, 3);
    q.setRPY(rpy[0] * kDegToRad, rpy[1] * kDegToRad, rpy[2] * kDegToRad);
  } else {
    const auto v = r.Vector(node, "rotation_quaternion", w, 4);
    q = tf2::Quaternion(v[0], v[1], v[2], v[3]);
    if (std::abs(q.length() - 1.0) > 1e-3) {
      r.Fail(w + "rotation_quaternion", "not a unit quaternion");
    }
    q.normalize();
  }
  t.transform = tf2::Transform(q, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
  t.source = r.String(node, "source", w);
  t.date = r.String(node, "date", w, false);
  return t;
}

}  // namespace

const char * ProfileModeName(ProfileMode mode)
{
  switch (mode) {
    case ProfileMode::MULTICAMERA: return "multicamera";
    case ProfileMode::INERTIAL: return "inertial";
    case ProfileMode::RGBD: return "rgbd";
    case ProfileMode::MONO: return "mono";
    case ProfileMode::MULTISENSOR: return "multisensor";
  }
  return "unknown";
}

std::string ResolveProfilePath(const std::string & profile, const std::string & share_directory)
{
  namespace fs = std::filesystem;
  if (fs::is_regular_file(profile)) {
    return fs::absolute(profile).string();
  }
  const fs::path installed = fs::path(share_directory) / "profiles" / (profile + ".yaml");
  if (profile.find('/') == std::string::npos && fs::is_regular_file(installed)) {
    return installed.string();
  }
  throw ProfileError(
          "profile '" + profile + "' not found (neither a file nor " + installed.string() + ")");
}

Profile LoadProfile(const std::string & path)
{
  const Reader r(path);
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw ProfileError(path + ": " + e.what());
  }
  if (!root.IsMap()) {
    throw ProfileError(path + ": not a YAML mapping");
  }

  static const std::set<std::string> kKeys = {
    "name", "description", "mode", "base_frame", "cameras", "imu", "depth", "extrinsics",
    "sync", "hardware", "parameters"};
  for (const auto & item : root) {
    const std::string key = item.first.as<std::string>();
    if (kKeys.count(key) == 0) {
      r.Fail(key, "unknown key");
    }
  }

  Profile p;
  p.path = path;
  p.name = r.String(root, "name", "");
  p.description = r.String(root, "description", "", false);
  p.mode_name = r.String(root, "mode", "");
  p.mode = ParseMode(r, p.mode_name);
  p.base_frame = r.String(root, "base_frame", "");

  const YAML::Node cameras = r.Required(root, "cameras", "");
  if (!cameras.IsSequence() || cameras.size() == 0) {
    r.Fail("cameras", "expected a non-empty list");
  }
  for (size_t i = 0; i < cameras.size(); ++i) {
    const std::string w = "cameras[" + std::to_string(i) + "].";
    ProfileCamera c;
    c.name = r.String(cameras[i], "name", w);
    c.device = r.String(cameras[i], "device", w, false);
    c.image_topic = r.String(cameras[i], "image_topic", w);
    c.camera_info_topic = r.String(cameras[i], "camera_info_topic", w);
    c.optical_frame = r.String(cameras[i], "optical_frame", w);
    p.cameras.push_back(c);
  }

  if (const YAML::Node imu = root["imu"]; imu && !imu.IsNull()) {
    ProfileImu m;
    m.topic = r.String(imu, "topic", "imu.");
    m.frame = r.String(imu, "frame", "imu.");
    const YAML::Node noise = r.Required(imu, "noise", "imu.");
    m.gyro_noise_density = r.Number(noise, "gyro_noise_density", "imu.noise.");
    m.gyro_random_walk = r.Number(noise, "gyro_random_walk", "imu.noise.");
    m.accel_noise_density = r.Number(noise, "accel_noise_density", "imu.noise.");
    m.accel_random_walk = r.Number(noise, "accel_random_walk", "imu.noise.");
    m.frequency = r.Number(noise, "frequency", "imu.noise.");
    m.noise_source = r.String(noise, "source", "imu.noise.");
    m.noise_date = r.String(noise, "date", "imu.noise.", false);
    p.imu = m;
  }

  if (const YAML::Node depth = root["depth"]; depth && !depth.IsNull()) {
    ProfileDepth d;
    d.topic = r.String(depth, "topic", "depth.");
    d.camera_index = r.As<int>(r.Required(depth, "camera_index", "depth."), "depth.camera_index");
    d.scale = r.Number(depth, "scale", "depth.");
    if (depth["stereo_tracking"]) {
      d.stereo_tracking = r.As<bool>(depth["stereo_tracking"], "depth.stereo_tracking");
    }
    p.depth = d;
  }

  if (const YAML::Node extrinsics = root["extrinsics"]; extrinsics && !extrinsics.IsNull()) {
    if (!extrinsics.IsSequence()) {
      r.Fail("extrinsics", "expected a list");
    }
    for (size_t i = 0; i < extrinsics.size(); ++i) {
      p.extrinsics.push_back(
        ParseTransform(r, extrinsics[i], "extrinsics[" + std::to_string(i) + "]."));
    }
  }

  if (const YAML::Node sync = root["sync"]; sync && !sync.IsNull()) {
    ProfileSync s;
    s.mode_name = r.String(sync, "mode", "sync.");
    if (s.mode_name == "none") {
      s.mode = SyncMode::NONE;
    } else if (s.mode_name == "software") {
      s.mode = SyncMode::SOFTWARE;
    } else if (s.mode_name == "hardware") {
      s.mode = SyncMode::HARDWARE;
    } else {
      r.Fail("sync.mode", "'" + s.mode_name + "' is not one of none, software, hardware");
    }
    s.max_offset_ms = s.mode == SyncMode::HARDWARE ? 2.0 : 34.0;
    if (sync["max_offset_ms"]) {
      s.max_offset_ms = r.Number(sync, "max_offset_ms", "sync.");
    }
    if (sync["rotation_compensation"]) {
      s.rotation_compensation =
        r.As<bool>(sync["rotation_compensation"], "sync.rotation_compensation");
    }
    if (sync["compensation_border_px"]) {
      s.compensation_border_px =
        r.As<int>(sync["compensation_border_px"], "sync.compensation_border_px");
    }
    const YAML::Node devices = r.Required(sync, "devices", "sync.");
    if (!devices.IsSequence()) {
      r.Fail("sync.devices", "expected a list");
    }
    for (size_t i = 0; i < devices.size(); ++i) {
      const std::string w = "sync.devices[" + std::to_string(i) + "].";
      ProfileSyncDevice d;
      d.name = r.String(devices[i], "name", w);
      d.imu_topic = r.String(devices[i], "imu_topic", w, false);
      d.imu_frame = r.String(devices[i], "imu_frame", w, false);
      if (devices[i]["clock_offset_ms"]) {
        d.clock_offset_ms = r.Number(devices[i], "clock_offset_ms", w);
      }
      s.devices.push_back(d);
    }
    p.sync = s;
  }

  if (const YAML::Node hw = root["hardware"]; hw && !hw.IsNull()) {
    if (hw["ir_emitter"] && !hw["ir_emitter"].IsNull()) {
      p.ir_emitter = r.As<bool>(hw["ir_emitter"], "hardware.ir_emitter");
    }
  }

  if (const YAML::Node params = root["parameters"]; params && !params.IsNull()) {
    if (!params.IsMap()) {
      r.Fail("parameters", "expected a mapping of node parameters");
    }
    for (const auto & item : params) {
      p.parameters.emplace_back(item.first.as<std::string>(), item.second);
    }
  }
  return p;
}

std::vector<std::pair<std::string, YAML::Node>> ProfileParameters(const Profile & p)
{
  std::vector<std::pair<std::string, YAML::Node>> out;
  const auto set = [&out](const std::string & name, const auto & value) {
      out.emplace_back(name, YAML::Node(value));
    };

  switch (p.mode) {
    case ProfileMode::MULTICAMERA: set("tracking_mode", 0); break;
    case ProfileMode::INERTIAL: set("tracking_mode", 1); break;
    case ProfileMode::RGBD: set("tracking_mode", 2); break;
    case ProfileMode::MONO: set("tracking_mode", 3); break;
    case ProfileMode::MULTISENSOR:
      set("tracking_mode", 4);
      set("multisensor_use_imu", p.imu.has_value());
      if (!p.depth) {
        set("depth_camera_id", -1);
      }
      break;
  }
  set("base_frame", p.base_frame);
  set("num_cameras", static_cast<int>(p.cameras.size()));
  set("min_num_images", static_cast<int>(p.cameras.size()));
  std::vector<std::string> frames;
  for (const auto & c : p.cameras) {
    frames.push_back(c.optical_frame);
  }
  set("camera_optical_frames", frames);
  if (p.imu) {
    set("imu_frame", p.imu->frame);
    set("gyro_noise_density", p.imu->gyro_noise_density);
    set("gyro_random_walk", p.imu->gyro_random_walk);
    set("accel_noise_density", p.imu->accel_noise_density);
    set("accel_random_walk", p.imu->accel_random_walk);
    set("calibration_frequency", p.imu->frequency);
  }
  if (p.depth) {
    set("depth_camera_id", p.depth->camera_index);
    set("depth_scale_factor", p.depth->scale);
    set("depth_enable_stereo_tracking", p.depth->stereo_tracking);
  }
  for (const auto & [name, value] : p.parameters) {
    out.emplace_back(name, value);
  }
  return out;
}

ProfileCheck CheckProfile(const Profile & p, int cuvslam_major, bool multisensor_available)
{
  ProfileCheck check;
  const size_t n = p.cameras.size();
  const std::string mode = p.mode_name;

  switch (p.mode) {
    case ProfileMode::MULTICAMERA:
      if (n < 2 || n % 2 != 0) {
        check.errors.push_back(
          "mode multicamera needs stereo pairs (an even number of cameras, at least 2); the "
          "profile has " + std::to_string(n));
      }
      break;
    case ProfileMode::INERTIAL:
      if (n != 2) {
        check.errors.push_back(
          "mode inertial supports exactly one stereo pair (2 cameras); the profile has " +
          std::to_string(n));
      }
      if (!p.imu) {
        check.errors.push_back("mode inertial needs the imu section");
      }
      break;
    case ProfileMode::RGBD:
      if (!p.depth) {
        check.errors.push_back("mode rgbd needs the depth section");
      }
      break;
    case ProfileMode::MONO:
      if (n != 1) {
        check.errors.push_back(
          "mode mono needs exactly one camera; the profile has " + std::to_string(n));
      }
      check.warnings.push_back("mode mono: the trajectory has no metric scale");
      break;
    case ProfileMode::MULTISENSOR:
      if (cuvslam_major < 17) {
        check.errors.push_back(
          "mode multisensor needs cuVSLAM >= 17; this build uses cuVSLAM " +
          std::to_string(cuvslam_major));
      } else if (!multisensor_available) {
        check.errors.push_back(
          "mode multisensor needs cuVSLAM built with cuNLS (USE_CUNLS)");
      } else if (!p.depth && n < 2) {
        check.errors.push_back(
          "mode multisensor needs a depth image (depth section) or at least two cameras with "
          "overlapping views (cuVSLAM rejects a single camera without depth, with or without IMU)");
      }
      check.warnings.push_back("mode multisensor is experimental in cuVSLAM 17");
      break;
  }

  if (p.depth && p.depth->camera_index >= static_cast<int>(n)) {
    check.errors.push_back(
      "depth.camera_index " + std::to_string(p.depth->camera_index) + " but the profile has " +
      std::to_string(n) + " cameras");
  }
  if (p.depth && p.mode != ProfileMode::RGBD && p.mode != ProfileMode::MULTISENSOR) {
    check.warnings.push_back("depth section ignored in mode " + mode);
  }
  if (p.imu && p.mode != ProfileMode::INERTIAL && p.mode != ProfileMode::MULTISENSOR) {
    check.warnings.push_back("imu section ignored in mode " + mode);
  }
  if (p.imu && (p.mode == ProfileMode::INERTIAL || p.mode == ProfileMode::MULTISENSOR) &&
    p.imu->noise_source == "default")
  {
    check.warnings.push_back(
      "IMU noise parameters are defaults (imu.noise.source: default), not measured on this IMU "
      "(Allan variance or Kalibr): VIO accuracy may suffer");
  }
  if (p.ir_emitter && *p.ir_emitter) {
    check.warnings.push_back(
      "hardware.ir_emitter: true: the projected IR dots degrade the cuVSLAM tracking");
  }

  std::set<std::string> names;
  std::set<std::string> topics;
  for (const auto & c : p.cameras) {
    if (!names.insert(c.name).second) {
      check.errors.push_back("duplicate camera name " + c.name);
    }
    if (!topics.insert(c.image_topic).second) {
      check.errors.push_back("duplicate image topic " + c.image_topic);
    }
  }
  if (p.sync && p.sync->mode != SyncMode::NONE) {
    const ProfileSync & s = *p.sync;
    std::set<std::string> devices;
    for (const auto & d : s.devices) {
      if (!devices.insert(d.name).second) {
        check.errors.push_back("sync: duplicate device " + d.name);
      }
      if (s.rotation_compensation && (d.imu_topic.empty() || d.imu_frame.empty())) {
        check.errors.push_back(
          "sync.rotation_compensation needs imu_topic and imu_frame for device " + d.name);
      }
    }
    for (const auto & c : p.cameras) {
      if (c.device.empty()) {
        check.errors.push_back("sync: camera " + c.name + " has no device");
      } else if (devices.count(c.device) == 0) {
        check.errors.push_back(
          "sync: device " + c.device + " of camera " + c.name + " is not in sync.devices");
      }
    }
    if (s.devices.size() < 2) {
      check.warnings.push_back("sync: fewer than two devices, nothing to synchronize");
    }
    if (s.max_offset_ms <= 0.0) {
      check.errors.push_back("sync.max_offset_ms must be positive");
    }
    if (s.mode == SyncMode::HARDWARE && s.rotation_compensation) {
      check.warnings.push_back("sync: rotation compensation is not needed in hardware mode");
    }
  }

  for (const auto & t : p.extrinsics) {
    if (t.parent == t.child) {
      check.errors.push_back("extrinsic with parent == child (" + t.parent + ")");
    }
  }
  return check;
}

}  // namespace visual_slam
}  // namespace pq
