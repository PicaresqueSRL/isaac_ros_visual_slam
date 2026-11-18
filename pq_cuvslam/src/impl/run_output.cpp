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

#include "pq_cuvslam/impl/run_output.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

#include "pq_cuvslam/impl/cuvslam_ros_conversion.hpp"

namespace pq
{
namespace visual_slam
{

namespace
{

const std::vector<std::string> kPoseHeader = {
  "timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "roll_deg", "pitch_deg", "yaw_deg"};

std::string JsonEscape(const std::string & s)
{
  std::string out = "\"";
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

std::string JsonNumber(double value)
{
  return std::isfinite(value) ? FormatDouble(value) : "null";
}

std::string Indent(const std::string & json, int indent)
{
  // Indents every line after the first one (nested objects).
  std::string out;
  for (const char c : json) {
    out += c;
    if (c == '\n') {
      out.append(indent, ' ');
    }
  }
  return out;
}

bool WriteCsv(
  const std::filesystem::path & path, const std::vector<std::string> & header,
  const std::vector<std::vector<std::string>> & rows)
{
  std::ofstream f(path);
  if (!f) {
    return false;
  }
  auto write_row = [&f](const std::vector<std::string> & row) {
      for (size_t i = 0; i < row.size(); ++i) {
        f << (i ? "," : "") << row[i];
      }
      f << "\r\n";  // as Python's csv module
    };
  write_row(header);
  for (const auto & row : rows) {
    write_row(row);
  }
  return static_cast<bool>(f);
}

// Same formula as extract_paths.py and run_output.py.
std::vector<double> RpyDeg(double qx, double qy, double qz, double qw)
{
  const double roll = std::atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
  const double pitch = std::asin(std::clamp(2 * (qw * qy - qz * qx), -1.0, 1.0));
  const double yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
  const double k = 180.0 / M_PI;
  return {roll * k, pitch * k, yaw * k};
}

JsonObject PathStats(
  const std::vector<int64_t> & stamps, const std::vector<tf2::Vector3> & positions)
{
  JsonObject stats;
  stats.Add("poses", static_cast<uint64_t>(positions.size()));
  if (positions.size() < 2) {
    return stats;
  }
  double length = 0.0;
  for (size_t i = 1; i < positions.size(); ++i) {
    length += (positions[i] - positions[i - 1]).length();
  }
  stats.Add("duration_s", (stamps.back() - stamps.front()) / 1e9);
  stats.Add("path_length_m", length);
  stats.Add("start_end_distance_m", (positions.back() - positions.front()).length());
  return stats;
}

double Median(std::vector<double> values)
{
  if (values.empty()) {
    return NAN;
  }
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  return n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

}  // namespace

std::string FormatDouble(double value)
{
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), value);
  std::string s(buf, res.ptr);
  // Keep a decimal point for integral values, as Python repr does for floats.
  if (s.find_first_of(".eEn") == std::string::npos) {
    s += ".0";
  }
  return s;
}

// ---------------------------------------------------------------------------------------------
// JsonObject

JsonObject & JsonObject::AddRaw(const std::string & key, const std::string & json)
{
  entries_.emplace_back(key, json);
  return *this;
}

JsonObject & JsonObject::Add(const std::string & key, const std::string & value)
{
  return AddRaw(key, JsonEscape(value));
}

JsonObject & JsonObject::Add(const std::string & key, const char * value)
{
  return AddRaw(key, JsonEscape(value));
}

JsonObject & JsonObject::Add(const std::string & key, double value)
{
  return AddRaw(key, JsonNumber(value));
}

JsonObject & JsonObject::Add(const std::string & key, int64_t value)
{
  return AddRaw(key, std::to_string(value));
}

JsonObject & JsonObject::Add(const std::string & key, uint64_t value)
{
  return AddRaw(key, std::to_string(value));
}

JsonObject & JsonObject::Add(const std::string & key, int value)
{
  return AddRaw(key, std::to_string(value));
}

JsonObject & JsonObject::Add(const std::string & key, bool value)
{
  return AddRaw(key, value ? "true" : "false");
}

JsonObject & JsonObject::Add(const std::string & key, const std::vector<std::string> & values)
{
  std::string json = "[";
  for (size_t i = 0; i < values.size(); ++i) {
    json += (i ? ", " : "") + JsonEscape(values[i]);
  }
  return AddRaw(key, json + "]");
}

JsonObject & JsonObject::Add(const std::string & key, const JsonObject & value)
{
  return AddRaw(key, value.Dump());
}

std::string JsonObject::Dump(int indent) const
{
  if (entries_.empty()) {
    return "{}";
  }
  std::string out = "{";
  for (size_t i = 0; i < entries_.size(); ++i) {
    out += (i ? ",\n" : "\n") + std::string(indent + 2, ' ') +
      JsonEscape(entries_[i].first) + ": " + Indent(entries_[i].second, indent + 2);
  }
  return out + "\n" + std::string(indent, ' ') + "}";
}

// ---------------------------------------------------------------------------------------------
// RunOutput

RunOutput::RunOutput(std::string out_dir, size_t num_cameras)
: out_dir_(std::move(out_dir)), num_cameras_(num_cameras), start_(std::chrono::steady_clock::now())
{
}

RunOutput::Row RunOutput::PoseRow(int64_t timestamp_ns, const tf2::Transform & pose) const
{
  tf2::Quaternion q = pose.getRotation().normalized();
  if (q.w() < 0) {
    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
  }
  const tf2::Vector3 & t = pose.getOrigin();
  Row row = {
    FormatDouble(timestamp_ns / 1e9),
    FormatDouble(t.x()), FormatDouble(t.y()), FormatDouble(t.z()),
    FormatDouble(q.x()), FormatDouble(q.y()), FormatDouble(q.z()), FormatDouble(q.w())};
  for (const double a : RpyDeg(q.x(), q.y(), q.z(), q.w())) {
    row.push_back(FormatDouble(a));
  }
  return row;
}

void RunOutput::AddFrame(
  int64_t timestamp_ns, bool tracked,
  const std::optional<tf2::Transform> & odom_pose,
  const std::optional<tf2::Transform> & slam_pose,
  double track_time_ms, size_t num_landmarks,
  const std::vector<size_t> & observations_per_camera, double callback_time_ms)
{
  if (tracked && odom_pose) {
    if (!odom_ref_) {
      odom_ref_ = odom_pose->inverse();
    }
    const tf2::Transform rel = *odom_ref_ * *odom_pose;
    odometry_.push_back(PoseRow(timestamp_ns, rel));
    odometry_positions_.push_back(rel.getOrigin());
    odometry_stamps_.push_back(timestamp_ns);
  }
  if (slam_pose) {
    if (!slam_ref_) {
      slam_ref_ = slam_pose->inverse();
    }
    const tf2::Transform rel = *slam_ref_ * *slam_pose;
    slam_online_.push_back(PoseRow(timestamp_ns, rel));
    slam_online_positions_.push_back(rel.getOrigin());
    slam_online_stamps_.push_back(timestamp_ns);
  }

  // Track time rounded to 3 decimals, as run_output.py.
  const double track_ms = std::round(track_time_ms * 1000.0) / 1000.0;
  Row row = {FormatDouble(timestamp_ns / 1e9), tracked ? "1" : "2", FormatDouble(track_ms),
    std::to_string(num_landmarks)};
  for (size_t k = 0; k < num_cameras_; ++k) {
    row.push_back(std::to_string(k < observations_per_camera.size() ?
      observations_per_camera[k] : 0));
  }
  row.push_back(FormatDouble(std::round(callback_time_ms * 1000.0) / 1000.0));
  status_.push_back(std::move(row));
  track_times_ms_.push_back(track_ms);
  if (!tracked) {
    ++frames_lost_;
  }
}

void RunOutput::AddSlamMetrics(int64_t timestamp_ns, const cuvslam::Slam::Metrics & m)
{
  slam_metrics_.push_back({
        FormatDouble(timestamp_ns / 1e9), std::to_string(m.timestamp_ns),
        m.lc_status ? "True" : "False", m.pgo_status ? "True" : "False",
        std::to_string(m.lc_selected_landmarks_count), std::to_string(m.lc_tracked_landmarks_count),
        std::to_string(m.lc_pnp_landmarks_count), std::to_string(m.lc_good_landmarks_count)});
  frames_with_lc_ += m.lc_status ? 1 : 0;
  frames_with_pgo_ += m.pgo_status ? 1 : 0;
}

bool RunOutput::Write(const cuvslam::Slam * slam, JsonObject info)
{
  namespace fs = std::filesystem;
  const fs::path dir(out_dir_);
  std::error_code ec;
  fs::create_directories(dir, ec);
  if (ec) {
    return false;
  }

  bool ok = true;
  std::vector<std::string> files;
  JsonObject summary;

  ok &= WriteCsv(dir / "odometry.csv", kPoseHeader, odometry_);
  files.push_back("odometry.csv");
  std::vector<std::string> status_header = {"timestamp", "vo_state", "track_time_ms", "landmarks"};
  for (size_t k = 0; k < num_cameras_; ++k) {
    status_header.push_back("observations_cam" + std::to_string(k));
  }
  status_header.push_back("callback_time_ms");
  ok &= WriteCsv(dir / "status.csv", status_header, status_);
  files.push_back("status.csv");

  summary.Add("odometry", PathStats(odometry_stamps_, odometry_positions_));
  summary.Add("frames", static_cast<uint64_t>(status_.size()));
  summary.Add("frames_lost", static_cast<uint64_t>(frames_lost_));
  summary.Add("track_time_ms_median", Median(track_times_ms_));
  const double wall_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_).count();
  summary.Add("wall_time_s", std::round(wall_s * 100.0) / 100.0);

  if (slam != nullptr) {
    ok &= WriteCsv(dir / "slam_online.csv", kPoseHeader, slam_online_);
    files.push_back("slam_online.csv");
    summary.Add("slam_online", PathStats(slam_online_stamps_, slam_online_positions_));

    // Final SLAM trajectory, optimized with the loop closures.
    std::vector<cuvslam::PoseStamped> poses;
    try {
      slam->GetAllSlamPoses(poses);
    } catch (const std::exception &) {
      poses.clear();
    }
    std::sort(
      poses.begin(), poses.end(),
      [](const auto & a, const auto & b) {return a.timestamp_ns < b.timestamp_ns;});
    std::vector<Row> rows;
    std::vector<tf2::Vector3> positions;
    std::vector<int64_t> stamps;
    std::optional<tf2::Transform> ref;
    for (const auto & p : poses) {
      const tf2::Transform pose = ChangeBasis(canonical_pose_cuvslam, FromcuVSLAMPose(p.pose));
      if (!ref) {
        ref = pose.inverse();
      }
      const tf2::Transform rel = *ref * pose;
      rows.push_back(PoseRow(p.timestamp_ns, rel));
      positions.push_back(rel.getOrigin());
      stamps.push_back(p.timestamp_ns);
    }
    ok &= WriteCsv(dir / "slam_poses.csv", kPoseHeader, rows);
    files.push_back("slam_poses.csv");
    summary.Add("slam_final", PathStats(stamps, positions));

    // Loop closure poses: SLAM world frame (rig, OpenCV axes), written as they are.
    std::vector<cuvslam::PoseStamped> lc;
    try {
      slam->GetLoopClosurePoses(lc);
    } catch (const std::exception &) {
      lc.clear();
    }
    std::sort(
      lc.begin(), lc.end(),
      [](const auto & a, const auto & b) {return a.timestamp_ns < b.timestamp_ns;});
    std::vector<Row> lc_rows;
    for (const auto & p : lc) {
      lc_rows.push_back({
            FormatDouble(p.timestamp_ns / 1e9),
            FormatDouble(p.pose.translation[0]), FormatDouble(p.pose.translation[1]),
            FormatDouble(p.pose.translation[2]),
            FormatDouble(p.pose.rotation[0]), FormatDouble(p.pose.rotation[1]),
            FormatDouble(p.pose.rotation[2]), FormatDouble(p.pose.rotation[3])});
    }
    ok &= WriteCsv(
      dir / "loop_closures.csv", {"timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"}, lc_rows);
    files.push_back("loop_closures.csv");
    summary.Add("loop_closure_poses_reported", static_cast<uint64_t>(lc.size()));
  }

  if (!slam_metrics_.empty()) {
    ok &= WriteCsv(
      dir / "slam_metrics.csv",
      {"timestamp", "metrics_timestamp_ns", "lc_status", "pgo_status", "lc_selected_landmarks",
        "lc_tracked_landmarks", "lc_pnp_landmarks", "lc_good_landmarks"},
      slam_metrics_);
    files.push_back("slam_metrics.csv");
    summary.Add("frames_with_pgo", static_cast<uint64_t>(frames_with_pgo_));
    summary.Add("frames_with_lc", static_cast<uint64_t>(frames_with_lc_));
  }

  files.push_back("run_info.json");
  info.Add("summary", summary);
  info.Add("files", files);
  std::ofstream f(dir / "run_info.json");
  f << info.Dump() << "\n";
  ok &= static_cast<bool>(f);
  return ok;
}

}  // namespace visual_slam
}  // namespace pq
