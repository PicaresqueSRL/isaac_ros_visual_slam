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
// Run output in the CSV/JSON format of VRHealth_Challenge13/scripts/run_output.py, so that
// the runs of this node can be compared directly with the PyCuVSLAM and Isaac ROS runs
// (compare_runs.py, ground truth pipeline).

#ifndef PQ_CUVSLAM__IMPL__RUN_OUTPUT_HPP_
#define PQ_CUVSLAM__IMPL__RUN_OUTPUT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cuvslam/cuvslam2.h"
#include "tf2/LinearMath/Transform.hpp"

namespace pq
{
namespace visual_slam
{

// Ordered JSON object built from already serialized values.
class JsonObject
{
public:
  JsonObject & Add(const std::string & key, const std::string & value);  // string value
  JsonObject & Add(const std::string & key, const char * value);
  JsonObject & Add(const std::string & key, double value);
  JsonObject & Add(const std::string & key, int64_t value);
  JsonObject & Add(const std::string & key, uint64_t value);
  JsonObject & Add(const std::string & key, int value);
  JsonObject & Add(const std::string & key, bool value);
  JsonObject & Add(const std::string & key, const std::vector<std::string> & values);
  JsonObject & Add(const std::string & key, const JsonObject & value);
  // Adds a value that is already valid JSON.
  JsonObject & AddRaw(const std::string & key, const std::string & json);

  std::string Dump(int indent = 0) const;
  bool Empty() const {return entries_.empty();}

private:
  std::vector<std::pair<std::string, std::string>> entries_;
};

// Collects the per-frame output of the tracker and writes it to a folder:
//   odometry.csv       one pose per tracked frame (visual(-inertial) odometry)
//   slam_online.csv    SLAM pose returned frame by frame during the run (with SLAM)
//   slam_poses.csv     final SLAM trajectory (GetAllSlamPoses at the end of the run, i.e.
//                      optimized with the loop closures; with SLAM)
//   status.csv         per frame: vo_state (1 tracked, 2 lost), Track() time, landmarks,
//                      observations per camera
//   slam_metrics.csv   per frame: loop closure and pose graph optimization status (with SLAM)
//   loop_closures.csv  last loop closure poses reported by SLAM, in the SLAM world frame
//                      (rig, OpenCV axes)
//   run_info.json      input, parameters, versions, frame counts and summary metrics
//
// Pose columns: timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg. Timestamps in seconds.
// Poses of the base frame (ROS axes), in meters, relative to the first pose of each file: the
// first row is the identity.
class RunOutput
{
public:
  RunOutput(std::string out_dir, size_t num_cameras);

  // Adds one processed frame. odom_pose and slam_pose are poses of the base frame in ROS
  // conventions; odom_pose is empty when tracking failed, slam_pose when SLAM is disabled.
  void AddFrame(
    int64_t timestamp_ns, bool tracked,
    const std::optional<tf2::Transform> & odom_pose,
    const std::optional<tf2::Transform> & slam_pose,
    double track_time_ms, size_t num_landmarks,
    const std::vector<size_t> & observations_per_camera);

  void AddSlamMetrics(int64_t timestamp_ns, const cuvslam::Slam::Metrics & metrics);

  // Writes all files. slam may be null (SLAM disabled). info is extended with the summary
  // and the list of files. Returns false if the folder cannot be written.
  bool Write(const cuvslam::Slam * slam, JsonObject info);

  size_t NumFrames() const {return status_.size();}
  const std::string & Directory() const {return out_dir_;}

private:
  using Row = std::vector<std::string>;

  Row PoseRow(int64_t timestamp_ns, const tf2::Transform & pose) const;

  std::string out_dir_;
  size_t num_cameras_;
  std::optional<tf2::Transform> odom_ref_;
  std::optional<tf2::Transform> slam_ref_;
  std::vector<Row> odometry_;
  std::vector<Row> slam_online_;
  std::vector<Row> status_;
  std::vector<Row> slam_metrics_;
  std::vector<double> track_times_ms_;
  std::vector<tf2::Vector3> odometry_positions_;
  std::vector<int64_t> odometry_stamps_;
  std::vector<tf2::Vector3> slam_online_positions_;
  std::vector<int64_t> slam_online_stamps_;
  size_t frames_lost_ = 0;
  size_t frames_with_lc_ = 0;
  size_t frames_with_pgo_ = 0;
  std::chrono::steady_clock::time_point start_;
};

// Shortest decimal representation that reads back to the same double (as Python repr).
std::string FormatDouble(double value);

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__RUN_OUTPUT_HPP_
