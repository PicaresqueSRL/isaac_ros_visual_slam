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
// Online health check of the odometry and guarded odometry. cuVSLAM can report a successful
// tracking while the pose jumps (live walk indoors: 45 m in one frame when cam2 faced the
// ceiling) or while it has no landmarks (saturated images). For every frame the increment of
// the odometry pose is checked with the information of that frame only (causal):
//   - speed of the increment above max_speed_mps;
//   - rotation of the increment against the gyroscope over the same interval;
//   - few landmarks (degraded, not rejected);
//   - speed estimated by the visual-inertial odometry (cuVSLAM >= 17) above the speed of the
//     accepted increments of the last vio_window_s (median) by more than max_vio_mismatch_mps:
//     the inertial state disagrees with the visual motion, often 1-2 s before the increments
//     become implausible (degraded, not rejected).
// The guarded odometry applies the accepted increments and, for a rejected or lost frame, keeps
// the position and applies the rotation of the gyroscope (identity without gyroscope): a jump
// is not carried into the output trajectory.

#ifndef PQ_CUVSLAM__IMPL__TRACKING_HEALTH_HPP_
#define PQ_CUVSLAM__IMPL__TRACKING_HEALTH_HPP_

#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "tf2/LinearMath/Transform.hpp"

namespace pq
{
namespace visual_slam
{

struct HealthConfig
{
  double max_speed_mps = 3.0;       // walking with a hand-held rig: ~1.5 m/s
  double max_gyro_error_deg = 5.0;  // per frame; tracked frames rarely exceed 2 deg
  int min_landmarks = 10;           // below: degraded
  // VIO speed against the odometry: 0 disables (the mismatch is still computed). Clean bags:
  // above 1 m/s in < 1 % of the frames.
  double max_vio_mismatch_mps = 0.0;
  double vio_window_s = 0.5;
  // Reset of the tracker: at least reset_bad_frames rejected or lost frames within the last
  // reset_window_s seconds (0 disables), at most one reset every reset_min_interval_s.
  int reset_bad_frames = 8;
  double reset_window_s = 1.0;
  double reset_min_interval_s = 2.0;
};

enum class HealthState : uint8_t
{
  OK = 0,
  DEGRADED = 1,  // accepted, but few landmarks
  REJECTED = 2,  // increment rejected (implausible speed or rotation)
  LOST = 3,      // no pose from the tracker
};

// Reasons, bit mask.
constexpr uint32_t kHealthSpeed = 1;
constexpr uint32_t kHealthGyro = 2;
constexpr uint32_t kHealthFewLandmarks = 4;
constexpr uint32_t kHealthLost = 8;
constexpr uint32_t kHealthVio = 16;

std::string HealthStateName(HealthState state);
std::string HealthReasons(uint32_t reasons);  // e.g. "speed+gyro"

struct HealthInput
{
  int64_t stamp_ns = 0;
  std::optional<tf2::Transform> pose;  // odometry pose of the base frame; empty: lost
  std::optional<size_t> landmarks;  // empty: not available
  std::optional<tf2::Quaternion> gyro_rotation;  // base frame rotation since the previous frame
  std::optional<double> vio_speed_mps;  // speed estimated by the VIO, once it is initialized
};

struct HealthOutput
{
  HealthState state = HealthState::OK;
  uint32_t reasons = 0;
  double speed_mps = std::numeric_limits<double>::quiet_NaN();
  double gyro_error_deg = std::numeric_limits<double>::quiet_NaN();
  // VIO speed - median speed of the accepted increments of the window; NaN without VIO state.
  double vio_mismatch_mps = std::numeric_limits<double>::quiet_NaN();
  std::optional<tf2::Transform> guarded_pose;   // empty until the first pose
  int consecutive_bad = 0;                       // rejected or lost frames in a row
  int bad_in_window = 0;                         // rejected or lost frames in reset_window_s
  bool reset_tracker = false;                    // the tracker should be reset now
};

class TrackingHealth
{
public:
  explicit TrackingHealth(const HealthConfig & config = HealthConfig())
  : config_(config) {}

  HealthOutput Update(const HealthInput & input);

  const HealthConfig & Config() const {return config_;}

  // The tracker was reset: its next pose starts a new odometry frame. The guarded odometry
  // continues from its last pose; the bad frame history is cleared.
  void OnTrackerReset(int64_t stamp_ns);

private:
  HealthConfig config_;
  std::optional<tf2::Transform> last_raw_;
  std::optional<tf2::Transform> guarded_;
  int64_t last_raw_stamp_ns_ = 0;
  // Gyroscope rotation since the last pose of the tracker (spans lost frames); empty when a
  // frame came without gyroscope.
  std::optional<tf2::Quaternion> gyro_since_raw_;
  int consecutive_bad_ = 0;
  std::deque<int64_t> bad_stamps_ns_;
  int64_t last_reset_ns_ = 0;
  std::deque<std::pair<int64_t, double>> accepted_speeds_;  // (stamp, speed), vio_window_s

  HealthOutput Finish(HealthOutput out, int64_t stamp_ns, bool bad);
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__TRACKING_HEALTH_HPP_
