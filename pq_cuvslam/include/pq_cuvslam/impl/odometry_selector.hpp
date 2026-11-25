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
// Selection between two (or more) odometries running in parallel on rigidly mounted cameras,
// e.g. the VIO of cam1 (down) and of cam2 (forward). Each tracker gives its guarded odometry
// and the health of every frame (impl/tracking_health.hpp). The fused odometry, in the base
// frame of the reference tracker, applies the increments of the selected tracker only,
// converted with the fixed transform between the base frames. Causal: decided on every frame
// with the data received so far.
//   - switch away from the selected tracker when it has at least bad_frames rejected, lost or
//     degraded frames within bad_window_s and another tracker has none;
//   - back to the primary tracker when it has been healthy for back_after_s.
// At a switch the motion since the first bad frame of the previous tracker (held by its guarded
// odometry) is taken from the new tracker, so that the detection delay loses no motion.

#ifndef PQ_CUVSLAM__IMPL__ODOMETRY_SELECTOR_HPP_
#define PQ_CUVSLAM__IMPL__ODOMETRY_SELECTOR_HPP_

#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Transform.hpp"

namespace pq
{
namespace visual_slam
{

struct SelectorConfig
{
  std::string primary;
  double bad_window_s = 0.5;
  int bad_frames = 3;
  double back_after_s = 2.0;
};

struct SelectorResult
{
  std::string selected;                      // tracker selected after this frame
  std::optional<tf2::Transform> fused;       // new fused pose (frames of the selected tracker)
  std::optional<std::string> switched_from;  // set when the selection changed at this frame
};

class OdometrySelector
{
public:
  explicit OdometrySelector(const SelectorConfig & config = SelectorConfig())
  : config_(config), selected_(config.primary) {}

  // ref_from_base: pose of the base frame of the tracker in the reference base frame.
  void AddTracker(const std::string & name, const tf2::Transform & ref_from_base);

  // One frame of a tracker: its guarded pose (any fixed origin) and whether it is bad.
  SelectorResult Update(
    const std::string & name, int64_t stamp_ns, const tf2::Transform & pose, bool bad);

  const std::string & Selected() const {return selected_;}
  const std::vector<std::string> & Trackers() const {return order_;}

private:
  struct Tracker
  {
    tf2::Transform ref_from_base;
    std::optional<tf2::Transform> last;
    std::deque<int64_t> bad_stamps_ns;
    std::optional<int64_t> good_since_ns;
    std::deque<std::pair<int64_t, tf2::Transform>> history;  // recent poses
  };

  // Last pose of a history at or before stamp_ns.
  static std::optional<tf2::Transform> At(
    const std::deque<std::pair<int64_t, tf2::Transform>> & history, int64_t stamp_ns);

  SelectorConfig config_;
  std::map<std::string, Tracker> trackers_;
  std::vector<std::string> order_;
  std::string selected_;
  tf2::Transform fused_ = tf2::Transform::getIdentity();
  std::deque<std::pair<int64_t, tf2::Transform>> fused_history_;
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__ODOMETRY_SELECTOR_HPP_
