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

#include "pq_cuvslam/impl/odometry_selector.hpp"

#include <stdexcept>

namespace pq
{
namespace visual_slam
{

void OdometrySelector::AddTracker(const std::string & name, const tf2::Transform & ref_from_base)
{
  if (trackers_.count(name)) {
    throw std::invalid_argument("selector: tracker " + name + " added twice");
  }
  trackers_[name].ref_from_base = ref_from_base;
  order_.push_back(name);
  if (selected_.empty()) {
    selected_ = name;
  }
}

std::optional<tf2::Transform> OdometrySelector::At(
  const std::deque<std::pair<int64_t, tf2::Transform>> & history, int64_t stamp_ns)
{
  std::optional<tf2::Transform> out;
  for (const auto & [t, pose] : history) {
    if (t > stamp_ns) {
      break;
    }
    out = pose;
  }
  return out;
}

SelectorResult OdometrySelector::Update(
  const std::string & name, int64_t stamp_ns, const tf2::Transform & pose, bool bad)
{
  SelectorResult result;
  const auto it = trackers_.find(name);
  if (it == trackers_.end()) {
    result.selected = selected_;
    return result;
  }
  Tracker & tr = it->second;
  const auto window_ns = static_cast<int64_t>(config_.bad_window_s * 1e9);
  if (bad) {
    tr.bad_stamps_ns.push_back(stamp_ns);
    tr.good_since_ns.reset();
  } else if (!tr.good_since_ns) {
    tr.good_since_ns = stamp_ns;
  }
  // The windows of all trackers advance with the time of the latest frame.
  for (auto & [n, t] : trackers_) {
    while (!t.bad_stamps_ns.empty() && t.bad_stamps_ns.front() < stamp_ns - window_ns) {
      t.bad_stamps_ns.pop_front();
    }
  }

  // Increment of this tracker since its previous frame, in the reference base frame.
  std::optional<tf2::Transform> increment;
  if (tr.last) {
    increment = tr.ref_from_base * (tr.last->inverse() * pose) * tr.ref_from_base.inverse();
  }
  tr.last = pose;
  const auto keep_ns = window_ns + static_cast<int64_t>(1e9);
  tr.history.emplace_back(stamp_ns, pose);
  while (!tr.history.empty() && tr.history.front().first < stamp_ns - keep_ns) {
    tr.history.pop_front();
  }

  // Selection.
  const std::string before = selected_;
  Tracker & current = trackers_.at(selected_);
  const std::optional<int64_t> first_bad = current.bad_stamps_ns.empty() ?
    std::nullopt : std::optional<int64_t>(current.bad_stamps_ns.front());
  if (static_cast<int>(current.bad_stamps_ns.size()) >= config_.bad_frames) {
    for (const auto & n : order_) {
      const Tracker & other = trackers_.at(n);
      if (n != selected_ && other.last && other.bad_stamps_ns.empty()) {
        selected_ = n;
        break;
      }
    }
  } else if (selected_ != config_.primary && trackers_.count(config_.primary)) {
    const Tracker & primary = trackers_.at(config_.primary);
    if (primary.good_since_ns && primary.last &&
      stamp_ns - *primary.good_since_ns >= static_cast<int64_t>(config_.back_after_s * 1e9))
    {
      selected_ = config_.primary;
    }
  }
  if (selected_ != before) {
    result.switched_from = before;
    // Replace the motion held by the previous tracker since its first bad frame with the
    // motion of the new one over the same time.
    const Tracker & next = trackers_.at(selected_);
    if (first_bad) {
      const int64_t t0 = *first_bad - 1;
      const auto fused_then = At(fused_history_, t0);
      const auto next_then = At(next.history, t0);
      if (fused_then && next_then && next.last) {
        const tf2::Transform motion =
          next.ref_from_base * (next_then->inverse() * *next.last) * next.ref_from_base.inverse();
        fused_ = *fused_then * motion;
        result.fused = fused_;
      }
    }
  } else if (name == selected_ && increment) {
    fused_ = fused_ * *increment;
    result.fused = fused_;
  }
  if (result.fused) {
    fused_history_.emplace_back(stamp_ns, fused_);
    while (!fused_history_.empty() && fused_history_.front().first < stamp_ns - keep_ns) {
      fused_history_.pop_front();
    }
  }
  result.selected = selected_;
  return result;
}

}  // namespace visual_slam
}  // namespace pq
