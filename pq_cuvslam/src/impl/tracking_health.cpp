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

#include "pq_cuvslam/impl/tracking_health.hpp"

#include <algorithm>
#include <cmath>

namespace pq
{
namespace visual_slam
{

namespace
{

double AngleDeg(const tf2::Quaternion & q)
{
  const double w = std::min(1.0, std::abs(q.normalized().getW()));
  return 2.0 * std::acos(w) * 180.0 / M_PI;
}

}  // namespace

std::string HealthStateName(HealthState state)
{
  switch (state) {
    case HealthState::OK: return "ok";
    case HealthState::DEGRADED: return "degraded";
    case HealthState::REJECTED: return "rejected";
    case HealthState::LOST: return "lost";
  }
  return "unknown";
}

std::string HealthReasons(uint32_t reasons)
{
  std::string out;
  const auto add = [&out](const char * name) {
      out += (out.empty() ? "" : "+") + std::string(name);
    };
  if (reasons & kHealthSpeed) {add("speed");}
  if (reasons & kHealthGyro) {add("gyro");}
  if (reasons & kHealthFewLandmarks) {add("few_landmarks");}
  if (reasons & kHealthLost) {add("lost");}
  return out;
}

HealthOutput TrackingHealth::Update(const HealthInput & in)
{
  HealthOutput out;
  // Gyroscope over the interval of the increment checked below (since the last tracker pose).
  if (in.gyro_rotation && gyro_since_raw_) {
    gyro_since_raw_ = *gyro_since_raw_ * *in.gyro_rotation;
  } else {
    gyro_since_raw_.reset();
  }

  // Rotation applied when the increment of the tracker is not used.
  const tf2::Transform held(
    in.gyro_rotation ? in.gyro_rotation->normalized() : tf2::Quaternion::getIdentity(),
    tf2::Vector3(0, 0, 0));

  if (!in.pose) {
    out.state = HealthState::LOST;
    out.reasons = kHealthLost;
    if (guarded_) {
      guarded_ = *guarded_ * held;
    }
    out.guarded_pose = guarded_;
    out.consecutive_bad = ++consecutive_bad_;
    return out;
  }

  if (!last_raw_) {
    last_raw_ = *in.pose;
    last_raw_stamp_ns_ = in.stamp_ns;
    gyro_since_raw_ = tf2::Quaternion::getIdentity();
    guarded_ = guarded_ ? *guarded_ : *in.pose;
    out.guarded_pose = guarded_;
    consecutive_bad_ = 0;
    return out;
  }

  // Increment of the tracker since its last pose, in the base frame of that pose.
  const tf2::Transform increment = last_raw_->inverse() * *in.pose;
  const double dt = in.stamp_ns > last_raw_stamp_ns_ ?
    (in.stamp_ns - last_raw_stamp_ns_) / 1e9 : 1.0 / 30.0;
  last_raw_ = *in.pose;
  last_raw_stamp_ns_ = in.stamp_ns;
  const std::optional<tf2::Quaternion> gyro = gyro_since_raw_;
  gyro_since_raw_ = tf2::Quaternion::getIdentity();
  out.speed_mps = increment.getOrigin().length() / dt;
  if (out.speed_mps > config_.max_speed_mps) {
    out.reasons |= kHealthSpeed;
  }
  if (gyro && in.gyro_rotation) {
    out.gyro_error_deg = AngleDeg(increment.getRotation().inverse() * *gyro);
    if (out.gyro_error_deg > config_.max_gyro_error_deg) {
      out.reasons |= kHealthGyro;
    }
  }

  if (out.reasons & (kHealthSpeed | kHealthGyro)) {
    out.state = HealthState::REJECTED;
    guarded_ = *guarded_ * held;
    out.consecutive_bad = ++consecutive_bad_;
  } else {
    guarded_ = *guarded_ * increment;
    consecutive_bad_ = 0;
    if (in.landmarks && static_cast<int>(*in.landmarks) < config_.min_landmarks) {
      out.state = HealthState::DEGRADED;
      out.reasons |= kHealthFewLandmarks;
    }
  }
  out.guarded_pose = guarded_;
  return out;
}

}  // namespace visual_slam
}  // namespace pq
