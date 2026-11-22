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

#include <gtest/gtest.h>

#include <cmath>

#include "pq_cuvslam/impl/tracking_health.hpp"

namespace
{

using pq::visual_slam::HealthInput;
using pq::visual_slam::HealthState;
using pq::visual_slam::TrackingHealth;

constexpr int64_t kFrameNs = 33'333'333;

tf2::Transform At(double x, double yaw_deg = 0.0)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_deg * M_PI / 180.0);
  return tf2::Transform(q, tf2::Vector3(x, 0, 0));
}

HealthInput Frame(int i, std::optional<tf2::Transform> pose, size_t landmarks = 100)
{
  HealthInput in;
  in.stamp_ns = 1'000'000'000 + i * kFrameNs;
  in.pose = pose;
  in.landmarks = landmarks;
  return in;
}

TEST(TrackingHealth, WalkingIsAccepted)
{
  TrackingHealth h;
  for (int i = 0; i < 30; ++i) {
    const auto out = h.Update(Frame(i, At(0.05 * i)));   // 1.5 m/s
    EXPECT_EQ(out.state, HealthState::OK);
    ASSERT_TRUE(out.guarded_pose);
    EXPECT_NEAR(out.guarded_pose->getOrigin().x(), 0.05 * i, 1e-9);
  }
}

TEST(TrackingHealth, JumpIsRejectedAndNotCarried)
{
  TrackingHealth h;
  for (int i = 0; i < 10; ++i) {
    h.Update(Frame(i, At(0.05 * i)));
  }
  // The tracker jumps by 45 m, then continues from there.
  auto out = h.Update(Frame(10, At(0.45 + 45.0)));
  EXPECT_EQ(out.state, HealthState::REJECTED);
  EXPECT_GT(out.speed_mps, 1000.0);
  EXPECT_NEAR(out.guarded_pose->getOrigin().x(), 0.45, 1e-9);   // held
  out = h.Update(Frame(11, At(0.50 + 45.0)));
  EXPECT_EQ(out.state, HealthState::OK);
  EXPECT_NEAR(out.guarded_pose->getOrigin().x(), 0.50, 1e-9);   // jump removed
  EXPECT_EQ(out.consecutive_bad, 0);
}

TEST(TrackingHealth, RotationAgainstGyro)
{
  TrackingHealth h;
  h.Update(Frame(0, At(0.0)));
  auto in = Frame(1, At(0.05, 20.0));       // the tracker turns by 20 deg
  in.gyro_rotation = tf2::Quaternion::getIdentity();   // the gyroscope does not
  auto out = h.Update(in);
  EXPECT_EQ(out.state, HealthState::REJECTED);
  EXPECT_NEAR(out.gyro_error_deg, 20.0, 1e-6);
  // Rotation of the held frame from the gyroscope: none here.
  EXPECT_NEAR(out.guarded_pose->getOrigin().x(), 0.0, 1e-9);
  EXPECT_NEAR(out.guarded_pose->getRotation().getAngle(), 0.0, 1e-9);

  in = Frame(2, At(0.10, 20.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.0);
  in.gyro_rotation = q;
  out = h.Update(in);
  EXPECT_EQ(out.state, HealthState::OK);
}

TEST(TrackingHealth, LostAndFewLandmarks)
{
  TrackingHealth h;
  h.Update(Frame(0, At(0.0)));
  auto out = h.Update(Frame(1, std::nullopt));
  EXPECT_EQ(out.state, HealthState::LOST);
  EXPECT_EQ(out.consecutive_bad, 1);
  out = h.Update(Frame(2, At(0.05), 3));
  EXPECT_EQ(out.state, HealthState::DEGRADED);
  EXPECT_EQ(pq::visual_slam::HealthReasons(out.reasons), "few_landmarks");
}

}  // namespace
