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

#include "pq_cuvslam/impl/odometry_selector.hpp"

namespace
{

using pq::visual_slam::OdometrySelector;
using pq::visual_slam::SelectorConfig;

constexpr int64_t kFrameNs = 33'333'333;

tf2::Transform X(double x)
{
  return tf2::Transform(tf2::Quaternion::getIdentity(), tf2::Vector3(x, 0, 0));
}

// cam1 base frame pitched by 90 deg about y in the reference (cam2) base frame: a forward
// motion of the rig (+x in cam2) is +z in cam1... here: x_ref = R * x_cam1.
tf2::Transform Pitch90()
{
  tf2::Quaternion q;
  q.setRPY(0, M_PI / 2, 0);
  return tf2::Transform(q, tf2::Vector3(0, 0, -0.06));
}

TEST(OdometrySelector, FollowsPrimaryAndSwitches)
{
  SelectorConfig cfg;
  cfg.primary = "cam2";
  OdometrySelector s(cfg);
  s.AddTracker("cam2", tf2::Transform::getIdentity());
  s.AddTracker("cam1", Pitch90());
  // The rig moves +5 cm along x of the reference frame per frame. In the cam1 base frame the
  // same motion is R^T * (0.05, 0, 0).
  const tf2::Vector3 step_cam1 = tf2::quatRotate(Pitch90().getRotation().inverse(),
      tf2::Vector3(0.05, 0, 0));
  tf2::Transform p1 = tf2::Transform::getIdentity();
  double x2 = 0.0;
  std::optional<tf2::Transform> fused;
  for (int i = 0; i < 40; ++i) {
    const int64_t t = 1'000'000'000 + i * kFrameNs;
    const bool cam2_bad = i >= 10 && i < 20;
    x2 += cam2_bad ? 0.0 : 0.05;  // cam2 guarded: held while bad
    p1.setOrigin(p1.getOrigin() + step_cam1);
    auto r2 = s.Update("cam2", t, X(x2), cam2_bad);
    if (r2.fused) {fused = r2.fused;}
    auto r1 = s.Update("cam1", t + 1000, p1, false);
    if (r1.fused) {fused = r1.fused;}
    if (i == 12) {
      EXPECT_EQ(s.Selected(), "cam1");
    }
  }
  // Back on cam2 after 2 s of good frames? Only 20 frames (0.67 s) of good cam2 here: still cam1.
  EXPECT_EQ(s.Selected(), "cam1");
  ASSERT_TRUE(fused);
  // The fused odometry kept moving at 5 cm per frame through the cam2 failure, including the
  // frames before the switch (taken from cam1).
  EXPECT_NEAR(fused->getOrigin().x(), 0.05 * 39, 1e-6);
  EXPECT_NEAR(fused->getOrigin().z(), 0.0, 1e-6);
}

TEST(OdometrySelector, BackToPrimary)
{
  SelectorConfig cfg;
  cfg.primary = "cam1";
  OdometrySelector s(cfg);
  s.AddTracker("cam1", tf2::Transform::getIdentity());
  s.AddTracker("cam2", tf2::Transform::getIdentity());
  for (int i = 0; i < 200; ++i) {
    const int64_t t = 1'000'000'000 + i * kFrameNs;
    s.Update("cam1", t, X(0.05 * i), i >= 5 && i < 10);
    s.Update("cam2", t + 1000, X(0.05 * i), false);
    if (i == 9) {
      EXPECT_EQ(s.Selected(), "cam2");
    }
  }
  EXPECT_EQ(s.Selected(), "cam1");
}

}  // namespace
