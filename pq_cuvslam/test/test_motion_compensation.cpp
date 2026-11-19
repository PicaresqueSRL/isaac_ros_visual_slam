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
#include <cstdint>

#include "pq_cuvslam/impl/motion_compensation.hpp"

namespace
{

using pq::visual_slam::CameraRotation;
using pq::visual_slam::GyroBuffer;
using pq::visual_slam::RotationAngleDeg;
using pq::visual_slam::RotationHomography;
using pq::visual_slam::WarpImage;

constexpr int64_t kMs = 1'000'000;

GyroBuffer ConstantRate(const Eigen::Vector3d & w)
{
  GyroBuffer gyro;
  for (int64_t t = 0; t <= 1000; t += 5) {
    gyro.Add(t * kMs, w);
  }
  return gyro;
}

TEST(MotionCompensation, IntegratesConstantRate)
{
  const GyroBuffer gyro = ConstantRate(Eigen::Vector3d(0.0, 1.0, 0.0));  // 1 rad/s about y
  const auto R = gyro.Integrate(100 * kMs, 150 * kMs);
  ASSERT_TRUE(R.has_value());
  EXPECT_NEAR(RotationAngleDeg(*R), 0.05 * 180.0 / M_PI, 1e-6);
  const Eigen::AngleAxisd aa(*R);
  EXPECT_NEAR(aa.axis().y(), 1.0, 1e-9);
}

TEST(MotionCompensation, BackwardsIsInverse)
{
  const GyroBuffer gyro = ConstantRate(Eigen::Vector3d(0.3, -0.2, 0.5));
  const auto forward = gyro.Integrate(200 * kMs, 230 * kMs);
  const auto backward = gyro.Integrate(230 * kMs, 200 * kMs);
  ASSERT_TRUE(forward && backward);
  EXPECT_TRUE((*forward * *backward).isIdentity(1e-12));
}

TEST(MotionCompensation, MissingSamples)
{
  const GyroBuffer gyro = ConstantRate(Eigen::Vector3d(0.0, 1.0, 0.0));
  EXPECT_FALSE(gyro.Integrate(990 * kMs, 1100 * kMs).has_value());
  EXPECT_FALSE(GyroBuffer().Integrate(0, 10 * kMs).has_value());
}

TEST(MotionCompensation, HomographyDirection)
{
  // Camera (OpenCV axes) turning right (positive rotation about y) between t_from and t_to:
  // the scene moves left in the image, so a pixel at the center goes to a smaller u.
  Eigen::Matrix3d K;
  K << 400, 0, 320, 0, 400, 240, 0, 0, 1;
  const Eigen::Matrix3d dR(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY()));
  const Eigen::Matrix3d H = RotationHomography(K, dR);
  const Eigen::Vector3d p = H * Eigen::Vector3d(320, 240, 1);
  EXPECT_NEAR(p.x() / p.z(), 320 - 400 * std::tan(0.05), 1e-6);
  EXPECT_NEAR(p.y() / p.z(), 240, 1e-6);
}

TEST(MotionCompensation, CameraRotationFromImu)
{
  // IMU x axis = camera y axis: a rotation about the IMU x axis is about the camera y axis.
  Eigen::Matrix3d R_camera_imu;
  R_camera_imu << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d dR_imu(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()));
  const Eigen::AngleAxisd aa(CameraRotation(R_camera_imu, dR_imu));
  EXPECT_NEAR(aa.angle(), 0.1, 1e-12);
  EXPECT_NEAR(std::abs(aa.axis().y()), 1.0, 1e-12);
}

TEST(MotionCompensation, WarpShiftsImage)
{
  sensor_msgs::msg::Image image;
  image.height = 20;
  image.width = 20;
  image.encoding = "mono8";
  image.step = 20;
  image.data.resize(400);
  image.data.data()[10 * 20 + 10] = 255;  // bright pixel at (10, 10)
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  H(0, 2) = 3.0;  // shift right by 3 px
  const auto out = WarpImage(image, H, 1500 * kMs);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->data.data()[10 * 20 + 13], 255);
  EXPECT_EQ(out->header.stamp.sec, 1);
  EXPECT_EQ(out->header.stamp.nanosec, 500'000'000u);
}

}  // namespace
