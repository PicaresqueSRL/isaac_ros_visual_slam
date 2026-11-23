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
#include <stdexcept>

#include "pq_cuvslam/impl/virtual_camera.hpp"

namespace
{

using pq::visual_slam::ApplyVirtualCamera;
using pq::visual_slam::MakeVirtualCamera;

sensor_msgs::msg::CameraInfo MakeInfo(bool distorted)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 64;
  info.height = 48;
  info.distortion_model = "plumb_bob";
  info.d = distorted ? std::vector<double>{-0.05, 0.06, 0.0, 0.0, -0.02} :
  std::vector<double>(5, 0.0);
  info.k = {40.0, 0.0, 32.0, 0.0, 40.0, 24.0, 0.0, 0.0, 1.0};
  return info;
}

sensor_msgs::msg::Image MakeImage()
{
  sensor_msgs::msg::Image image;
  image.width = 64;
  image.height = 48;
  image.encoding = "mono8";
  image.step = 64;
  image.data.resize(64 * 48);
  for (uint32_t y = 0; y < 48; ++y) {
    for (uint32_t x = 0; x < 64; ++x) {
      image.data[y * 64 + x] = static_cast<uint8_t>(x * 3 + y);
    }
  }
  return image;
}

TEST(VirtualCamera, PadKeepsPixelsAndMovesPrincipalPoint)
{
  const auto v = MakeVirtualCamera(MakeInfo(false), false, 20);
  EXPECT_EQ(v.height, 68u);
  EXPECT_DOUBLE_EQ(v.info->k[5], 44.0);
  EXPECT_DOUBLE_EQ(v.info->p[6], 44.0);
  EXPECT_EQ(v.info->height, 68u);
  const auto image = MakeImage();
  const auto out = ApplyVirtualCamera(v, image);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->height, 68u);
  EXPECT_EQ(out->data[5 * 64 + 10], 0);                         // band
  EXPECT_EQ(out->data[(20 + 7) * 64 + 10], image.data[7 * 64 + 10]);   // same pixel, shifted
}

TEST(VirtualCamera, SideBandsWidenTheImage)
{
  // 64x48 camera widened to 80x48 (e.g. a 640x480 RGB next to 848x480 IR images).
  const auto v = MakeVirtualCamera(MakeInfo(false), false, 0, 0, 6, 10);
  EXPECT_EQ(v.width, 80u);
  EXPECT_EQ(v.height, 48u);
  EXPECT_EQ(v.info->width, 80u);
  EXPECT_DOUBLE_EQ(v.info->k[2], 38.0);
  EXPECT_DOUBLE_EQ(v.info->k[5], 24.0);
  const auto out = ApplyVirtualCamera(v, MakeImage());
  ASSERT_NE(out, nullptr);
  EXPECT_EQ(out->width, 80u);
  EXPECT_EQ(out->data[10 * 80 + 2], 0);                       // left band
  EXPECT_EQ(out->data[10 * 80 + 75], 0);                      // right band
  EXPECT_EQ(out->data[10 * 80 + 6 + 5], 5 * 3 + 10);          // pixel (5, 10) moved right by 6
}

TEST(VirtualCamera, NearestKeepsDepthValues)
{
  // Depth step between two columns: nearest sampling keeps only the two values.
  sensor_msgs::msg::Image depth;
  depth.width = 64;
  depth.height = 48;
  depth.encoding = "16UC1";
  depth.step = 128;
  depth.data.resize(128 * 48);
  auto * d = reinterpret_cast<uint16_t *>(depth.data.data());
  for (uint32_t i = 0; i < 64 * 48; ++i) {
    d[i] = (i % 64) < 32 ? 1000 : 3000;
  }
  const auto v = MakeVirtualCamera(MakeInfo(true), true, 0);
  const auto out = ApplyVirtualCamera(v, depth, true);
  ASSERT_NE(out, nullptr);
  const auto * o = reinterpret_cast<const uint16_t *>(out->data.data());
  for (uint32_t i = 0; i < out->width * out->height; ++i) {
    EXPECT_TRUE(o[i] == 0 || o[i] == 1000 || o[i] == 3000) << i << ": " << o[i];
  }
}

TEST(VirtualCamera, UndistortGivesPinhole)
{
  const auto v = MakeVirtualCamera(MakeInfo(true), true, 0);
  for (double d : v.info->d) {
    EXPECT_EQ(d, 0.0);
  }
  const auto image = MakeImage();
  const auto out = ApplyVirtualCamera(v, image);
  ASSERT_TRUE(out);
  // The principal point is a fixed point of the undistortion.
  EXPECT_NEAR(out->data[24 * 64 + 32], image.data[24 * 64 + 32], 1);
}

TEST(VirtualCamera, BandOnDistortedCameraNeedsUndistort)
{
  EXPECT_THROW(MakeVirtualCamera(MakeInfo(true), false, 10), std::invalid_argument);
}

TEST(VirtualCamera, WrongImageSize)
{
  const auto v = MakeVirtualCamera(MakeInfo(false), false, 10);
  auto image = MakeImage();
  image.width = 32;
  image.step = 32;
  EXPECT_FALSE(ApplyVirtualCamera(v, image));
}

}  // namespace
