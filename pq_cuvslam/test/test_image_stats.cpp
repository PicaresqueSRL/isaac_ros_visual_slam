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
#include <cstring>
#include <string>

#include "pq_cuvslam/impl/image_stats.hpp"

namespace
{

using pq::visual_slam::ComputeImageStats;

sensor_msgs::msg::Image MakeImage(const std::string & encoding, int bytes_per_pixel)
{
  sensor_msgs::msg::Image image;
  image.width = 10;
  image.height = 10;
  image.encoding = encoding;
  image.step = 10 * bytes_per_pixel;
  image.data.resize(image.step * image.height);
  return image;
}

TEST(ImageStats, Mono8)
{
  auto image = MakeImage("mono8", 1);
  uint8_t * p = image.data.data();
  for (int i = 0; i < 100; ++i) {
    p[i] = i < 30 ? 255 : (i < 40 ? 0 : 100);  // 30 saturated, 10 dark, 60 mid
  }
  const auto s = ComputeImageStats(image, false, 1);
  ASSERT_TRUE(s.valid);
  EXPECT_EQ(s.samples, 100u);
  EXPECT_NEAR(s.saturated_pct, 30.0f, 1e-4);
  EXPECT_NEAR(s.dark_pct, 10.0f, 1e-4);
  EXPECT_NEAR(s.mean_intensity, (30 * 255 + 60 * 100) / 100.0f, 1e-3);
  EXPECT_TRUE(std::isnan(s.valid_depth_pct));
}

TEST(ImageStats, Stride)
{
  auto image = MakeImage("mono8", 1);
  const auto s = ComputeImageStats(image, false, 3);  // rows and columns 0, 3, 6, 9
  EXPECT_EQ(s.samples, 16u);
  EXPECT_NEAR(s.dark_pct, 100.0f, 1e-4);
}

TEST(ImageStats, Depth16)
{
  auto image = MakeImage("16UC1", 2);
  for (int i = 0; i < 100; ++i) {
    const uint16_t v = i < 75 ? 1200 : 0;
    std::memcpy(image.data.data() + 2 * i, &v, 2);
  }
  const auto s = ComputeImageStats(image, true, 1);
  ASSERT_TRUE(s.valid);
  EXPECT_NEAR(s.valid_depth_pct, 75.0f, 1e-4);
  EXPECT_TRUE(std::isnan(s.saturated_pct));
}

TEST(ImageStats, Rgb8Luma)
{
  auto image = MakeImage("rgb8", 3);
  std::memset(image.data.data(), 255, image.data.size());
  const auto s = ComputeImageStats(image, false, 1);
  EXPECT_NEAR(s.saturated_pct, 100.0f, 1e-4);
}

TEST(ImageStats, UnsupportedEncoding)
{
  auto image = MakeImage("yuv422", 2);
  EXPECT_FALSE(ComputeImageStats(image, false, 1).valid);
}

}  // namespace
