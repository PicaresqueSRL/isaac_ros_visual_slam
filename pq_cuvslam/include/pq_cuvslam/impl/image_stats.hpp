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
// Statistics of input images for the diagnostics: brightness, saturated and dark pixels of
// intensity images, valid pixels of depth images. Computed on a subsample of the pixels.

#ifndef PQ_CUVSLAM__IMPL__IMAGE_STATS_HPP_
#define PQ_CUVSLAM__IMPL__IMAGE_STATS_HPP_

#include <cstdint>
#include <limits>

#include "sensor_msgs/msg/image.hpp"

namespace pq
{
namespace visual_slam
{

struct ImageStats
{
  bool depth = false;
  bool valid = false;  // false for unsupported encodings or empty images
  uint32_t samples = 0;
  // Intensity images, 0-255 scale (16-bit values divided by 257).
  float mean_intensity = std::numeric_limits<float>::quiet_NaN();
  float saturated_pct = std::numeric_limits<float>::quiet_NaN();
  float dark_pct = std::numeric_limits<float>::quiet_NaN();
  // Depth images.
  float valid_depth_pct = std::numeric_limits<float>::quiet_NaN();
};

// Thresholds on the 0-255 scale.
constexpr int kSaturatedLevel = 250;
constexpr int kDarkLevel = 5;

// Supported encodings: mono8/8UC1, mono16/16UC1 (intensity, or depth when depth is true),
// rgb8/bgr8 (luma), 32FC1 (depth). stride >= 1: one pixel every stride in both directions.
ImageStats ComputeImageStats(const sensor_msgs::msg::Image & image, bool depth, int stride);

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__IMAGE_STATS_HPP_
