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

#include "pq_cuvslam/impl/image_stats.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "sensor_msgs/image_encodings.hpp"

namespace pq
{
namespace visual_slam
{

namespace
{

template<typename T>
T Read(const uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

}  // namespace

ImageStats ComputeImageStats(const sensor_msgs::msg::Image & image, bool depth, int stride)
{
  namespace enc = sensor_msgs::image_encodings;
  ImageStats s;
  s.depth = depth;
  stride = std::max(1, stride);
  const std::string & e = image.encoding;
  int bytes = 0;
  int channels = 1;
  if (e == enc::MONO8 || e == enc::TYPE_8UC1) {
    bytes = 1;
  } else if (e == enc::MONO16 || e == enc::TYPE_16UC1) {
    bytes = 2;
  } else if (e == enc::TYPE_32FC1) {
    bytes = 4;
  } else if (e == enc::RGB8 || e == enc::BGR8) {
    bytes = 1;
    channels = 3;
  }
  if (bytes == 0 || image.width == 0 || image.height == 0 ||
    image.data.size() < static_cast<size_t>(image.step) * image.height ||
    (bytes == 4 && !depth))
  {
    return s;
  }

  const uint8_t * data = image.data.data();
  uint64_t samples = 0, sum = 0, saturated = 0, dark = 0, valid = 0;
  for (uint32_t y = 0; y < image.height; y += stride) {
    const uint8_t * row = data + static_cast<size_t>(y) * image.step;
    for (uint32_t x = 0; x < image.width; x += stride) {
      const uint8_t * p = row + static_cast<size_t>(x) * bytes * channels;
      ++samples;
      if (depth) {
        bool ok = false;
        if (bytes == 2) {
          ok = Read<uint16_t>(p) != 0;
        } else if (bytes == 4) {
          const float d = Read<float>(p);
          ok = std::isfinite(d) && d > 0.0f;
        } else {
          ok = *p != 0;
        }
        valid += ok ? 1 : 0;
        continue;
      }
      int v = 0;
      if (channels == 3) {
        // Luma (BT.601), same weights for RGB and BGR up to the red/blue order.
        const int r = e == enc::RGB8 ? p[0] : p[2];
        const int b = e == enc::RGB8 ? p[2] : p[0];
        v = (299 * r + 587 * p[1] + 114 * b) / 1000;
      } else if (bytes == 2) {
        v = Read<uint16_t>(p) / 257;
      } else {
        v = *p;
      }
      sum += v;
      saturated += v >= kSaturatedLevel ? 1 : 0;
      dark += v <= kDarkLevel ? 1 : 0;
    }
  }
  if (samples == 0) {
    return s;
  }
  s.valid = true;
  s.samples = static_cast<uint32_t>(samples);
  const double n = static_cast<double>(samples);
  if (depth) {
    s.valid_depth_pct = static_cast<float>(100.0 * valid / n);
  } else {
    s.mean_intensity = static_cast<float>(sum / n);
    s.saturated_pct = static_cast<float>(100.0 * saturated / n);
    s.dark_pct = static_cast<float>(100.0 * dark / n);
  }
  return s;
}

}  // namespace visual_slam
}  // namespace pq
