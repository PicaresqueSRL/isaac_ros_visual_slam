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
// Virtual camera given to cuVSLAM instead of a physical one: the image undistorted to a
// pinhole camera with the same focal length (cuNLS, used by the Multisensor mode, supports
// pinhole cameras only), optionally extended with empty bands at the top (principal point
// moved down by the same amount) and at the bottom. Every real pixel keeps its viewing ray, so
// the geometry is exact; the extended field of view lets the frustum overlap check of cuVSLAM
// (> 50 % of the points of one camera visible in the other one, at 2-4 m; all cameras with the
// same resolution) accept two cameras whose real views overlap less, e.g. cam1 looking 49 deg
// lower than cam2. The bands are excluded from the feature search with the camera borders.

#ifndef PQ_CUVSLAM__IMPL__VIRTUAL_CAMERA_HPP_
#define PQ_CUVSLAM__IMPL__VIRTUAL_CAMERA_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "opencv2/core.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pq
{
namespace visual_slam
{

struct VirtualCamera
{
  int pad_top = 0;
  int pad_bottom = 0;
  uint32_t width = 0;
  uint32_t height = 0;           // with the bands
  cv::Mat map1, map2;            // remap tables, virtual pixel -> source pixel
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;   // virtual intrinsics (pinhole)
};

// Builds the virtual camera of a physical one (plumb_bob, rational_polynomial or pinhole
// calibration). Throws std::invalid_argument for other distortion models.
VirtualCamera MakeVirtualCamera(
  const sensor_msgs::msg::CameraInfo & info, bool undistort, int pad_top, int pad_bottom = 0);

// The image in the virtual camera (mono8, mono16, rgb8, bgr8); nullptr for other encodings or
// sizes different from the calibration.
std::shared_ptr<sensor_msgs::msg::Image> ApplyVirtualCamera(
  const VirtualCamera & camera, const sensor_msgs::msg::Image & image);

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__VIRTUAL_CAMERA_HPP_
