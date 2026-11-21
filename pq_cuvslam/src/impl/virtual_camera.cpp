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

#include "pq_cuvslam/impl/virtual_camera.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace pq
{
namespace visual_slam
{

VirtualCamera MakeVirtualCamera(
  const sensor_msgs::msg::CameraInfo & info, bool undistort, int pad_top, int pad_bottom)
{
  namespace dm = sensor_msgs::distortion_models;
  if (info.width == 0 || info.height == 0) {
    throw std::invalid_argument("virtual camera: camera_info without image size");
  }
  pad_top = std::max(0, pad_top);
  pad_bottom = std::max(0, pad_bottom);
  const bool distorted = std::any_of(
    info.d.begin(), info.d.end(), [](double v) {return v != 0.0;});
  if (distorted && !undistort && (pad_top > 0 || pad_bottom > 0)) {
    throw std::invalid_argument(
            "virtual camera: the bands need undistort (the distortion model cannot be extended "
            "beyond the image)");
  }
  if (distorted && undistort && info.distortion_model != dm::PLUMB_BOB &&
    info.distortion_model != dm::RATIONAL_POLYNOMIAL)
  {
    throw std::invalid_argument(
            "virtual camera: undistort supports plumb_bob and rational_polynomial, not " +
            info.distortion_model);
  }

  VirtualCamera v;
  v.pad_top = pad_top;
  v.pad_bottom = pad_bottom;
  v.width = info.width;
  v.height = info.height + static_cast<uint32_t>(pad_top + pad_bottom);

  const cv::Matx33d K(info.k[0], info.k[1], info.k[2], info.k[3], info.k[4], info.k[5],
    info.k[6], info.k[7], info.k[8]);
  cv::Matx33d K_virtual = K;
  K_virtual(1, 2) += pad_top;
  std::vector<double> D(info.d.begin(), info.d.end());
  if (!undistort || !distorted) {
    D.assign(5, 0.0);
  }
  cv::initUndistortRectifyMap(
    K, D, cv::Matx33d::eye(), K_virtual,
    cv::Size(static_cast<int>(v.width), static_cast<int>(v.height)), CV_16SC2, v.map1, v.map2);

  auto out = std::make_shared<sensor_msgs::msg::CameraInfo>(info);
  out->height = v.height;
  out->distortion_model = dm::PLUMB_BOB;
  out->d.assign(5, 0.0);   // all zeros: pinhole for cuVSLAM
  for (int i = 0; i < 9; ++i) {
    out->k[i] = K_virtual.val[i];
    out->r[i] = (i % 4 == 0) ? 1.0 : 0.0;
  }
  out->p = {K_virtual(0, 0), K_virtual(0, 1), K_virtual(0, 2), 0.0,
    K_virtual(1, 0), K_virtual(1, 1), K_virtual(1, 2), 0.0,
    0.0, 0.0, 1.0, 0.0};
  out->roi = sensor_msgs::msg::RegionOfInterest();
  v.info = out;
  return v;
}

std::shared_ptr<sensor_msgs::msg::Image> ApplyVirtualCamera(
  const VirtualCamera & camera, const sensor_msgs::msg::Image & image)
{
  namespace enc = sensor_msgs::image_encodings;
  int type = -1;
  if (image.encoding == enc::MONO8 || image.encoding == enc::TYPE_8UC1) {
    type = CV_8UC1;
  } else if (image.encoding == enc::MONO16 || image.encoding == enc::TYPE_16UC1) {
    type = CV_16UC1;
  } else if (image.encoding == enc::RGB8 || image.encoding == enc::BGR8) {
    type = CV_8UC3;
  }
  if (type < 0 || image.width != camera.width ||
    image.height + static_cast<uint32_t>(camera.pad_top + camera.pad_bottom) != camera.height ||
    image.data.size() < static_cast<size_t>(image.step) * image.height)
  {
    return nullptr;
  }
  const cv::Mat src(
    static_cast<int>(image.height), static_cast<int>(image.width), type,
    const_cast<uint8_t *>(image.data.data()), image.step);
  auto out = std::make_shared<sensor_msgs::msg::Image>();
  out->header = image.header;
  out->encoding = image.encoding;
  out->is_bigendian = image.is_bigendian;
  out->width = camera.width;
  out->height = camera.height;
  out->step = static_cast<uint32_t>(camera.width * CV_ELEM_SIZE(type));
  out->data.resize(static_cast<size_t>(out->step) * out->height);
  cv::Mat dst(
    static_cast<int>(out->height), static_cast<int>(out->width), type, out->data.data(),
    out->step);
  cv::remap(src, dst, camera.map1, camera.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
  return out;
}

}  // namespace visual_slam
}  // namespace pq
