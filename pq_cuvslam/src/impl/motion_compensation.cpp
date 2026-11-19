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

#include "pq_cuvslam/impl/motion_compensation.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace pq
{
namespace visual_slam
{

namespace
{

Eigen::Matrix3d Exp(const Eigen::Vector3d & v)
{
  const double theta = v.norm();
  if (theta < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(theta, v / theta).toRotationMatrix();
}

}  // namespace

void GyroBuffer::Add(int64_t stamp_ns, const Eigen::Vector3d & angular_velocity)
{
  if (!samples_.empty() && stamp_ns <= samples_.back().stamp_ns) {
    return;  // out of order or duplicated
  }
  samples_.push_back({stamp_ns, angular_velocity});
  while (!samples_.empty() && samples_.front().stamp_ns < stamp_ns - keep_ns_) {
    samples_.pop_front();
  }
}

Eigen::Vector3d GyroBuffer::At(int64_t t) const
{
  if (t <= samples_.front().stamp_ns) {
    return samples_.front().w;
  }
  if (t >= samples_.back().stamp_ns) {
    return samples_.back().w;
  }
  const auto it = std::lower_bound(
    samples_.begin(), samples_.end(), t,
    [](const Sample & s, int64_t value) {return s.stamp_ns < value;});
  const Sample & b = *it;
  const Sample & a = *(it - 1);
  const double alpha = static_cast<double>(t - a.stamp_ns) / (b.stamp_ns - a.stamp_ns);
  return a.w + alpha * (b.w - a.w);
}

std::optional<Eigen::Matrix3d> GyroBuffer::Integrate(
  int64_t t0_ns, int64_t t1_ns, int64_t max_extrapolation_ns) const
{
  if (t0_ns == t1_ns) {
    return Eigen::Matrix3d::Identity();
  }
  if (samples_.empty()) {
    return std::nullopt;
  }
  const int64_t a = std::min(t0_ns, t1_ns);
  const int64_t b = std::max(t0_ns, t1_ns);
  if (a < samples_.front().stamp_ns - max_extrapolation_ns ||
    b > samples_.back().stamp_ns + max_extrapolation_ns)
  {
    return std::nullopt;
  }

  std::vector<int64_t> grid = {a};
  for (const auto & s : samples_) {
    if (s.stamp_ns > a && s.stamp_ns < b) {
      grid.push_back(s.stamp_ns);
    }
  }
  grid.push_back(b);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  for (size_t i = 0; i + 1 < grid.size(); ++i) {
    const double dt = (grid[i + 1] - grid[i]) * 1e-9;
    R = R * Exp(0.5 * (At(grid[i]) + At(grid[i + 1])) * dt);
  }
  // Integrated forward over [a, b]; backwards in time is the inverse rotation.
  return t1_ns > t0_ns ? R : Eigen::Matrix3d(R.transpose());
}

Eigen::Matrix3d RotationHomography(const Eigen::Matrix3d & K, const Eigen::Matrix3d & dR_camera)
{
  // A direction x in the camera at t_from is dR^T x in the camera at t_to.
  return K * dR_camera.transpose() * K.inverse();
}

Eigen::Matrix3d CameraRotation(
  const Eigen::Matrix3d & R_camera_imu, const Eigen::Matrix3d & dR_imu)
{
  return R_camera_imu * dR_imu * R_camera_imu.transpose();
}

sensor_msgs::msg::Image::SharedPtr WarpImage(
  const sensor_msgs::msg::Image & image, const Eigen::Matrix3d & H, int64_t stamp_ns)
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
  if (type < 0 || image.data.size() < static_cast<size_t>(image.step) * image.height) {
    return nullptr;
  }

  // The source buffer is only read.
  const cv::Mat src(
    static_cast<int>(image.height), static_cast<int>(image.width), type,
    const_cast<uint8_t *>(image.data.data()), image.step);

  auto out = std::make_shared<sensor_msgs::msg::Image>();
  out->header = image.header;
  out->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000);
  out->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000);
  out->height = image.height;
  out->width = image.width;
  out->encoding = image.encoding;
  out->is_bigendian = image.is_bigendian;
  out->step = image.width * static_cast<uint32_t>(CV_ELEM_SIZE(type));
  out->data.resize(static_cast<size_t>(out->step) * out->height);
  cv::Mat dst(src.rows, src.cols, type, out->data.data(), out->step);

  cv::Mat Hcv(3, 3, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      Hcv.at<double>(r, c) = H(r, c);
    }
  }
  // Depth must not be interpolated across edges.
  const int interpolation = type == CV_16UC1 ? cv::INTER_NEAREST : cv::INTER_LINEAR;
  cv::warpPerspective(src, dst, Hcv, src.size(), interpolation, cv::BORDER_CONSTANT, 0);
  return out;
}

double RotationAngleDeg(const Eigen::Matrix3d & R)
{
  return Eigen::AngleAxisd(R).angle() * 180.0 / M_PI;
}

}  // namespace visual_slam
}  // namespace pq
