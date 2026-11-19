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
// Rotation compensation of the capture time difference between two devices: an image taken
// at t_from is warped to how it would look at t_to, using the camera rotation integrated from
// the gyroscope of the same device (pure rotation model: H = K * dR^T * K^-1). The translation
// in the interval is not compensated.

#ifndef PQ_CUVSLAM__IMPL__MOTION_COMPENSATION_HPP_
#define PQ_CUVSLAM__IMPL__MOTION_COMPENSATION_HPP_

#include <cstdint>
#include <deque>
#include <optional>

#include "Eigen/Dense"
#include "sensor_msgs/msg/image.hpp"

namespace pq
{
namespace visual_slam
{

// Recent gyroscope samples of one device (stamps in the corrected time of the device).
class GyroBuffer
{
public:
  explicit GyroBuffer(int64_t keep_ns = 2'000'000'000)
  : keep_ns_(keep_ns) {}

  void Add(int64_t stamp_ns, const Eigen::Vector3d & angular_velocity);

  // Rotation of the IMU from t0 to t1, expressed in the IMU frame at t0 (t1 may precede t0).
  // Samples are linearly interpolated and integrated with the trapezoidal rule. Empty when the
  // samples do not cover [min(t0, t1), max(t0, t1)] (up to max_extrapolation_ns at the ends).
  std::optional<Eigen::Matrix3d> Integrate(
    int64_t t0_ns, int64_t t1_ns, int64_t max_extrapolation_ns = 10'000'000) const;

  bool Empty() const {return samples_.empty();}

private:
  struct Sample
  {
    int64_t stamp_ns;
    Eigen::Vector3d w;
  };
  Eigen::Vector3d At(int64_t t) const;

  int64_t keep_ns_;
  std::deque<Sample> samples_;
};

// Homography that maps a pixel of the image taken at t_from to the image at t_to, given the
// camera rotation dR_camera from t_from to t_to (in the camera frame at t_from).
Eigen::Matrix3d RotationHomography(const Eigen::Matrix3d & K, const Eigen::Matrix3d & dR_camera);

// Camera rotation from the IMU rotation: dR_camera = R_camera_imu * dR_imu * R_camera_imu^T.
Eigen::Matrix3d CameraRotation(
  const Eigen::Matrix3d & R_camera_imu, const Eigen::Matrix3d & dR_imu);

// Warps an image (mono8, mono16, 8UC1, 16UC1, rgb8, bgr8) with the homography; the result
// keeps all fields of the input except data, and takes the given stamp. Pixels outside the
// source image are set to 0. Returns null for unsupported encodings.
sensor_msgs::msg::Image::SharedPtr WarpImage(
  const sensor_msgs::msg::Image & image, const Eigen::Matrix3d & H, int64_t stamp_ns);

// Rotation angle in degrees.
double RotationAngleDeg(const Eigen::Matrix3d & R);

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__MOTION_COMPENSATION_HPP_
