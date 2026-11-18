// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
// Modified by Picaresque SRL: derived from isaac_ros_common/qos.hpp
// (isaac_ros_common, release-5.1) so that the node does not depend on Isaac ROS.
// Behaviour is unchanged.

#ifndef ISAAC_ROS_CUVSLAM__IMPL__QOS_HPP_
#define ISAAC_ROS_CUVSLAM__IMPL__QOS_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// Declares the parameters `<parameter_name>` (QoS profile name) and
// `<parameter_name>_depth` on the node and returns the resulting QoS.
rclcpp::QoS AddQosParameter(
  rclcpp::Node & node,
  std::string default_qos = "SYSTEM_DEFAULT",
  std::string parameter_name = "qos",
  const int default_depth = 0);

// Converts a profile name (SYSTEM_DEFAULT, DEFAULT, PARAMETER_EVENTS,
// SERVICES_DEFAULT, PARAMETERS, SENSOR_DATA; case-insensitive) to a QoS.
// Unknown names fall back to DEFAULT with a warning. A depth of 0 keeps the
// depth of the default profile.
rclcpp::QoS ParseQosString(const std::string & str, const int depth = 0);

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_CUVSLAM__IMPL__QOS_HPP_
