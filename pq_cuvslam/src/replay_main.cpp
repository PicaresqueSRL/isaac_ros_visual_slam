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
// Deterministic bag replay: reads a ROS 2 bag and feeds its messages directly to
// VisualSlamNode, in the order in which they were recorded, as fast as the tracker processes
// them. No message is dropped and no executor is involved, so with processing_mode:=offline
// the run is reproducible.
//
// Usage:
//   pq_cuvslam_replay --bag <bag folder> [--max-frames N] --ros-args <node parameters/remaps>
//
// The node parameters and remappings are the same as for the live node: the input topics are
// the resolved names of the node subscriptions (e.g. visual_slam/image_0 remapped to
// /cam2/infra1/image_rect_raw). /tf_static of the bag provides the camera and IMU frames.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "pq_cuvslam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

constexpr char kTfStaticTopic[] = "/tf_static";

struct Options
{
  std::string bag;
  int64_t max_frames = -1;  // number of messages of the first image topic to replay; -1 = all
};

void PrintUsage()
{
  std::cerr <<
    "Usage: pq_cuvslam_replay --bag <bag folder> [--max-frames N] "
    "[--ros-args <node parameters and remappings>]\n";
}

bool ParseOptions(const std::vector<std::string> & args, Options & options)
{
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--bag" && i + 1 < args.size()) {
      options.bag = args[++i];
    } else if (args[i] == "--max-frames" && i + 1 < args.size()) {
      options.max_frames = std::stoll(args[++i]);
    } else if (args[i] == "-h" || args[i] == "--help") {
      return false;
    } else {
      std::cerr << "Unknown argument: " << args[i] << "\n";
      return false;
    }
  }
  return !options.bag.empty();
}

template<typename MsgT>
std::shared_ptr<MsgT> Deserialize(const rosbag2_storage::SerializedBagMessage & bag_msg)
{
  static rclcpp::Serialization<MsgT> serialization;
  auto msg = std::make_shared<MsgT>();
  const rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
  serialization.deserialize_message(&serialized, msg.get());
  return msg;
}

// Feeds every /tf_static message of the bag to the node, so that the frames are available
// when the tracker is initialized, whatever their position in the bag.
size_t LoadStaticTransforms(const std::string & bag, pq::visual_slam::VisualSlamNode & node)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {kTfStaticTopic};
  reader.set_filter(filter);
  size_t count = 0;
  while (reader.has_next()) {
    const auto bag_msg = reader.read_next();
    node.InputStaticTransforms(*Deserialize<tf2_msgs::msg::TFMessage>(*bag_msg));
    ++count;
  }
  return count;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

  Options options;
  if (!ParseOptions(args, options)) {
    PrintUsage();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  int exit_code = EXIT_SUCCESS;
  {
    auto node = std::make_shared<pq::visual_slam::VisualSlamNode>(rclcpp::NodeOptions());
    const rclcpp::Logger logger = node->get_logger().get_child("replay");
    node->SetRunInput(options.bag);

    // Input topics: resolved names of the node subscriptions.
    std::map<std::string, int> image_topics;
    std::map<std::string, int> camera_info_topics;
    for (const auto & [idx, topic] : node->GetImageTopics()) {
      image_topics[topic] = idx;
    }
    for (const auto & [idx, topic] : node->GetCameraInfoTopics()) {
      camera_info_topics[topic] = idx;
    }
    const std::string imu_topic = node->GetImuTopic();

    std::vector<std::string> topics;
    for (const auto & [topic, idx] : image_topics) {
      topics.push_back(topic);
    }
    for (const auto & [topic, idx] : camera_info_topics) {
      topics.push_back(topic);
    }
    if (!imu_topic.empty()) {
      topics.push_back(imu_topic);
    }

    try {
      const size_t num_tf_static = LoadStaticTransforms(options.bag, *node);
      RCLCPP_INFO(logger, "Bag: %s (%zu /tf_static messages)", options.bag.c_str(),
        num_tf_static);

      rosbag2_cpp::Reader reader;
      reader.open(options.bag);

      // Every input topic must be in the bag.
      std::set<std::string> bag_topics;
      for (const auto & t : reader.get_all_topics_and_types()) {
        bag_topics.insert(t.name);
      }
      for (const auto & topic : topics) {
        if (bag_topics.count(topic) == 0) {
          throw std::runtime_error("Input topic not in the bag: " + topic);
        }
        RCLCPP_INFO(logger, "Input topic: %s", topic.c_str());
      }

      rosbag2_storage::StorageFilter filter;
      filter.topics = topics;
      reader.set_filter(filter);

      // The first image topic counts the frames (for --max-frames and the progress log).
      const std::string frame_topic = node->GetImageTopics().front().second;
      int64_t frames = 0;
      size_t messages = 0;
      const auto start = std::chrono::steady_clock::now();
      auto last_log = start;

      while (reader.has_next() && rclcpp::ok()) {
        const auto bag_msg = reader.read_next();
        const std::string & topic = bag_msg->topic_name;
        if (topic == frame_topic) {
          if (options.max_frames >= 0 && frames >= options.max_frames) {
            break;
          }
          ++frames;
        }
        const auto image_it = image_topics.find(topic);
        const auto info_it = camera_info_topics.find(topic);
        if (image_it != image_topics.end()) {
          node->InputImage(image_it->second, Deserialize<sensor_msgs::msg::Image>(*bag_msg));
        } else if (info_it != camera_info_topics.end()) {
          node->InputCameraInfo(
            info_it->second, Deserialize<sensor_msgs::msg::CameraInfo>(*bag_msg));
        } else if (topic == imu_topic) {
          node->InputImu(Deserialize<sensor_msgs::msg::Imu>(*bag_msg));
        }
        ++messages;

        const auto now = std::chrono::steady_clock::now();
        if (now - last_log > std::chrono::seconds(10)) {
          RCLCPP_INFO(logger, "%ld frames replayed", frames);
          last_log = now;
        }
      }

      const double wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (!rclcpp::ok()) {
        RCLCPP_WARN(logger, "Interrupted after %ld frames", frames);
        exit_code = EXIT_FAILURE;
      }
      RCLCPP_INFO(
        logger, "Replay done: %ld frames, %zu messages in %.2f s (%.1f frames/s)",
        frames, messages, wall_s, wall_s > 0 ? frames / wall_s : 0.0);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Replay failed: %s", e.what());
      exit_code = EXIT_FAILURE;
    }

    node->WriteRunOutput();
  }

  rclcpp::shutdown();
  return exit_code;
}
