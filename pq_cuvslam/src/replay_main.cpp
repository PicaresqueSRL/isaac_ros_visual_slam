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
// Bag replay: reads a ROS 2 bag and feeds its messages directly to VisualSlamNode, without
// DDS and without executor.
//
// Two modes:
//   deterministic (default, --rate 0): the messages are given to the node in recorded order,
//     as fast as the tracker processes them. No message is dropped; with
//     processing_mode:=offline the run is reproducible.
//   simulated online (--rate R > 0): the messages arrive at their recorded times (scaled by R)
//     in one keep-last queue per topic, with the history depth of the node subscriptions, and
//     the node takes them in arrival order. If the node does not keep up, the oldest messages
//     are dropped, as with live subscriptions. Use it with processing_mode:=realtime and, on a
//     PC, with latency_profile or emulated_latency_ms to emulate the Jetson.
//
// Usage:
//   pq_cuvslam_replay --bag <bag folder> [--rate R] [--max-frames N]
//     --ros-args <node parameters and remappings>
//
// The node parameters and remappings are the same as for the live node: the input topics are
// the resolved names of the node subscriptions (e.g. visual_slam/image_0 remapped to
// /cam2/infra1/image_rect_raw). /tf_static of the bag provides the camera and IMU frames.
// The node is configured and activated by this program (autostart is not used).

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "pq_cuvslam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

using pq::visual_slam::VisualSlamNode;

constexpr char kTfStaticTopic[] = "/tf_static";

struct Options
{
  std::string bag;
  double rate = 0.0;        // 0 = as fast as possible (deterministic)
  int64_t max_frames = -1;  // messages of the first image topic to replay; -1 = all
};

void PrintUsage()
{
  std::cerr <<
    "Usage: pq_cuvslam_replay --bag <bag folder> [--rate R] [--max-frames N] "
    "[--ros-args <node parameters and remappings>]\n"
    "  --rate R        0 (default): deterministic, as fast as possible, nothing dropped\n"
    "                  R > 0: simulated online at R times the recorded speed\n"
    "  --max-frames N  stop after N frames of the first image topic\n";
}

bool ParseOptions(const std::vector<std::string> & args, Options & options)
{
  for (size_t i = 1; i < args.size(); ++i) {
    if (args[i] == "--bag" && i + 1 < args.size()) {
      options.bag = args[++i];
    } else if (args[i] == "--rate" && i + 1 < args.size()) {
      options.rate = std::stod(args[++i]);
    } else if (args[i] == "--max-frames" && i + 1 < args.size()) {
      options.max_frames = std::stoll(args[++i]);
    } else if (args[i] == "-h" || args[i] == "--help") {
      return false;
    } else {
      std::cerr << "Unknown argument: " << args[i] << "\n";
      return false;
    }
  }
  return !options.bag.empty() && options.rate >= 0.0;
}

template<typename MsgT>
std::shared_ptr<MsgT> Deserialize(const rosbag2_storage::SerializedBagMessage & bag_msg)
{
  rclcpp::Serialization<MsgT> serialization;
  auto msg = std::make_shared<MsgT>();
  const rclcpp::SerializedMessage serialized(*bag_msg.serialized_data);
  serialization.deserialize_message(&serialized, msg.get());
  return msg;
}

// Feeds every /tf_static message of the bag to the node, so that the frames are available
// when the tracker is initialized, whatever their position in the bag.
size_t LoadStaticTransforms(const std::string & bag, VisualSlamNode & node)
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

// A deserialized input message, ready to be given to the node.
struct InputMessage
{
  enum class Kind {IMAGE, CAMERA_INFO, IMU};
  Kind kind;
  int index = 0;
  std::shared_ptr<const sensor_msgs::msg::Image> image;
  std::shared_ptr<const sensor_msgs::msg::CameraInfo> camera_info;
  std::shared_ptr<const sensor_msgs::msg::Imu> imu;
  bool vio_imu = false;              // IMU of the visual-inertial odometry
  std::vector<int> sync_devices;     // devices whose gyroscope this IMU topic is
  uint64_t sequence = 0;  // arrival order
};

void Deliver(VisualSlamNode & node, const InputMessage & msg)
{
  switch (msg.kind) {
    case InputMessage::Kind::IMAGE: node.InputImage(msg.index, msg.image); break;
    case InputMessage::Kind::CAMERA_INFO: node.InputCameraInfo(msg.index, msg.camera_info); break;
    case InputMessage::Kind::IMU:
      if (msg.vio_imu) {
        node.InputImu(msg.imu);
      }
      for (const int device : msg.sync_devices) {
        node.InputSyncImu(device, msg.imu);
      }
      break;
  }
}

// Input topics of the node and conversion of bag messages.
class InputTopics
{
public:
  explicit InputTopics(const VisualSlamNode & node)
  {
    for (const auto & [idx, topic] : node.GetImageTopics()) {
      images_[topic] = idx;
      depth_[topic] = node.GetImageQueueDepth();
    }
    for (const auto & [idx, topic] : node.GetCameraInfoTopics()) {
      camera_infos_[topic] = idx;
      depth_[topic] = node.GetImageQueueDepth();
    }
    imu_ = node.GetImuTopic();
    if (!imu_.empty()) {
      depth_[imu_] = node.GetImuQueueDepth();
    }
    for (const auto & [device, topic] : node.GetSyncImuTopics()) {
      sync_imu_[topic].push_back(device);
      depth_[topic] = node.GetImuQueueDepth();
    }
    frame_topic_ = node.GetImageTopics().front().second;
  }

  std::vector<std::string> All() const
  {
    std::vector<std::string> topics;
    for (const auto & [topic, depth] : depth_) {
      topics.push_back(topic);
    }
    return topics;
  }

  InputMessage Convert(const rosbag2_storage::SerializedBagMessage & bag_msg) const
  {
    InputMessage msg;
    const std::string & topic = bag_msg.topic_name;
    if (const auto it = images_.find(topic); it != images_.end()) {
      msg.kind = InputMessage::Kind::IMAGE;
      msg.index = it->second;
      msg.image = Deserialize<sensor_msgs::msg::Image>(bag_msg);
    } else if (camera_infos_.count(topic)) {
      msg.kind = InputMessage::Kind::CAMERA_INFO;
      msg.index = camera_infos_.at(topic);
      msg.camera_info = Deserialize<sensor_msgs::msg::CameraInfo>(bag_msg);
    } else {
      msg.kind = InputMessage::Kind::IMU;
      msg.imu = Deserialize<sensor_msgs::msg::Imu>(bag_msg);
      msg.vio_imu = topic == imu_;
      if (const auto it = sync_imu_.find(topic); it != sync_imu_.end()) {
        msg.sync_devices = it->second;
      }
    }
    return msg;
  }

  size_t Depth(const std::string & topic) const {return depth_.at(topic);}
  const std::string & FrameTopic() const {return frame_topic_;}

private:
  std::map<std::string, int> images_;
  std::map<std::string, int> camera_infos_;
  std::string imu_;
  std::map<std::string, std::vector<int>> sync_imu_;
  std::map<std::string, size_t> depth_;
  std::string frame_topic_;
};

struct ReplayStats
{
  int64_t frames = 0;  // messages of the frame topic read from the bag
  std::map<std::string, uint64_t> received;
  std::map<std::string, uint64_t> dropped;
  double wall_s = 0.0;
};

// Deterministic replay: every message, in recorded order, as fast as possible.
void ReplayDeterministic(
  rosbag2_cpp::Reader & reader, const InputTopics & topics, const Options & options,
  VisualSlamNode & node, ReplayStats & stats, const rclcpp::Logger & logger)
{
  auto last_log = std::chrono::steady_clock::now();
  while (reader.has_next() && rclcpp::ok()) {
    const auto bag_msg = reader.read_next();
    if (bag_msg->topic_name == topics.FrameTopic()) {
      if (options.max_frames >= 0 && stats.frames >= options.max_frames) {
        break;
      }
      ++stats.frames;
    }
    ++stats.received[bag_msg->topic_name];
    Deliver(node, topics.Convert(*bag_msg));

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log > std::chrono::seconds(10)) {
      RCLCPP_INFO(logger, "%ld frames replayed", stats.frames);
      last_log = now;
    }
  }
}

// Simulated online replay: a reader thread delivers the messages at their recorded times into
// keep-last queues; this thread takes them in arrival order and gives them to the node.
void ReplayOnline(
  rosbag2_cpp::Reader & reader, const InputTopics & topics, const Options & options,
  VisualSlamNode & node, ReplayStats & stats, const rclcpp::Logger & logger)
{
  std::mutex mutex;
  std::condition_variable cv;
  std::map<std::string, std::deque<InputMessage>> queues;
  bool reader_done = false;

  std::thread reader_thread([&]() {
      const auto start = std::chrono::steady_clock::now();
      int64_t first_recv_ns = -1;
      uint64_t sequence = 0;
      while (reader.has_next() && rclcpp::ok()) {
        const auto bag_msg = reader.read_next();
        const std::string topic = bag_msg->topic_name;
        if (topic == topics.FrameTopic()) {
          if (options.max_frames >= 0 && stats.frames >= options.max_frames) {
            break;
          }
          ++stats.frames;
        }
        InputMessage msg = topics.Convert(*bag_msg);
        msg.sequence = sequence++;

        // Wait for the recorded arrival time.
        if (first_recv_ns < 0) {
          first_recv_ns = bag_msg->recv_timestamp;
        }
        const auto due = start + std::chrono::nanoseconds(
          static_cast<int64_t>((bag_msg->recv_timestamp - first_recv_ns) / options.rate));
        std::this_thread::sleep_until(due);

        {
          std::lock_guard<std::mutex> lock(mutex);
          auto & queue = queues[topic];
          ++stats.received[topic];
          if (queue.size() >= topics.Depth(topic)) {
            queue.pop_front();  // keep-last: the oldest message is lost
            ++stats.dropped[topic];
          }
          queue.push_back(std::move(msg));
        }
        cv.notify_one();
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        reader_done = true;
      }
      cv.notify_one();
    });

  auto last_log = std::chrono::steady_clock::now();
  uint64_t delivered = 0;
  while (true) {
    InputMessage msg;
    {
      std::unique_lock<std::mutex> lock(mutex);
      const auto any_message = [&queues]() {
          for (const auto & [topic, queue] : queues) {
            if (!queue.empty()) {
              return true;
            }
          }
          return false;
        };
      cv.wait(lock, [&]() {return reader_done || any_message();});
      if (!any_message()) {
        break;  // reader done and queues empty
      }
      // Oldest message among the queue fronts.
      std::deque<InputMessage> * oldest = nullptr;
      for (auto & [topic, queue] : queues) {
        if (!queue.empty() && (!oldest || queue.front().sequence < oldest->front().sequence)) {
          oldest = &queue;
        }
      }
      msg = std::move(oldest->front());
      oldest->pop_front();
    }
    Deliver(node, msg);
    ++delivered;

    const auto now = std::chrono::steady_clock::now();
    if (now - last_log > std::chrono::seconds(10)) {
      RCLCPP_INFO(logger, "%lu messages delivered", delivered);
      last_log = now;
    }
  }
  reader_thread.join();
}

std::string StatsJson(const Options & options, const ReplayStats & stats)
{
  const auto counts = [](const std::map<std::string, uint64_t> & m) {
      std::ostringstream ss;
      ss << "{";
      bool first = true;
      for (const auto & [topic, n] : m) {
        ss << (first ? "" : ", ") << "\"" << topic << "\": " << n;
        first = false;
      }
      ss << "}";
      return ss.str();
    };
  std::ostringstream ss;
  ss << "{\"mode\": \"" << (options.rate > 0.0 ? "simulated_online" : "deterministic") << "\", " <<
    "\"rate\": " << options.rate << ", " <<
    "\"max_frames\": " << options.max_frames << ", " <<
    "\"frames_read\": " << stats.frames << ", " <<
    "\"wall_time_s\": " << stats.wall_s << ", " <<
    "\"messages_received\": " << counts(stats.received) << ", " <<
    "\"messages_dropped\": " << counts(stats.dropped) << "}";
  return ss.str();
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
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("autostart", false);
    auto node = std::make_shared<VisualSlamNode>(node_options);
    const rclcpp::Logger logger = node->get_logger().get_child("replay");

    if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      node->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_FATAL(logger, "The node could not be configured and activated");
      node->shutdown();
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    node->SetRunInput(options.bag);

    ReplayStats stats;
    try {
      const InputTopics topics(*node);
      const size_t num_tf_static = LoadStaticTransforms(options.bag, *node);
      RCLCPP_INFO(
        logger, "Bag: %s (%zu /tf_static messages), %s", options.bag.c_str(), num_tf_static,
        options.rate > 0.0 ?
        ("simulated online at rate " + std::to_string(options.rate)).c_str() :
        "deterministic");

      rosbag2_cpp::Reader reader;
      reader.open(options.bag);

      // Every input topic must be in the bag.
      std::set<std::string> bag_topics;
      for (const auto & t : reader.get_all_topics_and_types()) {
        bag_topics.insert(t.name);
      }
      for (const auto & topic : topics.All()) {
        if (bag_topics.count(topic) == 0) {
          throw std::runtime_error("Input topic not in the bag: " + topic);
        }
        RCLCPP_INFO(logger, "Input topic: %s (queue depth %zu)", topic.c_str(),
          topics.Depth(topic));
      }

      rosbag2_storage::StorageFilter filter;
      filter.topics = topics.All();
      reader.set_filter(filter);

      const auto start = std::chrono::steady_clock::now();
      if (options.rate > 0.0) {
        ReplayOnline(reader, topics, options, *node, stats, logger);
      } else {
        ReplayDeterministic(reader, topics, options, *node, stats, logger);
      }
      stats.wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

      if (!rclcpp::ok()) {
        RCLCPP_WARN(logger, "Interrupted after %ld frames", stats.frames);
        exit_code = EXIT_FAILURE;
      }
      uint64_t dropped = 0;
      for (const auto & [topic, n] : stats.dropped) {
        dropped += n;
      }
      RCLCPP_INFO(
        logger, "Replay done: %ld frames in %.2f s (%.1f frames/s), %lu messages dropped",
        stats.frames, stats.wall_s, stats.wall_s > 0 ? stats.frames / stats.wall_s : 0.0,
        dropped);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Replay failed: %s", e.what());
      exit_code = EXIT_FAILURE;
    }

    node->AddRunInfo("replay", StatsJson(options, stats));
    // Writes the run output and destroys the tracker.
    node->deactivate();
    node->cleanup();
    node->shutdown();
  }

  rclcpp::shutdown();
  return exit_code;
}
