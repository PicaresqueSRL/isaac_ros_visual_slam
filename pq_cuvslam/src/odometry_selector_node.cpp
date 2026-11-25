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
// Odometry selector node (impl/odometry_selector.hpp): subscribes to the guarded odometry and
// the frame diagnostics of two or more pq_cuvslam nodes (one per camera) and publishes the
// odometry of the selected one, in the base frame of the reference camera.
//
// Parameters:
//   trackers          names, e.g. [cam1, cam2]
//   namespaces        topic prefix of each pq_cuvslam node, e.g. [/cam1_vio, /cam2_vio]
//   primary           tracker used whenever it is healthy
//   ref_from_<name>   [x, y, z, roll, pitch, yaw] of the base frame of the tracker in the
//                     reference base frame (m, rad); default identity
//   reference_frame   child frame of the fused odometry (base frame of the reference camera)
//   odom_frame        frame of the fused odometry
//   bad_window_s, bad_frames, back_after_s   see SelectorConfig
//   run_output_dir    if set: odometry.csv (fused) and selection.csv written at shutdown
// Topics: visual_slam/tracking/odometry_fused, visual_slam/selector/active (transient local),
// visual_slam/events (camera_switch).

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "pq_cuvslam/impl/odometry_selector.hpp"
#include "pq_cuvslam_interfaces/msg/frame_diagnostics.hpp"
#include "pq_cuvslam_interfaces/msg/slam_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pq
{
namespace visual_slam
{

class OdometrySelectorNode : public rclcpp::Node
{
public:
  explicit OdometrySelectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("odometry_selector", options)
  {
    const auto names = declare_parameter<std::vector<std::string>>(
      "trackers", std::vector<std::string>{"cam1", "cam2"});
    const auto namespaces = declare_parameter<std::vector<std::string>>(
      "namespaces", std::vector<std::string>{"/cam1_vio", "/cam2_vio"});
    if (names.size() < 2 || names.size() != namespaces.size()) {
      throw std::invalid_argument(
            "odometry_selector: trackers and namespaces: 2 or more, same size");
    }
    SelectorConfig cfg;
    cfg.primary = declare_parameter<std::string>("primary", names.front());
    cfg.bad_window_s = declare_parameter<double>("bad_window_s", cfg.bad_window_s);
    cfg.bad_frames = declare_parameter<int>("bad_frames", cfg.bad_frames);
    cfg.back_after_s = declare_parameter<double>("back_after_s", cfg.back_after_s);
    reference_frame_ = declare_parameter<std::string>("reference_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    run_output_dir_ = declare_parameter<std::string>("run_output_dir", "");
    selector_ = OdometrySelector(cfg);

    for (size_t i = 0; i < names.size(); ++i) {
      const auto v = declare_parameter<std::vector<double>>(
        "ref_from_" + names[i], std::vector<double>{0, 0, 0, 0, 0, 0});
      if (v.size() != 6) {
        throw std::invalid_argument("ref_from_" + names[i] + ": expected 6 values");
      }
      tf2::Quaternion q;
      q.setRPY(v[3], v[4], v[5]);
      selector_.AddTracker(names[i], tf2::Transform(q, tf2::Vector3(v[0], v[1], v[2])));
      const std::string ns = namespaces[i];
      const std::string name = names[i];
      odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          ns + "/visual_slam/tracking/odometry_guarded", rclcpp::QoS(50),
          [this, name](const nav_msgs::msg::Odometry::ConstSharedPtr & m) {OnOdometry(name, m);}));
      diag_subs_.push_back(create_subscription<pq_cuvslam_interfaces::msg::FrameDiagnostics>(
          ns + "/visual_slam/diagnostics/frame", rclcpp::QoS(50),
          [this, name](const pq_cuvslam_interfaces::msg::FrameDiagnostics::ConstSharedPtr & m) {
            OnDiagnostics(name, m);
          }));
    }
    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "visual_slam/tracking/odometry_fused", rclcpp::QoS(50));
    active_pub_ = create_publisher<std_msgs::msg::String>(
      "visual_slam/selector/active", rclcpp::QoS(1).reliable().transient_local());
    events_pub_ = create_publisher<pq_cuvslam_interfaces::msg::SlamEvent>(
      "visual_slam/events", rclcpp::QoS(100).reliable().transient_local());
    PublishActive();
    RCLCPP_INFO(
      get_logger(), "Odometry selector: %zu trackers, primary %s, reference frame %s",
      names.size(), cfg.primary.c_str(), reference_frame_.c_str());
  }

  ~OdometrySelectorNode() override
  {
    WriteRunOutput();
  }

private:
  static int64_t Stamp(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<int64_t>(t.sec) * 1'000'000'000 + t.nanosec;
  }

  struct PendingHealth
  {
    uint8_t health;
    std::string reasons;
  };

  void OnOdometry(const std::string & name, const nav_msgs::msg::Odometry::ConstSharedPtr & m)
  {
    tf2::Transform pose;
    tf2::fromMsg(m->pose.pose, pose);
    const int64_t stamp = Stamp(m->header.stamp);
    pending_odom_[name][stamp] = pose;
    TryProcess(name, stamp);
  }

  void OnDiagnostics(
    const std::string & name,
    const pq_cuvslam_interfaces::msg::FrameDiagnostics::ConstSharedPtr & d)
  {
    const int64_t stamp = Stamp(d->header.stamp);
    pending_health_[name][stamp] = {d->health, d->health_reasons};
    TryProcess(name, stamp);
  }

  // A frame is processed when its guarded odometry and its diagnostics (health) have both
  // arrived; the two topics may deliver them in either order.
  void TryProcess(const std::string & name, int64_t stamp)
  {
    auto & odom = pending_odom_[name];
    auto & health = pending_health_[name];
    const auto o = odom.find(stamp);
    const auto h = health.find(stamp);
    if (o == odom.end() || h == health.end()) {
      Trim(odom);
      Trim(health);
      return;
    }
    const tf2::Transform pose = o->second;
    const PendingHealth state = h->second;
    odom.erase(odom.begin(), std::next(o));
    health.erase(health.begin(), std::next(h));
    int64_t & last = last_stamp_[name];
    if (stamp <= last) {
      return;   // older than a frame already processed
    }
    last = stamp;
    Process(name, stamp, pose, state);
  }

  template<typename Map>
  static void Trim(Map & m)
  {
    while (m.size() > 60) {
      m.erase(m.begin());
    }
  }

  void Process(
    const std::string & name, int64_t stamp, const tf2::Transform & pose,
    const PendingHealth & state)
  {
    builtin_interfaces::msg::Time stamp_msg;
    stamp_msg.sec = static_cast<int32_t>(stamp / 1'000'000'000);
    stamp_msg.nanosec = static_cast<uint32_t>(stamp % 1'000'000'000);
    const SelectorResult r = selector_.Update(name, stamp, pose, state.health != 0);
    if (r.switched_from) {
      const std::string text = "odometry from " + r.selected + " instead of " +
        *r.switched_from + " (" + state.reasons + ")";
      RCLCPP_WARN(get_logger(), "Camera switch: %s", text.c_str());
      pq_cuvslam_interfaces::msg::SlamEvent e;
      e.header.stamp = stamp_msg;
      e.header.frame_id = odom_frame_;
      e.type = pq_cuvslam_interfaces::msg::SlamEvent::CAMERA_SWITCH;
      e.description = text;
      events_pub_->publish(e);
      PublishActive();
      switches_.push_back({stamp, *r.switched_from + "->" + r.selected});
    }
    if (r.fused) {
      nav_msgs::msg::Odometry out;
      out.header.stamp = stamp_msg;
      out.header.frame_id = odom_frame_;
      out.child_frame_id = reference_frame_;
      tf2::toMsg(*r.fused, out.pose.pose);
      fused_pub_->publish(out);
      rows_.push_back({stamp, *r.fused, r.selected});
    }
  }

  void PublishActive()
  {
    std_msgs::msg::String s;
    s.data = selector_.Selected();
    active_pub_->publish(s);
  }

  void WriteRunOutput()
  {
    if (run_output_dir_.empty() || rows_.empty()) {
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(run_output_dir_, ec);
    std::ofstream f(std::filesystem::path(run_output_dir_) / "odometry.csv");
    f << "timestamp,x,y,z,qx,qy,qz,qw,tracker\n";
    f << std::fixed << std::setprecision(9);
    for (const auto & r : rows_) {
      const auto & o = r.pose.getOrigin();
      const auto q = r.pose.getRotation();
      f << r.stamp_ns / 1e9 << ',' << o.x() << ',' << o.y() << ',' << o.z() << ',' << q.x() <<
        ',' << q.y() << ',' << q.z() << ',' << q.w() << ',' << r.tracker << '\n';
    }
    std::ofstream s(std::filesystem::path(run_output_dir_) / "selection.csv");
    s << "timestamp,switch\n";
    s << std::fixed << std::setprecision(9);
    for (const auto & [stamp, text] : switches_) {
      s << stamp / 1e9 << ',' << text << '\n';
    }
  }

  struct Row
  {
    int64_t stamp_ns;
    tf2::Transform pose;
    std::string tracker;
  };

  OdometrySelector selector_;
  std::string reference_frame_;
  std::string odom_frame_;
  std::string run_output_dir_;
  std::map<std::string, std::map<int64_t, tf2::Transform>> pending_odom_;
  std::map<std::string, std::map<int64_t, PendingHealth>> pending_health_;
  std::map<std::string, int64_t> last_stamp_;
  std::vector<Row> rows_;
  std::vector<std::pair<int64_t, std::string>> switches_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Subscription<pq_cuvslam_interfaces::msg::FrameDiagnostics>::SharedPtr>
  diag_subs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_pub_;
  rclcpp::Publisher<pq_cuvslam_interfaces::msg::SlamEvent>::SharedPtr events_pub_;
};

}  // namespace visual_slam
}  // namespace pq

RCLCPP_COMPONENTS_REGISTER_NODE(pq::visual_slam::OdometrySelectorNode)
