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
// Frame synchronization between devices that are not triggered together (e.g. two D455 on a
// rigid mount). Two stages:
//   1. device frames: the images of the cameras of one device (e.g. IR1 and IR2), which share
//      the shutter, are grouped by timestamp (within intra_device_tolerance_ns);
//   2. rig frames: for each frame of the reference device (device 0), the nearest frame of
//      every other device is chosen, within max_offset_ns. The chosen frame stays available
//      for the next reference frame: when frames are missing, one frame can be the nearest of
//      two reference frames, and consuming it would shift all the following pairs by one
//      frame. Only frames older than the chosen one are discarded. The rig frame takes the
//      timestamp of the reference device.
// The stamps of each device are corrected by its clock offset before the comparison.
// A frame that is followed by the next frame of the same device only after more than
// stale_gap_ns is dropped as stale (the D455 driver publishes an old frame at startup).

#ifndef PQ_CUVSLAM__IMPL__DEVICE_FRAME_SYNCHRONIZER_HPP_
#define PQ_CUVSLAM__IMPL__DEVICE_FRAME_SYNCHRONIZER_HPP_

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace pq
{
namespace visual_slam
{

template<typename Message>
class DeviceFrameSynchronizer
{
public:
  struct Config
  {
    // Camera indices of each device; device 0 is the reference.
    std::vector<std::vector<int>> devices;
    // Clock offset of each device in ns, subtracted from its stamps (empty = all zero).
    std::vector<int64_t> clock_offset_ns;
    int64_t max_offset_ns = 34'000'000;
    int64_t intra_device_tolerance_ns = 1'000'000;
    int64_t stale_gap_ns = 1'000'000'000;
    // Reference frames waiting for the other devices before being dropped.
    size_t max_pending_reference_frames = 10;
  };

  // A rig frame: images of all cameras, reference stamp, and the corrected stamp of the frame
  // of each device (device_stamps_ns[0] == reference_stamp_ns).
  struct RigFrame
  {
    int64_t reference_stamp_ns = 0;
    std::vector<int64_t> device_stamps_ns;
    std::vector<std::pair<int, Message>> images;  // (camera index, message)
  };

  struct Stats
  {
    uint64_t rig_frames = 0;
    uint64_t incomplete_device_frames = 0;    // e.g. IR1 without IR2
    uint64_t stale_device_frames = 0;
    uint64_t late_device_frames = 0;          // older than frames already used
    uint64_t unmatched_reference_frames = 0;  // no frame of another device close enough
    uint64_t unused_device_frames = 0;        // frames of other devices never paired
    uint64_t reused_device_frames = 0;        // frames paired with two reference frames
  };

  using Callback = std::function<void (const RigFrame &)>;

  explicit DeviceFrameSynchronizer(Config config)
  : config_(std::move(config))
  {
    if (config_.devices.empty()) {
      throw std::invalid_argument("DeviceFrameSynchronizer: no devices");
    }
    config_.clock_offset_ns.resize(config_.devices.size(), 0);
    for (size_t d = 0; d < config_.devices.size(); ++d) {
      for (const int camera : config_.devices[d]) {
        device_of_camera_[camera] = static_cast<int>(d);
      }
    }
    devices_.resize(config_.devices.size());
  }

  void RegisterCallback(Callback callback) {callback_ = std::move(callback);}

  void Add(int camera_index, int64_t stamp_ns, const Message & message)
  {
    const auto it = device_of_camera_.find(camera_index);
    if (it == device_of_camera_.end()) {
      return;
    }
    const int d = it->second;
    DeviceState & device = devices_[d];
    const int64_t stamp = stamp_ns - config_.clock_offset_ns[d];

    // Stage 1: group the cameras of the device.
    auto partial = device.partial.end();
    for (auto p = device.partial.begin(); p != device.partial.end(); ++p) {
      if (std::llabs(p->first - stamp) <= config_.intra_device_tolerance_ns) {
        partial = p;
        break;
      }
    }
    if (partial == device.partial.end()) {
      partial = device.partial.emplace(stamp, Partial{}).first;
    }
    partial->second.images[camera_index] = message;
    if (partial->second.images.size() < config_.devices[d].size()) {
      return;
    }

    DeviceFrame frame{partial->first, {}};
    for (const auto & [camera, msg] : partial->second.images) {
      frame.images.emplace_back(camera, msg);
    }
    // Older incomplete groups of this device will not be completed anymore.
    for (auto p = device.partial.begin(); p != device.partial.end(); ) {
      if (p->first <= frame.stamp) {
        if (p->first < frame.stamp) {
          ++stats_.incomplete_device_frames;
        }
        p = device.partial.erase(p);
      } else {
        ++p;
      }
    }
    PushDeviceFrame(d, std::move(frame));
    Match();
  }

  const Stats & GetStats() const {return stats_;}

private:
  struct Partial
  {
    std::map<int, Message> images;
  };

  struct DeviceFrame
  {
    int64_t stamp;
    std::vector<std::pair<int, Message>> images;
    bool used = false;
  };

  struct DeviceState
  {
    std::map<int64_t, Partial> partial;
    std::deque<DeviceFrame> frames;
    int64_t last_stamp = std::numeric_limits<int64_t>::min();
  };

  void PushDeviceFrame(int d, DeviceFrame frame)
  {
    DeviceState & device = devices_[d];
    // Frames already overtaken by the paired reference frames.
    if (frame.stamp < last_reference_stamp_ - config_.max_offset_ns) {
      ++stats_.late_device_frames;
      return;
    }
    // A long gap before this frame: the queued older frames are stale.
    while (!device.frames.empty() &&
      frame.stamp - device.frames.back().stamp > config_.stale_gap_ns)
    {
      device.frames.pop_back();
      ++stats_.stale_device_frames;
    }
    device.last_stamp = std::max(device.last_stamp, frame.stamp);
    device.frames.push_back(std::move(frame));
  }

  void Match()
  {
    auto & reference = devices_[0].frames;
    while (!reference.empty()) {
      const DeviceFrame & ref = reference.front();
      const int64_t window = config_.max_offset_ns;
      std::vector<int> chosen(devices_.size(), -1);
      bool decided = true;
      bool complete = true;
      for (size_t d = 1; d < devices_.size(); ++d) {
        auto & frames = devices_[d].frames;
        // Frames too old for this and the following reference frames.
        while (!frames.empty() && frames.front().stamp < ref.stamp - window) {
          stats_.unused_device_frames += frames.front().used ? 0 : 1;
          frames.pop_front();
        }
        // The nearest frame is known once a frame at or after the reference time is queued,
        // or once the device has gone past the pairing window.
        bool has_later = false;
        for (const auto & f : frames) {
          if (f.stamp >= ref.stamp) {
            has_later = true;
            break;
          }
        }
        if (!has_later && devices_[d].last_stamp < ref.stamp + window) {
          decided = false;
          break;
        }
        int64_t best = std::numeric_limits<int64_t>::max();
        for (size_t i = 0; i < frames.size(); ++i) {
          const int64_t dt = std::llabs(frames[i].stamp - ref.stamp);
          if (dt <= window && dt < best) {
            best = dt;
            chosen[d] = static_cast<int>(i);
          }
        }
        complete = complete && chosen[d] >= 0;
      }

      if (!decided) {
        if (reference.size() > config_.max_pending_reference_frames) {
          reference.pop_front();
          ++stats_.unmatched_reference_frames;
          continue;
        }
        return;
      }
      if (!complete) {
        reference.pop_front();
        ++stats_.unmatched_reference_frames;
        continue;
      }

      RigFrame rig;
      rig.reference_stamp_ns = ref.stamp;
      rig.device_stamps_ns.resize(devices_.size());
      rig.device_stamps_ns[0] = ref.stamp;
      rig.images = ref.images;
      for (size_t d = 1; d < devices_.size(); ++d) {
        auto & frames = devices_[d].frames;
        const DeviceFrame & f = frames[chosen[d]];
        rig.device_stamps_ns[d] = f.stamp;
        rig.images.insert(rig.images.end(), f.images.begin(), f.images.end());
        if (f.used) {
          ++stats_.reused_device_frames;
        }
        // Frames before the chosen one are not used anymore; the chosen one stays available
        // for the next reference frame.
        for (int i = 0; i < chosen[d]; ++i) {
          stats_.unused_device_frames += frames[i].used ? 0 : 1;
        }
        frames.erase(frames.begin(), frames.begin() + chosen[d]);
        frames.front().used = true;
      }
      last_reference_stamp_ = ref.stamp;
      reference.pop_front();
      ++stats_.rig_frames;
      callback_(rig);
    }
  }

  Config config_;
  std::map<int, int> device_of_camera_;
  std::vector<DeviceState> devices_;
  int64_t last_reference_stamp_ = std::numeric_limits<int64_t>::min() / 2;
  Callback callback_ = [](const RigFrame &) {};
  Stats stats_;
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__DEVICE_FRAME_SYNCHRONIZER_HPP_
