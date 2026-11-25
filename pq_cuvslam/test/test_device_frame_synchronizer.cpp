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

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "pq_cuvslam/impl/device_frame_synchronizer.hpp"

namespace
{

using Sync = pq::visual_slam::DeviceFrameSynchronizer<int>;
constexpr int64_t kMs = 1'000'000;

// Two devices with two cameras each: device 0 = cameras 0, 1; device 1 = cameras 2, 3.
Sync::Config TwoStereo(int64_t max_offset_ms = 34)
{
  Sync::Config c;
  c.devices = {{0, 1}, {2, 3}};
  c.max_offset_ns = max_offset_ms * kMs;
  return c;
}

struct Recorder
{
  std::vector<Sync::RigFrame> frames;
  void Attach(Sync & sync)
  {
    sync.RegisterCallback([this](const Sync::RigFrame & f) {frames.push_back(f);});
  }
};

// Adds the two images of one device at time t (ms); message = time in ms.
void AddDevice(Sync & sync, int device, int64_t t_ms)
{
  sync.Add(device * 2, t_ms * kMs, static_cast<int>(t_ms));
  sync.Add(device * 2 + 1, t_ms * kMs, static_cast<int>(t_ms));
}

TEST(DeviceFrameSynchronizer, PairsDevicesWithReferenceStamp)
{
  Sync sync(TwoStereo());
  Recorder rec;
  rec.Attach(sync);
  for (int64_t t = 1000; t < 1400; t += 66) {
    AddDevice(sync, 0, t);
    AddDevice(sync, 1, t + 20);
  }
  ASSERT_GE(rec.frames.size(), 5u);
  for (const auto & f : rec.frames) {
    EXPECT_EQ(f.images.size(), 4u);
    EXPECT_EQ(f.device_stamps_ns[1] - f.device_stamps_ns[0], 20 * kMs);
    EXPECT_EQ(f.reference_stamp_ns, f.device_stamps_ns[0]);
  }
}

TEST(DeviceFrameSynchronizer, ChoosesNearestFrame)
{
  Sync sync(TwoStereo());
  Recorder rec;
  rec.Attach(sync);
  // Device 1 at 15 ms before and 18 ms after the reference: the earlier one is nearer.
  AddDevice(sync, 1, 985);
  AddDevice(sync, 0, 1000);
  AddDevice(sync, 1, 1018);
  ASSERT_EQ(rec.frames.size(), 1u);
  EXPECT_EQ(rec.frames[0].device_stamps_ns[1], 985 * kMs);
}

TEST(DeviceFrameSynchronizer, WaitsUntilNearestIsKnown)
{
  Sync sync(TwoStereo());
  Recorder rec;
  rec.Attach(sync);
  AddDevice(sync, 0, 1000);
  AddDevice(sync, 1, 990);  // a later frame of device 1 could still be nearer
  EXPECT_TRUE(rec.frames.empty());
  AddDevice(sync, 1, 1056);
  ASSERT_EQ(rec.frames.size(), 1u);
  EXPECT_EQ(rec.frames[0].device_stamps_ns[1], 990 * kMs);
}

TEST(DeviceFrameSynchronizer, DropsReferenceWithoutMatch)
{
  Sync sync(TwoStereo(10));
  Recorder rec;
  rec.Attach(sync);
  AddDevice(sync, 0, 1000);
  AddDevice(sync, 1, 1030);  // beyond 10 ms
  EXPECT_TRUE(rec.frames.empty());
  EXPECT_EQ(sync.GetStats().unmatched_reference_frames, 1u);
}

TEST(DeviceFrameSynchronizer, DropsIncompleteDeviceFrame)
{
  Sync sync(TwoStereo());
  Recorder rec;
  rec.Attach(sync);
  sync.Add(0, 1000 * kMs, 1000);  // camera 1 of this frame never arrives
  AddDevice(sync, 0, 1066);
  AddDevice(sync, 1, 1070);
  AddDevice(sync, 1, 1136);
  ASSERT_EQ(rec.frames.size(), 1u);
  EXPECT_EQ(rec.frames[0].reference_stamp_ns, 1066 * kMs);
  EXPECT_EQ(sync.GetStats().incomplete_device_frames, 1u);
}

TEST(DeviceFrameSynchronizer, DropsStaleFirstFrame)
{
  Sync sync(TwoStereo());
  Recorder rec;
  rec.Attach(sync);
  // Old frame left in the driver: 400 s before the stream.
  AddDevice(sync, 0, 1000);
  for (int64_t t = 401000; t < 401300; t += 66) {
    AddDevice(sync, 0, t);
    AddDevice(sync, 1, t + 5);
  }
  ASSERT_FALSE(rec.frames.empty());
  EXPECT_EQ(rec.frames.front().reference_stamp_ns, 401000 * kMs);
  EXPECT_EQ(sync.GetStats().stale_device_frames, 1u);
}

TEST(DeviceFrameSynchronizer, KeepsNearestPhaseWhenFirstFrameIsMissing)
{
  // 30 fps; device 1 frames 13 ms before the reference frames (and 20 ms after the previous
  // ones). The first device 1 frame is missing: the first reference frame must not take the
  // frame that belongs to the second one.
  Sync sync(TwoStereo(34));
  Recorder rec;
  rec.Attach(sync);
  const int64_t period = 33;
  for (int i = 0; i < 12; ++i) {
    const int64_t t = 1000 + i * period;
    if (i > 0) {
      AddDevice(sync, 1, t - 13);
    }
    AddDevice(sync, 0, t);
  }
  AddDevice(sync, 1, 1000 + 12 * period - 13);
  ASSERT_GE(rec.frames.size(), 8u);
  // The first reference frame takes the +20 ms frame; the following ones keep the -13 ms
  // phase (the +20 ms frame is reused, not consumed).
  EXPECT_EQ(rec.frames[0].device_stamps_ns[1] - rec.frames[0].reference_stamp_ns, 20 * kMs);
  for (size_t i = 1; i < rec.frames.size(); ++i) {
    EXPECT_EQ(rec.frames[i].device_stamps_ns[1] - rec.frames[i].reference_stamp_ns, -13 * kMs)
      << "rig frame " << i;
  }
}

TEST(DeviceFrameSynchronizer, PairsIrregularFramesWithNearest)
{
  // 30 fps grid with frames missing on both devices (as in the recorded bags): every
  // reference frame gets its nearest frame, which may be shared with the next one.
  Sync sync(TwoStereo(34));
  Recorder rec;
  rec.Attach(sync);
  for (const int64_t t : {0, 33, 133, 167, 267, 300}) {
    AddDevice(sync, 0, 1000 + t);
  }
  for (const int64_t t : {-11, 22, 122, 156, 256, 289, 389}) {
    AddDevice(sync, 1, 1000 + t);
  }
  ASSERT_EQ(rec.frames.size(), 6u);
  for (const auto & f : rec.frames) {
    EXPECT_EQ(f.device_stamps_ns[1] - f.reference_stamp_ns, -11 * kMs);
  }
  EXPECT_EQ(sync.GetStats().unmatched_reference_frames, 0u);
}

TEST(DeviceFrameSynchronizer, AppliesClockOffset)
{
  Sync::Config c = TwoStereo(5);
  c.clock_offset_ns = {0, 100 * kMs};  // device 1 stamps 100 ms ahead
  Sync sync(c);
  Recorder rec;
  rec.Attach(sync);
  AddDevice(sync, 0, 1000);
  AddDevice(sync, 1, 1102);
  AddDevice(sync, 1, 1168);
  ASSERT_EQ(rec.frames.size(), 1u);
  EXPECT_EQ(rec.frames[0].device_stamps_ns[1], 1002 * kMs);
}

}  // namespace
