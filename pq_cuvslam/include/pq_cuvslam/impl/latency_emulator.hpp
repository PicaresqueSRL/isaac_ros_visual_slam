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
// Emulation of the per-frame processing time of another platform, e.g. the Jetson Orin Nano
// on a desktop PC: the node waits after each frame until the frame has taken as long as it
// would on that platform. Frames that the target platform would not process in time are then
// dropped by the input queues in the same way.

#ifndef PQ_CUVSLAM__IMPL__LATENCY_EMULATOR_HPP_
#define PQ_CUVSLAM__IMPL__LATENCY_EMULATOR_HPP_

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace pq
{
namespace visual_slam
{

class LatencyEmulator
{
public:
  // Constant target time per frame.
  static LatencyEmulator Constant(double latency_ms);

  // Profile measured on the target platform: a status.csv of the run output (columns
  // "timestamp" and "callback_time_ms", or "track_time_ms") or a frames.csv of the Isaac ROS
  // benchmark ("frame_timestamp_s", "frame_processing_time_ms"). Frames of the same bag are
  // matched by timestamp; other frames get a time drawn from the profile (deterministic for a
  // given timestamp). Throws std::runtime_error if the file cannot be used.
  static LatencyEmulator FromProfile(const std::string & path);

  // Target processing time of the frame with this timestamp, in ms.
  double TargetMs(int64_t timestamp_ns) const;

  // Short description for the log and run_info.json.
  std::string Describe() const;

  // Number of frames matched by timestamp / drawn from the profile so far.
  uint64_t Matched() const {return matched_;}
  uint64_t Drawn() const {return drawn_;}

private:
  static int64_t Key(int64_t timestamp_ns);

  std::string source_;
  std::string column_;
  double constant_ms_ = 0.0;
  std::unordered_map<int64_t, double> by_stamp_;
  std::vector<double> samples_;
  mutable uint64_t matched_ = 0;
  mutable uint64_t drawn_ = 0;
};

}  // namespace visual_slam
}  // namespace pq

#endif  // PQ_CUVSLAM__IMPL__LATENCY_EMULATOR_HPP_
