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

#include "pq_cuvslam/impl/latency_emulator.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <stdexcept>

namespace pq
{
namespace visual_slam
{

namespace
{

std::vector<std::string> SplitCsvLine(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    if (!field.empty() && field.back() == '\r') {
      field.pop_back();
    }
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

LatencyEmulator LatencyEmulator::Constant(double latency_ms)
{
  LatencyEmulator emulator;
  emulator.constant_ms_ = latency_ms;
  emulator.source_ = "constant";
  return emulator;
}

int64_t LatencyEmulator::Key(int64_t timestamp_ns)
{
  // 0.1 ms resolution: robust to the rounding of timestamps written in seconds.
  return static_cast<int64_t>(std::llround(timestamp_ns / 1e5));
}

LatencyEmulator LatencyEmulator::FromProfile(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("cannot open latency profile " + path);
  }
  std::string line;
  if (!std::getline(f, line)) {
    throw std::runtime_error("empty latency profile " + path);
  }
  const auto header = SplitCsvLine(line);
  const auto column_index = [&header](const std::string & name) -> int {
      const auto it = std::find(header.begin(), header.end(), name);
      return it == header.end() ? -1 : static_cast<int>(it - header.begin());
    };
  // Supported formats: status.csv of the run output (timestamp, callback_time_ms or
  // track_time_ms) and frames.csv of the Isaac ROS benchmark (frame_timestamp_s,
  // frame_processing_time_ms).
  int ts_col = column_index("timestamp");
  if (ts_col < 0) {
    ts_col = column_index("frame_timestamp_s");
  }
  int ms_col = -1;
  std::string column;
  for (const char * name : {"callback_time_ms", "frame_processing_time_ms", "track_time_ms"}) {
    ms_col = column_index(name);
    if (ms_col >= 0) {
      column = name;
      break;
    }
  }
  if (ts_col < 0 || ms_col < 0) {
    throw std::runtime_error(
            "latency profile " + path + " needs a timestamp column (timestamp or "
            "frame_timestamp_s) and a time column (callback_time_ms, frame_processing_time_ms "
            "or track_time_ms)");
  }

  LatencyEmulator emulator;
  emulator.source_ = path;
  emulator.column_ = column;
  while (std::getline(f, line)) {
    const auto fields = SplitCsvLine(line);
    if (static_cast<int>(fields.size()) <= std::max(ts_col, ms_col)) {
      continue;
    }
    try {
      const double ts_s = std::stod(fields[ts_col]);
      const double ms = std::stod(fields[ms_col]);
      emulator.by_stamp_[Key(static_cast<int64_t>(std::llround(ts_s * 1e9)))] = ms;
      emulator.samples_.push_back(ms);
    } catch (const std::exception &) {
      continue;  // skip malformed rows
    }
  }
  if (emulator.samples_.empty()) {
    throw std::runtime_error("no valid rows in latency profile " + path);
  }
  return emulator;
}

double LatencyEmulator::TargetMs(int64_t timestamp_ns) const
{
  if (samples_.empty()) {
    return constant_ms_;
  }
  if (const auto it = by_stamp_.find(Key(timestamp_ns)); it != by_stamp_.end()) {
    ++matched_;
    return it->second;
  }
  // Frame not in the profile: draw a time from it, seeded with the timestamp so that the same
  // frame always gets the same time.
  std::mt19937_64 rng(static_cast<uint64_t>(timestamp_ns));
  std::uniform_int_distribution<size_t> index(0, samples_.size() - 1);
  ++drawn_;
  return samples_[index(rng)];
}

std::string LatencyEmulator::Describe() const
{
  std::ostringstream ss;
  if (samples_.empty()) {
    ss << "constant " << constant_ms_ << " ms per frame";
    return ss.str();
  }
  std::vector<double> sorted = samples_;
  std::sort(sorted.begin(), sorted.end());
  const auto pct = [&sorted](double p) {
      return sorted[static_cast<size_t>(p * (sorted.size() - 1))];
    };
  ss << "profile " << source_ << " (" << column_ << ", " << sorted.size() <<
    " frames, median " << pct(0.5) << " ms, p95 " << pct(0.95) << " ms)";
  return ss.str();
}

}  // namespace visual_slam
}  // namespace pq
