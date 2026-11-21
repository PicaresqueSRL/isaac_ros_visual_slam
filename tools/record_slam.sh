#!/bin/bash
# Copyright (c) 2026 Picaresque SRL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
#
# Replays one bag with pq_cuvslam and records the SLAM output to MCAP.
# Runs in the pq_cuvslam container (ROS 2 and the workspace sourced).
#
# Usage: record_slam.sh <bag> <out dir> <profile> [rate] [image topics]
#
# Default: real-time replay (rate 1, processing_mode realtime: frames dropped when the tracker
# is late, bundle adjustment and SLAM in background threads), i.e. what the node gives online.
# MODE=offline: every frame processed, reproducible, at <rate> times the recorded speed.
# The visualization topics are on. Output in <out dir>:
#   bag/        MCAP with odometry, paths, status, TF and the visualization topics
#   rerun/      the same for Rerun (to_rerun.py), with the camera images of the bag (default
#               the two RGB streams; comma-separated topics, "none" for no images)
#   run/        CSV files and run_info.json
#   *.log       replay, record and conversion logs

set -u

if [ $# -lt 3 ]; then
  echo "Usage: $0 <bag> <out dir> <profile> [rate] [image topics]" >&2
  exit 2
fi
BAG=$1
OUT=$2
PROFILE=$3
MODE=${MODE:-realtime}
if [ "$MODE" = offline ]; then RATE=${4:-2.0}; else RATE=${4:-1.0}; fi
IMAGES=${5:-/cam2/color/image_raw,/cam1/color/image_raw}
[ "$IMAGES" = none ] && IMAGES=

# Discovery limited to this host (container): other ROS processes on the network (e.g. another
# replay in a parallel container) must not end up in the recording.
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}

RECORD_TOPICS=(
  /visual_slam/tracking/odometry
  /visual_slam/tracking/vo_path
  /visual_slam/tracking/slam_path
  /visual_slam/tracking/vo_pose_covariance
  /visual_slam/status
  /tf /tf_static
)

rm -rf "$OUT"
mkdir -p "$OUT"
NAME=$(basename "${BAG%/}")

# The recorder runs in its own process group so that it can be stopped cleanly.
setsid ros2 bag record -s mcap -o "$OUT/bag" --topics "${RECORD_TOPICS[@]}" \
  --regex '^/visual_slam/vis/.*' > "$OUT/record.log" 2>&1 < /dev/null &
RECORD_PID=$!
sleep 3
if ! kill -0 "$RECORD_PID" 2>/dev/null; then
  echo "[$NAME] ERROR: ros2 bag record exited, see $OUT/record.log" >&2
  exit 1
fi

ros2 launch pq_cuvslam slam.launch.py profile:="$PROFILE" bag:="$BAG" rate:="$RATE" \
  processing_mode:="$MODE" enable_vis:=true run_output_dir:="$OUT/run" \
  > "$OUT/replay.log" 2>&1 < /dev/null
RC=$?

# Let the recorder receive the last messages, then stop it. Background processes of a script
# ignore SIGINT, so SIGTERM follows; SIGKILL only as last resort (the MCAP would not be closed).
sleep 2
kill -INT -- -"$RECORD_PID" 2>/dev/null
for i in $(seq 60); do
  kill -0 "$RECORD_PID" 2>/dev/null || break
  [ "$i" -eq 4 ] && kill -TERM -- -"$RECORD_PID" 2>/dev/null
  sleep 0.5
done
kill -KILL -- -"$RECORD_PID" 2>/dev/null

if [ $RC -ne 0 ] || [ ! -f "$OUT/run/run_info.json" ]; then
  echo "[$NAME] ERROR: replay failed, see $OUT/replay.log" >&2
  exit 1
fi
if [ ! -f "$OUT/bag/metadata.yaml" ]; then
  echo "[$NAME] ERROR: recording not closed, see $OUT/record.log" >&2
  exit 1
fi
if ! python3 "$(dirname "$0")/to_rerun.py" "$OUT/bag" "$BAG" "$OUT/rerun" --images "$IMAGES" \
  --max-image-rate 0 > "$OUT/rerun.log" 2>&1; then
  echo "[$NAME] WARNING: Rerun conversion failed, see $OUT/rerun.log" >&2
fi
echo "[$NAME] done -> $OUT"
