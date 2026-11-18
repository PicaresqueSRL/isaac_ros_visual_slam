#!/bin/bash
# Run pq_cuvslam on one bag at real-time speed and record its output topics.
# Runs inside the pq_cuvslam container (ROS 2 and the workspace sourced).
#
# Usage: run_bag.sh <bag_dir> <out_dir> [launch args...]
#   e.g. run_bag.sh /bags/x /out/x profile:=cam1_cam2_ir run_output_dir:=/out/x/run
#
# Output in <out_dir>:
#   bag/          MCAP recording of the cuVSLAM output topics
#   launch.log    node log
#   record.log    ros2 bag record log
#   play.log      ros2 bag play log
#
# The node runs with slam.launch.py (default profile cam2_stereo_imu). Played topics: IR,
# IMU, RGB and aligned depth of both cameras and /tf_static (PLAY_TOPICS to change them).
# Recorded topics: those of the Isaac ROS reference runs (slam_cam2.sh).

set -u

if [ $# -lt 2 ]; then
  echo "Usage: $0 <bag_dir> <out_dir> [launch args...]" >&2
  exit 2
fi

BAG=$1
OUT=$2
shift 2
LAUNCH=$(ros2 pkg prefix pq_cuvslam)/share/pq_cuvslam/launch/slam.launch.py
LAUNCH_ARGS=("$@")

if [ -n "${PLAY_TOPICS:-}" ]; then
  read -r -a PLAY_TOPICS <<< "$PLAY_TOPICS"
else
  PLAY_TOPICS=(/tf_static)
  for cam in cam1 cam2; do
    PLAY_TOPICS+=(
      /$cam/infra1/image_rect_raw /$cam/infra1/camera_info
      /$cam/infra2/image_rect_raw /$cam/infra2/camera_info
      /$cam/imu
      /$cam/color/image_raw /$cam/color/camera_info
      /$cam/aligned_depth_to_color/image_raw /$cam/aligned_depth_to_color/camera_info
    )
  done
fi
RECORD_TOPICS=(
  /visual_slam/tracking/odometry
  /visual_slam/tracking/vo_path
  /visual_slam/tracking/slam_path
  /visual_slam/status
  /tf /tf_static
)

LAUNCH_PID=""
RECORD_PID=""

# Stop a process group: SIGINT, then SIGTERM, then SIGKILL.
stop_group() {
  local pid=$1
  [ -z "$pid" ] && return
  kill -0 "$pid" 2>/dev/null || return
  kill -INT -- -"$pid" 2>/dev/null
  for i in $(seq 60); do
    kill -0 "$pid" 2>/dev/null || return
    [ "$i" -eq 10 ] && kill -TERM -- -"$pid" 2>/dev/null
    sleep 0.5
  done
  echo "WARNING: process $pid does not exit, sending SIGKILL" >&2
  kill -KILL -- -"$pid" 2>/dev/null
}

cleanup() {
  stop_group "$RECORD_PID"
  stop_group "$LAUNCH_PID"
}
trap 'echo "Interrupted"; cleanup; exit 130' INT TERM

wait_for_node() {
  for _ in $(seq 60); do
    ros2 node list 2>/dev/null < /dev/null | grep -q visual_slam_node && return 0
    sleep 1
  done
  return 1
}

[ -f "$LAUNCH" ] || { echo "Launch file not found: $LAUNCH" >&2; exit 1; }
[ -f "$BAG/metadata.yaml" ] || { echo "Not a bag folder: $BAG" >&2; exit 1; }

rm -rf "$OUT"
mkdir -p "$OUT"
NAME=$(basename "${BAG%/}")

echo "[$NAME] starting pq_cuvslam"
setsid ros2 launch "$LAUNCH" "${LAUNCH_ARGS[@]}" > "$OUT/launch.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
if ! wait_for_node; then
  echo "[$NAME] ERROR: visual_slam_node did not start, see $OUT/launch.log" >&2
  cleanup
  exit 1
fi
sleep 3

setsid ros2 bag record -s mcap -o "$OUT/bag" --topics "${RECORD_TOPICS[@]}" \
  > "$OUT/record.log" 2>&1 < /dev/null &
RECORD_PID=$!
sleep 3
if ! kill -0 "$RECORD_PID" 2>/dev/null; then
  echo "[$NAME] ERROR: ros2 bag record exited, see $OUT/record.log" >&2
  RECORD_PID=""
  cleanup
  exit 1
fi

echo "[$NAME] playing"
ros2 bag play "$BAG" --topics "${PLAY_TOPICS[@]}" > "$OUT/play.log" 2>&1 < /dev/null
PLAY_RC=$?

# Give cuVSLAM time to publish the last frames.
sleep 5
cleanup
LAUNCH_PID=""
RECORD_PID=""

if [ $PLAY_RC -ne 0 ]; then
  echo "[$NAME] ERROR: bag play failed, see $OUT/play.log" >&2
  exit 1
fi
if ! [ -f "$OUT/bag/metadata.yaml" ]; then
  echo "[$NAME] ERROR: recording not closed (no metadata.yaml), see $OUT/record.log" >&2
  exit 1
fi
echo "[$NAME] done -> $OUT/bag"
