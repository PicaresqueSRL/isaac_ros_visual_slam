#!/bin/bash
# Copyright (c) 2026 Picaresque SRL
# SPDX-License-Identifier: Apache-2.0
#
# Runs tools/benchmark.py in the pq_cuvslam image, with the GPU.
#
# Usage:
#   benchmark_docker.sh [options] -- <benchmark.py arguments>
# Options:
#   -i <image>      Docker image (default pq_cuvslam:dev)
#   -b <dir>        bags root, mounted at /bags (read only)
#   -o <dir>        output root, mounted at /out
#   -w <dir>        colcon workspace with build/ and install/ (e.g. a development build of
#                   another cuVSLAM version), used instead of the one of the image
#   -g <dir>        folder with the ground truth and compare_runs.py, mounted at /gt (read only)
#
# Paths given to benchmark.py are paths in the container (/bags, /out, /gt). Example:
#   tools/benchmark_docker.sh -b ~/bags -o ~/bench -- run --bags /out/bench_list.txt \
#     --bags-root /bags --profiles cam2_stereo_imu,cam1_cam2_ir --out /out/v15 --repeat 2

set -euo pipefail

IMAGE=pq_cuvslam:dev
BAGS=""
OUT=""
WS=""
GT=""
while getopts "i:b:o:w:g:h" opt; do
  case $opt in
    i) IMAGE=$OPTARG ;;
    b) BAGS=$(readlink -f "$OPTARG") ;;
    o) OUT=$(readlink -f "$OPTARG") ;;
    w) WS=$(readlink -f "$OPTARG") ;;
    g) GT=$(readlink -f "$OPTARG") ;;
    *) sed -n '5,20p' "$0"; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
[ "${1:-}" = "--" ] && shift
[ -n "$OUT" ] || { echo "missing -o <output root>" >&2; exit 2; }
mkdir -p "$OUT"

TOOLS=$(dirname "$(readlink -f "$0")")
# No network: ROS discovery would otherwise reach the nodes of other containers.
ARGS=(--rm --gpus all --network none --user "$(id -u):$(id -g)" -e HOME=/tmp
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      -v "$TOOLS:/tools:ro" -v "$OUT:/out")
[ -n "$BAGS" ] && ARGS+=(-v "$BAGS:/bags:ro")
[ -n "$GT" ] && ARGS+=(-v "$GT:/gt:ro")
SETUP="source /opt/ros/lyrical/setup.bash && source /ws/install/setup.bash"
if [ -n "$WS" ]; then
  ARGS+=(-v "$WS/build:/ws/build:ro" -v "$WS/install:/ws/install:ro")
fi

exec docker run "${ARGS[@]}" --entrypoint bash "$IMAGE" -c \
  "$SETUP && python3 /tools/benchmark.py $(printf '%q ' "$@")"
