#!/bin/bash
# Copyright (c) 2026 Picaresque SRL
# SPDX-License-Identifier: Apache-2.0
#
# pq: build and use pq_cuvslam (cuVSLAM SLAM for two D455) on the PC or the Jetson.
# Everything runs in Docker; the local settings are in pq.conf (see pq.conf.example).
# `./pq help` for the commands.

set -euo pipefail

REPO=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)
CONF=$REPO/pq.conf

usage() {
  cat <<'EOF'
Usage: ./pq <command> [arguments] [--cuvslam <version>]

cuVSLAM version: every version is built into its own image (pq_cuvslam:<version>) and all the
commands take --cuvslam <version> to choose it, e.g. --cuvslam 15. A short version picks the
latest release (15: 15.0.1, 16: 16.0.0, 17: 17.0.1); 15.0.0 is the version of the Isaac ROS
node. Default: CUVSLAM in pq.conf, otherwise the submodule (17.0.1). Supported: 15 to 17.

Setup
  setup [options]            build the image of a cuVSLAM version (platform detected), create
                             pq.conf; --cuvslam 15,16,17 builds several versions
                             --all-gpus       kernels for every NVIDIA GPU (PC; slower build)
                             --jobs <n>       parallel compile jobs
                             --no-cache       rebuild every step (no Docker cache)
  versions                   cuVSLAM versions built on this machine, and available ones
  profiles                   list the camera profiles

Live (cameras connected)
  cams start|stop|restart|reset|list|logs [cam1|cam2]
                             the two D455; reset: USB reset of the cameras at start (use it
                             when no images come after a reboot); list: connected cameras
  slam [profile|dual] [name] live SLAM in the background (default cam1_stereo_imu; dual: VIO
                             of cam1 and cam2 with the odometry selector); starts the cameras
                             if needed; output in runs/<name>
  record <name> [--depth]    raw recording of both cameras to bags/<name> (background)
  walk <name> <seconds> [profile]
                             live SLAM + recording for <seconds>, then everything stopped and
                             the Rerun file built; needs no connection (nohup ./pq walk ... &)
  stop [slam|record]         stop the SLAM and/or the recording cleanly (default both)

Recorded bags
  replay <bag> [profile|dual] [--offline] [--name <name>] [--rate <r>]
                             SLAM on a bag (folder, or name under bags/ or BAGS_DIR): real time
                             by default, as live; --offline: every frame, reproducible.
                             Output in runs/<name>: run/ (CSV), bag/ (SLAM MCAP), rerun/
  bench <benchmark.py args>  regression benchmark (tools/README.md); container paths:
                             /bags = bags, /out = bench/

Other
  status                     containers, cameras, disk
  logs [cams|slam|record]    follow a log
  shell                      shell in the image (ROS 2 and the workspace sourced)
  test                       unit tests
  console install|update|config|start|stop|logs|uninstall
                             optional web app (branch console, checked out in .pq_console/);
                             config: regenerate its configuration (e.g. after a new version)
EOF
}

die() { echo "pq: $*" >&2; exit 1; }
info() { echo "pq: $*"; }

is_jetson() { [ -f /etc/nv_tegra_release ] || grep -qi tegra /proc/device-tree/compatible 2>/dev/null; }

# ---------------------------------------------------------------------------------------------
# cuVSLAM versions. The tags come from the cuvslam submodule (fetched if missing).
cuvslam_tags() {
  [ -f "$REPO/cuvslam/CMakeLists.txt" ] || git -C "$REPO" submodule update --init cuvslam >&2
  git -C "$REPO/cuvslam" tag -l 'v[0-9]*' | sort -V
}

# Version of the submodule: its tag (v17.0.1), or its commit.
submodule_ref() {
  [ -f "$REPO/cuvslam/CMakeLists.txt" ] || git -C "$REPO" submodule update --init cuvslam >&2
  git -C "$REPO/cuvslam" describe --tags --exact-match HEAD 2>/dev/null \
    || git -C "$REPO/cuvslam" rev-parse --short=12 HEAD
}

# Version argument -> git ref: 15 -> latest v15.x.y, 15.0 -> latest v15.0.y, 15.0.0 or v15.0.0
# -> v15.0.0; anything else (commit, branch) unchanged; empty -> the submodule.
cuvslam_ref() {
  local v=${1#v} t
  [ -n "$v" ] || { submodule_ref; return; }
  if [[ $v =~ ^[0-9]+(\.[0-9]+){0,2}$ ]]; then
    t=$(cuvslam_tags | grep -E "^v${v//./\\.}(\.|$)" | tail -1)
    [ -n "$t" ] || die "cuVSLAM $1: no such version (available: $(cuvslam_tags | tr '\n' ' '))"
    echo "$t"
  else
    echo "$1"
  fi
}

# Image of a version: pq_cuvslam:17.0.1 (commit refs: pq_cuvslam:<commit>).
image_of() { local r=${1#v}; echo "pq_cuvslam:${r//[^A-Za-z0-9_.-]/_}"; }

# Sets CUVSLAM_REF, VERSION and IMAGE from --cuvslam (or CUVSLAM of pq.conf).
select_version() {
  CUVSLAM_REF=$(cuvslam_ref "$1")
  VERSION=${CUVSLAM_REF#v}
  IMAGE=$(image_of "$CUVSLAM_REF")
  if [[ $VERSION =~ ^[0-9]+\. ]] && ! [[ $VERSION =~ ^1[5-7]\. ]]; then
    info "warning: cuVSLAM $VERSION is outside the supported versions (15 to 17)"
  fi
}

load_conf() {
  DATA_DIR=$HOME/pq_data; BAGS_DIR=; CAM1_SERIAL=; CAM2_SERIAL=; CUVSLAM=
  CAMERA_IMAGE=; CAMERA_PARAMS=; ROS_DOMAIN_ID=7; BROWSE="$HOME /media /mnt"
  # shellcheck disable=SC1090
  [ -f "$CONF" ] && source "$CONF"
  [[ ${CUVSLAM_ARG:-} == *,* ]] && die "--cuvslam: one version only (several only with setup)"
  select_version "${CUVSLAM_ARG:-$CUVSLAM}"
  CAMERA_IMAGE=${CAMERA_IMAGE:-$IMAGE}
  if is_jetson; then GPU=(--runtime nvidia); else GPU=(--gpus all); fi
  # Containers that talk to the cameras: host network and IPC, root (the Fast DDS shared memory
  # is owned by root in the camera container), large data transport for the images.
  ROSNET=(--network host --ipc host -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
          -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-LARGE_DATA}")
}

need_image() {
  docker image inspect "$1" > /dev/null 2>&1 && return
  if [ "$1" = "$IMAGE" ]; then
    die "cuVSLAM $VERSION not built on this machine: ./pq setup --cuvslam $VERSION (./pq versions)"
  fi
  die "image $1 missing"
}

running() { docker ps -q --filter "name=^$1\$" | grep -q .; }

# Runs a command in an image with ROS 2 (and the pq_cuvslam workspace, if any) sourced.
ros_cmd() {
  echo "source /opt/ros/\${ROS_DISTRO:-lyrical}/setup.bash; " \
       "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash; $1"
}

# Gives files written by root in the containers back to the user.
own() {
  local d
  for d in "$@"; do
    [ -e "$d" ] || continue
    docker run --rm --network none -v "$d:/d" --entrypoint chown "$IMAGE" -R \
      "$(id -u):$(id -g)" /d > /dev/null 2>&1 || true
  done
}

# Profile argument: name of an installed profile, YAML file (mounted), or dual.
# Sets PROFILE_ARG (value for the launch) and PROFILE_MOUNT (docker -v arguments).
resolve_profile() {
  local p=$1
  PROFILE_MOUNT=()
  if [ "$p" = dual ]; then PROFILE_ARG=dual; return; fi
  if [ -f "$p" ]; then
    local f; f=$(readlink -f "$p")
    PROFILE_MOUNT=(-v "$(dirname "$f"):/profiles:ro")
    PROFILE_ARG=/profiles/$(basename "$f")
    return
  fi
  p=${p%.yaml}
  [ -f "$REPO/pq_cuvslam/profiles/$p.yaml" ] || die "unknown profile '$1' (./pq profiles)"
  PROFILE_ARG=$p
}

stamp() { date +%Y%m%d_%H%M%S; }

# ---------------------------------------------------------------------------------------------
cmd_setup() {
  local all_gpus=0 jobs="" nocache=()
  while [ $# -gt 0 ]; do
    case $1 in
      --all-gpus) all_gpus=1 ;;
      --jobs) jobs=${2:?}; shift ;;
      --no-cache) nocache=(--no-cache) ;;
      *) die "setup: unknown option $1" ;;
    esac
    shift
  done
  command -v docker > /dev/null || die "docker not installed"
  docker info > /dev/null 2>&1 || die "cannot use docker (user in the docker group?)"
  if [ ! -f "$CONF" ]; then
    cp "$REPO/pq.conf.example" "$CONF"
    info "created pq.conf: set CAM1_SERIAL and CAM2_SERIAL there for the live cameras"
  fi
  info "cuVSLAM submodule"
  git -C "$REPO" submodule update --init cuvslam
  local CUVSLAM_ARG_ALL=${CUVSLAM_ARG:-}
  CUVSLAM_ARG=
  load_conf

  local args=()
  if is_jetson; then
    # JetPack 7.2 (CUDA 13), Orin Nano: 8 GB without swap, 2 jobs; kernels for sm_87 only;
    # the CUDA forward compatibility packages crash the container toolkit.
    info "platform: Jetson"
    args+=(--build-arg BASE_IMAGE=nvidia/cuda:13.2.0-devel-ubuntu24.04
           --build-arg CUDA_COMPAT=remove --build-arg "BUILD_JOBS=${jobs:-2}")
    [ $all_gpus = 1 ] || args+=(--build-arg CUDA_ARCHITECTURES=87)
  else
    info "platform: PC ($(uname -m))"
    command -v nvidia-smi > /dev/null && nvidia-smi > /dev/null 2>&1 \
      || die "NVIDIA driver missing or not loaded (nvidia-smi fails): see README.md, Quick start"
    docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' \
      || die "NVIDIA Container Toolkit missing (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)"
    local cc=""
    if [ $all_gpus = 0 ] && command -v nvidia-smi > /dev/null; then
      cc=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | sort -u | tr -d . | paste -sd';')
    fi
    if [ -n "$cc" ]; then
      info "GPU architecture $cc (--all-gpus for an image valid on every NVIDIA GPU)"
      args+=(--build-arg "CUDA_ARCHITECTURES=$cc")
    fi
    # The CUDA kernels need about 2 GB per job.
    local mem_jobs=$(( $(awk '/MemTotal/{print int($2/1048576)}' /proc/meminfo) / 2 ))
    local n=$(( $(nproc) < mem_jobs ? $(nproc) : mem_jobs ))
    args+=(--build-arg "BUILD_JOBS=${jobs:-$(( n > 0 ? n : 1 ))}")
  fi
  mkdir -p "$DATA_DIR"/{bags,runs,rerun,bench}
  local v versions sub built=()
  IFS=, read -r -a versions <<< "${CUVSLAM_ARG_ALL:-$CUVSLAM}"
  [ ${#versions[@]} -gt 0 ] || versions=("")
  sub=$(submodule_ref)
  for v in "${versions[@]}"; do
    select_version "$v"
    local vargs=("${args[@]}")
    # The submodule version is built from the submodule; other versions are downloaded from
    # GitHub during the build (CUVSLAM_REF).
    [ "$CUVSLAM_REF" = "$sub" ] || vargs+=(--build-arg "CUVSLAM_REF=$CUVSLAM_REF")
    info "building cuVSLAM $VERSION -> $IMAGE (first time: about 20 min on a PC, 30-45 min" \
         "on the Orin Nano; a further version: only the workspace is rebuilt)"
    docker build "${nocache[@]}" -f "$REPO/docker/Dockerfile" -t "$IMAGE" "${vargs[@]}" "$REPO"
    docker run --rm "${GPU[@]}" "$IMAGE" bash -c \
      'ls /ws/install/pq_cuvslam/lib/pq_cuvslam > /dev/null' || die "$IMAGE built but not usable"
    built+=("$VERSION")
  done
  info "done: cuVSLAM ${built[*]} (./pq versions), data in $DATA_DIR"
  info "next: ./pq replay <bag>  or, with the cameras, ./pq slam"
}

cmd_versions() {
  local default; default=$(cuvslam_ref "$CUVSLAM")
  printf '%-14s %-24s %-16s %s\n' VERSION IMAGE BUILT SIZE
  local tag created size found=0
  while read -r tag created size; do
    [ -n "$tag" ] || continue
    found=1
    printf '%-14s %-24s %-16s %s%s\n' "$tag" "pq_cuvslam:$tag" "${created:0:16}" "$size" \
      "$([ "v$tag" = "$default" ] || [ "$tag" = "$default" ] && echo '   (default)')"
  done < <(docker images pq_cuvslam --format '{{.Tag}} {{.CreatedAt}} {{.Size}}' \
             | awk '{print $1, $2"_"$3, $NF}' | sort -V)
  [ $found = 1 ] || echo "(none built: ./pq setup)"
  echo
  echo "available: $(cuvslam_tags | grep -E '^v1[5-7]\.' | tr '\n' ' ')"
  echo "default: ${default#v} (CUVSLAM in pq.conf; empty: the submodule)"
  echo "build another one: ./pq setup --cuvslam <version>; use it: ./pq <command> --cuvslam <version>"
}

cmd_profiles() {
  local f
  printf '%-28s %-12s %s\n' PROFILE MODE DESCRIPTION
  for f in "$REPO"/pq_cuvslam/profiles/*.yaml; do
    printf '%-28s %-12s %s\n' "$(basename "$f" .yaml)" "$(sed -n 's/^mode: *//p' "$f" | head -1)" \
      "$(sed -n 's/^description: *//p' "$f" | head -1)"
  done
  printf '%-28s %-12s %s\n' dual inertial \
    "cam1_stereo_imu and cam2_stereo_imu in parallel, odometry selector (real time)"
}

# ---------------------------------------------------------------------------------------------
cams_start() {
  local reset=$1; shift
  [ -n "$CAM1_SERIAL" ] && [ -n "$CAM2_SERIAL" ] \
    || die "set CAM1_SERIAL and CAM2_SERIAL in pq.conf (./pq cams list shows the serials)"
  need_image "$CAMERA_IMAGE"
  docker rm -f pq_cams > /dev/null 2>&1 || true
  local extra="$CAMERA_PARAMS $* $([ "$reset" = 1 ] && echo initial_reset:=true)"
  extra=$(echo "$extra" | tr '\n' ' ')
  # --runtime nvidia also for the driver: librealsense of the Isaac ROS image uses CUDA.
  docker run -d --name pq_cams --privileged -v /dev:/dev "${ROSNET[@]}" "${GPU[@]}" \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --entrypoint bash "$CAMERA_IMAGE" -c "$(ros_cmd "
cam() { ros2 launch realsense2_camera rs_launch.py camera_name:=\$1 camera_namespace:=/ \
  serial_no:=_\$2 $extra; }
cam cam1 $CAM1_SERIAL > /tmp/cam1.log 2>&1 &
sleep 4
cam cam2 $CAM2_SERIAL > /tmp/cam2.log 2>&1 &
# The emitter argument of the launch is not applied: switch it off once the nodes are up.
for i in \$(seq 30); do sleep 2
  ros2 param set /cam1 depth_module.emitter_enabled 0 > /dev/null 2>&1 \
    && ros2 param set /cam2 depth_module.emitter_enabled 0 > /dev/null 2>&1 \
    && echo 'emitters off' > /tmp/emitter.log && break
done
wait")" > /dev/null
  info "cameras starting (cam1 $CAM1_SERIAL, cam2 $CAM2_SERIAL)"
  local i
  for i in $(seq 40); do
    sleep 2
    running pq_cams || { docker logs pq_cams 2>&1 | tail -5; die "camera container stopped"; }
    if docker exec pq_cams test -f /tmp/emitter.log 2>/dev/null; then
      info "cameras up, IR emitters off"; return 0
    fi
  done
  docker exec pq_cams bash -c 'grep -h -i -E "error|warn" /tmp/cam1.log /tmp/cam2.log | tail -8' || true
  die "cameras not up after 80 s (./pq logs cams; after a reboot try ./pq cams reset)"
}

cmd_cams() {
  local sub=${1:-start}; shift || true
  case $sub in
    start) running pq_cams && { info "cameras already running (./pq cams restart)"; return; }
           cams_start 0 "$@" ;;
    restart) cams_start 0 "$@" ;;
    reset) cams_start 1 "$@" ;;
    stop) docker rm -f pq_cams > /dev/null 2>&1 || true; info "cameras stopped" ;;
    list) need_image "$CAMERA_IMAGE"
          docker run --rm --privileged -v /dev:/dev --entrypoint bash "$CAMERA_IMAGE" -c \
            "$(ros_cmd 'if command -v rs-enumerate-devices > /dev/null; then rs-enumerate-devices -s; else echo "RealSense tools missing in the camera image: rebuild it (./pq setup)"; fi')" || true ;;
    logs) docker exec pq_cams tail -n 50 -f "/tmp/${1:-cam2}.log" ;;
    *) die "cams: start|stop|restart|reset|list|logs" ;;
  esac
}

ensure_cams() {
  running pq_cams && return
  info "cameras not running: starting them"
  cams_start 0
}

slam_start() {   # <profile arg> <name> <vis 0|1>
  local profile=$1 name=$2 vis=$3 launch
  local visarg; visarg=$([ "$vis" = 1 ] && echo true || echo false)
  if [ "$profile" = dual ]; then
    launch="dual_vio.launch.py run_output_dir:=/out/$name enable_vis:=$visarg"
  else
    launch="slam.launch.py profile:=$profile run_output_dir:=/out/$name enable_vis:=$visarg"
  fi
  need_image "$IMAGE"
  docker rm -f pq_slam > /dev/null 2>&1 || true
  mkdir -p "$DATA_DIR/runs"
  docker run -d --name pq_slam "${GPU[@]}" "${ROSNET[@]}" -v "$DATA_DIR/runs:/out" \
    "${PROFILE_MOUNT[@]}" --entrypoint bash "$IMAGE" -c \
    "$(ros_cmd "exec ros2 launch pq_cuvslam $launch > /out/$name.log 2>&1")" > /dev/null
  info "SLAM $profile, cuVSLAM $VERSION -> $DATA_DIR/runs/$name (log $DATA_DIR/runs/$name.log)"
}

cmd_slam() {
  local profile=${1:-cam1_stereo_imu} name=${2:-}
  resolve_profile "$profile"
  name=${name:-$(basename "${PROFILE_ARG%.yaml}")_$(stamp)}
  ensure_cams
  slam_start "$PROFILE_ARG" "$name" "${VIS:-0}"
  info "stop with ./pq stop"
}

record_start() {   # <name> <depth 0|1>
  local name=$1 depth=$2 t="" c s
  for c in cam1 cam2; do
    for s in infra1 infra2 color; do
      t="$t /$c/$s/image_rect_raw /$c/$s/image_raw /$c/$s/camera_info /$c/$s/metadata"
    done
    t="$t /$c/imu /$c/gyro/metadata /$c/accel/metadata"
    [ "$depth" = 1 ] && t="$t /$c/depth/image_rect_raw /$c/depth/camera_info /$c/depth/metadata"
  done
  [ -e "$DATA_DIR/bags/$name" ] && die "bags/$name exists"
  mkdir -p "$DATA_DIR/bags"
  docker rm -f pq_rec > /dev/null 2>&1 || true
  docker run -d --name pq_rec "${ROSNET[@]}" -v "$DATA_DIR/bags:/out" --entrypoint bash \
    "$CAMERA_IMAGE" -c "$(ros_cmd "exec ros2 bag record -s mcap --storage-preset-profile zstd_fast \
      --include-unpublished-topics -o /out/$name --topics $t /tf_static > /out/$name.log 2>&1")" \
    > /dev/null
  info "recording -> $DATA_DIR/bags/$name"
}

cmd_record() {
  local name=${1:?name} depth=0
  [ "${2:-}" = --depth ] && depth=1
  ensure_cams
  record_start "$name" "$depth"
  info "stop with ./pq stop record"
}

stop_bag_record() {   # <container>
  running "$1" || return 0
  docker exec "$1" bash -c 'pkill -INT -f "bag record"' 2> /dev/null || true
}

wait_stopped() {
  local i c any
  for i in $(seq 90); do
    any=0
    for c in "$@"; do running "$c" && any=1; done
    [ $any = 0 ] && return 0
    sleep 1
  done
  for c in "$@"; do docker rm -f "$c" > /dev/null 2>&1 || true; done
}

cmd_stop() {
  local what=${1:-all}
  case $what in
    record) stop_bag_record pq_rec; wait_stopped pq_rec ;;
    slam) stop_bag_record pq_slamrec; docker kill -s INT pq_slam > /dev/null 2>&1 || true
          wait_stopped pq_slam pq_slamrec ;;
    all) stop_bag_record pq_rec; stop_bag_record pq_slamrec
         docker kill -s INT pq_slam > /dev/null 2>&1 || true
         wait_stopped pq_rec pq_slam pq_slamrec ;;
    *) die "stop: slam|record" ;;
  esac
  for c in pq_rec pq_slam pq_slamrec; do docker rm "$c" > /dev/null 2>&1 || true; done
  own "$DATA_DIR/runs" "$DATA_DIR/bags"
  info "stopped ($what)"
}

slam_topics_regex='^(/tf|/tf_static|(/cam[12]_vio)?/visual_slam/.*)$'

to_rerun() {   # <slam bag (container path)> <source bag (container path)> <output .mcap>
  docker run --rm --network none -v "$DATA_DIR:/data" -v "$REPO/tools:/tools:ro" \
    --entrypoint bash "$IMAGE" -c "$(ros_cmd "rm -rf /tmp/o; \
    python3 /tools/to_rerun.py $1 $2 /tmp/o --max-image-rate 15 \
      --images /cam2/infra1/image_rect_raw,/cam2/color/image_raw,/cam1/infra1/image_rect_raw,/cam1/color/image_raw \
      > $3.log 2>&1 && cp /tmp/o/*.mcap $3")" || info "Rerun conversion failed, see $DATA_DIR/${3#/data/}.log"
}

cmd_walk() {
  local name=${1:?name} secs=${2:?seconds} profile=${3:-cam1_stereo_imu}
  resolve_profile "$profile"
  ensure_cams
  [ -e "$DATA_DIR/bags/$name" ] && die "bags/$name exists"
  echo "$(date +%T) $name start" >> "$DATA_DIR/walk.log"
  slam_start "$PROFILE_ARG" "$name" "${VIS:-1}"
  record_start "$name" "${DEPTH:-0}"
  docker rm -f pq_slamrec > /dev/null 2>&1 || true
  docker run -d --name pq_slamrec "${ROSNET[@]}" -v "$DATA_DIR/bags:/out" --entrypoint bash \
    "$CAMERA_IMAGE" -c "$(ros_cmd "exec ros2 bag record -s mcap --storage-preset-profile zstd_fast \
      --include-unpublished-topics -o /out/${name}_slam --regex '$slam_topics_regex' \
      > /out/${name}_slam.log 2>&1")" > /dev/null
  info "walk $name: ${secs} s"
  sleep "$secs"
  cmd_stop all
  mkdir -p "$DATA_DIR/rerun"
  to_rerun "/data/bags/${name}_slam" "/data/bags/$name" "/data/rerun/$name.mcap"
  own "$DATA_DIR/rerun"
  echo "$(date +%T) $name done" >> "$DATA_DIR/walk.log"
  info "done: runs/$name, bags/$name, bags/${name}_slam, rerun/$name.mcap"
}

# ---------------------------------------------------------------------------------------------
find_bag() {
  local b=$1 d
  if [ -d "$b" ]; then readlink -f "$b"; return; fi
  for d in "$DATA_DIR/bags" ${BAGS_DIR:+"$BAGS_DIR"}; do
    [ -d "$d/$b" ] && { readlink -f "$d/$b"; return; }
    # Name of a bag in a category folder (bags/<category>/<bag>).
    local m; m=$(find "$d" -mindepth 2 -maxdepth 3 -type d -name "$b" 2>/dev/null | head -1)
    [ -n "$m" ] && { readlink -f "$m"; return; }
  done
  die "bag '$b' not found (folder, or name under $DATA_DIR/bags${BAGS_DIR:+ or $BAGS_DIR})"
}

cmd_replay() {
  local bag="" profile="" name="" mode=realtime rate=""
  while [ $# -gt 0 ]; do
    case $1 in
      --offline) mode=offline ;;
      --name) name=${2:?}; shift ;;
      --rate) rate=${2:?}; shift ;;
      -*) die "replay: unknown option $1" ;;
      *) if [ -z "$bag" ]; then bag=$1; elif [ -z "$profile" ]; then profile=$1
         else die "replay: too many arguments"; fi ;;
    esac
    shift
  done
  [ -n "$bag" ] || die "replay <bag> [profile|dual] [--offline] [--name <name>] [--rate <r>]"
  bag=$(find_bag "$bag")
  [ -f "$bag/metadata.yaml" ] || die "$bag is not a ros2 bag (no metadata.yaml)"
  resolve_profile "${profile:-cam2_stereo_imu}"
  [ "$PROFILE_ARG" = dual ] && [ $mode = offline ] && die "dual runs in real time only"
  name=${name:-$(basename "$bag")_$(basename "${PROFILE_ARG%.yaml}")_${mode}_$VERSION}
  # Offline: every frame, at twice the recorded speed (the recorder of the output keeps up).
  [ -n "$rate" ] || rate=$([ $mode = offline ] && echo 2.0 || echo 1.0)
  need_image "$IMAGE"
  mkdir -p "$DATA_DIR/runs"
  info "replay $(basename "$bag"), $PROFILE_ARG, $mode, cuVSLAM $VERSION -> $DATA_DIR/runs/$name"
  # No network: the replay cannot see (or disturb) the cameras or other replays.
  local tty=(); [ -t 0 ] && tty=(-it)
  docker run --rm "${tty[@]}" --init --network none --ipc private "${GPU[@]}" \
    -v "$bag:/bag:ro" -v "$DATA_DIR/runs:/out" -v "$REPO/tools:/tools:ro" "${PROFILE_MOUNT[@]}" \
    -e MODE="$mode" --entrypoint bash "$IMAGE" -c \
    "$(ros_cmd "/tools/record_slam.sh /bag /out/$name $PROFILE_ARG $rate \
      /cam2/infra1/image_rect_raw,/cam2/color/image_raw,/cam1/infra1/image_rect_raw,/cam1/color/image_raw")" \
    || { own "$DATA_DIR/runs/$name"; die "replay failed (logs in $DATA_DIR/runs/$name)"; }
  own "$DATA_DIR/runs/$name"
  info "Rerun: $DATA_DIR/runs/$name/rerun/*.mcap"
}

cmd_bench() {
  local bags=${BAGS_DIR:-$DATA_DIR/bags}
  mkdir -p "$DATA_DIR/bench"
  "$REPO/tools/benchmark_docker.sh" -i "$IMAGE" -b "$bags" -o "$DATA_DIR/bench" -- "$@"
  own "$DATA_DIR/bench"
}

# ---------------------------------------------------------------------------------------------
cmd_status() {
  local c
  for c in pq_cams pq_slam pq_rec pq_slamrec pq_console; do
    if running "$c"; then
      printf '%-11s running  %s\n' "$c" "$(docker ps --filter "name=^$c\$" --format '{{.Status}}')"
    else
      printf '%-11s -\n' "$c"
    fi
  done
  if running pq_cams; then
    docker exec pq_cams bash -c 'cat /tmp/emitter.log 2>/dev/null || echo "emitters: not set yet"'
  fi
  [ -d "$DATA_DIR" ] && df -h "$DATA_DIR" | awk -v d="$DATA_DIR" 'NR==2{print "disk " d ": " $4 " free of " $2}'
  docker image inspect "$IMAGE" > /dev/null 2>&1 \
    && echo "cuVSLAM $VERSION ($IMAGE); built: $(docker images pq_cuvslam --format '{{.Tag}}' | sort -V | tr '\n' ' ')" \
    || echo "cuVSLAM $VERSION not built (./pq setup --cuvslam $VERSION)"
}

cmd_logs() {
  case ${1:-slam} in
    cams) docker exec pq_cams tail -n 50 -f /tmp/cam1.log /tmp/cam2.log ;;
    slam) local f; f=$(ls -t "$DATA_DIR"/runs/*.log 2>/dev/null | head -1)
          [ -n "$f" ] || die "no SLAM log"; tail -n 50 -f "$f" ;;
    record) local f; f=$(ls -t "$DATA_DIR"/bags/*.log 2>/dev/null | head -1)
            [ -n "$f" ] || die "no recording log"; tail -n 50 -f "$f" ;;
    *) die "logs: cams|slam|record" ;;
  esac
}

cmd_shell() {
  need_image "$IMAGE"
  docker run --rm -it "${GPU[@]}" "${ROSNET[@]}" -v "$DATA_DIR:/data" -v "$REPO/tools:/tools:ro" \
    ${BAGS_DIR:+-v "$BAGS_DIR:/bags:ro"} -w /data "$IMAGE" "${@:-bash}"
}

cmd_test() {
  need_image "$IMAGE"
  docker run --rm "${GPU[@]}" "$IMAGE" bash -c \
    'cd /ws && colcon test --packages-select pq_cuvslam --event-handlers console_cohesion- \
     && colcon test-result --verbose'
}

# ---------------------------------------------------------------------------------------------
# The web app lives on the branch `console`, checked out next to this one (git worktree in
# .pq_console/), with a configuration generated from pq.conf.
CONSOLE_DIR=$REPO/.pq_console

console_config() {
  local gpu; if is_jetson; then gpu='[--runtime, nvidia]'; else gpu='[--gpus, all]'; fi
  local cfg=$CONSOLE_DIR/pq_console/config.yaml
  [ -f "$cfg" ] && [ "${1:-}" != force ] && return
  # Folders the console can browse (read only): BAGS_DIR and BROWSE, the existing ones.
  local d roots=
  for d in $BAGS_DIR $BROWSE; do [ -d "$d" ] && roots+="${roots:+, }\"$d\""; done
  sed -e "s|^data_dir:.*|data_dir: $DATA_DIR|" \
      -e "s|^# bags_dir:.*|bags_dir: $DATA_DIR/bags|" \
      -e "s|^browse_roots:.*|browse_roots: [$roots]|" \
      -e "s|^ros_domain_id:.*|ros_domain_id: $ROS_DOMAIN_ID|" \
      -e "s|^camera_image:.*|camera_image: $CAMERA_IMAGE|" \
      -e "s|^slam_image:.*|slam_image: $IMAGE|" \
      -e "s|^gpu_args:.*|gpu_args: $gpu|" \
      -e "s|^tools_dir:.*|tools_dir: $REPO/tools|" \
      -e "s|^pylib_dir:.*|pylib_dir:|" \
      -e "0,/serial: \"000000000000\"/s//serial: \"$CAM1_SERIAL\"/" \
      -e "0,/serial: \"000000000000\"/s//serial: \"$CAM2_SERIAL\"/" \
      "$CONSOLE_DIR/pq_console/config.example.yaml" > "$cfg"
  # cuVSLAM versions selectable in the console: the built images, the default first.
  local t tags; tags=$(docker images pq_cuvslam --format '{{.Tag}}' | sort -rV)
  if [ -n "$tags" ]; then
    { echo "cuvslam_versions:"
      echo "  \"$VERSION\": {image: $IMAGE}"
      for t in $tags; do [ "$t" = "$VERSION" ] || echo "  \"$t\": {image: pq_cuvslam:$t}"; done
    } >> "$cfg"
  fi
  info "console configuration: $cfg (from pq.conf; ./pq console config regenerates it)"
}

cmd_console() {
  local sub=${1:-start}
  case $sub in
    install)
      [ -d "$CONSOLE_DIR" ] && { info "console already installed (./pq console update)"; return; }
      git -C "$REPO" fetch -q origin console
      git -C "$REPO" worktree add -q --detach "$CONSOLE_DIR" origin/console
      console_config
      info "installed; start with ./pq console start, then from your machine:"
      info "  ssh -N -L 8080:127.0.0.1:8080 <user>@<this host>  and open http://localhost:8080" ;;
    update)
      [ -d "$CONSOLE_DIR" ] || die "console not installed (./pq console install)"
      git -C "$REPO" fetch -q origin console
      git -C "$CONSOLE_DIR" checkout -q --detach origin/console
      "$CONSOLE_DIR/pq_console/run.sh" restart ;;
    config)
      [ -d "$CONSOLE_DIR" ] || die "console not installed (./pq console install)"
      local cfg=$CONSOLE_DIR/pq_console/config.yaml
      [ -f "$cfg" ] && cp "$cfg" "$cfg.bak"
      console_config force ;;
    start|stop|restart|logs)
      [ -d "$CONSOLE_DIR" ] || die "console not installed (./pq console install)"
      "$CONSOLE_DIR/pq_console/run.sh" "$sub" ;;
    uninstall)
      [ -d "$CONSOLE_DIR" ] || { info "console not installed"; return; }
      # Stop the console only if it is the one of this checkout.
      if docker inspect pq_console --format '{{range .Mounts}}{{.Source}} {{end}}' 2>/dev/null \
          | grep -q "$CONSOLE_DIR"; then
        "$CONSOLE_DIR/pq_console/run.sh" stop || true
      fi
      git -C "$REPO" worktree remove --force "$CONSOLE_DIR"
      info "console removed" ;;
    *) die "console: install|update|config|start|stop|restart|logs|uninstall" ;;
  esac
}

# ---------------------------------------------------------------------------------------------
main() {
  # --cuvslam <version> (or --cuvslam=<version>) anywhere on the command line.
  CUVSLAM_ARG=
  local rest=()
  while [ $# -gt 0 ]; do
    case $1 in
      --cuvslam) CUVSLAM_ARG=${2:?--cuvslam <version>}; shift ;;
      --cuvslam=*) CUVSLAM_ARG=${1#--cuvslam=} ;;
      *) rest+=("$1") ;;
    esac
    shift
  done
  set -- "${rest[@]+"${rest[@]}"}"
  local cmd=${1:-help}; shift || true
  case $cmd in
    help|-h|--help) usage; return ;;
    setup) cmd_setup "$@"; return ;;
  esac
  load_conf
  case $cmd in
    profiles) cmd_profiles ;;
    versions) cmd_versions ;;
    cams) cmd_cams "$@" ;;
    slam) cmd_slam "$@" ;;
    record) cmd_record "$@" ;;
    walk) cmd_walk "$@" ;;
    stop) cmd_stop "$@" ;;
    replay) cmd_replay "$@" ;;
    bench) cmd_bench "$@" ;;
    status) cmd_status ;;
    logs) cmd_logs "$@" ;;
    shell) cmd_shell "$@" ;;
    test) cmd_test ;;
    console) cmd_console "$@" ;;
    *) usage; die "unknown command '$cmd'" ;;
  esac
}

# On one line: bash reads the script while running it, so a change of this file (git pull)
# during a long command (walk) must not be read after main returns.
main "$@"; exit $?
