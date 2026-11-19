# Changes from Isaac ROS

This repository is a fork of
[NVIDIA-ISAAC-ROS/isaac_ros_visual_slam](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam),
release 5.1 (commit `c7c42a0`), with cuVSLAM 15.0.0 (submodule commit `33da606`).
This file records every change made to it. Update it with each change.

## Current differences

| Area | Isaac ROS 5.1 | This fork |
|---|---|---|
| Package | `isaac_ros_cuvslam` | `pq_cuvslam` |
| C++ namespace / plugin | `nvidia::isaac_ros::visual_slam::VisualSlamNode` | `pq::visual_slam::VisualSlamNode` |
| Interfaces | `isaac_ros_visual_slam_interfaces` | unchanged (same name and types) |
| Topics, services, TF | `/visual_slam/...`, `map -> odom -> base` | unchanged |
| Isaac ROS dependencies | `isaac_ros_common`, `isaac_common`, `cuda_buffer` | none required; `cuda_buffer` optional |
| Image transfer | zero-copy GPU (`cuda_buffer`) | GPU with `cuda_buffer`, host memory without (`USE_CUDA_BUFFER`) |
| cuNLS | `USE_CUNLS=OFF` fixed | CMake option, default `OFF`; requires cuVSLAM >= 17 |
| Node type | `rclcpp::Node` | lifecycle node (`configure`/`activate`/`deactivate`/`cleanup`), `autostart` |
| Startup errors | `exit()` / "no CUDA-capable device" | `configure` fails with an explicit message (version, modes, GPU) |
| Processing mode | realtime only (async BA and SLAM) | `processing_mode`: `realtime` (default) or `offline` (reproducible) |
| Multi-camera sync | images grouped by timestamp, rig frame at the latest stamp | per-device pairing (nearest frame) + gyroscope rotation compensation (`sync` in the profile) |
| Bag input | `ros2 bag play` at real-time speed | also `pq_cuvslam_replay`: deterministic (every frame) or simulated online (`--rate`) |
| Latency emulation | none | `latency_profile` / `emulated_latency_ms`: frames take as long as on the target platform |
| Run output | topics only | also CSV + `run_info.json` (`run_output_dir`), format of `run_output.py` |
| Build | Isaac ROS container | `docker/Dockerfile` (ROS 2 Lyrical, Ubuntu 24.04, CUDA 13) |
| Configuration | launch parameters + remappings + `static_transform_publisher` | YAML profiles (`profile`): cameras, topics, frames, IMU, depth, extrinsics with source, mode |
| Modes | 0, 1, 2 | same (multicamera, inertial, rgbd); profiles for mono and multisensor rejected with a clear message |
| Launch files | Isaac Sim, RealSense, ZED, multi-camera, core | standalone node, `slam.launch.py profile:=...` |
| Tests | integration tests on NVIDIA r2b bags | unit tests only (sequencer, vendored isaac_common) |

## Log

### 2026-11-18

**Removed Isaac ROS-specific files** (`2b924c7`)
- Launch files for Isaac Sim, RealSense (stereo, RGBD, mask, multi-camera) and the core
  launch fragment (needs `isaac_ros_examples` and `isaac_ros_image_proc`). Kept the
  standalone `isaac_ros_visual_slam.launch.py` (later renamed, see below).
- `config/`, `foxglove_layouts/`, all RViz configurations except `default.cfg.rviz`.
- Python integration tests, helpers and r2b bags (depend on `isaac_ros_test` and NVIDIA data).
- `CONTRIBUTING.md`, `SECURITY.md`; matching `exec_depend` / `test_depend` in `package.xml`.

**Removed the Isaac ROS build dependencies** (`16f2cf2`)
- `isaac_ros_common/qos.hpp` replaced by `impl/qos.{hpp,cpp}` (same behaviour).
- `isaac_common` (`MessageBuffer`, `MessageStreamSynchronizer`) vendored in
  `include/pq_cuvslam/third_party/isaac_common/` with its unit tests, from `isaac_ros_common`
  release-5.1 (`fddf2d2`). Namespace unchanged, include guards renamed.
- `cuda_buffer` optional: `-DUSE_CUDA_BUFFER=AUTO|ON|OFF` (default `AUTO`). Without it the image
  is passed to cuVSLAM in host memory (`is_gpu_mem=false`). The conversion helpers take an
  `ImagePixels {ptr, is_gpu_mem}` instead of a `cuda_buffer` `ReadHandle`.
- `isaac_ros_common` version-info CMake removed.
- `USE_CUNLS` is a CMake option (default `OFF`, as Isaac ROS).

**Renamed the package** (`3c90a65`)
- `isaac_ros_cuvslam` -> `pq_cuvslam`: package, directory, executable, include directory and
  guards. Namespace `nvidia::isaac_ros::visual_slam` -> `pq::visual_slam`, so that this node and
  the Isaac ROS one can be loaded in the same process. Launch file `pq_cuvslam.launch.py`.

**Build fixes from the first container build** (`9541cf4`)
- `isaac_ros_visual_slam_interfaces`: `isaac_ros_common` build dependency and version-info
  removed; `ament_cmake_auto` declared. Messages and services unchanged.
- cuVSLAM version read from the submodule `VERSION`; `USE_CUNLS=ON` with cuVSLAM < 17 fails
  with a clear message (cuVSLAM 15 aborts with "cuNLS is not supported yet").
- Deprecated `ament_target_dependencies` call removed from the sequencer test.

**Docker build** (`89f8c45`)
- `docker/Dockerfile`: one file for PC and Jetson (`BASE_IMAGE`), ROS 2 Lyrical from the Isaac
  ROS buildfarm, rosdep keys resolved as `ubuntu:resolute`, colcon ignores the `cuvslam`
  submodule (it is built by `pq_cuvslam`). See `docker/README.md`.

**cam2 launch and bag runner** (`16edd93`)
- `pq_cuvslam/launch/cam2_stereo_imu.launch.py`: parameters of the Isaac ROS 5 reference runs.
- `tools/run_bag.sh`: plays a bag at real-time speed and records the output topics, as
  `slam_cam2.sh`.

**Deterministic replay and run output** (`43d66b1`)
- New parameter `processing_mode`: `realtime` (default, unchanged behaviour) or `offline`.
  Offline sets `Odometry::Config::async_sba = false` and `Slam::Config::sync_mode = true`:
  bundle adjustment and SLAM run inside the tracking call, so the result does not depend on
  thread timing.
- New executable `pq_cuvslam_replay --bag <bag> [--max-frames N]`: reads the bag with
  `rosbag2_cpp` and feeds the messages to the node in recorded order, without executor and
  without drops. Input topics are the resolved subscription names, so the same parameters and
  remappings as the live node apply. `/tf_static` is loaded from the whole bag first.
- New public methods on `VisualSlamNode` for direct input (`InputImage`, `InputCameraInfo`,
  `InputImu`, `InputStaticTransforms`) and topic introspection.
- New parameter `run_output_dir`: the node writes `odometry.csv`, `slam_online.csv`,
  `slam_poses.csv`, `status.csv`, `slam_metrics.csv`, `loop_closures.csv` and `run_info.json`
  in the format of `VRHealth_Challenge13/scripts/run_output.py` (`impl/run_output.{hpp,cpp}`).
  Written at the end of the replay or when the node is destroyed. `run_info.json` records the
  cuVSLAM version and the wrapper commit (`git describe` at configure time).
- `cam2_stereo_imu.launch.py`: arguments `bag`, `processing_mode`, `run_output_dir`,
  `max_frames`; with `bag` it starts the replay instead of the live container.
- New dependencies: `rosbag2_cpp`, `rosbag2_storage_mcap` (exec).
- Apache 2.0 headers (`Copyright (c) 2026 Picaresque SRL`) on the new files;
  `pose_cache.cpp` reformatted with `ament_uncrustify` (upstream file, whitespace only).
  All linters and unit tests pass.
- Verified on `building_in_108` and `loop_closure`: two offline replays give byte-identical
  CSV files (except the measured track times). Replay speed on the PC: about 85-90 frames/s
  offline, 155 frames/s realtime (bags recorded at 15 fps).

**Lifecycle node and simulated online replay** (`d164dc7`)
- `VisualSlamNode` is a `rclcpp_lifecycle::LifecycleNode`. Parameters are declared in the
  constructor and read at every `configure`, which also checks the cuVSLAM version, the modes
  and the GPU (explicit error with hints for containers and Jetson device permissions) and
  creates the interfaces and the tracker. `activate` processes input, `deactivate` drops it
  (tracker and map kept), `cleanup` writes the run output, saves the map if requested and
  destroys everything. Invalid parameters make `configure` fail instead of exiting.
- `autostart` parameter (default `true`): the node configures and activates itself when it
  starts spinning, as the Isaac ROS node does; launch files and scripts are unchanged.
- The run output is not written when no frame was processed (an empty cycle does not
  overwrite a previous run). A second run after `cleanup`/`configure` with the same
  `run_output_dir` does overwrite the first one.
- `pq_cuvslam_replay --rate R`: simulated online replay. Messages arrive at their recorded
  times in keep-last queues with the history depth of the node subscriptions, so frames not
  processed in time are dropped as with live subscriptions. Received and dropped messages per
  topic are written to `run_info.json` (`replay`).
- Latency emulation: `latency_profile` (`status.csv` of a run on the target platform, or
  `frames.csv` of the Isaac ROS benchmark) or `emulated_latency_ms`. After each frame the node
  waits until the frame has taken as long as on the target platform; frames of the same bag
  are matched by timestamp, the others draw a time from the profile. Statistics in
  `run_info.json` (`latency_emulation`).
- `status.csv` has a new last column `callback_time_ms` (whole frame callback, measured
  without the emulation), to build latency profiles.
- Launch arguments `rate`, `latency_profile`, `emulated_latency_ms`; new dependencies
  `rclcpp_lifecycle`, `lifecycle_msgs`.
- Verified: deterministic replay output unchanged; live run with `ros2 bag play` and
  `ros2 lifecycle` transitions (deactivate, cleanup, reconfigure, failing configure with an
  invalid parameter); simulated online replay of `building_in_108` with the Jetson profile
  (no drops: median 4.7 ms per frame on the Orin Nano) and with 100 ms per frame (223 of 300
  frames processed).

**Camera layout profiles** (`e2765e3`)
- YAML profiles in `pq_cuvslam/profiles/` (format in `profiles/README.md`): cameras with
  topics and optical frames, IMU with the noise model and its source/date, aligned depth,
  extrinsics missing from the data TF with their source/date, IR emitter state, cuVSLAM mode,
  other node parameters.
- Parameter `profile` (installed name or path): at `configure` the profile is loaded with
  yaml-cpp, checked, and turned into node parameters; subscriptions use its topics (no
  remapping). Errors name the file and the key; warnings for default IMU noise in inertial
  mode, IR emitter on, ignored sections. Mono: rejected (not implemented in the node yet,
  although cuVSLAM 15 supports it); multisensor: rejected (cuVSLAM >= 17).
- Profile extrinsics go into the TF buffer of the node and are published on `/tf_static`, in
  place of the `static_transform_publisher` between the two D455.
- At tracker initialization the node checks that all optical frames and the IMU frame are
  reachable from `base_frame`, and reports the missing ones while waiting (before, a missing
  frame silently gave an identity transform after a 10 s timeout).
- `run_info.json` records the profile.
- Profiles: `cam2_stereo_imu`, `cam1_cam2_ir`, `cam2_rgbd`, `cam2_rgb_mono`, `cam2_rgbd_imu`.
- `slam.launch.py` replaces `cam2_stereo_imu.launch.py`; `tools/run_bag.sh` uses it and plays
  the topics of both cameras. New dependencies: `yaml_cpp_vendor` (yaml-cpp), `ament_index_cpp`.
- Verified: `cam2_stereo_imu` gives byte-identical trajectories to the previous configuration;
  `cam1_cam2_ir` on `loop_closure`: deterministic, rigid RMSE to ground truth 1.20 m (Isaac
  ROS reference 1.18 m); `cam2_rgbd` on `building_in_108`: 0 frames lost, RMSE 0.80 m.

### 2026-11-19

**Bag analysis tools** (`ab256be`)
- `tools/analyze_sync.py`: timing of the two cameras (gaps, stale stamps, capture phase and its
  drift, clock offset from the gyroscopes).
- `tools/check_rotation_compensation.py`: validation of the gyroscope rotation compensation
  on consecutive frames, and gyroscope - image time offset scan.
- `tools/analyze_images.py`: saturation, dark pixels and projector dot density per IR stream.

**Software sync between devices** (`479b7e9`)
- Profile: `cameras[].device` and a `sync` section (`none` / `software` / `hardware`,
  `max_offset_ms`, `rotation_compensation`, `compensation_border_px`, devices with gyroscope
  and clock offset).
- `DeviceFrameSynchronizer`: images grouped per device, then each reference frame (cam2) paired
  with the nearest frame of the other devices; the chosen frame is not consumed (with missing
  frames, consuming it shifted all the following pairs by one frame); stale and incomplete
  device frames dropped; rig frame at the reference timestamp.
- Rotation compensation: images of the other devices warped to the reference capture time with
  `H = K dR^T K^-1` (gyroscope of their device), border excluded from tracking.
- `status.csv` column `sync_offset_ms`, `run_info.json` section `sync`. `cam1_cam2_ir` uses
  software sync with compensation by default. New dependency: OpenCV (core, imgproc).
- `tools/gyro_consistency.py`: ground-truth-free check (odometry rotation vs gyroscope).
- Evaluation on 13 bags (deterministic replay, rotation error vs gyroscope, RMS per frame):
  lower with compensation on 9 bags (`build_in_out` 0.258 -> 0.092 deg, `build_out_in`
  0.267 -> 0.102, `staircase_fix_1` 0.385 -> 0.247, `low_detail` 0.827 -> 0.579), about the
  same on 3, higher on `building_in_108` (0.787 -> 0.876). Pairing without compensation alone
  is not better than the Isaac ROS grouping.

## Notes

- Frames processed: only complete stereo pairs reach the tracker (e.g. `loop_closure`: 826
  infra1 and 823 infra2 images, 804 pairs). The replay processes all of them except the
  first, which arrives before the tracker is initialized (as in the live node), and the
  last, which waits for a later IMU message.

- IMU queue: with the default QoS (keep-last 10, i.e. 50 ms at 200 Hz) a frame that takes
  more than about 50 ms makes the IMU queue overflow; in the 100 ms stress test 4233 of 4264
  IMU messages were dropped. The same happens with live subscriptions: on a slow platform
  `imu_qos_depth` should be increased.
- Recorded bags (see `tools/`): the first message of every image stream has a stale stamp
  (21-486 s old); most "15 fps" bags are on a 30 fps grid with about half of the frames missing
  (intervals of 33, 67 and 100 ms); several bags have saturated IR or the IR projector on.
- `slam_poses.csv` of the Isaac ROS reference runs (`extract_paths.py`, last `slam_path`
  message) is the sequence of per-frame SLAM poses, capped at `path_max_size` (1024). The
  equivalent file here is `slam_online.csv`; `slam_poses.csv` is the final optimized trajectory
  (`GetAllSlamPoses`), as in `run_output.py`.
