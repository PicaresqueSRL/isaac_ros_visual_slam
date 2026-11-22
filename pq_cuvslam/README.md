# pq_cuvslam

ROS 2 node for cuVSLAM (versions 15 to 17), derived from the Isaac ROS 5.1 node
(`isaac_ros_cuvslam`). Lifecycle node, camera layout profiles, deterministic and simulated
online bag replay, software sync between two D455, run output for offline analysis and
diagnostics. All changes from Isaac ROS are listed in `../CHANGES.md`.

## Run

```bash
# Live (topics of the profile), as a composable node with autostart.
ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu

# Deterministic replay of a bag (every frame, reproducible) with the run output.
ros2 launch pq_cuvslam slam.launch.py profile:=cam1_cam2_ir bag:=<bag> run_output_dir:=<dir>

# Simulated online replay at the recorded speed, emulating the Jetson processing time.
ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu bag:=<bag> rate:=1.0 \
  latency_profile:=<status.csv or frames.csv>
```

Profiles: `profiles/` (format in `profiles/README.md`). Build and container: `../docker/`.
Benchmark and analysis tools: `../tools/`.

## Parameters added to the Isaac ROS node

| Parameter | Default | Meaning |
|---|---|---|
| `profile` | `""` | camera layout profile (installed name or YAML path) |
| `autostart` | `true` | configure and activate at startup (lifecycle) |
| `processing_mode` | `realtime` | `offline`: bundle adjustment and SLAM in the tracking call, reproducible runs |
| `run_output_dir` | `""` | write CSV files and `run_info.json` there |
| `latency_profile`, `emulated_latency_ms` | `""`, `0` | emulated processing time per frame |
| `enable_frame_diagnostics` | `true` | publish `visual_slam/diagnostics/frame` (when it has subscribers) |
| `diagnostics_pixel_stride` | `4` | image statistics on one pixel every N in both directions |
| `imu_qos_depth` | `400` (Isaac ROS: 0, i.e. the SENSOR_DATA depth 5) | IMU subscription queue; a short queue loses IMU samples when a frame is slow, and the real-time VIO diverges on the Orin Nano |
| `multisensor_use_imu` | `true` | Multisensor mode: fuse the IMU |
| `health_max_speed_mps` | `3.0` | odometry increments faster than this are rejected (guarded odometry) |
| `health_max_gyro_error_deg` | `5.0` | increments whose rotation differs from the gyroscope by more are rejected |
| `health_min_landmarks` | `10` | fewer landmarks: health degraded |
| `health_reset_bad_frames` | `8` | reset of the tracker after this many rejected or lost frames within 1 s, at most every 2 s (0: never); the SLAM map is lost, the guarded odometry continues |

## Topics

The Isaac ROS topics are unchanged (`visual_slam/tracking/odometry`, `vo_path`, `slam_path`,
`visual_slam/status`, TF `map -> odom -> base`, `visual_slam/vis/*`, services). Added:

| Topic | Type | Content |
|---|---|---|
| `visual_slam/diagnostics/frame` | `pq_cuvslam_interfaces/FrameDiagnostics` | per frame: mode, profile, tracked, Track() and callback time, latency (live), sync offset, observations per camera, landmarks, loop closure, statistics of every input image (`StreamStats`: brightness, saturated and dark pixels, valid depth), health of the odometry (ok, degraded, rejected, lost; speed, rotation error against the gyroscope) |
| `visual_slam/tracking/odometry_guarded` | `nav_msgs/Odometry` | odometry with the rejected increments removed: for a rejected or lost frame the position is kept and the rotation of the gyroscope applied (no jump is carried into the trajectory) |
| `visual_slam/events` | `pq_cuvslam_interfaces/SlamEvent` | configured, tracker initialized, tracking lost / recovered, loop closure, odometry rejected / accepted again, tracker reset (reliable, transient local) |

With `run_output_dir` the same information is written to `status.csv`, `streams.csv`,
`events.csv`, `health.csv`, `odometry_guarded.csv` and summarized in `run_info.json`.

Note: on fully saturated images cuVSLAM can report a successful tracking with no landmarks;
the saturation of the streams is the signal to watch.
