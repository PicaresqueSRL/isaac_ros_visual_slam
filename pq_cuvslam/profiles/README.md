# Camera layout profiles

A profile describes one camera configuration: cameras and streams, IMU, depth, extrinsics
(with their source), cuVSLAM mode and node parameters. The node loads it at `configure`
(parameter `profile`: the name of a file in this folder, or a path):

```bash
ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu
ros2 launch pq_cuvslam slam.launch.py profile:=/path/to/my_profile.yaml bag:=<bag>
```

The values of the profile become node parameters (visible with `ros2 param get`); they
override the parameters given at launch. The input topics are taken from the profile, so no
remapping is needed. To change profile at runtime: `deactivate`, `cleanup`, set `profile`,
`configure`, `activate`.

## Profiles

| Profile | Mode | Streams | cuVSLAM 15 |
|---|---|---|---|
| `cam2_stereo_imu` | inertial | cam2 IR1 + IR2, cam2 IMU | yes (benchmark configuration) |
| `cam1_cam2_ir` | multicamera | cam2 IR1 + IR2, cam1 IR1 + IR2 | yes |
| `cam2_rgbd` | rgbd | cam2 RGB + aligned depth | yes |
| `cam2_rgb_mono` | mono | cam2 RGB | no: Mono not implemented in the node yet |
| `cam2_rgbd_imu` | multisensor | cam2 RGB + aligned depth + IMU | no: needs cuVSLAM >= 17 |

## Format

```yaml
name: cam2_stereo_imu                 # required
description: free text
mode: inertial                        # multicamera | inertial | rgbd | mono | multisensor
base_frame: cam2_link                 # frame whose poses are published

cameras:                              # order = cuVSLAM camera index; stereo pair: left, right
  - name: cam2_ir1                    # unique
    image_topic: /cam2/infra1/image_rect_raw
    camera_info_topic: /cam2/infra1/camera_info
    optical_frame: cam2_infra1_optical_frame   # overrides the frame of the camera_info

imu:                                  # required for inertial and multisensor
  topic: /cam2/imu
  frame: cam2_gyro_optical_frame
  noise:
    gyro_noise_density: 0.000244      # rad/s/sqrt(Hz)
    gyro_random_walk: 0.000019393     # rad/s^2/sqrt(Hz)
    accel_noise_density: 0.001862     # m/s^2/sqrt(Hz)
    accel_random_walk: 0.003          # m/s^3/sqrt(Hz)
    frequency: 200.0                  # Hz
    source: default                   # where the values come from: default, allan_variance, kalibr...
    date: null                        # date of the measurement

depth:                                # required for rgbd and multisensor
  topic: /cam2/aligned_depth_to_color/image_raw
  camera_index: 0                     # camera the depth is aligned to
  scale: 1000.0                       # raw depth units per meter (D455: millimeters)
  stereo_tracking: false

extrinsics:                           # transforms missing from the TF of the data
  - parent: cam2_link
    child: cam1_link
    translation: [-0.0134, 0.0, -0.0634]       # m
    rotation_rpy: [0.0, 0.8604, 0.0]           # rad; or rotation_rpy_deg, or
                                               # rotation_quaternion: [x, y, z, w]
    source: CAD + gyroscope estimate           # required
    date: null

hardware:
  ir_emitter: false                   # D455 IR projector during the recording (warning if true)

parameters:                           # any other node parameter, applied last
  rectified_images: true
  image_jitter_threshold_ms: 150.0
```

The profile sets these node parameters: `tracking_mode` (from `mode`), `base_frame`,
`num_cameras` and `min_num_images` (number of cameras), `camera_optical_frames`; with `imu`:
`imu_frame`, the noise parameters and `calibration_frequency`; with `depth`:
`depth_camera_id`, `depth_scale_factor`, `depth_enable_stereo_tracking`; then everything in
`parameters` (an unknown parameter name is an error).

## Checks

At `configure` the node rejects the profile, with a message naming the file and the key, if:
a key is unknown or missing, a value has the wrong type, the number of cameras does not fit
the mode (multicamera: stereo pairs; inertial: exactly one pair), a required section is
missing, the mode is not available in the cuVSLAM version of the build, names or topics are
duplicated.

It warns if: the IMU noise values are defaults in inertial mode (`imu.noise.source: default`),
the IR emitter was on, a section is ignored by the mode.

When the tracker is initialized (first camera info) the node checks that every optical frame
and the IMU frame are reachable in TF from `base_frame`; until then it waits and reports the
missing frames.
