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

| Profile | Mode | Streams | cuVSLAM |
|---|---|---|---|
| `cam2_stereo_imu` | inertial | cam2 IR1 + IR2, cam2 IMU | 15-17 (benchmark configuration) |
| `cam1_stereo_imu` | inertial | cam1 IR1 + IR2, cam1 IMU (down-looking) | 15-17 |
| `cam1_cam2_ir` | multicamera | cam2 IR1 + IR2, cam1 IR1 + IR2 | 15-17 |
| `cam2_rgbd` | rgbd | cam2 RGB + aligned depth | 15-17 |
| `cam2_rgb_mono` | mono | cam2 RGB | 15-17; trajectory without metric scale |
| `cam2_rgbd_imu` | multisensor | cam2 RGB + aligned depth + IMU | 17 with cuNLS; experimental in cuVSLAM |
| `cam1_cam2_rgb_imu` | multisensor | cam2 RGB + cam1 RGB + cam2 IMU, no IR, no depth | 17 with cuNLS; experimental in cuVSLAM |
| `cam1_cam2_ir_imu` | multisensor | cam2 + cam1 IR stereo, cam2 IMU: one SLAM | 17 with cuNLS; experimental in cuVSLAM |
| `cam1_cam2_rgbd` | multisensor | cam2 + cam1 RGB with aligned depth: one SLAM | 17 with cuNLS; experimental in cuVSLAM |
| `cam1_cam2_rgbd_imu` | multisensor | cam2 + cam1 RGB with aligned depth, cam2 IMU: one SLAM | 17 with cuNLS; experimental in cuVSLAM |
| `cam1_cam2_ir_rgb_depth_imu` | multisensor | cam2 + cam1 IR stereo, RGB and depth (on IR1), cam2 IMU: one SLAM | 17 with cuNLS; experimental in cuVSLAM |

The `aligned_depth_to_color` topics come from the RealSense driver with
`align_depth.enable:=true`; for bags recorded without it, `tools/align_depth.py` computes them
from the depth and the factory calibration in the bag.

## Format

```yaml
name: cam2_stereo_imu                 # required
description: free text
mode: inertial                        # multicamera | inertial | rgbd | mono | multisensor
base_frame: cam2_link                 # frame whose poses are published

cameras:                              # order = cuVSLAM camera index; stereo pair: left, right
  - name: cam2_ir1                    # unique
    device: cam2                      # physical camera; required with sync
    image_topic: /cam2/infra1/image_rect_raw
    camera_info_topic: /cam2/infra1/camera_info
    optical_frame: cam2_infra1_optical_frame   # overrides the frame of the camera_info

imu:                                  # required for inertial, optional for multisensor
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

depth:                                # required for rgbd; multisensor: depth or >= 2 cameras
  topic: /cam2/aligned_depth_to_color/image_raw
  camera_index: 0                     # camera the depth is aligned to
  scale: 1000.0                       # raw depth units per meter (D455: millimeters)
  stereo_tracking: false
# multisensor: a list of depth images, one per camera (same scale and stereo_tracking):
# depth:
#   - {topic: /cam2/depth/image_rect_raw, camera_index: 0, scale: 1000.0}
#   - {topic: /cam1/depth/image_rect_raw, camera_index: 3, scale: 1000.0}

# Per camera, optional: virtual camera given to cuVSLAM (impl/virtual_camera.hpp).
#   undistort: true                     undistorted to pinhole (cuNLS / Multisensor: pinhole only)
#   pad_top: 200                        empty band on top (principal point moved down) or at the
#   pad_bottom: 200                     bottom: a wider declared view for the 50 % frustum
#                                       overlap check (all cameras must have the same size)
#   pad_left: 104                       empty bands on the sides: e.g. a 640x480 RGB widened to
#   pad_right: 104                      848x480, the size of the IR images
# A depth image goes through the virtual camera of its camera (nearest-neighbour sampling).

extrinsics:                           # transforms missing from the TF of the data
  - parent: cam2_link
    child: cam1_link
    translation: [-0.0134, 0.0, -0.0634]       # m
    rotation_rpy: [0.0, 0.8604, 0.0]           # rad; or rotation_rpy_deg, or
                                               # rotation_quaternion: [x, y, z, w]
    source: CAD + gyroscope estimate           # required
    date: null

sync:                                 # between devices (multicamera, multisensor); optional
  mode: software                      # none | software | hardware
  max_offset_ms: 34.0                 # max capture time difference of paired frames
                                      # (default 34 for software, 2 for hardware)
  rotation_compensation: true         # software only: warp to the reference capture time
  compensation_border_px: 16          # border excluded from tracking on warped images
  compensation_min_deg: 0.1           # smaller rotations are not compensated (under one pixel:
                                      # warping every image costs CPU time on the Orin Nano)
  devices:                            # first = reference device (rig frame timestamps)
    - name: cam2
      imu_topic: /cam2/imu            # gyroscope, needed for rotation_compensation
      imu_frame: cam2_gyro_optical_frame
      clock_offset_ms: 0.0            # subtracted from the stamps of this device

hardware:
  ir_emitter: false                   # D455 IR projector during the recording (warning if true)

parameters:                           # any other node parameter, applied last
  rectified_images: true
  image_jitter_threshold_ms: 150.0
```

The profile sets these node parameters: `tracking_mode` (from `mode`), `base_frame`,
`num_cameras` and `min_num_images` (number of cameras), `camera_optical_frames`; with `imu`:
`imu_frame`, the noise parameters and `calibration_frequency`; with `depth`:
`depth_camera_id` (multisensor: also `depth_camera_ids`), `depth_scale_factor`,
`depth_enable_stereo_tracking`; then everything in
`parameters` (an unknown parameter name is an error).

## Synchronization between devices

Two D455 that are not triggered together capture at slightly different rates: the capture
time difference changes slowly along the recording and reaches half a frame (33 ms at 15 fps).
cuVSLAM needs the images of a rig frame to be simultaneous.

- `none`: images grouped by timestamp within `sync_matching_threshold_ms` (Isaac ROS
  behaviour); the rig frame takes the latest stamp.
- `software`: the images of each device are grouped first (IR1 and IR2 share the shutter);
  then each frame of the reference device is paired with the nearest frame of every other
  device within `max_offset_ms` (a frame can serve two reference frames when frames are
  missing); the stale first frame of the driver is discarded. With `rotation_compensation`
  the images of the other devices are warped to the reference capture time with the rotation
  integrated from their gyroscope (homography `K dR^T K^-1`; translation not compensated),
  and their border is excluded from tracking. The rig frame takes the reference timestamp.
- `hardware`: same pairing with a tight threshold, for cameras triggered by the sync cable.

`status.csv` gets a `sync_offset_ms` column and `run_info.json` a `sync` section (pairs,
dropped frames, offsets, compensated rotation).

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
