# Docker

One Dockerfile for desktop PC and Jetson. It installs ROS 2 Lyrical on
Ubuntu 24.04 and builds the workspace (`pq_cuvslam`, the interfaces and
cuVSLAM from the `cuvslam` submodule) with colcon.

## Build

From the repository root, with the submodule initialized:

```bash
git submodule update --init
docker build -f docker/Dockerfile -t pq_cuvslam:dev .
```

Build arguments:

| Argument | Default | Notes |
|---|---|---|
| `BASE_IMAGE` | `nvidia/cuda:13.1.1-devel-ubuntu24.04` | CUDA devel image on Ubuntu 24.04. Jetson: JetPack 7.2 based image, to be defined. |
| `CUVSLAM_REF` | (empty) | cuVSLAM version to build: git tag or commit (`v15.0.0`, `v16.0.0`, `v17.0.1`, ...); empty: the `cuvslam` submodule (15.0.0). Supported: 15 to 17. |
| `USE_CUNLS` | `AUTO` | cuNLS solver: `AUTO` = on from cuVSLAM 17 (needed by Multisensor), off before (not supported there, and off in the Isaac ROS node). |
| `USE_CUDA_BUFFER` | `AUTO` | Zero-copy images through Isaac ROS `cuda_buffer`, enabled if the package is installed. Not installed by this image, so images go to cuVSLAM in host memory. |
| `BUILD_TYPE` | `Release` | CMake build type. |

Outside Docker, build with `colcon build --packages-ignore cuVSLAM`: the
`cuvslam` submodule is a plain CMake project that colcon would otherwise build
a second time as a separate package (pq_cuvslam already builds it).

Example, cuVSLAM 17: `docker build -f docker/Dockerfile --build-arg CUVSLAM_REF=v17.0.1 -t pq_cuvslam:cuvslam17 .`
Outside Docker: `colcon build --packages-ignore cuVSLAM --cmake-args -DCUVSLAM_REF=v17.0.1`
(or `-DCUVSLAM_SOURCE_DIR=<checkout>`). The node logs the cuVSLAM version and its features at
`configure`, refuses a library that differs from the one of the build, and writes both to
`run_info.json`.

GPU architecture: cuVSLAM 15 compiles its CUDA kernels for all architectures
supported by the toolkit (`-arch=all`), so the image runs on any supported
NVIDIA GPU. Selecting a single architecture would require changing cuVSLAM.

## Run

```bash
docker run --rm -it --gpus all --network host --ipc host \
  -v /path/to/bags:/bags \
  pq_cuvslam:dev
```

Inside the container ROS 2 and the workspace are already sourced:

```bash
ros2 launch pq_cuvslam pq_cuvslam.launch.py
```

## Bag replay

Deterministic replay of a bag (every frame processed, as fast as possible, tracker in
`offline` mode), with the run output written to a folder:

```bash
docker run --rm --gpus all -v /path/to/bags:/bags:ro -v /path/to/runs:/out \
  pq_cuvslam:dev ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu \
    bag:=/bags/<category>/<bag> run_output_dir:=/out/<bag>
```

`processing_mode:=realtime` keeps the background threads of the live node; `max_frames:=N`
stops after N frames. The output files (`odometry.csv`, `slam_poses.csv`, `run_info.json`, ...)
have the format of `run_output.py` and can be compared with `compare_runs.py`.

Simulated online replay: the bag is played at its recorded speed (`rate:=1.0`), the tracker
runs in `realtime` mode and frames that are not processed in time are dropped by keep-last
queues, as with live subscriptions. On the PC, `latency_profile:=<csv>` makes every frame
take at least as long as on the Jetson (a `status.csv` of a Jetson run, or the `frames.csv`
of the Isaac ROS benchmark); `emulated_latency_ms:=N` sets a constant time instead:

```bash
docker run --rm --gpus all -v /path/to/bags:/bags:ro -v /path/to/runs:/out -v /path/to/profiles:/p:ro \
  pq_cuvslam:dev ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu \
    bag:=/bags/<category>/<bag> rate:=1.0 latency_profile:=/p/frames.csv run_output_dir:=/out/<bag>
```

## Lifecycle

The node is a lifecycle (managed) node. With `autostart:=true` (default) it configures and
activates itself; otherwise, or to reconfigure it at runtime:

```bash
ros2 lifecycle get /visual_slam_node
ros2 lifecycle set /visual_slam_node deactivate   # input ignored, tracker and map kept
ros2 lifecycle set /visual_slam_node cleanup      # run output written, tracker destroyed
ros2 param set /visual_slam_node <param> <value>  # applied at the next configure
ros2 lifecycle set /visual_slam_node configure
ros2 lifecycle set /visual_slam_node activate
```

## Tests

```bash
docker run --rm --gpus all pq_cuvslam:dev \
  bash -c 'cd /ws && colcon test --packages-select pq_cuvslam && colcon test-result --verbose'
```
