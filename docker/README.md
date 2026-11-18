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
| `USE_CUNLS` | `OFF` | cuNLS solver; requires cuVSLAM >= 17. OFF as in the Isaac ROS node. |
| `USE_CUDA_BUFFER` | `AUTO` | Zero-copy images through Isaac ROS `cuda_buffer`, enabled if the package is installed. Not installed by this image, so images go to cuVSLAM in host memory. |
| `BUILD_TYPE` | `Release` | CMake build type. |

Outside Docker, build with `colcon build --packages-ignore cuVSLAM`: the
`cuvslam` submodule is a plain CMake project that colcon would otherwise build
a second time as a separate package (pq_cuvslam already builds it).

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
  pq_cuvslam:dev ros2 launch pq_cuvslam cam2_stereo_imu.launch.py \
    bag:=/bags/<category>/<bag> run_output_dir:=/out/<bag>
```

`processing_mode:=realtime` keeps the background threads of the live node; `max_frames:=N`
stops after N frames. The output files (`odometry.csv`, `slam_poses.csv`, `run_info.json`, ...)
have the format of `run_output.py` and can be compared with `compare_runs.py`.

## Tests

```bash
docker run --rm --gpus all pq_cuvslam:dev \
  bash -c 'cd /ws && colcon test --packages-select pq_cuvslam && colcon test-result --verbose'
```
