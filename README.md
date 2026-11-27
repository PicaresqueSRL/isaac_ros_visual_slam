# pq_cuvslam

Visual-inertial SLAM with [cuVSLAM](https://github.com/nvidia-isaac/cuVSLAM) (versions 15 to 17)
for a rig of two Intel RealSense D455, on a desktop PC with an NVIDIA GPU or on a Jetson Orin
Nano. ROS 2 Lyrical, everything in Docker.

The project is the `pq_cuvslam` ROS 2 package. It started as a fork of the Isaac ROS Visual SLAM
node (release 5.1), which is why the repository is still called `isaac_ros_visual_slam`. The
original Isaac ROS package has been replaced; every change is listed in [CHANGES.md](CHANGES.md).

## Quick start

Requirements: an NVIDIA GPU with its driver, git and Docker with the [NVIDIA Container
Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) (on the Jetson,
JetPack 7.2 has them). Everything else (ROS 2, cuVSLAM, the RealSense driver) is installed in
the image by `./pq setup`. On a new Ubuntu PC:

```bash
sudo ubuntu-drivers install                 # NVIDIA driver, then reboot; check with nvidia-smi
sudo apt install -y git curl
curl -fsSL https://get.docker.com | sudo sh  # Docker
sudo usermod -aG docker $USER               # then log out and in again
# NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

Then:

```bash
git clone --recursive https://github.com/PicaresqueSRL/isaac_ros_visual_slam.git
cd isaac_ros_visual_slam
./pq setup          # builds cuVSLAM 17.0.1 into the image pq_cuvslam:17.0.1, creates pq.conf
```

`./pq setup` detects the platform (PC or Jetson) and uses the right build options. The first
build takes about 20 minutes on a PC and 30-45 minutes on the Orin Nano. Run it again after a
`git pull` (`--no-cache` rebuilds every step from scratch).

Local settings (data folder, camera serial numbers) are in `pq.conf`. It is created from
[pq.conf.example](pq.conf.example) and is not tracked by git.

## Use

`./pq help` lists all the commands.

### cuVSLAM versions

The default is cuVSLAM 17.0.1 (the `cuvslam` submodule). Every version is built into its own
image, `pq_cuvslam:<version>`, so several versions can be on the same machine, and every
command takes `--cuvslam <version>` to choose one:

```bash
./pq setup                          # 17.0.1
./pq setup --cuvslam 15.0.0         # also 15.0.0 (the version of the Isaac ROS node)
./pq setup --cuvslam 15,16          # several at once
./pq versions                       # built and available versions

./pq replay building_in_108                      # 17.0.1
./pq replay building_in_108 --cuvslam 15.0.0     # same bag with 15.0.0
./pq slam cam1_stereo_imu --cuvslam 16
```

A short version is the latest release: `15` = 15.0.1 (same library as 15.0.0, only Python
tool fixes), `16` = 16.0.0, `17` = 17.0.1. Supported: 15 to 17. To change the default, set
`CUVSLAM=<version>` in `pq.conf`. After the first one, building a further version only
rebuilds the workspace (about 5-10 minutes on a PC). The images share their system layers, so
each further version takes about 1-2 GB of disk. A replay writes to
`runs/<bag>_<profile>_<mode>_<version>`, so the runs of different versions sit side by side.

### SLAM on a recorded bag

```bash
./pq replay ~/bags/8_Building/building_in_108                  # cam2 stereo IR + IMU, real time
./pq replay building_in_108 cam1_stereo_imu                    # bag name under bags/ or BAGS_DIR
./pq replay building_in_108 dual                               # VIO of cam1 and cam2 + selector
./pq replay building_in_108 cam1_cam2_ir --offline             # every frame, reproducible
```

By default the bag plays at its recorded speed with the tracker in real-time mode, as it
would run live. With `--offline` every frame is processed and two runs give the same
trajectory. `./pq profiles` lists the camera profiles.

### Live on the Jetson

Put the serial numbers of the two cameras in `pq.conf` (`./pq cams list` shows them). cam1
looks at the floor, cam2 forward.

```bash
./pq cams start                    # both D455, IR emitter off
./pq slam                          # live SLAM (default profile cam1_stereo_imu) in the background
./pq record walk_1                 # raw recording of both cameras
./pq stop                          # stops SLAM and recording cleanly
./pq status                        # what is running, free disk

# A whole session that needs no connection: SLAM + recording for 300 s, then the Rerun file.
nohup ./pq walk walk_2 300 &
```

`./pq slam` and `./pq record` start the cameras if they are not running. After a reboot the
cameras sometimes publish no images: use `./pq cams reset`.

### Benchmark

```bash
./pq bench run --bags /out/bench_list.txt --bags-root /bags \
  --profiles cam2_stereo_imu,cam1_cam2_ir --out /out/v17 --label v17
./pq bench compare /out/v15 /out/v17 --out /out/v15_v17.md
```

Here `/bags` is the bags folder (`BAGS_DIR`) and `/out` is `bench/` in the data folder. The
metrics are described in [tools/README.md](tools/README.md).

### Web app (optional)

pq_console is a browser page served from the Jetson: camera previews, recording, SLAM,
camera settings and the rig editor. It lives on the `console` branch and is installed only if
you want it:

```bash
./pq console install               # checks out the console branch in .pq_console/
./pq console start
# from your computer: ssh -N -L 8080:127.0.0.1:8080 <user>@<jetson>, then http://localhost:8080
```

## Results

All the results are in `DATA_DIR` (default `~/pq_data`):

| Folder | Content |
|---|---|
| `bags/<name>` | raw recordings of the cameras (MCAP); `<name>_slam` holds the SLAM output recorded live |
| `runs/<name>/run` | CSV files of a run: `odometry.csv`, `slam_poses.csv`, `health.csv`, `streams.csv`, `events.csv`, `run_info.json` (versions, settings, summary) |
| `runs/<name>/rerun`, `rerun/` | MCAP for the [Rerun](https://rerun.io) viewer: trajectory, landmarks, camera images |
| `bench/` | benchmarks |

## Repository

| Path | What |
|---|---|
| `pq` | the command line above |
| `pq_cuvslam/` | the ROS 2 node: [README](pq_cuvslam/README.md) (parameters, topics), [profiles/](pq_cuvslam/profiles/README.md) (camera layouts), launch files |
| `pq_cuvslam_interfaces/` | diagnostics and event messages |
| `isaac_ros_visual_slam_interfaces/` | messages and services of the Isaac ROS node, unchanged (compatible topics) |
| `cuvslam/` | cuVSLAM, git submodule (17.0.1) |
| `docker/` | the Dockerfile and its build options: [README](docker/README.md) |
| `tools/` | benchmark and analysis scripts: [README](tools/README.md) |
| `CHANGES.md` | changes from the Isaac ROS node |

Inside the container the node runs with plain ROS 2 commands (`./pq shell`):
`ros2 launch pq_cuvslam slam.launch.py profile:=cam2_stereo_imu [bag:=...]`. See
[pq_cuvslam/README.md](pq_cuvslam/README.md).

## License

Apache 2.0 (see [LICENSE](LICENSE)), derived from NVIDIA Isaac ROS Visual SLAM. cuVSLAM is
distributed by NVIDIA under the NVIDIA Community License, which allows its use only on NVIDIA
hardware.
