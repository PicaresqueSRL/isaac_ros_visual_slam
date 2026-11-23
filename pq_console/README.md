# pq_console

Small web console for the D455 rig, opened in the browser of a laptop: camera previews while
recording, IR emitter check, recording start/stop, SLAM replay of a recording with a profile and
live view of the trajectory, ROS nodes and topics.

It runs in a container on the machine with the cameras (Jetson) or the bags (PC), listens on
127.0.0.1 only and is reached through an ssh tunnel.

## Start

From the laptop:

```bash
pq_console/connect.sh user@jetson            # starts the console, opens the tunnel and the page
```

On the machine itself: `pq_console/run.sh [start|restart|stop|logs] [config.yaml]`.
The console keeps running when the ssh session ends; so do the cameras, recordings and replays
(separate containers).

## Configuration

Copy `config.example.yaml` to `config.yaml` (not tracked by git: it holds the camera serial
numbers) and set the paths, images and cameras of the machine. Paths are host paths: the
containers mount them at the same path.

## Pages

- **Bags**: browse the folders of the configuration (`bags_dir`, the SLAM output and
  `browse_roots`) and open a bag (rosbag2 folder, `.mcap` or `.db3`). The page lists its
  streams by camera (resolution, rate, distortion, IMU), the camera settings saved with it and
  the other topics; *Play* shows every image stream (pause, rate, seek). *SLAM on this bag* plays
  it in real time with pq_cuvslam, with a profile (the ones whose topics are in the bag) or with
  the chosen sensors (IR stereo, RGB, depth per camera, one IMU: the console picks the cuVSLAM
  mode and writes the profile), on the chosen cuVSLAM build; trajectory, tracker state and
  events are shown live, the runs of the bag are listed with their Rerun file.
- **Cameras**: cameras on/off, IR emitter of each camera (a recording does not start with an
  emitter on, unless confirmed), recording with or without depth, four previews, settings of
  the camera drivers side by side, changed at runtime on one or all cameras; *Save as startup
  settings* writes `data_dir/camera_settings.yaml`, applied after *Start cameras*. The
  parameters of the drivers at the start of each recording are saved in
  `bags/<name>.camera_settings.yaml`.
- **Rig 3D**: (next) cameras in 3D, coordinates import, camera models.
- **Graph**: ROS nodes and topics; rates on request.

cuVSLAM builds: `cuvslam_versions` in the configuration maps a label to an image and/or an
install folder; profiles and sensors that need the Multisensor mode are refused below 17.

## How it works

- `console.py`: HTTP server (Python standard library): page, JSON API, JPEG previews (all the
  visible ones in one request: browsers open at most 6 connections to a host).
- `bags.py`: folder browsing, content of a bag, profiles for the chosen sensors.
- `ros_bridge.py`: ROS node (graph, rates, previews, SLAM state with landmarks, parameters of
  the camera drivers). Light by design: image and landmark topics are not received
  continuously; their latest sample is taken only when the page asks (history 1, node not spun
  by an executor), images are subsampled to about 320 px and JPEG-compressed (~6 kB). Eight
  previews at ~5 fps cost about 15 % of a PC core.
- `sessions.py`: containers `pq_cams` (RealSense driver), `pq_rec` (recorder), `pq_slam`,
  `pq_slamrec` and `pq_play` (replay), started through the Docker socket with the docker client
  of the host.

The container image is the pq_cuvslam image (rclpy, OpenCV, pq_cuvslam messages); nothing else
is installed.

Without cameras (development on a PC): `mock_cameras.py` starts /cam1 and /cam2 nodes with D455
driver parameters, and a played bag gives the previews.
