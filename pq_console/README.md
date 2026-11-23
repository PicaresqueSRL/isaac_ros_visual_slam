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

- **Live**: cameras on/off, IR emitter of each camera (a recording does not start with an
  emitter on, unless confirmed), recording with or without depth, four previews (any image
  topic; fps, saturated pixels, valid depth), SLAM trajectory (top view), tracker state and
  events of pq_cuvslam.
- **Sessions**: recordings; *Play + SLAM* plays a recording in real time with the chosen
  profile (the bag `<name>_aligned` from `tools/align_depth.py` is played with it), records the
  SLAM output and builds the Rerun file; SLAM runs with their results.
- **Nodes & topics**: ROS graph; rates on request.

## How it works

- `console.py`: HTTP server (Python standard library): page, JSON API, MJPEG previews.
- `ros_bridge.py`: ROS node (graph, rates, previews, SLAM state, parameters of the camera
  drivers). Light by design: rates only on request, previews decoded at 5 fps and dropped
  10 s after the page stops showing them.
- `sessions.py`: containers `pq_cams` (RealSense driver), `pq_rec` (recorder), `pq_slam`,
  `pq_slamrec` and `pq_play` (replay), started through the Docker socket with the docker client
  of the host.

The container image is the pq_cuvslam image (rclpy, OpenCV, pq_cuvslam messages); nothing else
is installed.
