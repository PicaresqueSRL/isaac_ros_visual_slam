# tools

Scripts to run, evaluate and analyze pq_cuvslam on recorded bags. The Python scripts run in
the pq_cuvslam container (ROS 2 Lyrical, rosbag2_py, numpy, cv2); paths are paths in the
container.

| Script | Purpose |
|---|---|
| `benchmark.py` | Regression benchmark: deterministic replay of bags × profiles, ground-truth-free metrics, comparison of two or more runs |
| `benchmark_docker.sh` | Runs `benchmark.py` in the image from the host, with bags, output, optional workspace and ground truth mounted |
| `gyro_consistency.py` | Rotation of a run (`odometry.csv`) against the gyroscope of the base camera |
| `analyze_sync.py` | Timing of a two-camera bag: gaps, stale stamps, capture phase and its drift, clock offset from the gyroscopes |
| `analyze_images.py` | Image quality of the IR streams: brightness, saturated and dark pixels, projector dots |
| `check_rotation_compensation.py` | Check of the gyroscope rotation compensation on consecutive frames; gyroscope - image time offset |
| `mono_scale.py` | Metric scale of a Mono run from the IMU: vertical oscillations of the walk (band 1-3 Hz) in the visual positions and in the double-integrated accelerometer, on sliding windows; evaluation against a metric reference run; scaled `odometry.csv` |
| `scale_recording.py` | Applies a constant scale to a SLAM recording (MCAP): odometry, paths, landmarks, map/odom TF |
| `record_slam.sh` | Replays a bag (offline, all frames) and records the SLAM output to MCAP, plus the Rerun MCAP and the CSV files |
| `run_bag.sh` | Plays a bag at real-time speed with `ros2 bag play` and records the output topics (live-like run) |

## Benchmark

```bash
# Bag list: one bag folder per line, relative to --bags-root; # for comments.
tools/benchmark_docker.sh -b ~/bags -o ~/bench -- run \
  --bags /out/bench_list.txt --bags-root /bags \
  --profiles cam2_stereo_imu,cam1_cam2_ir --out /out/v15 --label v15 --repeat 2

# Same bags with another build (e.g. cuVSLAM 17 built in a development workspace).
tools/benchmark_docker.sh -b ~/bags -o ~/bench -w ~/ws_cuvslam17 -- run \
  --bags /out/bench_list.txt --bags-root /bags \
  --profiles cam2_stereo_imu,cam1_cam2_ir --out /out/v17 --label v17 --repeat 2

tools/benchmark_docker.sh -o ~/bench -- compare /out/v15 /out/v17 --out /out/v15_v17.md
```

`run` writes, under `--out`: one folder per profile and bag with the run output
(`run/odometry.csv`, `run/run_info.json`, ...) and the replay log; `summary.json`,
`summary.csv` and `report.md`. Metrics per run:

| Metric | Meaning |
|---|---|
| `frames`, `frames_lost` | rig frames processed, frames with tracking lost |
| `gyro_rms_deg`, `gyro_median_deg`, `gyro_p95_deg` | rotation error per frame against the gyroscope of the base camera (no ground truth needed; includes the small gyroscope bias, the same for all runs of a bag) |
| `gyro_glitches` | frames with a rotation error above 2 deg |
| `track_ms_median`, `replay_wall_s` | median `Track()` time, wall time of the replay |
| `saturated_pct` | highest median share of saturated pixels among the input images (diagnostics of the node) |
| `deterministic` | with `--repeat N`: the N runs give identical trajectories |
| `gt_rmse_m` | with `--gt-root` and `--compare-runs`: rigid RMSE against the ground truth (an indication only: the ground truth is approximate) |

A bag without the topics needed by a profile is reported as `skipped` with the missing topics;
a replay that fails is `failed`, with its log. `compare` prints the metrics of two or more
summaries side by side with the change relative to the first, and lists as regressions the gyro
RMS increases above `--rms-tolerance` (default 10 %), more frames lost, and runs that fail only
in a later summary; its exit code is 1 when there are regressions.

## SLAM recordings (MCAP)

```bash
docker run --rm --gpus all --network none -v ~/bags:/bags:ro -v ~/slam_runs:/out \
  -v $PWD/tools:/tools:ro pq_cuvslam:dev \
  /tools/record_slam.sh /bags/<category>/<bag> /out/<profile>/<bag> <profile> 2.0 [to_rerun_mcap.py]
```

Output: `bag/` (MCAP with odometry, paths, status, TF and visualization topics), `rerun/` (with
`to_rerun_mcap.py`), `run/` (CSV and `run_info.json`). Run recording containers with
`--network none`: with the default Docker network the ROS discovery reaches other containers,
and their messages would end up in the recording.

## Mono scale

```bash
# Estimate (and evaluate against the stereo IR + IMU run of the same bag, in metres).
python3 tools/mono_scale.py <bag> <mono run folder> --reference <stereo run folder> \
  --window 8 --json scale.json --scaled-out <folder>
# Scaled recording for Rerun.
python3 tools/scale_recording.py <recording>/bag <recording>/bag_scaled <scale>
```

The scale is observable only while walking (step bounce); with the camera still or in an
elevator no window is valid. On the recorded bags the estimate of a window is within 7-17 %
(median) of the true scale and tends to be 10-15 % low; a Mono run that never initializes
(all poses at zero) gives no estimate.
