#!/usr/bin/env python3
# Copyright (c) 2026 Picaresque SRL
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
"""
Ground-truth-free check of the rotation of a run: for each pair of consecutive poses of
odometry.csv, the rotation estimated by the tracker is compared with the rotation measured by
the gyroscope of the base camera over the same interval (the cameras are rigidly mounted).
The error does not need a ground truth; it includes the small gyroscope bias, the same for all
runs of a bag, so it is meant to compare runs of the same bag.

Runs in the pq_cuvslam container (rosbag2_py, numpy).

Usage:
  gyro_consistency.py <bag> label=<run folder> [label=<run folder> ...]
      [--imu /cam2/imu] [--imu-frame cam2_gyro_optical_frame] [--base-frame cam2_link]
      [--json out.json]
"""

import argparse
import csv
import json

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def quat_to_rot(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rodrigues(v):
    theta = np.linalg.norm(v)
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * kx @ kx


def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)))


def root_rotation(parents, frame):
    r = np.eye(3)
    while frame in parents:
        frame, rot = parents[frame]
        r = rot @ r
    return r


def read_bag(path, imu_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[imu_topic, '/tf_static']))
    parents = {}
    gyro = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            for t in deserialize_message(data, TFMessage).transforms:
                q = t.transform.rotation
                parents[t.child_frame_id] = (t.header.frame_id, quat_to_rot(q.x, q.y, q.z, q.w))
        else:
            m = deserialize_message(data, Imu)
            w = m.angular_velocity
            gyro.append((m.header.stamp.sec * 1e9 + m.header.stamp.nanosec, w.x, w.y, w.z))
    return parents, np.array(gyro)


def integrate(gyro, t0, t1):
    ts = gyro[:, 0]
    grid = np.concatenate(([t0], ts[(ts > t0) & (ts < t1)], [t1]))
    w = np.stack([np.interp(grid, ts, gyro[:, k]) for k in (1, 2, 3)], axis=1)
    r = np.eye(3)
    for i in range(len(grid) - 1):
        r = r @ rodrigues(0.5 * (w[i] + w[i + 1]) * (grid[i + 1] - grid[i]) / 1e9)
    return r


def read_poses(folder):
    rows = []
    with open(f'{folder}/odometry.csv') as f:
        for r in csv.DictReader(f):
            rows.append((float(r['timestamp']) * 1e9,
                         quat_to_rot(*(float(r[k]) for k in ('qx', 'qy', 'qz', 'qw')))))
    return rows


def evaluate(poses, gyro, R_base_gyro, max_dt_s=0.2):
    errors, rotations, rates = [], [], []
    for (t0, R0), (t1, R1) in zip(poses, poses[1:]):
        dt = (t1 - t0) / 1e9
        if dt <= 0 or dt > max_dt_s or t0 < gyro[0, 0] or t1 > gyro[-1, 0]:
            continue
        dR_odom = R0.T @ R1
        dR_gyro = R_base_gyro @ integrate(gyro, t0, t1) @ R_base_gyro.T
        e = angle_deg(dR_odom.T @ dR_gyro)
        errors.append(e)
        rates.append(e / dt)
        rotations.append(angle_deg(dR_gyro))
    e = np.array(errors)
    return {
        'pairs': int(len(e)),
        'rotation_median_deg': float(np.median(rotations)) if rotations else None,
        'error_median_deg': float(np.median(e)) if len(e) else None,
        'error_p95_deg': float(np.percentile(e, 95)) if len(e) else None,
        'error_rms_deg': float(np.sqrt(np.mean(e ** 2))) if len(e) else None,
        'error_rate_median_deg_s': float(np.median(rates)) if rates else None,
        # pairs with an error above 2 deg: tracking glitches
        'glitches_2deg': int(np.sum(e > 2.0)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('runs', nargs='+', help='label=folder with odometry.csv')
    ap.add_argument('--imu', default='/cam2/imu')
    ap.add_argument('--imu-frame', default='cam2_gyro_optical_frame')
    ap.add_argument('--base-frame', default='cam2_link')
    ap.add_argument('--json')
    args = ap.parse_args()

    parents, gyro = read_bag(args.bag, args.imu)
    R_base_gyro = root_rotation(parents, args.base_frame).T @ root_rotation(parents, args.imu_frame)
    results = {}
    print(f"{'run':14s} {'pairs':>5s} {'rot_med':>7s} {'err_med':>7s} {'err_p95':>7s} "
          f"{'err_rms':>7s} {'>2deg':>5s}")
    for spec in args.runs:
        label, folder = spec.split('=', 1) if '=' in spec else (spec.rstrip('/').split('/')[-1], spec)
        r = evaluate(read_poses(folder), gyro, R_base_gyro)
        results[label] = r
        if r['pairs']:
            print(f"{label:14s} {r['pairs']:5d} {r['rotation_median_deg']:7.3f} "
                  f"{r['error_median_deg']:7.3f} {r['error_p95_deg']:7.3f} "
                  f"{r['error_rms_deg']:7.3f} {r['glitches_2deg']:5d}")
        else:
            print(f'{label:14s} no pose pairs')
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results, f, indent=2)


if __name__ == '__main__':
    main()
