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
Timing analysis of a two-camera bag (two D455, cam1 and cam2), for the software sync.

For each camera: frame rate, gaps (dropped frames), duplicated or non-monotonic stamps,
IR1/IR2 pairs. Between the cameras: offset of the nearest frames (capture phase) and its
drift, and the offset between the camera clocks estimated by cross-correlating the angular
speed |w| of the two gyroscopes (|w| does not depend on the fixed rotation between them).

Runs in the pq_cuvslam container (rosbag2_py, numpy).

Usage:
  analyze_sync.py <bag> [<bag> ...] [--json out.json]
"""

import argparse
import json
import math
import struct
import sys

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Imu

CAMERAS = ('cam1', 'cam2')


def header_stamp_ns(data):
    """Stamp of a serialized message whose first field is std_msgs/Header (CDR, LE)."""
    sec, nsec = struct.unpack_from('<iI', data, 4)
    return sec * 1_000_000_000 + nsec


def read_bag(path):
    topics = []
    for cam in CAMERAS:
        topics += [f'/{cam}/infra1/image_rect_raw', f'/{cam}/infra2/image_rect_raw',
                   f'/{cam}/imu']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {t.name for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in available]))
    stamps = {t: [] for t in topics}
    gyro = {cam: [] for cam in CAMERAS}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic.endswith('/imu'):
            msg = deserialize_message(data, Imu)
            w = msg.angular_velocity
            t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            gyro[topic.split('/')[1]].append((t, w.x, w.y, w.z))
        else:
            stamps[topic].append(header_stamp_ns(data))
    return stamps, {c: np.array(g) for c, g in gyro.items()}, available


def stream_stats(stamps_ns):
    s = np.array(stamps_ns, dtype=np.int64)
    if len(s) < 3:
        return {'messages': int(len(s))}
    d = np.diff(s)
    period = float(np.median(d[d > 0])) if np.any(d > 0) else float('nan')
    return {
        'messages': int(len(s)),
        'rate_hz': 1e9 / period,
        'non_monotonic': int(np.sum(d < 0)),
        'duplicated': int(np.sum(d == 0)),
        # gap = more than 1.5 periods between consecutive stamps
        'gaps': int(np.sum(d > 1.5 * period)),
        'missing_frames': int(np.sum(np.round(d[d > 1.5 * period] / period) - 1)),
        'max_gap_ms': float(d.max() / 1e6),
    }


def nearest_offsets(ref_ns, other_ns):
    """For each reference stamp, (other - ref) of the nearest other stamp, in ms."""
    ref = np.sort(np.array(ref_ns, dtype=np.int64))
    other = np.sort(np.array(other_ns, dtype=np.int64))
    idx = np.clip(np.searchsorted(other, ref), 1, len(other) - 1)
    before, after = other[idx - 1], other[idx]
    nearest = np.where(np.abs(before - ref) <= np.abs(after - ref), before, after)
    return ref, (nearest - ref) / 1e6


def gyro_offset(g_ref, g_other, t0, t1, max_lag_ms=150.0, step_ms=1.0):
    """Lag (ms) that maximizes the correlation of |w|: other(t - lag) ~ ref(t).

    A positive lag means that the other camera stamps its samples later than the reference
    for the same motion (other clock ahead): subtract it from the other camera's stamps.
    Returns (lag_ms, peak correlation, std of |w| in the window).
    """
    grid = np.arange(t0, t1, step_ms * 1e6)
    if len(grid) < 1000:
        return None

    def speed(g):
        mag = np.linalg.norm(g[:, 1:4], axis=1)
        return np.interp(grid, g[:, 0], mag)
    a = speed(g_ref)
    b = speed(g_other)
    if a.std() < 1e-3 or b.std() < 1e-3:
        return None
    a = (a - a.mean()) / a.std()
    b = (b - b.mean()) / b.std()
    max_lag = int(max_lag_ms / step_ms)
    lags = np.arange(-max_lag, max_lag + 1)
    n = len(a)
    corr = np.array([
        np.dot(a[max(0, -k):n - max(0, k)], b[max(0, k):n - max(0, -k)]) / (n - abs(k))
        for k in lags])
    i = int(np.argmax(corr))
    lag = float(lags[i])
    if 0 < i < len(corr) - 1:  # parabolic refinement
        y0, y1, y2 = corr[i - 1], corr[i], corr[i + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            lag += 0.5 * (y0 - y2) / den
    return lag * step_ms, float(corr[i]), float(np.linalg.norm(g_ref[:, 1:4], axis=1).std())


def linear_drift(t_ns, values):
    """Slope of values over time, per second, and residual std."""
    t = (np.asarray(t_ns) - t_ns[0]) / 1e9
    if len(t) < 3 or t[-1] - t[0] < 1:
        return float('nan'), float('nan')
    p = np.polyfit(t, values, 1)
    return float(p[0]), float(np.std(values - np.polyval(p, t)))


def analyze(path):
    stamps, gyro, available = read_bag(path)
    out = {'bag': path, 'cameras': {}}
    for cam in CAMERAS:
        ir1 = stamps[f'/{cam}/infra1/image_rect_raw']
        ir2 = stamps[f'/{cam}/infra2/image_rect_raw']
        imu = gyro[cam][:, 0] if len(gyro[cam]) else []
        out['cameras'][cam] = {
            'ir1': stream_stats(ir1),
            'ir2': stream_stats(ir2),
            'stereo_pairs': len(set(ir1) & set(ir2)),
            'imu': stream_stats(imu),
        }

    c1 = stamps['/cam1/infra1/image_rect_raw']
    c2 = stamps['/cam2/infra1/image_rect_raw']
    if len(c1) > 2 and len(c2) > 2:
        ref, off = nearest_offsets(c2, c1)
        slope, resid = linear_drift(ref, off)
        out['frame_offset_ms'] = {
            'description': 'cam1 - cam2, nearest cam1 IR1 frame for each cam2 IR1 frame',
            'median': float(np.median(off)), 'p5': float(np.percentile(off, 5)),
            'p95': float(np.percentile(off, 95)), 'abs_max': float(np.abs(off).max()),
            'drift_ms_per_s': slope, 'residual_std_ms': resid,
        }

    if len(gyro['cam1']) > 100 and len(gyro['cam2']) > 100:
        t0 = max(gyro['cam1'][0, 0], gyro['cam2'][0, 0])
        t1 = min(gyro['cam1'][-1, 0], gyro['cam2'][-1, 0])
        whole = gyro_offset(gyro['cam2'], gyro['cam1'], t0, t1)
        windows = []
        w = 10e9
        for start in np.arange(t0, t1 - w, w / 2):
            r = gyro_offset(gyro['cam2'], gyro['cam1'], start, start + w)
            if r and r[1] > 0.5:
                windows.append(((start + w / 2 - t0) / 1e9, r[0], r[1]))
        res = {'description': 'clock offset cam1 - cam2 from the gyroscopes (|w| cross-correlation)'}
        if whole:
            res.update(lag_ms=whole[0], correlation=whole[1], gyro_speed_std=whole[2])
        if len(windows) >= 2:
            tw = np.array([x[0] for x in windows])
            lw = np.array([x[1] for x in windows])
            p = np.polyfit(tw, lw, 1)
            res.update(windows=len(windows), window_lag_min=float(lw.min()),
                       window_lag_max=float(lw.max()), drift_ms_per_s=float(p[0]))
        out['gyro_offset_ms'] = res
    return out


def fmt(v, nd=2):
    return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{nd}f}'


def print_report(r):
    print(f"\n=== {r['bag']}")
    for cam, s in r['cameras'].items():
        ir1, ir2, imu = s['ir1'], s['ir2'], s['imu']
        print(f"  {cam}: IR1 {ir1.get('messages', 0)} @ {fmt(ir1.get('rate_hz'), 1)} Hz, "
              f"missing {ir1.get('missing_frames', '-')} (max gap {fmt(ir1.get('max_gap_ms'), 0)} ms), "
              f"dup {ir1.get('duplicated', '-')}, non-mono {ir1.get('non_monotonic', '-')}; "
              f"IR2 {ir2.get('messages', 0)}; pairs {s['stereo_pairs']}; "
              f"IMU {imu.get('messages', 0)} @ {fmt(imu.get('rate_hz'), 0)} Hz, "
              f"non-mono {imu.get('non_monotonic', '-')}")
    f = r.get('frame_offset_ms')
    if f:
        print(f"  frames cam1-cam2: median {fmt(f['median'])} ms, p5..p95 {fmt(f['p5'])}..{fmt(f['p95'])}, "
              f"|max| {fmt(f['abs_max'])}, drift {fmt(f['drift_ms_per_s'], 3)} ms/s")
    g = r.get('gyro_offset_ms')
    if g:
        print(f"  gyro clock cam1-cam2: {fmt(g.get('lag_ms'))} ms (corr {fmt(g.get('correlation'))}), "
              f"windows {g.get('windows', 0)}: "
              f"{fmt(g.get('window_lag_min'))}..{fmt(g.get('window_lag_max'))} ms, "
              f"drift {fmt(g.get('drift_ms_per_s'), 3)} ms/s")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--json', help='write the results to this file')
    args = ap.parse_args()
    results = []
    for bag in args.bags:
        try:
            r = analyze(bag)
        except Exception as e:  # noqa: BLE001 (report and continue with the other bags)
            print(f'\n=== {bag}\n  ERROR: {e}', file=sys.stderr)
            continue
        results.append(r)
        print_report(r)
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results, f, indent=2)


if __name__ == '__main__':
    main()
