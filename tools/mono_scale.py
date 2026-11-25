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
Metric scale of a Mono run (odometry.csv up to scale) from the IMU of the base camera.

Visual-inertial alignment on sliding windows (as in the initialization of VINS-Mono): in a
window of poses t_0..t_K the Mono positions p_k (unknown scale s) must match the double
integral of the accelerometer rotated to the odometry frame with the Mono orientations
(reliable in Mono) and propagated between poses with the gyroscope:

    s (p_k - p_0) = v_0 dt_k + 1/2 g dt_k^2 + alpha_k        (alpha_k: integrated IMU)

Unknowns s, v_0, g (3 + 3 + 1), linear least squares; then the gravity norm is fixed to
9.81 m/s^2 with a few tangent-space refinements. Windows with little acceleration, a negative
scale or a large residual are rejected. The lever arm between IMU and base frame and the
accelerometer bias are neglected (optionally the bias is estimated, --bias).

With --reference (an odometry.csv in metres, e.g. stereo IR + IMU), the true scale of every
window is computed by a similarity alignment of the two trajectories, and the estimate is
evaluated.

Runs in the pq_cuvslam container (rosbag2_py, numpy).

Usage:
  mono_scale.py <bag> <mono run folder> [--reference <run folder>] [--imu /cam2/imu]
      [--base-frame cam2_link] [--window 4.0] [--step 1.0] [--bias] [--out scale.csv]
      [--scaled-out <folder>]
"""

import argparse
import csv
import importlib.util
import json
import os

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

_spec = importlib.util.spec_from_file_location(
    'gyro_consistency', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     'gyro_consistency.py'))
gc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gc)

GRAVITY = 9.81


def read_imu(path, imu_topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[imu_topic, '/tf_static']))
    parents, rows, frame = {}, [], None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            for t in deserialize_message(data, TFMessage).transforms:
                q = t.transform.rotation
                parents[t.child_frame_id] = (t.header.frame_id,
                                             gc.quat_to_rot(q.x, q.y, q.z, q.w))
        else:
            m = deserialize_message(data, Imu)
            frame = frame or m.header.frame_id
            a, w = m.linear_acceleration, m.angular_velocity
            rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                         a.x, a.y, a.z, w.x, w.y, w.z))
    imu = np.array(rows)
    imu = imu[np.argsort(imu[:, 0])]
    return parents, frame, imu


def read_odometry(folder):
    t, p, R = [], [], []
    with open(os.path.join(folder, 'odometry.csv')) as f:
        for r in csv.DictReader(f):
            t.append(float(r['timestamp']))
            p.append([float(r['x']), float(r['y']), float(r['z'])])
            R.append(gc.quat_to_rot(*(float(r[k]) for k in ('qx', 'qy', 'qz', 'qw'))))
    t, p, R = np.array(t), np.array(p), np.array(R)
    # The first image of the D455 streams has an old timestamp (driver leftover): drop the
    # poses before a gap of more than 1 s.
    gaps = np.where(np.diff(t[:10]) > 1.0)[0]
    if len(gaps):
        k = gaps[-1] + 1
        t, p, R = t[k:], p[k:] - p[k], R[k:]
    return t, p, R


def world_specific_force(imu, t_pose, R_pose, R_base_imu):
    """Accelerometer of every IMU sample in the odometry frame: orientation of the last pose,
    propagated to the sample time with the gyroscope."""
    ts = imu[:, 0]
    acc = imu[:, 1:4] @ R_base_imu.T          # base frame
    gyr = imu[:, 4:7] @ R_base_imu.T
    out = np.full((len(ts), 3), np.nan)
    k = 0
    R = None
    for i in range(len(ts)):
        while k + 1 < len(t_pose) and t_pose[k + 1] <= ts[i]:
            k += 1
            R = None
        if ts[i] < t_pose[0]:
            continue
        if R is None:                          # re-anchor on the pose k
            R, t_prev = R_pose[k].copy(), t_pose[k]
            if i > 0 and ts[i] > t_prev:
                R = R @ gc.rodrigues(gyr[i] * (ts[i] - t_prev))
        else:
            R = R @ gc.rodrigues(0.5 * (gyr[i] + gyr[i - 1]) * (ts[i] - ts[i - 1]))
        out[i] = R @ acc[i]
    return out


def double_integrals(ts, f, R_list_t, t0, times):
    """alpha(t_k) = int_t0^tk int_t0^s f, and beta(t_k) = int int R (3x3, for the bias), for
    the IMU samples between t0 and the last time."""
    sel = (ts >= t0) & (ts <= times[-1])
    tt, ff, RR = ts[sel], f[sel], R_list_t[sel]
    v = np.zeros(3)
    p = np.zeros(3)
    V = np.zeros((3, 3))
    P = np.zeros((3, 3))
    alpha, beta = [], []
    j = 0
    for i in range(1, len(tt)):
        dt = tt[i] - tt[i - 1]
        a = 0.5 * (ff[i] + ff[i - 1])
        Rm = 0.5 * (RR[i] + RR[i - 1])
        p = p + v * dt + 0.5 * a * dt * dt
        v = v + a * dt
        P = P + V * dt + 0.5 * Rm * dt * dt
        V = V + Rm * dt
        while j < len(times) and times[j] <= tt[i]:
            alpha.append(p.copy())
            beta.append(P.copy())
            j += 1
    while j < len(times):
        alpha.append(p.copy())
        beta.append(P.copy())
        j += 1
    return np.array(alpha), np.array(beta)


def tangent_basis(g):
    n = g / np.linalg.norm(g)
    tmp = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
    b1 = np.cross(n, tmp)
    b1 /= np.linalg.norm(b1)
    return np.stack([b1, np.cross(n, b1)], axis=1)


def solve_window(dp, dt, alpha, beta, bias):
    """Least squares for s, v0, g (and the accelerometer bias); g norm fixed afterwards."""
    n = len(dt)
    cols = 7 + (3 if bias else 0)
    A = np.zeros((3 * n, cols))
    b = alpha.reshape(-1)
    for k in range(n):
        r = slice(3 * k, 3 * k + 3)
        A[r, 0] = dp[k]
        A[r, 1:4] = -dt[k] * np.eye(3)
        A[r, 4:7] = -0.5 * dt[k] ** 2 * np.eye(3)
        if bias:
            A[r, 7:10] = beta[k]   # alpha measured with (a - b): alpha_true = alpha - beta b
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    g = x[4:7]
    if np.linalg.norm(g) < 1e-3:
        return None
    # Gravity refinement: g = 9.81 n + B w.
    for _ in range(4):
        g = GRAVITY * g / np.linalg.norm(g)
        B = tangent_basis(g)
        A2 = np.zeros((3 * n, 4 + 2 + (3 if bias else 0)))
        b2 = b.copy()
        for k in range(n):
            r = slice(3 * k, 3 * k + 3)
            A2[r, 0] = dp[k]
            A2[r, 1:4] = -dt[k] * np.eye(3)
            A2[r, 4:6] = -0.5 * dt[k] ** 2 * B
            if bias:
                A2[r, 6:9] = beta[k]
            b2[3 * k:3 * k + 3] += 0.5 * dt[k] ** 2 * g
        y, *_ = np.linalg.lstsq(A2, b2, rcond=None)
        g = g + B @ y[4:6]
    g = GRAVITY * g / np.linalg.norm(g)
    residual = A2 @ y - b2
    return {'scale': float(y[0]), 'gravity': g.tolist(),
            'residual_rms_m': float(np.sqrt(np.mean(residual ** 2))),
            'bias': y[6:9].tolist() if bias else None}


def band_scale(t_imu, a_w, t_pose, p, t0, t1, f_lo, f_hi, up=None, estimator='ratio',
               rate=100.0):
    """Scale from the oscillations of the motion (walking steps): positions of the Mono run and
    double-integrated acceleration (odometry frame) band-passed in [f_lo, f_hi] Hz, where the
    gravity error and the accelerometer bias (slow) do not enter. Least squares on the complex
    spectra: s * P_vis(f) = A(f) / -(2 pi f)^2. With up (unit vector, opposite to gravity) only
    the vertical component is used: the bounce of the steps is the strongest oscillation and
    the best observed by the camera (the horizontal ones mix sway, which is weak in the
    accelerometer, and the noise along the optical axis). Returns scale, quality (share of the
    IMU band energy explained, 0-1) and the band-passed IMU position amplitude (m, RMS)."""
    n = int((t1 - t0) * rate)
    if n < 64:
        return None
    tg = t0 + np.arange(n) / rate
    win = np.hanning(n)
    P = np.stack([np.interp(tg, t_pose, p[:, k]) for k in range(3)], axis=1)
    A = np.stack([np.interp(tg, t_imu, a_w[:, k]) for k in range(3)], axis=1)
    # Remove the slow motion before the window: a quadratic fit of the positions (constant
    # acceleration), the mean of the accelerations (the same constant acceleration). Otherwise
    # the displacement of the window leaks into the band.
    x = (tg - tg.mean()) / (tg[-1] - tg[0])
    V = np.stack([np.ones(n), x, x * x], axis=1)
    P = P - V @ np.linalg.lstsq(V, P, rcond=None)[0]
    P = P * win[:, None]
    A = (A - A.mean(0)) * win[:, None]
    if up is not None:
        P = (P @ up)[:, None]
        A = (A @ up)[:, None]
    f = np.fft.rfftfreq(n, 1 / rate)
    band = (f >= f_lo) & (f <= f_hi)
    if band.sum() < 3:
        return None
    Pv = np.fft.rfft(P, axis=0)[band]
    Pi = np.fft.rfft(A, axis=0)[band] / -(2 * np.pi * f[band])[:, None] ** 2
    num = np.real(np.sum(np.conj(Pv) * Pi))
    den = np.real(np.sum(np.conj(Pv) * Pv))
    if den <= 0:
        return None
    if estimator == 'ratio':
        # Amplitude ratio with the sign of the correlation: not attenuated by the noise of the
        # visual positions (the least squares slope is, by den), slightly inflated by the IMU one.
        s_est = np.sign(num) * np.sqrt(np.real(np.sum(np.conj(Pi) * Pi)) / den)
    else:
        s_est = num / den
    # Fraction of the IMU band energy explained by the scaled visual motion.
    resid = Pi - s_est * Pv
    e_imu = np.real(np.sum(np.conj(Pi) * Pi))
    quality = 1 - np.real(np.sum(np.conj(resid) * resid)) / e_imu if e_imu > 0 else 0.0
    amp = np.sqrt(2 * e_imu) / np.sum(win ** 2) ** 0.5 / np.sqrt(n)
    return float(s_est), float(quality), float(amp)


def umeyama_scale(src, dst):
    """Scale of the similarity transform dst ~ s R src + t."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    var_s = (xs ** 2).sum() / len(src)
    return float(np.trace(np.diag(D) @ S) / var_s) if var_s > 0 else float('nan')


def interp_positions(t_ref, p_ref, t):
    return np.stack([np.interp(t, t_ref, p_ref[:, k]) for k in range(3)], axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('mono')
    ap.add_argument('--reference', default='', help='run folder with a metric odometry.csv')
    ap.add_argument('--imu', default='/cam2/imu')
    ap.add_argument('--imu-frame', default='cam2_gyro_optical_frame',
                    help='IMU frame in /tf_static (the frame_id of the messages may be missing)')
    ap.add_argument('--base-frame', default='cam2_link')
    ap.add_argument('--window', type=float, default=4.0, help='window length (s)')
    ap.add_argument('--step', type=float, default=1.0, help='window step (s)')
    ap.add_argument('--pose-dt', type=float, default=0.1, help='pose spacing in a window (s)')
    ap.add_argument('--min-accel', type=float, default=0.3,
                    help='minimum std of the gravity-free acceleration in a window (m/s^2)')
    ap.add_argument('--bias', action='store_true', help='estimate the accelerometer bias')
    ap.add_argument('--method', choices=['band', 'alignment'], default='band',
                    help='band: oscillations of the motion in [--f-lo, --f-hi] Hz (default); '
                    'alignment: visual-inertial alignment of the positions')
    ap.add_argument('--axes', choices=['vertical', 'all'], default='vertical',
                    help='band method: vertical component only (default) or all axes')
    ap.add_argument('--estimator', choices=['ratio', 'lsq'], default='ratio',
                    help='band method: amplitude ratio (default) or least squares slope')
    ap.add_argument('--f-lo', type=float, default=1.0)
    ap.add_argument('--f-hi', type=float, default=3.0)
    ap.add_argument('--min-quality', type=float, default=0.5,
                    help='band method: minimum share of the IMU band energy explained')
    ap.add_argument('--out', default='', help='CSV with the scale of every window')
    ap.add_argument('--json', default='')
    ap.add_argument('--scaled-out', default='',
                    help='folder for odometry.csv scaled with the windowed estimate')
    args = ap.parse_args()

    parents, imu_frame, imu = read_imu(args.bag, args.imu)
    known = set(parents) | {parent for parent, _ in parents.values()}
    for frame in (args.base_frame, args.imu_frame):
        if frame not in known:
            raise SystemExit(f'frame {frame} not in /tf_static of the bag')
    R_base_imu = gc.root_rotation(parents, args.base_frame).T @ \
        gc.root_rotation(parents, args.imu_frame)
    t, p, R = read_odometry(args.mono)
    f_w = world_specific_force(imu, t, R, R_base_imu)
    ok = ~np.isnan(f_w[:, 0])
    ts, f_w = imu[ok, 0], f_w[ok]
    # Orientation of every IMU sample (for the bias term): nearest previous pose.
    idx = np.clip(np.searchsorted(t, ts, side='right') - 1, 0, len(t) - 1)
    R_imu = R[idx]

    ref = None
    if args.reference:
        tr, pr, _ = read_odometry(args.reference)
        ref = (tr, pr)

    # Gravity in the odometry frame for the band method: mean specific force (the mean
    # acceleration over the run is about zero); only its slow part matters, removed by the band.
    gravity_w = -f_w.mean(0)
    gravity_w *= GRAVITY / np.linalg.norm(gravity_w)

    windows = []
    start = t[0]
    while start + args.window <= min(t[-1], ts[-1]):
        sel = np.where((t >= start) & (t <= start + args.window))[0]
        # Poses spaced by about pose_dt.
        keep, last = [], -1e9
        for i in sel:
            if t[i] - last >= args.pose_dt - 1e-6:
                keep.append(i)
                last = t[i]
        keep = np.array(keep)
        w = {'start': float(start - t[0]), 'end': float(start + args.window - t[0])}
        if args.method == 'band':
            res = band_scale(ts, f_w + gravity_w, t, p, start, start + args.window,
                             args.f_lo, args.f_hi,
                             up=-gravity_w / GRAVITY if args.axes == 'vertical' else None,
                             estimator=args.estimator)
            if res:
                w['scale'], w['quality'], w['imu_amp_m'] = res
                w['valid'] = bool(w['scale'] > 0 and w['quality'] >= args.min_quality)
            if ref is not None:
                tr, pr = ref
                sel_r = (t >= start) & (t <= start + args.window)
                if sel_r.sum() >= 8 and start >= tr[0] and start + args.window <= tr[-1]:
                    w['true_scale'] = umeyama_scale(p[sel_r], interp_positions(tr, pr, t[sel_r]))
        elif len(keep) >= 8:
            times = t[keep]
            alpha, beta = double_integrals(ts, f_w, R_imu, times[0], times)
            dt = times - times[0]
            dp = p[keep] - p[keep[0]]
            fsel = (ts >= times[0]) & (ts <= times[-1])
            res = solve_window(dp, dt, alpha, beta, args.bias)
            if res:
                accel = f_w[fsel] + np.array(res['gravity'])   # f = a - g -> a = f + g
                w['accel_std'] = float(np.linalg.norm(accel.std(0)))
                w.update(res)
                w['mono_motion'] = float(np.linalg.norm(dp, axis=1).max())
                w['valid'] = bool(res['scale'] > 0 and w['accel_std'] >= args.min_accel)
            if ref is not None:
                tr, pr = ref
                if times[0] >= tr[0] and times[-1] <= tr[-1]:
                    w['true_scale'] = umeyama_scale(p[keep], interp_positions(tr, pr, times))
        windows.append(w)
        start += args.step

    valid = [w for w in windows if w.get('valid')]
    scales = np.array([w['scale'] for w in valid])
    summary = {
        'windows': len(windows), 'valid_windows': len(valid),
        'scale_median': float(np.median(scales)) if len(scales) else None,
        'scale_iqr': (float(np.percentile(scales, 25)), float(np.percentile(scales, 75)))
        if len(scales) else None,
    }
    if ref is not None:
        tr, pr = ref
        m = (t >= tr[0]) & (t <= tr[-1])
        summary['true_scale_global'] = umeyama_scale(p[m], interp_positions(tr, pr, t[m]))
        errs = [abs(w['scale'] / w['true_scale'] - 1) for w in valid
                if w.get('true_scale') and w['true_scale'] > 0]
        if errs:
            summary['window_error_median_pct'] = float(100 * np.median(errs))
            summary['window_error_p90_pct'] = float(100 * np.percentile(errs, 90))
        if summary['scale_median']:
            summary['median_vs_global_error_pct'] = float(
                100 * (summary['scale_median'] / summary['true_scale_global'] - 1))

    # Scaled trajectory: every pose displacement scaled with the estimate of the window centred
    # nearest to it (median of the valid windows if none).
    if args.scaled_out and len(scales):
        centres = np.array([(w['start'] + w['end']) / 2 for w in valid])
        s_t = np.array([scales[np.argmin(np.abs(centres - (ti - t[0])))] for ti in t])
        q = np.zeros_like(p)
        for i in range(1, len(p)):
            q[i] = q[i - 1] + s_t[i] * (p[i] - p[i - 1])
        os.makedirs(args.scaled_out, exist_ok=True)
        with open(os.path.join(args.mono, 'odometry.csv')) as f:
            rows = list(csv.DictReader(f))
        with open(os.path.join(args.scaled_out, 'odometry.csv'), 'w', newline='') as f:
            wtr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            wtr.writeheader()
            for r, pos in zip(rows, q):
                r['x'], r['y'], r['z'] = (f'{v:.6f}' for v in pos)
                wtr.writerow(r)
        summary['scaled_path_m'] = float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
        if ref is not None:
            tr, pr = ref
            m = (t >= tr[0]) & (t <= tr[-1])
            pr_i = interp_positions(tr, pr, t[m])
            summary['reference_path_m'] = float(
                np.linalg.norm(np.diff(pr_i, axis=0), axis=1).sum())

    if args.out:
        keys = ['start', 'end', 'scale', 'true_scale', 'quality', 'imu_amp_m', 'accel_std',
                'residual_rms_m', 'mono_motion', 'valid']
        with open(args.out, 'w', newline='') as f:
            wtr = csv.writer(f)
            wtr.writerow(keys)
            for w in windows:
                wtr.writerow([w.get(k, '') for k in keys])
    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'summary': summary, 'windows': windows}, f, indent=1)
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
