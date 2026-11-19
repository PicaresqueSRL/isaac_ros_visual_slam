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
Check of the gyroscope-based rotation compensation on real data.

For consecutive IR1 frames of one camera, the earlier frame is warped to the time of the
later one with the homography H = K * dR^T * K^-1, where dR is the camera rotation integrated
from the gyroscope of the same camera. If the sign and the IMU -> camera rotation are right,
the warped frame matches the later frame better than the unwarped one (and better than with
the opposite sign). Similarity: normalized cross-correlation of the image gradients, on the
central part of the image.

With --scan-offset it also searches the time offset tau between the gyroscope and the image
stamps (gyroscope integrated over [t0 + tau, t1 + tau]) that gives the best match: an estimate
of the camera <-> IMU time offset of the recording.

Runs in the pq_cuvslam container (rosbag2_py, numpy, cv2).

Usage:
  check_rotation_compensation.py <bag> [--camera cam1] [--frames 300] [--scan-offset]
"""

import argparse

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage


def quat_to_rot(x, y, z, w):
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


class StaticTf:
    def __init__(self):
        self.parent = {}

    def add(self, msg):
        for t in msg.transforms:
            q = t.transform.rotation
            self.parent[t.child_frame_id] = (t.header.frame_id, quat_to_rot(q.x, q.y, q.z, q.w))

    def root_rotation(self, frame):
        """Rotation of frame in the root of its tree."""
        r = np.eye(3)
        while frame in self.parent:
            frame, rot = self.parent[frame]
            r = rot @ r
        return r

    def rotation(self, target, source):
        """Rotation that maps vectors from source frame to target frame."""
        return self.root_rotation(target).T @ self.root_rotation(source)


def integrate(gyro, t0, t1):
    """Rotation of the IMU from t0 to t1 (in the IMU frame at t0); t1 may be < t0."""
    if t1 == t0:
        return np.eye(3)
    sign = 1.0 if t1 > t0 else -1.0
    a, b = min(t0, t1), max(t0, t1)
    ts = gyro[:, 0]
    grid = np.concatenate(([a], ts[(ts > a) & (ts < b)], [b]))
    w = np.stack([np.interp(grid, ts, gyro[:, k]) for k in (1, 2, 3)], axis=1)
    r = np.eye(3)
    for i in range(len(grid) - 1):
        dt = (grid[i + 1] - grid[i]) / 1e9
        r = r @ rodrigues(0.5 * (w[i] + w[i + 1]) * dt)
    return r if sign > 0 else r.T


def gradient_ncc(a, b, margin):
    def grad(img):
        img = img.astype(np.float32)
        gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
        g = np.sqrt(gx * gx + gy * gy)[margin:-margin, margin:-margin]
        return (g - g.mean()) / (g.std() + 1e-6)
    return float(np.mean(grad(a) * grad(b)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--camera', default='cam1')
    ap.add_argument('--frames', type=int, default=300)
    ap.add_argument('--margin', type=int, default=40)
    ap.add_argument('--scan-offset', action='store_true',
                    help='search the gyroscope - image time offset (-150..150 ms)')
    args = ap.parse_args()
    cam = args.camera
    image_topic = f'/{cam}/infra1/image_rect_raw'
    info_topic = f'/{cam}/infra1/camera_info'
    imu_topic = f'/{cam}/imu'

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=[image_topic, info_topic, imu_topic, '/tf_static']))
    tf = StaticTf()
    K = None
    images = []
    gyro = []
    while reader.has_next() and len(images) < args.frames + 1:
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            tf.add(deserialize_message(data, TFMessage))
        elif topic == info_topic and K is None:
            K = np.array(deserialize_message(data, CameraInfo).k).reshape(3, 3)
        elif topic == imu_topic:
            m = deserialize_message(data, Imu)
            w = m.angular_velocity
            gyro.append((m.header.stamp.sec * 1e9 + m.header.stamp.nanosec, w.x, w.y, w.z))
        elif topic == image_topic:
            m = deserialize_message(data, Image)
            img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width)
            images.append((m.header.stamp.sec * 1e9 + m.header.stamp.nanosec, img))
    gyro = np.array(gyro)
    images = images[1:]  # the first frame of the bags has a stale stamp
    R_ci = tf.rotation(f'{cam}_infra1_optical_frame', f'{cam}_gyro_optical_frame')
    print(f'{cam}: {len(images)} frames, {len(gyro)} gyro samples, K fx={K[0, 0]:.1f}')
    print('camera <- gyro rotation:\n', np.round(R_ci, 3))

    Kinv = np.linalg.inv(K)
    rows = []
    for (t0, a), (t1, b) in zip(images, images[1:]):
        if not (gyro[0, 0] <= t0 and t1 <= gyro[-1, 0]) or t1 - t0 > 0.1e9:
            continue
        dR_c = R_ci @ integrate(gyro, t0, t1) @ R_ci.T
        angle = np.degrees(np.linalg.norm(cv2.Rodrigues(dR_c)[0]))
        if angle < 0.5:
            continue  # too little rotation to tell
        h, w = a.shape
        res = {}
        for name, R in (('none', np.eye(3)), ('compensated', dR_c.T), ('opposite', dR_c)):
            warped = cv2.warpPerspective(a, K @ R @ Kinv, (w, h), flags=cv2.INTER_LINEAR)
            res[name] = gradient_ncc(warped, b, args.margin)
        rows.append((angle, res['none'], res['compensated'], res['opposite']))

    if not rows:
        print('no frame pair with enough rotation')
        return
    r = np.array(rows)
    print(f'frame pairs with rotation >= 0.5 deg: {len(r)} '
          f'(rotation median {np.median(r[:, 0]):.2f} deg, max {r[:, 0].max():.2f} deg)')
    print(f'gradient NCC with the next frame, median: none {np.median(r[:, 1]):.3f}, '
          f'compensated {np.median(r[:, 2]):.3f}, opposite sign {np.median(r[:, 3]):.3f}')
    print(f'compensated better than none in {np.mean(r[:, 2] > r[:, 1]) * 100:.0f}% of the pairs, '
          f'better than the opposite sign in {np.mean(r[:, 2] > r[:, 3]) * 100:.0f}%')

    if args.scan_offset:
        pairs = [(t0, a, t1, b) for (t0, a), (t1, b) in zip(images, images[1:])
                 if gyro[0, 0] + 0.2e9 <= t0 and t1 <= gyro[-1, 0] - 0.2e9 and t1 - t0 < 0.1e9]
        pairs = pairs[::max(1, len(pairs) // 120)]
        h, w = images[0][1].shape
        scores = []
        for tau_ms in range(-150, 151, 10):
            tau = tau_ms * 1e6
            ncc = []
            for t0, a, t1, b in pairs:
                dR_c = R_ci @ integrate(gyro, t0 + tau, t1 + tau) @ R_ci.T
                warped = cv2.warpPerspective(a, K @ dR_c.T @ Kinv, (w, h), flags=cv2.INTER_LINEAR)
                ncc.append(gradient_ncc(warped, b, args.margin))
            scores.append((tau_ms, float(np.median(ncc))))
        best = max(scores, key=lambda x: x[1])
        print('time offset scan (gyro time = image time + tau): ' +
              ', '.join(f'{t}:{v:.3f}' for t, v in scores if t % 30 == 0))
        print(f'best tau {best[0]} ms (median NCC {best[1]:.3f}; tau 0: '
              f'{dict(scores)[0]:.3f})')


if __name__ == '__main__':
    main()
