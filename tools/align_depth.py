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
Depth aligned to the RGB camera for bags recorded without align_depth.

The RealSense driver publishes <cam>/aligned_depth_to_color/image_raw with
align_depth.enable:=true; bags recorded without it have only the depth aligned to IR1. This
tool computes the aligned depth as the driver does (rs2::align): every depth pixel is
deprojected with the depth intrinsics, moved to the color frame with the factory extrinsics
(/tf_static of the bag) and projected with the color intrinsics and distortion; the footprint
of the pixel (its two corners) is filled with its depth value, keeping the nearest value where
pixels overlap. Missing depth stays 0.

The output is a separate bag with <cam>/aligned_depth_to_color/image_raw and camera_info (the
color camera_info), with the stamps of the depth images and the receive times of the original
bag, to be played together with it:
  ros2 bag play -i <bag> -i <bag>_aligned

Usage:
  align_depth.py <bag> <out bag> [--cameras cam1,cam2]
"""

import argparse
import time

import numpy as np


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class StaticTf:
    """Static transforms of a bag: pose of a frame in another one through the tree."""

    def __init__(self):
        self.parent = {}   # child -> (parent, 4x4 pose of child in parent)

    def add(self, msg):
        for t in msg.transforms:
            r, p = t.transform.rotation, t.transform.translation
            m = np.eye(4)
            m[:3, :3] = quat_to_matrix((r.x, r.y, r.z, r.w))
            m[:3, 3] = (p.x, p.y, p.z)
            self.parent[t.child_frame_id] = (t.header.frame_id, m)

    def to_root(self, frame):
        m = np.eye(4)   # pose of the frame in the root
        while frame in self.parent:
            parent, pose = self.parent[frame]
            m = pose @ m
            frame = parent
        return frame, m

    def pose(self, target, source):
        """Transform from source coordinates to target coordinates."""
        root_t, t = self.to_root(target)
        root_s, s = self.to_root(source)
        if root_t != root_s:
            raise KeyError(f'no static transform between {target} and {source}')
        return np.linalg.inv(t) @ s


def project(points, k, d):
    """Pinhole + plumb_bob projection (OpenCV convention), points N x 3 in the camera frame."""
    x = points[:, 0] / points[:, 2]
    y = points[:, 1] / points[:, 2]
    if d is not None and np.any(d):
        k1, k2, p1, p2, k3 = (list(d) + [0.0] * 5)[:5]
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x, y = xd, yd
    return k[0, 0] * x + k[0, 2], k[1, 1] * y + k[1, 2]


class Aligner:
    """Depth (aligned to IR1) to the color camera, rs2::align style."""

    def __init__(self, depth_info, color_info, color_from_depth):
        self.w, self.h = color_info.width, color_info.height
        self.kc = np.array(color_info.k).reshape(3, 3)
        self.dc = np.array(color_info.d, dtype=float) if len(color_info.d) else None
        kd = np.array(depth_info.k).reshape(3, 3)
        if any(depth_info.d):
            raise ValueError('distorted depth camera_info: not supported')
        self.r = color_from_depth[:3, :3]
        self.t = color_from_depth[:3, 3]
        v, u = np.mgrid[0:depth_info.height, 0:depth_info.width].astype(np.float64)
        # Viewing rays (z = 1) of the two corners of every depth pixel.
        self.rays = [np.stack([(u + c - kd[0, 2]) / kd[0, 0], (v + c - kd[1, 2]) / kd[1, 1]],
                              axis=-1).reshape(-1, 2) for c in (-0.5, 0.5)]

    def align(self, depth_raw, scale):
        flat = depth_raw.reshape(-1)
        idx = np.nonzero(flat)[0]
        out = np.zeros(self.w * self.h, np.uint16)
        if len(idx) == 0:
            return out.reshape(self.h, self.w)
        z = flat[idx].astype(np.float64) / scale
        corners = []
        for rays in self.rays:
            p = np.empty((len(idx), 3))
            p[:, :2] = rays[idx] * z[:, None]
            p[:, 2] = z
            pc = p @ self.r.T + self.t
            u, v = project(pc, self.kc, self.dc)
            corners.append((np.floor(u + 0.5).astype(np.int64), np.floor(v + 0.5).astype(np.int64)))
        (x0, y0), (x1, y1) = corners
        ok = (x0 >= 0) & (y0 >= 0) & (x1 < self.w) & (y1 < self.h) & (x1 >= x0) & (y1 >= y0)
        x0, y0, x1, y1, vals = x0[ok], y0[ok], x1[ok], y1[ok], flat[idx][ok]
        # The footprint is at most a few color pixels: fill it offset by offset.
        big = np.iinfo(np.uint16).max
        best = np.full(self.w * self.h, big, np.uint16)
        span = int(max((x1 - x0).max(initial=0), (y1 - y0).max(initial=0))) + 1
        for dy in range(min(span, 4)):
            for dx in range(min(span, 4)):
                m = (x0 + dx <= x1) & (y0 + dy <= y1)
                np.minimum.at(best, (y0[m] + dy) * self.w + x0[m] + dx, vals[m])
        out[best != big] = best[best != big]
        return out.reshape(self.h, self.w)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--cameras', default='cam1,cam2')
    ap.add_argument('--scale', type=float, default=1000.0, help='raw depth units per meter')
    args = ap.parse_args()

    from rclpy.serialization import deserialize_message, serialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import CameraInfo, Image

    cams = args.cameras.split(',')
    types_ok = {f'/{c}/{s}' for c in cams for s in
                ('depth/image_rect_raw', 'depth/camera_info', 'color/camera_info')} | {'/tf_static'}

    def reader():
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=''),
               rosbag2_py.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in types if t in types_ok]))
        return r, types

    # Pass 1: calibrations (first camera_info of each stream, static transforms).
    r, types = reader()
    infos, tf = {}, StaticTf()
    while r.has_next() and len(infos) < 2 * len(cams):
        topic, data, _ = r.read_next()
        if topic == '/tf_static':
            tf.add(deserialize_message(data, get_message(types[topic])))
        elif topic.endswith('camera_info') and topic not in infos:
            infos[topic] = deserialize_message(data, CameraInfo)
    del r
    aligners, color_infos = {}, {}
    for c in cams:
        di, ci = infos.get(f'/{c}/depth/camera_info'), infos.get(f'/{c}/color/camera_info')
        if di is None or ci is None:
            raise SystemExit(f'{c}: depth or color camera_info missing')
        m = tf.pose(ci.header.frame_id, di.header.frame_id)
        print(f'{c}: depth {di.width}x{di.height} -> color {ci.width}x{ci.height}, '
              f'translation {np.round(m[:3, 3] * 1000, 1).tolist()} mm, rotation '
              f'{np.degrees(np.arccos(np.clip((np.trace(m[:3, :3]) - 1) / 2, -1, 1))):.2f} deg')
        aligners[c] = Aligner(di, ci, m)
        color_infos[c] = ci

    # Pass 2: depth images.
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=args.out, storage_id='mcap',
                                     storage_preset_profile='zstd_fast'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for c in cams:
        for name, type_ in (('image_raw', 'sensor_msgs/msg/Image'),
                            ('camera_info', 'sensor_msgs/msg/CameraInfo')):
            w.create_topic(rosbag2_py.TopicMetadata(
                0, f'/{c}/aligned_depth_to_color/{name}', type_, 'cdr'))
    r, types = reader()
    counts = dict.fromkeys(cams, 0)
    valid = dict.fromkeys(cams, 0.0)
    start = time.time()
    while r.has_next():
        topic, data, t = r.read_next()
        c = topic.split('/')[1]
        if topic != f'/{c}/depth/image_rect_raw' or c not in aligners:
            continue
        msg = deserialize_message(data, Image)
        if msg.encoding not in ('16UC1', 'mono16'):
            continue
        depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)
        aligned = aligners[c].align(depth[:, :msg.width], args.scale)
        out = Image()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = color_infos[c].header.frame_id
        out.height, out.width = aligned.shape
        out.encoding = '16UC1'
        out.step = out.width * 2
        out.data = aligned.tobytes()
        info = color_infos[c]
        info.header.stamp = msg.header.stamp
        w.write(f'/{c}/aligned_depth_to_color/image_raw', serialize_message(out), t)
        w.write(f'/{c}/aligned_depth_to_color/camera_info', serialize_message(info), t)
        counts[c] += 1
        valid[c] += np.count_nonzero(aligned) / aligned.size
        if sum(counts.values()) % 500 == 0:
            print(f'  {sum(counts.values())} images, {time.time() - start:.0f} s', flush=True)
    del w
    for c in cams:
        n = max(counts[c], 1)
        print(f'{c}: {counts[c]} aligned depth images, valid depth {100 * valid[c] / n:.1f} %')


if __name__ == '__main__':
    main()
