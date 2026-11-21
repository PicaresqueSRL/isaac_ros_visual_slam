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
Applies a constant scale to a SLAM recording of record_slam.sh (MCAP), e.g. a Mono run with
the scale estimated by mono_scale.py: positions of odometry, paths and pose arrays, point
clouds (landmarks) and the TF transforms whose parent is the map or odom frame. Other
messages are copied unchanged.

Runs in the pq_cuvslam container (rosbag2_py, numpy).

Usage:
  scale_recording.py <input bag> <output bag> <scale> [--frames map,odom]
"""

import argparse
import os

import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def topic_metadata(i, name, type_):
    try:
        return rosbag2_py.TopicMetadata(id=i, name=name, type=type_, serialization_format='cdr')
    except TypeError:
        return rosbag2_py.TopicMetadata(name=name, type=type_, serialization_format='cdr')


def scale_position(p, s):
    p.x, p.y, p.z = p.x * s, p.y * s, p.z * s


def scale_cloud(msg, s):
    offsets = {f.name: f.offset for f in msg.fields if f.datatype == 7}  # FLOAT32
    if not all(k in offsets for k in 'xyz'):
        return False
    n = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step).copy()
    for k in 'xyz':
        o = offsets[k]
        v = raw[:, o:o + 4].copy().view(np.float32)
        raw[:, o:o + 4] = (v * s).view(np.uint8)
    msg.data = raw.tobytes()
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('scale', type=float)
    ap.add_argument('--frames', default='map,odom',
                    help='TF parents whose transforms are scaled')
    args = ap.parse_args()
    if os.path.exists(args.dst):
        raise SystemExit(f'{args.dst} exists')
    s = args.scale
    tf_parents = set(args.frames.split(','))

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (name, type_) in enumerate(sorted(types.items())):
        writer.create_topic(topic_metadata(i, name, type_))
    classes = {}
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        type_ = types[topic]
        changed = True
        if type_ in ('nav_msgs/msg/Odometry', 'nav_msgs/msg/Path', 'geometry_msgs/msg/PoseArray',
                     'geometry_msgs/msg/PoseStamped', 'sensor_msgs/msg/PointCloud2',
                     'tf2_msgs/msg/TFMessage'):
            msg = deserialize_message(data, classes.setdefault(type_, get_message(type_)))
            if type_ == 'nav_msgs/msg/Odometry':
                scale_position(msg.pose.pose.position, s)
                v = msg.twist.twist.linear
                v.x, v.y, v.z = v.x * s, v.y * s, v.z * s
            elif type_ == 'nav_msgs/msg/Path':
                for ps in msg.poses:
                    scale_position(ps.pose.position, s)
            elif type_ == 'geometry_msgs/msg/PoseArray':
                for pose in msg.poses:
                    scale_position(pose.position, s)
            elif type_ == 'geometry_msgs/msg/PoseStamped':
                scale_position(msg.pose.position, s)
            elif type_ == 'sensor_msgs/msg/PointCloud2':
                changed = scale_cloud(msg, s)
            else:
                changed = False
                for tr in msg.transforms:
                    if tr.header.frame_id in tf_parents:
                        scale_position(tr.transform.translation, s)
                        changed = True
            if changed:
                data = serialize_message(msg)
                counts[topic] = counts.get(topic, 0) + 1
        writer.write(topic, data, t)
    del writer
    print(f'written {args.dst}, scale {s}')
    for topic, n in sorted(counts.items()):
        print(f'  scaled {topic}: {n}')


if __name__ == '__main__':
    main()
