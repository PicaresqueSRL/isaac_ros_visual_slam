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
MCAP for Rerun from a SLAM recording of record_slam.sh (or a run output folder with the CSV
files, e.g. of a live run), with the camera images of the source bag, all on the sensor time
(header stamps), so that the trajectory and the images play together.

Rerun reads PoseStamped, PointCloud2, TFMessage and CompressedImage from MCAP, not Odometry or
Path. Converted:
  /visual_slam/tracking/odometry   -> /rerun/odometry/pose (PoseStamped) and
                                      /rerun/odometry/trail (PointCloud2, path so far)
  nav_msgs/Path, PoseArray         -> /rerun/<name> (PointCloud2)
  TFMessage, PointCloud2           copied (landmarks, map -> odom -> base)
  images of the source bag         -> <topic>/compressed (JPEG), and its /tf_static
With --scale the positions are scaled (Mono run with the scale of mono_scale.py).

Runs in the pq_cuvslam container (rosbag2_py, cv2, numpy).

Usage:
  to_rerun.py <recording bag> <source bag> <output> [--images /cam2/color/image_raw,...]
      [--scale S] [--jpeg-quality 80] [--max-image-rate 15]
"""

import argparse
import csv
import importlib.util
import os

import cv2
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

_spec = importlib.util.spec_from_file_location(
    'scale_recording', os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    'scale_recording.py'))
sr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sr)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def open_reader(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def to_bgr(img):
    enc = img.encoding
    a = np.frombuffer(bytes(img.data), dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        a = a.reshape(img.height, img.step)[:, :img.width * 3].reshape(img.height, img.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else a
    if enc in ('mono8', '8UC1'):
        return a.reshape(img.height, img.step)[:, :img.width]
    if enc in ('mono16', '16UC1'):
        a16 = np.frombuffer(bytes(img.data), dtype=np.uint16).reshape(img.height, img.step // 2)
        return cv2.convertScaleAbs(a16[:, :img.width], alpha=255.0 / max(1, a16.max()))
    return None


def rerun_base(topic):
    """Rerun name of a SLAM topic: /visual_slam/tracking/odometry -> /rerun/odometry,
    /cam1_vio/visual_slam/tracking/odometry_guarded -> /rerun/cam1_vio/odometry_guarded."""
    rel = topic.replace('/visual_slam/tracking', '').replace('/visual_slam', '').strip('/')
    return '/rerun/' + rel


# Throttling (the Rerun viewer keeps every message: a full trail or path at every frame grows
# quadratically with the length of the run).
TRAIL_PERIOD_NS = 500_000_000   # trail update every 0.5 s (the pose is written at every frame)
PATH_PERIOD_NS = 1_000_000_000  # paths and pose arrays: one per second, plus the last one
# Topics already written as converted odometry (the same poses again).
SKIP_TOPICS = ('/vo_pose', '/slam_odometry')


# All SLAM data in one frame: the trackers have their own map/odom frames (cam1_odom, cam2_map,
# odom of the selector, ...) that the recording does not connect, and the Rerun viewer does not
# place entities whose frames it cannot resolve. Every trajectory starts at its own origin.
WORLD_FRAME = 'world'


def write_recording(path, write, s):
    """SLAM topics of a recording of record_slam.sh. Returns the first odometry stamp (ns)."""
    reader = open_reader(path)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    trails = {}
    last_trail = {}
    last_path = {}
    pending_path = {}
    first_odom = None
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        type_ = types[topic]
        if type_ not in ('nav_msgs/msg/Odometry', 'nav_msgs/msg/Path',
                         'geometry_msgs/msg/PoseArray', 'sensor_msgs/msg/PointCloud2',
                         'tf2_msgs/msg/TFMessage', 'geometry_msgs/msg/PoseStamped'):
            continue
        if any(topic.endswith(x) for x in SKIP_TOPICS):
            continue
        msg = deserialize_message(data, classes.setdefault(type_, get_message(type_)))
        if type_ == 'tf2_msgs/msg/TFMessage':
            continue   # map/odom transforms of the trackers: the poses are written in WORLD_FRAME
        t = stamp_ns(msg.header) or t_rec
        msg.header.frame_id = WORLD_FRAME
        if type_ == 'nav_msgs/msg/Odometry':
            first_odom = t if first_odom is None else min(first_odom, t)
            sr.scale_position(msg.pose.pose.position, s)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            base = rerun_base(topic)
            write(base + '/pose', 'geometry_msgs/msg/PoseStamped', pose, t)
            p = msg.pose.pose.position
            trail = trails.setdefault(topic, [])
            trail.append((p.x, p.y, p.z))
            if t - last_trail.get(topic, -TRAIL_PERIOD_NS) >= TRAIL_PERIOD_NS:
                last_trail[topic] = t
                write(base + '/trail', 'sensor_msgs/msg/PointCloud2',
                      create_cloud_xyz32(msg.header, trail[::2]), t)
        elif type_ in ('nav_msgs/msg/Path', 'geometry_msgs/msg/PoseArray'):
            poses = [ps.pose for ps in msg.poses] if type_ == 'nav_msgs/msg/Path' \
                else list(msg.poses)
            pts = [(q.position.x * s, q.position.y * s, q.position.z * s) for q in poses]
            cloud = create_cloud_xyz32(msg.header, pts)
            if t - last_path.get(topic, -PATH_PERIOD_NS) >= PATH_PERIOD_NS:
                last_path[topic] = t
                write(rerun_base(topic), 'sensor_msgs/msg/PointCloud2', cloud, t)
                pending_path.pop(topic, None)
            else:
                pending_path[topic] = (cloud, t)
        elif type_ == 'sensor_msgs/msg/PointCloud2':
            if s != 1.0:
                sr.scale_cloud(msg, s)
            write(topic, type_, msg, t)
        else:
            sr.scale_position(msg.pose.position, s)
            write(topic, type_, msg, t)
    del reader
    for topic, (cloud, t) in pending_path.items():   # last state of every path
        write(rerun_base(topic), 'sensor_msgs/msg/PointCloud2', cloud, t)
    return first_odom


def write_run_folder(path, write, s):
    """Trajectories of a run output folder: odometry.csv (pose and trail of the base frame) and
    slam_poses.csv (final SLAM path, written at the end). Returns the first odometry stamp (ns)."""
    first_odom, t, header = None, 0, None
    trail = []
    with open(os.path.join(path, 'odometry.csv')) as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        t = int(round(float(r['timestamp']) * 1e9))
        first_odom = t if first_odom is None else first_odom
        header = Header()
        header.stamp.sec, header.stamp.nanosec = t // 1_000_000_000, t % 1_000_000_000
        header.frame_id = WORLD_FRAME
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(r[k]) * s for k in 'xyz')
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = (float(r[k]) for k in ('qx', 'qy', 'qz', 'qw'))
        write('/rerun/odometry/pose', 'geometry_msgs/msg/PoseStamped', pose, t)
        trail.append((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        write('/rerun/odometry/trail', 'sensor_msgs/msg/PointCloud2',
              create_cloud_xyz32(header, trail), t)
    slam = os.path.join(path, 'slam_poses.csv')
    if header is not None and os.path.isfile(slam):
        with open(slam) as f:
            pts = [tuple(float(r[k]) * s for k in 'xyz') for r in csv.DictReader(f)]
        if pts:
            header.frame_id = WORLD_FRAME
            write('/rerun/slam_path', 'sensor_msgs/msg/PointCloud2',
                  create_cloud_xyz32(header, pts), t)
    return first_odom


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('recording', help='recording of record_slam.sh, or a run output folder '
                    '(odometry.csv, slam_poses.csv), e.g. of a live run')
    ap.add_argument('source')
    ap.add_argument('out')
    ap.add_argument('--images', default='/cam2/color/image_raw,/cam1/color/image_raw')
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--jpeg-quality', type=int, default=80)
    ap.add_argument('--max-image-rate', type=float, default=15.0,
                    help='images per second and topic at most (0: all)')
    args = ap.parse_args()
    if os.path.exists(args.out):
        raise SystemExit(f'{args.out} exists')

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=args.out, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    created = {}

    def write(topic, type_, msg, t):
        if topic not in created:
            writer.create_topic(sr.topic_metadata(len(created), topic, type_))
            created[topic] = 0
        created[topic] += 1
        writer.write(topic, serialize_message(msg), t)

    if os.path.isfile(os.path.join(args.recording, 'odometry.csv')):
        first_odom = write_run_folder(args.recording, write, args.scale)
    else:
        first_odom = write_recording(args.recording, write, args.scale)

    # Images and static TF of the source bag.
    image_topics = [x for x in args.images.split(',') if x]
    reader = open_reader(args.source)
    available = {t.name for t in reader.get_all_topics_and_types()}
    missing = [x for x in image_topics if x not in available]
    image_topics = [x for x in image_topics if x in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=image_topics + ['/tf_static']))
    tf_class = get_message('tf2_msgs/msg/TFMessage')
    last = {}
    min_dt = 1e9 / args.max_image_rate if args.max_image_rate > 0 else 0
    while reader.has_next():
        topic, data, t_rec = reader.read_next()
        if topic == '/tf_static':
            write('/tf_static', 'tf2_msgs/msg/TFMessage', deserialize_message(data, tf_class),
                  t_rec)
            continue
        img = deserialize_message(data, Image)
        t = stamp_ns(img.header)
        if first_odom is not None and t < first_odom - 1_000_000_000:
            continue   # stale first frame of the D455 streams (old timestamp)
        if topic in last and (t - last[topic] < min_dt or t < last[topic]):
            continue   # rate limit
        last[topic] = t
        bgr = to_bgr(img)
        if bgr is None:
            continue
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
        if not ok:
            continue
        out = CompressedImage()
        out.header = img.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        write(topic + '/compressed', 'sensor_msgs/msg/CompressedImage', out, t)
    writer.close()   # writes the summary and metadata.yaml
    print(f'written {args.out}')
    for name, n in sorted(created.items()):
        print(f'  {name}: {n}')
    if missing:
        print('image topics not in the source bag: ' + ', '.join(missing))


if __name__ == '__main__':
    main()
