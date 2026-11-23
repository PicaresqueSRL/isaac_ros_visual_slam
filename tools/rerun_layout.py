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
Rerun recording (.rrd) with a fixed layout from the MCAP of to_rerun.py: camera images on the
left (cam2 IR and RGB on top, cam1 IR and RGB below), the SLAM in 3D on the right at full
height (trajectories as lines, current poses as axes, landmarks). An MCAP cannot carry a Rerun
layout (blueprint), an .rrd can. Open with: rerun <file>.rrd (viewer of the same minor version
as the SDK used here).

Runs in the pq_cuvslam container (rosbag2_py) with rerun-sdk and pyarrow<19 installed in a
separate folder appended to the Python path (the ROS numpy 1.x is kept):
  pip install --target <pylib> rerun-sdk "pyarrow<19"

Usage:
  rerun_layout.py <to_rerun.py MCAP folder> <out.rrd> [--pylib <folder>]
"""

import argparse
import sys

import numpy as np

# Trajectories: Rerun name under /rerun -> (entity, color, radius). Unknown ones get grey.
TRAJECTORIES = {
    'odometry_fused': ('fused', [230, 40, 40], 0.03),
    'cam1_vio/odometry_guarded': ('cam1/guarded', [40, 110, 230], 0.02),
    'cam2_vio/odometry_guarded': ('cam2/guarded', [40, 170, 60], 0.02),
    'cam1_vio/odometry': ('cam1/raw', [150, 170, 210], 0.01),
    'cam2_vio/odometry': ('cam2/raw', [160, 200, 160], 0.01),
    'odometry': ('odometry', [230, 40, 40], 0.03),
}
CAMERAS = {
    '/cam2/infra1/image_rect_raw/compressed': 'cameras/cam2/ir',
    '/cam2/color/image_raw/compressed': 'cameras/cam2/rgb',
    '/cam1/infra1/image_rect_raw/compressed': 'cameras/cam1/ir',
    '/cam1/color/image_raw/compressed': 'cameras/cam1/rgb',
}


def cloud_xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    if n == 0 or not all(k in offsets for k in 'xyz'):
        return np.zeros((0, 3), np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step)
    cols = [raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0] for k in 'xyz']
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mcap')
    ap.add_argument('out')
    ap.add_argument('--pylib', default='', help='folder with rerun-sdk (appended to sys.path)')
    args = ap.parse_args()
    if args.pylib:
        sys.path.append(args.pylib)
    import rerun as rr
    import rerun.blueprint as rrb
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    image_views = [rrb.Spatial2DView(origin=e, name=e.split('/', 1)[1].replace('/', ' '))
                   for e in CAMERAS.values()]
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Grid(*image_views, grid_columns=2),
            rrb.Spatial3DView(origin='world', name='SLAM'),
            column_shares=[1, 1]),
        collapse_panels=True)
    rr.init('pq_cuvslam')
    rr.save(args.out, default_blueprint=blueprint)
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.mcap, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        type_ = types[topic]
        if type_ in ('tf2_msgs/msg/TFMessage',):
            continue
        msg = deserialize_message(data, classes.setdefault(type_, get_message(type_)))
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec or t
        rr.set_time('time', timestamp=np.datetime64(stamp, 'ns'))
        entity = None
        if topic in CAMERAS:
            entity = CAMERAS[topic]
            rr.log(entity, rr.EncodedImage(contents=bytes(msg.data), media_type='image/jpeg'))
        elif topic.startswith('/rerun/'):
            name = topic[len('/rerun/'):]
            for suffix in ('/pose', '/trail'):
                if name.endswith(suffix):
                    key = name[:-len(suffix)]
                    ent, color, radius = TRAJECTORIES.get(key, (key, [128, 128, 128], 0.01))
                    entity = f'world/{ent}'
                    if suffix == '/pose':
                        p, q = msg.pose.position, msg.pose.orientation
                        rr.log(entity + '/pose', rr.Transform3D(
                            translation=[p.x, p.y, p.z],
                            quaternion=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])),
                            rr.TransformAxes3D(0.3))
                    else:
                        pts = cloud_xyz(msg)
                        if len(pts) > 1:
                            rr.log(entity + '/trail', rr.LineStrips3D(
                                [pts], colors=[color], radii=[radius]))
                    break
            else:
                # SLAM paths and pose graph nodes: thin lines / points.
                pts = cloud_xyz(msg)
                if len(pts):
                    entity = 'world/slam/' + name.replace('/', '_')
                    if name.endswith('path'):
                        rr.log(entity, rr.LineStrips3D(
                            [pts], colors=[[200, 120, 0]], radii=[0.01]))
                    else:
                        rr.log(entity, rr.Points3D(pts, colors=[[200, 120, 0]], radii=[0.02]))
        elif type_ == 'sensor_msgs/msg/PointCloud2' and 'landmarks' in topic:
            cam = 'cam1' if topic.startswith('/cam1') else 'cam2'
            entity = f'world/{cam}/landmarks'
            color = [90, 140, 240] if cam == 'cam1' else [90, 200, 110]
            rr.log(entity, rr.Points3D(cloud_xyz(msg), colors=[color], radii=[0.01]))
        if entity:
            counts[entity] = counts.get(entity, 0) + 1
    print(f'written {args.out}')
    for e, n in sorted(counts.items()):
        print(f'  {e}: {n}')


if __name__ == '__main__':
    main()
