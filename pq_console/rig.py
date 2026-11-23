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
Camera rigs: camera models with their sensors, rig files, import of coordinates.

Frames follow ROS: the pose of each camera is the pose of its <name>_link (x forward, y left,
z up; for a D455 the origin is the left IR imager) in the rig frame, as translation (m) and
roll, pitch, yaw (degrees, rotations about the fixed x, y, z axes: R = Rz(yaw) Ry(pitch) Rx(roll),
as tf2 setRPY). The first camera is usually at the origin.
"""

import csv
import io
import math
import os
import re

import numpy as np
import yaml

# Camera models: body (box in the link frame) and sensors (position in the link frame, field of
# view in degrees for the imagers). D455: factory transforms of the RealSense driver.
MODELS = {
    'D455': {
        'label': 'Intel RealSense D455',
        'body': {'center': [-0.0145, -0.0475, 0.0], 'size': [0.029, 0.124, 0.026]},
        'sensors': [
            {'name': 'ir1', 'label': 'IR left (depth)', 'kind': 'ir', 'position': [0.0, 0.0, 0.0],
             'fov': [87.0, 58.0]},
            {'name': 'ir2', 'label': 'IR right', 'kind': 'ir', 'position': [0.0, -0.095, 0.0],
             'fov': [87.0, 58.0]},
            {'name': 'rgb', 'label': 'RGB', 'kind': 'rgb', 'position': [-0.0003, -0.0591, 0.0],
             'fov': [90.0, 65.0]},
            {'name': 'imu', 'label': 'IMU (BMI055)', 'kind': 'imu',
             'position': [-0.016, -0.0302, 0.0074]},
        ],
    },
}

# The holder used so far: cam2 forward at the origin, cam1 49.3 deg down (CAD + gyroscopes).
DEFAULT_RIG = {
    'name': 'holder_v1',
    'description': 'cam2 forward, cam1 down (holder CAD translation, pitch from the gyroscopes)',
    'cameras': [
        {'name': 'cam2', 'model': 'D455', 'position': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]},
        {'name': 'cam1', 'model': 'D455', 'position': [-0.0134, 0.0, -0.0634],
         'rotation': [0.0, 49.3, 0.0]},
    ],
}

NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,60}$')


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def matrix_rpy(m):
    pitch = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(math.cos(pitch)) < 1e-9:   # gimbal lock: yaw folded into roll
        return math.atan2(-m[1, 2], m[1, 1]), pitch, 0.0
    return math.atan2(m[2, 1], m[2, 2]), pitch, math.atan2(m[1, 0], m[0, 0])


def pose_matrix(cam):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*[math.radians(a) for a in cam['rotation']])
    t[:3, 3] = cam['position']
    return t


def validate(rig):
    if not isinstance(rig, dict) or not isinstance(rig.get('cameras'), list) or not rig['cameras']:
        raise ValueError('a rig needs a list of cameras')
    names = set()
    out = {'name': str(rig.get('name') or 'rig'), 'description': str(rig.get('description') or ''),
           'cameras': []}
    for c in rig['cameras']:
        name = str(c.get('name', ''))
        if not NAME_RE.match(name) or name in names:
            raise ValueError(f'camera name {name!r}: missing, invalid or repeated')
        names.add(name)
        model = str(c.get('model') or 'D455')
        if model not in MODELS:
            raise ValueError(f'{name}: unknown camera model {model}')
        pos = [float(x) for x in c.get('position', [0, 0, 0])]
        rot = [float(x) for x in c.get('rotation', [0, 0, 0])]
        if len(pos) != 3 or len(rot) != 3 or not all(map(math.isfinite, pos + rot)):
            raise ValueError(f'{name}: position and rotation need 3 finite numbers')
        out['cameras'].append({'name': name, 'model': model, 'position': pos, 'rotation': rot})
    return out


def extrinsics(rig):
    """{(parent camera, child camera): transform of child_link in parent_link} for all pairs."""
    cams = rig['cameras']
    out = {}
    for a in cams:
        ta = pose_matrix(a)
        for b in cams:
            if a is b:
                continue
            rel = np.linalg.inv(ta) @ pose_matrix(b)
            out[(a['name'], b['name'])] = {
                'translation': [round(float(x), 6) for x in rel[:3, 3]],
                'rotation_rpy': [round(float(x), 6) for x in matrix_rpy(rel[:3, :3])],
                'source': f"rig {rig['name']}"}
    return out


# ---- files ----

class Rigs:
    """Rig files in <data_dir>/rigs/<name>.yaml; the active one feeds the profile builder."""

    def __init__(self, data_dir):
        self.dir = os.path.join(data_dir, 'rigs')
        self.active_file = os.path.join(self.dir, 'active')
        os.makedirs(self.dir, exist_ok=True)

    def names(self):
        return sorted(os.path.splitext(f)[0] for f in os.listdir(self.dir) if f.endswith('.yaml'))

    def load(self, name):
        if name == DEFAULT_RIG['name'] and name not in self.names():
            return validate(DEFAULT_RIG)
        if not NAME_RE.match(name or ''):
            raise ValueError(f'invalid rig name {name!r}')
        with open(os.path.join(self.dir, name + '.yaml')) as f:
            return validate(yaml.safe_load(f))

    def save(self, rig):
        rig = validate(rig)
        if not NAME_RE.match(rig['name']):
            raise ValueError(f"invalid rig name {rig['name']!r}")
        with open(os.path.join(self.dir, rig['name'] + '.yaml'), 'w') as f:
            f.write('# Camera rig of pq_console: pose of each <camera>_link in the rig frame\n'
                    '# (position m, rotation roll pitch yaw deg about the fixed x y z axes).\n')
            yaml.safe_dump(rig, f, sort_keys=False, default_flow_style=None)
        return rig

    def active(self):
        try:
            with open(self.active_file) as f:
                return self.load(f.read().strip())
        except (OSError, ValueError, yaml.YAMLError):
            return validate(DEFAULT_RIG)

    def set_active(self, name):
        self.load(name)
        with open(self.active_file, 'w') as f:
            f.write(name)


# ---- import ----

def import_text(filename, text):
    """
    Rig from a file of coordinates.

    - CSV: name, x, y, z, roll, pitch, yaw [, model] (m and degrees; a header row is allowed);
    - YAML rig file of the console (cameras with position and rotation);
    - pq_cuvslam profile YAML: the base camera at the origin, the others from its extrinsics.
    """
    name = re.sub(r'[^A-Za-z0-9_.-]', '_', os.path.splitext(os.path.basename(filename or 'rig'))[0])
    ext = os.path.splitext(filename or '')[1].lower()
    data = None
    if ext != '.csv':
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError:
            data = None
    if ext == '.csv' or not isinstance(data, dict):
        cams = []
        for row in csv.reader(io.StringIO(text)):
            row = [c.strip() for c in row]
            if not row or row[0].startswith('#'):
                continue
            try:
                nums = [float(x) for x in row[1:7]]
            except ValueError:
                continue                  # header
            if len(nums) != 6:
                raise ValueError(f'CSV row {row}: name, x, y, z, roll, pitch, yaw [, model]')
            cams.append({'name': row[0], 'model': row[7] if len(row) > 7 and row[7] else 'D455',
                         'position': nums[:3], 'rotation': nums[3:]})
        return validate({'name': name, 'cameras': cams})
    if isinstance(data, dict) and isinstance(data.get('cameras'), list) and \
            data['cameras'] and 'position' in data['cameras'][0]:
        data.setdefault('name', name)
        return validate(data)
    if isinstance(data, dict) and 'extrinsics' in data:
        return from_profile(data, name)
    raise ValueError('unknown format: CSV (name, x, y, z, roll, pitch, yaw), rig YAML or profile YAML')


def from_profile(profile, name):
    """Cameras of a pq_cuvslam profile (devices), placed by its extrinsics."""
    base = str(profile.get('base_frame', 'cam2_link')).replace('_link', '')
    devices = []
    for c in profile.get('cameras') or []:
        d = c.get('device') or c.get('name', '').split('_')[0]
        if d and d not in devices:
            devices.append(d)
    if base not in devices:
        devices.insert(0, base)
    cams = {base: {'name': base, 'model': 'D455', 'position': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}}
    for e in profile.get('extrinsics') or []:
        parent, child = e['parent'].replace('_link', ''), e['child'].replace('_link', '')
        if parent != base:
            continue
        if 'rotation_rpy_deg' in e:
            rpy = [float(x) for x in e['rotation_rpy_deg']]
        elif 'rotation_quaternion' in e:
            x, y, z, w = [float(v) for v in e['rotation_quaternion']]
            m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            rpy = [math.degrees(a) for a in matrix_rpy(m)]
        else:
            rpy = [math.degrees(float(a)) for a in e.get('rotation_rpy', [0, 0, 0])]
        cams[child] = {'name': child, 'model': 'D455',
                       'position': [float(x) for x in e['translation']], 'rotation': rpy}
    missing = [d for d in devices if d not in cams]
    if missing:
        raise ValueError(f'profile without extrinsics for {", ".join(missing)}')
    return validate({'name': profile.get('name', name),
                     'description': 'from a pq_cuvslam profile', 'cameras': list(cams.values())})
