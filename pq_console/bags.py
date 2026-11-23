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
Bags: folder browsing, content of a bag, SLAM profiles for a bag.

A bag is a rosbag2 folder (metadata.yaml) or a single .mcap / .db3 file. Its streams are
grouped by camera (/<camera>/<stream>/...), the D455 naming of the RealSense driver.
"""

import math
import os
import re
import time

import yaml

BAG_FILES = ('.mcap', '.db3')
# Image streams of a D455, in display order: (key, topic suffix, label)
STREAMS = [('infra1', 'infra1/image_rect_raw', 'IR left'),
           ('infra2', 'infra2/image_rect_raw', 'IR right'),
           ('color', 'color/image_raw', 'RGB'),
           ('depth', 'depth/image_rect_raw', 'depth (IR1)'),
           ('aligned', 'aligned_depth_to_color/image_raw', 'depth (RGB)')]
IMU_SUFFIXES = ('imu', 'gyro/sample', 'accel/sample')
# cam2_link -> cam1_link of the current holder (CAD translation, pitch from the gyroscopes);
# replaced by the rig of the console when one is saved.
DEFAULT_EXTRINSICS = {('cam2', 'cam1'): {'translation': [-0.0134, 0.0, -0.0634],
                                         'rotation_rpy': [0.0, 0.8604, 0.0],
                                         'source': 'holder CAD + gyroscope pitch'}}
IMU_NOISE = {'gyro_noise_density': 0.000244, 'gyro_random_walk': 0.000019393,
             'accel_noise_density': 0.001862, 'accel_random_walk': 0.003,
             'frequency': 200.0, 'source': 'default', 'date': None}


# ---- browsing ----

def inside(path, roots):
    real = os.path.realpath(path)
    return any(real == r or real.startswith(r.rstrip('/') + '/') for r in map(os.path.realpath, roots))


def is_bag_dir(path):
    return os.path.isfile(os.path.join(path, 'metadata.yaml'))


def browse(path, roots):
    """Folders and bags of a folder (only inside the configured roots)."""
    roots = [r for r in roots if r and os.path.isdir(r)]
    if not path:
        return {'path': '', 'parent': None, 'roots': roots,
                'entries': [{'name': r, 'path': r, 'kind': 'dir'} for r in roots]}
    if not inside(path, roots) or not os.path.isdir(path):
        raise ValueError(f'{path}: not a folder of the console')
    entries = []
    for name in sorted(os.listdir(path), key=str.lower):
        if name.startswith('.'):
            continue
        full = os.path.join(path, name)
        try:
            st = os.stat(full)
        except OSError:
            continue
        if os.path.isdir(full):
            kind = 'bag' if is_bag_dir(full) else 'dir'
            entries.append({'name': name, 'path': full, 'kind': kind, 'mtime': st.st_mtime})
        elif name.endswith(BAG_FILES) and not is_bag_dir(path):
            entries.append({'name': name, 'path': full, 'kind': 'bagfile', 'size': st.st_size,
                            'mtime': st.st_mtime})
    parent = os.path.dirname(path.rstrip('/'))
    is_root = any(os.path.realpath(path) == os.path.realpath(r) for r in roots)
    return {'path': path, 'parent': None if is_root else parent, 'roots': roots,
            'entries': entries, 'is_bag': is_bag_dir(path)}


# ---- content of a bag ----

def dir_size(path):
    if os.path.isfile(path):
        return os.path.getsize(path)
    return sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(path) for f in fs)


def bag_name(path):
    base = os.path.basename(path.rstrip('/'))
    return os.path.splitext(base)[0] if os.path.isfile(path) else base


def sidecar(path, suffix):
    """File or bag next to the bag: <bag><suffix> (e.g. _aligned, .camera_settings.yaml)."""
    base = path.rstrip('/')
    if os.path.isfile(base):
        base = os.path.splitext(base)[0]
    return base + suffix


def read_first(path, storage, topics, type_of):
    """First message of each topic (camera_info: resolution), stopping early."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {}
    if not topics:
        return out
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    start = time.time()
    while reader.has_next() and len(out) < len(topics) and time.time() - start < 10:
        topic, data, _ = reader.read_next()
        if topic not in out:
            out[topic] = deserialize_message(data, get_message(type_of[topic]))
    return out


def bag_info(path, roots):
    """Content of a bag: duration, streams by camera, other topics, settings, aligned depth."""
    import rosbag2_py
    if not inside(path, roots):
        raise ValueError(f'{path}: not a folder of the console')
    m = rosbag2_py.Info().read_metadata(path, '')
    duration = m.duration.nanoseconds / 1e9
    topics = {t.topic_metadata.name: {'type': t.topic_metadata.type, 'count': t.message_count}
              for t in m.topics_with_message_count}
    aligned_path = sidecar(path, '_aligned')
    extra = {}
    if is_bag_dir(aligned_path):
        ma = rosbag2_py.Info().read_metadata(aligned_path, '')
        extra = {t.topic_metadata.name: {'type': t.topic_metadata.type, 'count': t.message_count,
                                         'sidecar': True} for t in ma.topics_with_message_count}
    all_topics = {**topics, **extra}

    # Resolution of each image stream (first camera_info).
    infos = [t for t in topics if t.endswith('/camera_info')]
    first = read_first(path, m.storage_identifier, infos, {t: topics[t]['type'] for t in infos})

    cameras = {}
    used = set()
    for name in sorted({t.split('/')[1] for t in all_topics if t.count('/') >= 3}):
        streams = []
        for key, suffix, label in STREAMS:
            topic = f'/{name}/{suffix}'
            if topic not in all_topics:
                continue
            t = all_topics[topic]
            info_topic = topic.rsplit('/', 1)[0] + '/camera_info'
            ci = first.get(info_topic)
            streams.append({'key': key, 'label': label, 'topic': topic, 'count': t['count'],
                            'hz': round(t['count'] / duration, 1) if duration else None,
                            'width': ci.width if ci else None, 'height': ci.height if ci else None,
                            'distortion': ci.distortion_model if ci and any(ci.d) else 'none',
                            'sidecar': t.get('sidecar', False)})
            used.update({topic, info_topic})
        imu = None
        for suffix in IMU_SUFFIXES:
            topic = f'/{name}/{suffix}'
            if topic in all_topics:
                t = all_topics[topic]
                imu = {'topic': topic, 'count': t['count'],
                       'hz': round(t['count'] / duration, 1) if duration else None}
                used.add(topic)
                break
        if streams or imu:
            cameras[name] = {'streams': streams, 'imu': imu}
    for name in cameras:
        used.update(t for t in all_topics if t.startswith(f'/{name}/') and t.endswith('metadata'))
    others = [{'topic': t, 'type': v['type'], 'count': v['count']}
              for t, v in sorted(all_topics.items()) if t not in used]

    settings = None
    settings_file = sidecar(path, '.camera_settings.yaml')
    if os.path.isfile(settings_file):
        with open(settings_file) as f:
            settings = yaml.safe_load(f)
    return {'path': path, 'name': bag_name(path), 'storage': m.storage_identifier,
            'start': m.starting_time.nanoseconds / 1e9, 'duration_s': duration,
            'size': dir_size(path), 'messages': m.message_count,
            'aligned_bag': aligned_path if extra else None,
            'cameras': cameras, 'others': others, 'topics': sorted(all_topics),
            'camera_settings': settings}


# ---- SLAM profiles for a bag ----

def profile_topics(profile):
    """Topics a profile reads (images, camera_info, depth, IMU, gyroscopes of the sync)."""
    topics = set()
    for c in profile.get('cameras') or []:
        topics.update({c.get('image_topic'), c.get('camera_info_topic')})
    depth = profile.get('depth')
    for d in (depth if isinstance(depth, list) else [depth] if depth else []):
        topics.add(d.get('topic'))
    if profile.get('imu'):
        topics.add(profile['imu'].get('topic'))
    sync = profile.get('sync') or {}
    if sync.get('rotation_compensation'):
        topics.update(d.get('imu_topic') for d in sync.get('devices') or [])
    return {t for t in topics if t}


def compatibility(profile_topics_, bag_topics):
    missing = sorted(set(profile_topics_) - set(bag_topics))
    return {'ok': not missing, 'missing': missing}


def stream_info(info, cam, key):
    for s in info['cameras'].get(cam, {}).get('streams', []):
        if s['key'] == key:
            return s
    return None


def build_profile(info, selection, extrinsics=None):
    """
    Profile for the sensors chosen on a bag.

    selection: {'cameras': {cam: {'ir': bool, 'rgb': bool, 'depth': bool}}, 'imu': cam or ''}
    Mode: one IR pair (+ its IMU) -> multicamera / inertial; several IR pairs -> multicamera;
    one RGB -> mono; one image + depth -> rgbd; anything else -> multisensor (cuVSLAM >= 17).
    Returns (profile dict, notes).
    """
    extrinsics = extrinsics or DEFAULT_EXTRINSICS
    sel = {c: v for c, v in (selection.get('cameras') or {}).items()
           if v.get('ir') or v.get('rgb') or v.get('depth')}
    imu_cam = selection.get('imu') or ''
    if not sel:
        raise ValueError('choose at least one camera stream')
    notes = []
    order = sorted(sel, key=lambda c: (c != 'cam2', c))   # cam2 (forward) first: base frame
    cameras, depths = [], []
    for cam in order:
        s = sel[cam]
        if s.get('ir'):
            for k in ('infra1', 'infra2'):
                if not stream_info(info, cam, k):
                    raise ValueError(f'{cam}: no {k} in the bag')
                cameras.append({'name': f'{cam}_ir{k[-1]}', 'device': cam, 'key': k,
                                'image_topic': f'/{cam}/{k}/image_rect_raw',
                                'camera_info_topic': f'/{cam}/{k}/camera_info',
                                'optical_frame': f'{cam}_{k}_optical_frame'})
        if s.get('rgb'):
            if not stream_info(info, cam, 'color'):
                raise ValueError(f'{cam}: no RGB in the bag')
            cameras.append({'name': f'{cam}_rgb', 'device': cam, 'key': 'color',
                            'image_topic': f'/{cam}/color/image_raw',
                            'camera_info_topic': f'/{cam}/color/camera_info',
                            'optical_frame': f'{cam}_color_optical_frame'})
        if s.get('depth'):
            # Depth with IR1 (native alignment) when IR is used, else aligned to the RGB.
            if s.get('ir') and stream_info(info, cam, 'depth'):
                idx = next(i for i, c in enumerate(cameras) if c['name'] == f'{cam}_ir1')
                depths.append({'topic': f'/{cam}/depth/image_rect_raw', 'camera_index': idx})
            elif s.get('rgb') and stream_info(info, cam, 'aligned'):
                idx = next(i for i, c in enumerate(cameras) if c['name'] == f'{cam}_rgb')
                depths.append({'topic': f'/{cam}/aligned_depth_to_color/image_raw',
                               'camera_index': idx})
            else:
                raise ValueError(f'{cam}: depth needs IR (depth aligned to IR1) or RGB with the '
                                 'depth aligned to it (aligned_depth_to_color, '
                                 'tools/align_depth.py)')
    devices = sorted({c['device'] for c in cameras}, key=lambda c: (c != 'cam2', c))
    n_ir_pairs = sum(1 for c in cameras if c['key'] == 'infra1')
    only_ir = all(c['key'] in ('infra1', 'infra2') for c in cameras)
    if imu_cam and not (info['cameras'].get(imu_cam) or {}).get('imu'):
        raise ValueError(f'{imu_cam}: no IMU in the bag')

    if only_ir and not depths and not imu_cam:
        mode = 'multicamera'
    elif only_ir and not depths and n_ir_pairs == 1 and imu_cam == devices[0]:
        mode = 'inertial'
    elif len(cameras) == 1 and not depths and not imu_cam:
        mode = 'mono'
        notes.append('mono: no metric scale')
    elif len(cameras) == 1 and len(depths) == 1 and not imu_cam:
        mode = 'rgbd'
    else:
        mode = 'multisensor'
        notes.append('multisensor: cuVSLAM >= 17 with cuNLS, experimental')

    # Multisensor: pinhole cameras of one resolution (virtual cameras).
    if mode == 'multisensor':
        sizes = {}
        for c in cameras:
            s = stream_info(info, c['device'], c['key'])
            sizes[c['name']] = (s['width'], s['height'], s['distortion'])
        width = max(w or 0 for w, _, _ in sizes.values())
        height = max(h or 0 for _, h, _ in sizes.values())
        for c in cameras:
            w, h, dist = sizes[c['name']]
            if dist != 'none':
                c['undistort'] = True
            if w and w < width:
                c['pad_left'] = (width - w) // 2
                c['pad_right'] = width - w - c['pad_left']
            if h and h < height:
                c['pad_top'] = (height - h) // 2
                c['pad_bottom'] = height - h - c['pad_top']
            if c.get('pad_left') or c.get('pad_top'):
                notes.append(f"{c['name']}: {w}x{h} widened to {width}x{height}")
    elif mode in ('mono', 'rgbd'):
        s = stream_info(info, cameras[0]['device'], cameras[0]['key'])
        if s['distortion'] != 'none':
            notes.append(f"{cameras[0]['name']}: distorted camera, used as is")

    name = 'custom_' + '_'.join(
        f"{c}{''.join(k[0] for k in ('ir', 'rgb', 'depth') if sel[c].get(k))}" for c in order) + \
        (f'_imu{imu_cam[-1]}' if imu_cam else '')
    base = f'{devices[0]}_link'
    profile = {'name': name, 'description': 'built by pq_console', 'mode': mode,
               'base_frame': base,
               'cameras': [{k: v for k, v in c.items() if k != 'key'} for c in cameras]}
    if depths:
        profile['depth'] = [dict(d, scale=1000.0, stereo_tracking=True) for d in depths] \
            if mode == 'multisensor' else dict(depths[0], scale=1000.0)
    if imu_cam:
        profile['imu'] = {'topic': info['cameras'][imu_cam]['imu']['topic'],
                          'frame': f'{imu_cam}_gyro_optical_frame', 'noise': dict(IMU_NOISE)}
    if len(devices) > 1:
        ex = []
        for d in devices[1:]:
            e = extrinsics.get((devices[0], d))
            if not e:
                raise ValueError(f'no extrinsics {devices[0]} -> {d} (rig)')
            ex.append({'parent': base, 'child': f'{d}_link', 'translation': e['translation'],
                       'rotation_rpy': e['rotation_rpy'], 'source': e.get('source', 'rig'),
                       'date': None})
        profile['extrinsics'] = ex
        gyros = {d: (info['cameras'].get(d) or {}).get('imu') for d in devices}
        comp = all(gyros.values())
        if not comp:
            notes.append('no gyroscope for every camera: sync without rotation compensation')
        profile['sync'] = {'mode': 'software', 'max_offset_ms': 34.0,
                           'rotation_compensation': comp, 'compensation_border_px': 16,
                           'devices': [{'name': d, 'imu_topic': gyros[d]['topic'] if gyros[d] else '',
                                        'imu_frame': f'{d}_gyro_optical_frame',
                                        'clock_offset_ms': 0.0} for d in devices]}
    profile['hardware'] = {'ir_emitter': False}
    rectified = mode in ('multicamera', 'inertial')
    profile['parameters'] = {'rectified_images': rectified, 'enable_image_denoising': False,
                             'image_jitter_threshold_ms': 150.0,
                             'enable_localization_n_mapping': True}
    if mode == 'mono':
        profile['parameters']['rectified_images'] = False
    return profile, notes


def round_floats(obj):
    if isinstance(obj, float):
        return float(f'{obj:.6g}') if math.isfinite(obj) else obj
    if isinstance(obj, dict):
        return {k: round_floats(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [round_floats(v) for v in obj]
    return obj


def profile_yaml(profile):
    return yaml.safe_dump(round_floats(profile), sort_keys=False, default_flow_style=None)


def safe_name(name):
    return re.sub(r'[^A-Za-z0-9_.-]', '_', name)[:80]
