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
Image quality of the IR streams of a bag, on a sample of frames: brightness, saturated and
dark pixels, and density of the D455 projector dots (IR emitter on).

Dots: small isolated spots brighter than their surroundings, found with a white top-hat
(3 px spots, 9 px background) and counted per 10 000 pixels. With the emitter off the
density is close to zero; with the emitter on, surfaces within a few meters are covered.

Runs in the pq_cuvslam container (rosbag2_py, numpy, cv2).

Usage:
  analyze_images.py <bag> [<bag> ...] [--every 10] [--json out.json]
"""

import argparse
import json
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

STREAMS = [f'/{cam}/{ir}/image_rect_raw' for cam in ('cam1', 'cam2') for ir in ('infra1', 'infra2')]
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))


def image_stats(img):
    tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, KERNEL)
    spots = (tophat > 25).astype(np.uint8)
    n, _, stats, _ = cv2.connectedComponentsWithStats(spots, connectivity=8)
    small = int(np.sum((stats[1:, cv2.CC_STAT_AREA] >= 2) & (stats[1:, cv2.CC_STAT_AREA] <= 12)))
    return {
        'mean': float(img.mean()),
        'saturated_pct': float(np.mean(img >= 250) * 100),
        'dark_pct': float(np.mean(img <= 5) * 100),
        'dots_per_10k_px': small / img.size * 1e4,
    }


def analyze(path, every):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    available = {t.name for t in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in STREAMS if t in available]))
    counts = {t: 0 for t in STREAMS}
    samples = {t: [] for t in STREAMS}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] += 1
        if counts[topic] % every:
            continue
        m = deserialize_message(data, Image)
        if m.encoding not in ('mono8', '8UC1'):
            continue
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width]
        samples[topic].append(image_stats(img))
    out = {'bag': path, 'streams': {}}
    for topic, rows in samples.items():
        if not rows:
            continue
        keys = rows[0].keys()
        out['streams'][topic] = {
            'frames_sampled': len(rows),
            **{k: float(np.median([r[k] for r in rows])) for k in keys},
            # share of sampled frames with more than 20% saturated pixels
            'frames_saturated_20pct': float(np.mean([r['saturated_pct'] > 20 for r in rows]) * 100),
        }
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--every', type=int, default=10, help='analyze one frame every N')
    ap.add_argument('--json')
    args = ap.parse_args()
    results = []
    print(f"{'bag':28s} {'stream':12s} {'mean':>5s} {'sat%':>6s} {'>20%sat':>7s} "
          f"{'dark%':>6s} {'dots':>6s}")
    for bag in args.bags:
        try:
            r = analyze(bag, args.every)
        except Exception as e:  # noqa: BLE001 (report and continue)
            print(f'{bag}: ERROR {e}', file=sys.stderr)
            continue
        results.append(r)
        name = bag.rstrip('/').split('/')[-1]
        for topic, s in r['streams'].items():
            stream = topic.split('/')[1] + '/' + topic.split('/')[2].replace('infra', 'ir')
            print(f"{name:28s} {stream:12s} {s['mean']:5.0f} {s['saturated_pct']:6.1f} "
                  f"{s['frames_saturated_20pct']:6.0f}% {s['dark_pct']:6.1f} "
                  f"{s['dots_per_10k_px']:6.1f}")
    if args.json:
        with open(args.json, 'w') as f:
            json.dump(results, f, indent=2)


if __name__ == '__main__':
    main()
