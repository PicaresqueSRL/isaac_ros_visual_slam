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
Regression benchmark of pq_cuvslam on recorded bags.

run      Deterministic replay (processing_mode offline) of every bag with every profile, then
         metrics that do not need a ground truth:
           - frames processed and lost, Track() time, wall time;
           - rotation error against the gyroscope of the base camera (gyro_consistency.py):
             RMS, median, p95 per frame and frames with an error above 2 deg;
           - determinism, with --repeat N (identical trajectories in N runs);
           - optionally the ground truth (compare_runs.py), as an indication only.
         A bag without the topics of a profile is skipped, with the reason.
         Output: <out>/<profile>/<bag>/ (run output, log), summary.json, summary.csv,
         report.md.
compare  Tables of two or more summaries (e.g. two cuVSLAM versions or two commits) with the
         relative change of the main metrics, and the regressions above the thresholds.

Runs in the pq_cuvslam container (ROS 2 and the workspace sourced). The cuVSLAM version is
the one of the workspace: run the benchmark once per version, then compare.

Usage:
  benchmark.py run --bags <list file or bag folders...> --profiles cam2_stereo_imu,cam1_cam2_ir
      --out <dir> [--bags-root <dir>] [--label <name>] [--repeat N] [--max-frames N]
      [--gt-root <dir> --compare-runs <compare_runs.py>] [--gt-config profile=config ...]
      [--gt-alias bag=name ...]
  benchmark.py compare <summary dir or json> <summary dir or json> [...]
      [--rms-tolerance 0.10] [--out report.md]

Bag list file: one bag folder per line (absolute, or relative to --bags-root); empty lines
and lines starting with # are ignored.
"""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def _load_tool(name):
    """Import a script of this folder as a module."""
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gyro_consistency = _load_tool('gyro_consistency')

METRICS = [
    # key, label, unit, higher_is_worse
    ('frames', 'frames', '', False),
    ('frames_lost', 'lost', '', True),
    ('gyro_rms_deg', 'gyro RMS', 'deg', True),
    ('gyro_median_deg', 'gyro med', 'deg', True),
    ('gyro_p95_deg', 'gyro p95', 'deg', True),
    ('gyro_glitches', '>2deg', '', True),
    ('track_ms_median', 'track ms', 'ms', True),
    ('gt_rmse_m', 'GT RMSE', 'm', True),
]


# ---------------------------------------------------------------------------------------------
# Inputs

def read_bag_list(entries, bags_root):
    bags = []
    for entry in entries:
        if os.path.isfile(entry) and not os.path.isfile(os.path.join(entry, 'metadata.yaml')):
            with open(entry) as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        bags.append(line)
        else:
            bags.append(entry)
    out = []
    for b in bags:
        path = b if os.path.isabs(b) or not bags_root else os.path.join(bags_root, b)
        out.append(os.path.normpath(path))
    return out


def resolve_profile(profile):
    if os.path.isfile(profile):
        return profile
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('pq_cuvslam'), 'profiles', profile + '.yaml')


def profile_info(profile):
    """Topics needed by a profile and the gyroscope used for the rotation check."""
    with open(resolve_profile(profile)) as f:
        p = yaml.safe_load(f)
    topics = set()
    for c in p.get('cameras', []):
        topics.update([c['image_topic'], c['camera_info_topic']])
    if p.get('depth'):
        topics.add(p['depth']['topic'])
    if p.get('imu'):
        topics.add(p['imu']['topic'])
    sync = p.get('sync') or {}
    if sync.get('mode', 'none') != 'none' and sync.get('rotation_compensation'):
        topics.update(d['imu_topic'] for d in sync.get('devices', []) if d.get('imu_topic'))
    base = p['base_frame']
    # Gyroscope of the base camera: the profile IMU, the reference sync device, or the D455
    # naming convention (<cam>_link -> /<cam>/imu, <cam>_gyro_optical_frame).
    if p.get('imu'):
        imu_topic, imu_frame = p['imu']['topic'], p['imu']['frame']
    elif sync.get('devices') and sync['devices'][0].get('imu_topic'):
        imu_topic, imu_frame = sync['devices'][0]['imu_topic'], sync['devices'][0]['imu_frame']
    else:
        cam = base.split('_')[0]
        imu_topic, imu_frame = f'/{cam}/imu', f'{cam}_gyro_optical_frame'
    return {'topics': sorted(topics), 'base_frame': base, 'imu_topic': imu_topic,
            'imu_frame': imu_frame, 'name': p.get('name', profile)}


def bag_topics(bag):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        meta = yaml.safe_load(f)
    info = meta.get('rosbag2_bagfile_information', meta)
    return {t['topic_metadata']['name'] for t in info.get('topics_with_message_count', [])}


# ---------------------------------------------------------------------------------------------
# Run

def file_hash(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def replay(bag, profile, out_dir, max_frames, log_path):
    os.makedirs(out_dir, exist_ok=True)
    cmd = ['ros2', 'run', 'pq_cuvslam', 'pq_cuvslam_replay', '--bag', bag,
           '--max-frames', str(max_frames), '--ros-args', '-r', '__node:=visual_slam_node',
           '-p', f'profile:={profile}', '-p', 'processing_mode:=offline',
           '-p', f'run_output_dir:={out_dir}']
    start = time.time()
    with open(log_path, 'w') as log:
        rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT).returncode
    return rc, time.time() - start


def gyro_metrics(bag, run_dir, info):
    parents, gyro = gyro_consistency.read_bag(bag, info['imu_topic'])
    if len(gyro) == 0:
        return {}
    R = (gyro_consistency.root_rotation(parents, info['base_frame']).T @
         gyro_consistency.root_rotation(parents, info['imu_frame']))
    r = gyro_consistency.evaluate(gyro_consistency.read_poses(run_dir), gyro, R)
    if not r['pairs']:
        return {}
    return {'gyro_rms_deg': r['error_rms_deg'], 'gyro_median_deg': r['error_median_deg'],
            'gyro_p95_deg': r['error_p95_deg'], 'gyro_glitches': r['glitches_2deg'],
            'gyro_pairs': r['pairs']}


def gt_metrics(args, profile, bag_name, run_dir, work_dir):
    config = dict(kv.split('=', 1) for kv in args.gt_config).get(profile, profile)
    name = dict(kv.split('=', 1) for kv in args.gt_alias).get(bag_name, bag_name)
    gt = os.path.join(args.gt_root, config, name, 'paths3d.csv')
    if not os.path.isfile(gt):
        return {}
    out = os.path.join(work_dir, 'gt_compare')
    subprocess.run([sys.executable, '-I', args.compare_runs, f'run={run_dir}', '--gt', gt,
                    '--out', out, '--no-plots'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        with open(os.path.join(out, 'metrics.json')) as f:
            m = json.load(f)
        rigid = m['ground_truth']['odometry']['run']['rigid']
        return {'gt_rmse_m': rigid['rmse_m']}
    except (OSError, KeyError, ValueError):
        return {}


def run_one(args, bag, profile, info, available):
    bag_name = os.path.basename(bag.rstrip('/'))
    work = os.path.join(args.out, profile, bag_name)
    row = {'profile': profile, 'bag': bag_name, 'bag_path': bag}
    missing = [t for t in info['topics'] if t not in available]
    if missing:
        row.update(status='skipped', reason='missing topics: ' + ', '.join(missing))
        return row

    hashes = []
    for i in range(max(1, args.repeat)):
        run_dir = os.path.join(work, 'run' if i == 0 else f'run_repeat{i}')
        rc, wall = replay(bag, profile, run_dir, args.max_frames,
                          os.path.join(work, f'replay{i}.log'))
        if rc != 0 or not os.path.isfile(os.path.join(run_dir, 'run_info.json')):
            row.update(status='failed', reason=f'replay exit code {rc}, see replay{i}.log')
            return row
        hashes.append(tuple(file_hash(os.path.join(run_dir, f))
                            for f in ('odometry.csv', 'slam_poses.csv')
                            if os.path.isfile(os.path.join(run_dir, f))))
        if i == 0:
            row['replay_wall_s'] = round(wall, 2)

    run_dir = os.path.join(work, 'run')
    with open(os.path.join(run_dir, 'run_info.json')) as f:
        run_info = json.load(f)
    s = run_info.get('summary', {})
    row.update(
        status='ok',
        cuvslam=run_info.get('cuvslam_version'),
        wrapper=run_info.get('wrapper_commit'),
        frames=s.get('frames'), frames_lost=s.get('frames_lost'),
        track_ms_median=s.get('track_time_ms_median'),
        path_m=(s.get('odometry') or {}).get('path_length_m'),
        sync_offset_ms=((run_info.get('sync') or {}).get('offset_ms') or {}).get('median'),
    )
    if args.repeat > 1:
        row['deterministic'] = len(set(hashes)) == 1
    row.update(gyro_metrics(bag, run_dir, info))
    if args.gt_root and args.compare_runs:
        row.update(gt_metrics(args, profile, bag_name, run_dir, work))
    return row


def fmt(v, nd=3):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '-'
    if isinstance(v, bool):
        return 'yes' if v else 'no'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def write_summary(args, rows, meta):
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump({'meta': meta, 'runs': rows}, f, indent=2)
    keys = ['profile', 'bag', 'status', 'reason', 'frames', 'frames_lost', 'gyro_rms_deg',
            'gyro_median_deg', 'gyro_p95_deg', 'gyro_glitches', 'track_ms_median',
            'replay_wall_s', 'path_m', 'sync_offset_ms', 'deterministic', 'gt_rmse_m',
            'cuvslam', 'wrapper']
    with open(os.path.join(args.out, 'summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)

    lines = [f"# Benchmark {meta['label']}", '',
             f"cuVSLAM {meta.get('cuvslam', '-')}, wrapper {meta.get('wrapper', '-')}, "
             f"{meta['date']}, {len(rows)} runs.", '',
             'Rotation error against the gyroscope of the base camera, per frame; the ground '
             'truth (GT) is an indication only.', '',
             '| profile | bag | status | frames | lost | gyro RMS deg | gyro p95 deg | >2deg | '
             'track ms | determ. | GT RMSE m |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        if r['status'] != 'ok':
            lines.append(f"| {r['profile']} | {r['bag']} | {r['status']}: {r.get('reason', '')} "
                         '| | | | | | | | |')
            continue
        lines.append(
            f"| {r['profile']} | {r['bag']} | ok | {fmt(r.get('frames'))} | "
            f"{fmt(r.get('frames_lost'))} | {fmt(r.get('gyro_rms_deg'))} | "
            f"{fmt(r.get('gyro_p95_deg'))} | {fmt(r.get('gyro_glitches'))} | "
            f"{fmt(r.get('track_ms_median'), 2)} | {fmt(r.get('deterministic'))} | "
            f"{fmt(r.get('gt_rmse_m'))} |")
    with open(os.path.join(args.out, 'report.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    return lines


def cmd_run(args):
    bags = read_bag_list(args.bags, args.bags_root)
    profiles = [p for p in args.profiles.split(',') if p]
    infos = {p: profile_info(p) for p in profiles}
    rows = []
    for bag in bags:
        try:
            available = bag_topics(bag)
        except OSError as e:
            for p in profiles:
                rows.append({'profile': p, 'bag': os.path.basename(bag), 'bag_path': bag,
                             'status': 'failed', 'reason': f'cannot read the bag: {e}'})
            continue
        for p in profiles:
            row = run_one(args, bag, p, infos[p], available)
            rows.append(row)
            print(f"{p:18s} {row['bag']:24s} {row['status']:8s} "
                  f"gyro RMS {fmt(row.get('gyro_rms_deg'))} lost {fmt(row.get('frames_lost'))} "
                  f"{row.get('reason', '')}", flush=True)
    ok = [r for r in rows if r['status'] == 'ok']
    meta = {'label': args.label or os.path.basename(os.path.abspath(args.out)),
            'date': time.strftime('%Y-%m-%d %H:%M'),
            'cuvslam': ok[0]['cuvslam'] if ok else None,
            'wrapper': ok[0]['wrapper'] if ok else None,
            'profiles': profiles, 'repeat': args.repeat, 'max_frames': args.max_frames}
    write_summary(args, rows, meta)
    failed = [r for r in rows if r['status'] == 'failed']
    print(f'\n{len(ok)} ok, {len(rows) - len(ok) - len(failed)} skipped, {len(failed)} failed. '
          f"Report: {os.path.join(args.out, 'report.md')}")
    if args.repeat > 1 and any(r.get('deterministic') is False for r in ok):
        print('WARNING: some runs are not deterministic')
    return 1 if failed else 0


# ---------------------------------------------------------------------------------------------
# Compare

def load_summary(path):
    if os.path.isdir(path):
        path = os.path.join(path, 'summary.json')
    with open(path) as f:
        s = json.load(f)
    return s['meta'].get('label') or os.path.basename(os.path.dirname(path)), s


def cmd_compare(args):
    summaries = [load_summary(p) for p in args.summaries]
    base_label, base = summaries[0]
    index = [{(r['profile'], r['bag']): r for r in s['runs']} for _, s in summaries]
    keys = sorted(set().union(*index), key=lambda k: (k[0], k[1]))

    lines = ['# Benchmark comparison', '',
             'Baseline: ' + base_label + '. ' +
             ', '.join(f"{label}: cuVSLAM {s['meta'].get('cuvslam', '-')}, wrapper "
                       f"{s['meta'].get('wrapper', '-')}" for label, s in summaries), '']
    regressions = []
    for key, label, unit, worse_up in METRICS:
        lines += [f'## {label}' + (f' ({unit})' if unit else ''), '',
                  '| profile | bag | ' + ' | '.join(lab for lab, _ in summaries) + ' |',
                  '|---|---|' + '---|' * len(summaries)]
        for k in keys:
            cells = []
            ref = index[0].get(k, {})
            for i, (lab, _) in enumerate(summaries):
                r = index[i].get(k)
                if r is None or r.get('status') != 'ok':
                    cells.append(r['status'] if r else '-')
                    continue
                v = r.get(key)
                cell = fmt(v)
                rv = ref.get(key) if ref.get('status') == 'ok' else None
                if i > 0 and isinstance(v, (int, float)) and isinstance(rv, (int, float)) and rv:
                    change = (v - rv) / abs(rv)
                    cell += f' ({change:+.0%})'
                    if key == 'gyro_rms_deg' and change > args.rms_tolerance:
                        regressions.append(f'{k[0]} {k[1]}: gyro RMS {rv:.3f} -> {v:.3f} deg '
                                           f'({change:+.0%}) in {lab}')
                if i > 0 and key == 'frames_lost' and isinstance(v, int) and \
                        isinstance(rv, int) and v > rv:
                    regressions.append(f'{k[0]} {k[1]}: frames lost {rv} -> {v} in {lab}')
                cells.append(cell)
            lines.append(f'| {k[0]} | {k[1]} | ' + ' | '.join(cells) + ' |')
        lines.append('')
    for i, (lab, _) in enumerate(summaries[1:], start=1):
        for k in keys:
            a, b = index[0].get(k), index[i].get(k)
            if a and b and a.get('status') == 'ok' and b.get('status') != 'ok':
                regressions.append(f"{k[0]} {k[1]}: {b.get('status')} in {lab} "
                                   f"({b.get('reason', '')})")
    lines += ['## Regressions', '',
              f'Gyro RMS above +{args.rms_tolerance:.0%}, more frames lost, or runs that '
              'failed only in a later summary.', '']
    lines += [f'- {r}' for r in regressions] or ['- none']
    text = '\n'.join(lines) + '\n'
    if args.out:
        with open(args.out, 'w') as f:
            f.write(text)
    print(text)
    return 1 if regressions else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='command', required=True)

    r = sub.add_parser('run', help='replay the bags and compute the metrics')
    r.add_argument('--bags', nargs='+', required=True,
                   help='bag list files and/or bag folders')
    r.add_argument('--bags-root', default='', help='prefix of relative bag paths')
    r.add_argument('--profiles', required=True, help='comma-separated profile names or paths')
    r.add_argument('--out', required=True)
    r.add_argument('--label', default='')
    r.add_argument('--repeat', type=int, default=1, help='runs per bag, to check determinism')
    r.add_argument('--max-frames', type=int, default=-1)
    r.add_argument('--gt-root', default='', help='ground truth: <root>/<config>/<bag>/paths3d.csv')
    r.add_argument('--compare-runs', default='', help='compare_runs.py, for the ground truth')
    r.add_argument('--gt-config', nargs='*', default=['cam1_cam2_ir=cam1_cam2_stereo'],
                   help='profile=config folder of the ground truth')
    r.add_argument('--gt-alias', nargs='*', default=['staircase_fix_1=staircases_fix_1'],
                   help='bag=name of the ground truth folder')

    c = sub.add_parser('compare', help='compare two or more summaries')
    c.add_argument('summaries', nargs='+', help='summary folders or summary.json files')
    c.add_argument('--rms-tolerance', type=float, default=0.10,
                   help='relative increase of the gyro RMS reported as regression')
    c.add_argument('--out', default='')

    args = ap.parse_args()
    if args.command == 'run':
        return cmd_run(args)
    if len(args.summaries) < 2:
        ap.error('compare needs at least two summaries')
    return cmd_compare(args)


if __name__ == '__main__':
    sys.exit(main())
