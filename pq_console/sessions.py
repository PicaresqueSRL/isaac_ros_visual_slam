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
Cameras, recordings and SLAM replays, run as Docker containers.

The containers are the ones of the live scripts: pq_cams (RealSense driver of both D455),
pq_rec (raw data recorder), pq_slam (pq_cuvslam), pq_slamrec (SLAM output recorder) and
pq_play (bag player). They keep running if the console stops. Host paths are used in the
volume arguments: the console container mounts the data folder at the same path.
"""

import csv
import glob
import json
import os
import re
import shlex
import shutil
import subprocess
import threading
import time

import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,80}$')
PROFILES_DIR = '/ws/install/pq_cuvslam/share/pq_cuvslam/profiles'
SLAM_TOPICS_RE = r'^(/tf|(/cam[12]_vio)?/visual_slam/(tracking|vis|diagnostics|events|selector|status).*)$'


def docker(*args, timeout=60):
    p = subprocess.run(['docker', *args], capture_output=True, text=True, timeout=timeout)
    return p.returncode, (p.stdout + p.stderr).strip()


_ps_cache = {'time': 0.0, 'names': set()}
_ps_lock = threading.Lock()


def running(name, max_age=1.0):
    """Container running; docker ps is called at most once per max_age seconds."""
    with _ps_lock:
        if time.time() - _ps_cache['time'] > max_age:
            code, out = docker('ps', '--format', '{{.Names}}')
            _ps_cache['names'] = set(out.split()) if code == 0 else set()
            _ps_cache['time'] = time.time()
        return name in _ps_cache['names']


def dir_size(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


class Job:
    """A background task (replay, recording stop) with a log shown in the console."""

    def __init__(self, kind, name):
        self.kind, self.name = kind, name
        self.state = 'running'
        self.started = time.time()
        self.finished = None
        self.log = []
        self.meta = {}
        self.stop_event = threading.Event()

    def info(self, text):
        self.log.append(f'{time.strftime("%H:%M:%S")} {text}')
        del self.log[:-200]

    def to_dict(self):
        return {'kind': self.kind, 'name': self.name, 'state': self.state, 'meta': self.meta,
                'started': self.started, 'finished': self.finished, 'log': self.log[-40:]}


class Sessions:

    def __init__(self, config):
        self.c = config
        self.lock = threading.Lock()
        self.job = None
        self.recording = None   # {'name', 'started', 'depth'}
        for d in (self.c.bags_dir, self.c.runs_dir, self.c.rerun_dir):
            os.makedirs(d, exist_ok=True)
        if running('pq_rec'):
            self.recording = {'name': '(started outside the console)', 'started': None,
                              'depth': None}

    def slam_args(self):
        """Arguments of the containers of the SLAM image (local build, extra arguments)."""
        install = ['-v', f'{self.c.install_dir}:/ws/install:ro'] if self.c.install_dir else []
        return install + list(self.c.slam_extra_args)

    # Common container arguments: host network and IPC (DDS shared memory between containers).
    def ros_args(self):
        return ['--network', 'host', '--ipc', 'host',
                '-e', f'ROS_DOMAIN_ID={self.c.ros_domain_id}',
                '-e', f'FASTDDS_BUILTIN_TRANSPORTS={self.c.dds_transports}']

    def chown(self, *paths):
        if not self.c.host_uid:
            return
        args = []
        for p in paths:
            args += ['-v', f'{p}:{p}']
        docker('run', '--rm', *args, '--entrypoint', 'chown', self.c.slam_image, '-R',
               f'{self.c.host_uid}:{self.c.host_gid}', *paths, timeout=300)

    # ---- cameras ----

    def cameras_running(self):
        return running('pq_cams')

    def start_cameras(self):
        if not self.c.cameras or not self.c.camera_image:
            raise ValueError('no cameras in the configuration')
        params = ' '.join(f'{k}:={shlex.quote(str(v).lower() if isinstance(v, bool) else str(v))}'
                          for k, v in self.c.camera_params.items())
        launches = []
        for i, cam in enumerate(self.c.cameras):
            launches.append(
                f'ros2 launch realsense2_camera rs_launch.py camera_name:={cam.name} '
                f'camera_namespace:=/ serial_no:=_{cam.serial} {params} > /tmp/{cam.name}.log 2>&1 &')
            if i + 1 < len(self.c.cameras):
                launches.append('sleep 4')
        # The emitter argument of the launch is not applied by the driver: set it at runtime.
        off = ' && '.join(f'ros2 param set /{cam.name} depth_module.emitter_enabled 0 > /dev/null 2>&1'
                          for cam in self.c.cameras)
        script = ('source /opt/ros/lyrical/setup.bash\n' + '\n'.join(launches) +
                  f'\nfor i in $(seq 30); do sleep 2; {off} && echo emitters off > /tmp/emitter.log '
                  '&& break; done\nwait')
        docker('rm', '-f', 'pq_cams')
        code, out = docker('run', '-d', '--name', 'pq_cams', *self.c.gpu_args, '--privileged',
                           '-v', '/dev:/dev', *self.ros_args(), '--entrypoint', 'bash',
                           self.c.camera_image, '-c', script)
        if code:
            raise RuntimeError(out)

    def stop_cameras(self):
        if self.recording:
            raise ValueError('a recording is running: stop it first')
        docker('rm', '-f', 'pq_cams')

    # ---- recording ----

    def record_topics(self, depth, aligned):
        topics = []
        for cam in self.c.cameras or []:
            n = cam.name
            for s in ('infra1', 'infra2', 'color'):
                topics += [f'/{n}/{s}/image_rect_raw', f'/{n}/{s}/image_raw',
                           f'/{n}/{s}/camera_info', f'/{n}/{s}/metadata']
            topics += [f'/{n}/imu', f'/{n}/gyro/metadata', f'/{n}/accel/metadata']
            if depth:
                topics += [f'/{n}/depth/image_rect_raw', f'/{n}/depth/camera_info',
                           f'/{n}/depth/metadata']
            if aligned:
                topics += [f'/{n}/aligned_depth_to_color/image_raw',
                           f'/{n}/aligned_depth_to_color/camera_info']
        return topics + ['/tf_static']

    def start_recording(self, name, depth=True, aligned=False):
        with self.lock:
            if self.recording:
                raise ValueError('a recording is already running')
            if not NAME_RE.match(name or ''):
                raise ValueError('name: letters, digits, _ . - only')
            if os.path.exists(os.path.join(self.c.bags_dir, name)):
                raise ValueError(f'a recording named {name} exists')
            if self.job and self.job.state == 'running' and self.job.kind == 'replay':
                raise ValueError('a replay is running (same topics)')
            topics = ' '.join(self.record_topics(depth, aligned))
            docker('rm', '-f', 'pq_rec')
            code, out = docker(
                'run', '-d', '--name', 'pq_rec', *self.ros_args(), '-v', f'{self.c.bags_dir}:/out',
                '--entrypoint', 'bash', self.c.camera_image, '-c',
                'source /opt/ros/lyrical/setup.bash; exec ros2 bag record -s mcap '
                '--storage-preset-profile zstd_fast --include-unpublished-topics '
                f'-o /out/{name} --topics {topics} > /out/{name}.log 2>&1')
            if code:
                raise RuntimeError(out)
            self.recording = {'name': name, 'started': time.time(), 'depth': depth}

    def recording_status(self):
        if not self.recording:
            return None
        r = dict(self.recording)
        r['running'] = running('pq_rec')
        path = os.path.join(self.c.bags_dir, r['name'])
        r['size'] = dir_size(path) if os.path.isdir(path) else 0
        return r

    def stop_recording(self):
        with self.lock:
            if not self.recording:
                raise ValueError('no recording running')
            name = self.recording['name']
            docker('exec', 'pq_rec', 'bash', '-c', 'pkill -INT -f "bag record"')
            for _ in range(90):
                if not running('pq_rec', 0):
                    break
                time.sleep(1)
            docker('rm', '-f', 'pq_rec')
            self.recording = None
        self.chown(self.c.bags_dir)
        return name

    # ---- SLAM replay ----

    def start_replay(self, bag, profile, rate=1.0, rerun=True):
        with self.lock:
            if self.job and self.job.state == 'running':
                raise ValueError(f'{self.job.kind} {self.job.name} is running')
            if self.recording or self.cameras_running():
                raise ValueError('stop the cameras first: the bag uses the same topics')
            if not os.path.isfile(os.path.join(self.c.bags_dir, bag, 'metadata.yaml')):
                raise ValueError(f'no recording {bag}')
            if profile not in self.profile_names():
                raise ValueError(f'no profile {profile}')
            job = Job('replay', f'{bag}_{profile}')
            job.meta = self.bag_info(bag)
            job.meta.update(rate=float(rate), profile=profile)
            self.job = job
        threading.Thread(target=self._replay, args=(job, bag, profile, float(rate), rerun),
                         daemon=True).start()
        return job.name

    def bag_info(self, bag):
        """Start (s, receive time) and duration of a recording, from its metadata."""
        with open(os.path.join(self.c.bags_dir, bag, 'metadata.yaml')) as f:
            info = yaml.safe_load(f)['rosbag2_bagfile_information']
        return {'bag': bag,
                'start': info['starting_time']['nanoseconds_since_epoch'] / 1e9,
                'duration_s': info['duration']['nanoseconds'] / 1e9}

    def play_inputs(self, bag):
        inputs = f'-i /bags/{bag}'
        if os.path.isfile(os.path.join(self.c.bags_dir, f'{bag}_aligned', 'metadata.yaml')):
            inputs += f' -i /bags/{bag}_aligned'
        return inputs

    def play_args(self, inputs, rate, extra=''):
        """Player container (image of the cameras: it knows the RealSense metadata messages)."""
        return ['run', '--rm', '--name', 'pq_play', *self.ros_args(), '-v', f'{self.c.bags_dir}:/bags:ro',
                '--entrypoint', 'bash', self.c.camera_image or self.c.slam_image, '-c',
                f'source /opt/ros/lyrical/setup.bash; ros2 bag play {inputs} --rate {rate} {extra} '
                '> /dev/null 2>&1']

    def start_view(self, bag, rate=1.0, paused=False):
        """Play a recording without SLAM, to look at it (pause, rate and seek via the player)."""
        with self.lock:
            if self.job and self.job.state == 'running':
                raise ValueError(f'{self.job.kind} {self.job.name} is running')
            if self.recording or self.cameras_running():
                raise ValueError('stop the cameras first: the bag uses the same topics')
            if not os.path.isfile(os.path.join(self.c.bags_dir, bag, 'metadata.yaml')):
                raise ValueError(f'no recording {bag}')
            job = Job('view', bag)
            job.meta = self.bag_info(bag)
            job.meta.update(rate=float(rate), paused=bool(paused))
            self.job = job
        threading.Thread(target=self._view, args=(job, bag, float(rate), paused), daemon=True).start()
        return job.name

    def _view(self, job, bag, rate, paused):
        try:
            docker('rm', '-f', 'pq_play')
            job.info(f'play {bag} at rate {rate}')
            # /clock at 10 Hz: position of the playback for the console.
            extra = '--clock 10' + (' --start-paused' if paused else '')
            docker(*self.play_args(self.play_inputs(bag), rate, extra), timeout=24 * 3600)
            job.state = 'stopped' if job.stop_event.is_set() else 'done'
        except Exception as e:   # noqa: B902 - shown in the console
            job.info(f'error: {e}')
            job.state = 'failed'
        job.finished = time.time()
        job.info(job.state)

    def stop_job(self):
        if self.job and self.job.state == 'running':
            self.job.stop_event.set()
            docker('rm', '-f', 'pq_play')

    def _replay(self, job, bag, profile, rate, rerun):
        c, name = self.c, job.name
        try:
            for n in ('pq_slam', 'pq_slamrec', 'pq_play'):
                docker('rm', '-f', n)
            if profile == 'dual':
                launch = (f'dual_vio.launch.py cam1_profile:={c.dual_cam1_profile} '
                          f'run_output_dir:=/out/{name} enable_vis:=true')
            else:
                launch = f'slam.launch.py profile:={profile} run_output_dir:=/out/{name} enable_vis:=true'
            shutil.rmtree(os.path.join(c.runs_dir, name), ignore_errors=True)
            job.info(f'SLAM {profile}')
            code, out = docker(
                'run', '-d', '--name', 'pq_slam', *c.gpu_args, *self.ros_args(), *self.slam_args(),
                '-v', f'{c.runs_dir}:/out', '--entrypoint', 'bash', c.slam_image, '-c',
                'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; '
                f'exec ros2 launch pq_cuvslam {launch} > /out/{name}.log 2>&1')
            if code:
                raise RuntimeError(out)
            slam_bag = os.path.join(c.bags_dir, f'{name}_slam')
            self.chown_rm(slam_bag)
            code, out = docker(
                'run', '-d', '--name', 'pq_slamrec', *self.ros_args(), *self.slam_args(),
                '-v', f'{c.bags_dir}:/out',
                '--entrypoint', 'bash', c.slam_image, '-c',
                'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; '
                'exec ros2 bag record -s mcap --storage-preset-profile zstd_fast '
                f"--include-unpublished-topics -o /out/{name}_slam --regex '{SLAM_TOPICS_RE}' "
                f'> /out/{name}_slam.log 2>&1')
            if code:
                raise RuntimeError(out)
            time.sleep(8)   # nodes active and recorder subscribed before the first frame
            job.info(f'play {bag} at rate {rate}')
            if not job.stop_event.is_set():
                docker(*self.play_args(self.play_inputs(bag), rate), timeout=24 * 3600)
            job.info('play finished, stopping the SLAM')
            time.sleep(3)
            docker('exec', 'pq_slamrec', 'bash', '-c', 'pkill -INT -f "bag record"')
            docker('kill', '-s', 'INT', 'pq_slam')
            for _ in range(90):
                if not running('pq_slam', 0) and not running('pq_slamrec', 0):
                    break
                time.sleep(1)
            for n in ('pq_slam', 'pq_slamrec'):
                docker('rm', '-f', n)
            run = (self.run_summary(os.path.join(c.runs_dir, name)) or
                   self.dual_summary(os.path.join(c.runs_dir, name)))
            if run:
                keys = ('frames', 'path_m', 'start_end_m')
                job.info('run: ' + ', '.join(f'{k} {run[k]}' for k in keys if k in run))
            if rerun and not job.stop_event.is_set() and c.tools_dir:
                job.info('Rerun conversion')
                self._rerun(name, bag)
            self.chown(c.runs_dir, c.bags_dir, c.rerun_dir)
            job.state = 'stopped' if job.stop_event.is_set() else 'done'
        except Exception as e:   # noqa: B902 - shown in the console
            job.info(f'error: {e}')
            job.state = 'failed'
        job.finished = time.time()
        job.info(job.state)

    def chown_rm(self, path):
        if os.path.exists(path):
            docker('run', '--rm', '-v', f'{os.path.dirname(path)}:/d', '--entrypoint', 'rm',
                   self.c.slam_image, '-rf', f'/d/{os.path.basename(path)}')

    def _rerun(self, name, bag):
        c = self.c
        layout = ''
        if c.pylib_dir:
            layout = (f'python3 /tools/rerun_layout.py /live/rerun/{name}.mcap '
                      f'/live/rerun/{name}.rrd --pylib /pylib >> /live/rerun/{name}.log 2>&1')
        images = ','.join(f'/{cam}/{s}' for cam in ('cam2', 'cam1')
                          for s in ('infra1/image_rect_raw', 'color/image_raw'))
        pylib = ['-v', f'{c.pylib_dir}:/pylib:ro'] if c.pylib_dir else []
        docker('run', '--rm', '--network', 'none', '-v', f'{c.data_dir}:/live',
               '-v', f'{c.bags_dir}:/live/bags', '-v', f'{c.tools_dir}:/tools:ro', *pylib,
               '--entrypoint', 'bash', c.slam_image, '-c',
               'source /opt/ros/lyrical/setup.bash; mkdir -p /live/rerun; rm -rf /tmp/o; '
               f'python3 /tools/to_rerun.py /live/bags/{name}_slam /live/bags/{bag} /tmp/o '
               f'--max-image-rate 15 --images {images} > /live/rerun/{name}.log 2>&1 '
               f'&& cp /tmp/o/*.mcap /live/rerun/{name}.mcap; {layout}', timeout=3 * 3600)

    # ---- lists ----

    def profiles(self, max_age=60.0):
        """Profiles installed in the SLAM image (as the SLAM container sees them), plus dual."""
        cache = getattr(self, '_profiles', None)
        if cache and time.time() - cache[0] < max_age:
            return cache[1]
        script = ('import glob, json, os, yaml\n'
                  f'out = []\nfor p in sorted(glob.glob("{PROFILES_DIR}/*.yaml")):\n'
                  '    try:\n        y = yaml.safe_load(open(p))\n    except Exception:\n        y = {}\n'
                  '    out.append({"name": os.path.splitext(os.path.basename(p))[0], '
                  '"mode": y.get("mode", ""), "description": y.get("description", "")})\n'
                  'print(json.dumps(out))')
        code, out = docker('run', '--rm', '--network', 'none', *self.slam_args(),
                           '--entrypoint', 'python3', self.c.slam_image, '-c', script)
        found = []
        if code == 0:
            try:
                found = json.loads(out.splitlines()[-1])
            except (ValueError, IndexError):
                pass
        result = [{'name': 'dual', 'mode': 'dual VIO',
                   'description': 'cam1 and cam2 VIO in parallel with the odometry selector'}] + found
        self._profiles = (time.time(), result)
        return result

    def profile_names(self):
        return [p['name'] for p in self.profiles()]

    def recordings(self):
        out = []
        for meta in sorted(glob.glob(os.path.join(self.c.bags_dir, '*', 'metadata.yaml'))):
            path = os.path.dirname(meta)
            name = os.path.basename(path)
            if name.endswith(('_slam', '_aligned')):
                continue
            try:
                with open(meta) as f:
                    info = yaml.safe_load(f)['rosbag2_bagfile_information']
            except (OSError, yaml.YAMLError, KeyError, TypeError):
                continue
            topics = [t['topic_metadata']['name'] for t in info.get('topics_with_message_count', [])]
            start = info.get('starting_time', {}).get('nanoseconds_since_epoch', 0)
            out.append({
                'name': name,
                'start': start / 1e9,
                'duration_s': info.get('duration', {}).get('nanoseconds', 0) / 1e9,
                'size': dir_size(path),
                'cameras': sorted({t.split('/')[1] for t in topics if t.count('/') > 1}),
                'depth': any(t.endswith('/depth/image_rect_raw') for t in topics),
                'aligned': any('aligned_depth_to_color/image_raw' in t for t in topics) or
                os.path.isfile(os.path.join(self.c.bags_dir, f'{name}_aligned', 'metadata.yaml')),
            })
        return sorted(out, key=lambda r: r['start'], reverse=True)

    def run_summary(self, path):
        try:
            with open(os.path.join(path, 'run_info.json')) as f:
                info = json.load(f)
        except (OSError, ValueError):
            return None
        s = info.get('summary', {})
        profile = info.get('profile') or {}
        r = {'name': os.path.basename(path), 'time': os.path.getmtime(path),
             'profile': profile.get('name', '') if isinstance(profile, dict) else str(profile),
             'mode': info.get('mode', ''), 'processing': info.get('processing_mode', ''),
             'frames': s.get('frames'), 'health': s.get('health', {}),
             'events': s.get('events', {}), 'track_ms': s.get('track_time_ms_median')}
        for f in ('odometry_guarded.csv', 'odometry.csv'):
            p = os.path.join(path, f)
            if os.path.isfile(p):
                r.update(self.path_stats(p))
                r['trajectory'] = f
                break
        rrd = os.path.join(self.c.rerun_dir, r['name'] + '.rrd')
        r['rrd'] = rrd if os.path.isfile(rrd) else ''
        return r

    @staticmethod
    def path_stats(path):
        pts = []
        with open(path) as f:
            for row in csv.DictReader(f):
                try:
                    pts.append((float(row['x']), float(row['y']), float(row['z'])))
                except (KeyError, ValueError):
                    pass
        if len(pts) < 2:
            return {}
        length = sum(sum((a - b) ** 2 for a, b in zip(p, q)) ** 0.5 for p, q in zip(pts, pts[1:]))
        se = sum((a - b) ** 2 for a, b in zip(pts[0], pts[-1])) ** 0.5
        return {'path_m': round(length, 2), 'start_end_m': round(se, 2)}

    def dual_summary(self, path):
        """Dual VIO run: fused trajectory, frames and events of the two trackers."""
        fused = os.path.join(path, 'fused', 'odometry.csv')
        if not os.path.isfile(fused):
            return None
        r = {'name': os.path.basename(path), 'time': os.path.getmtime(path), 'profile': 'dual',
             'mode': 'dual VIO', 'trajectory': 'fused/odometry.csv', 'health': {}, 'events': {}}
        r.update(self.path_stats(fused))
        for cam in ('cam1', 'cam2'):
            sub = self.run_summary(os.path.join(path, cam))
            if sub:
                r['frames'] = max(r.get('frames') or 0, sub.get('frames') or 0)
                for key in ('health', 'events'):
                    for k, v in sub[key].items():
                        r[key][f'{cam} {k}'] = v
        try:
            with open(os.path.join(path, 'fused', 'selection.csv')) as f:
                r['events']['camera switches'] = max(sum(1 for _ in f) - 1, 0)
        except OSError:
            pass
        rrd = os.path.join(self.c.rerun_dir, r['name'] + '.rrd')
        r['rrd'] = rrd if os.path.isfile(rrd) else ''
        return r

    def runs(self):
        out = []
        for path in glob.glob(os.path.join(self.c.runs_dir, '*')):
            if os.path.isdir(path):
                r = self.run_summary(path) or self.dual_summary(path)
                if r:
                    out.append(r)
        return sorted(out, key=lambda r: r['time'], reverse=True)

    def disk(self):
        u = shutil.disk_usage(self.c.data_dir)
        return {'free': u.free, 'total': u.total}

    def state(self):
        return {
            'cameras_running': self.cameras_running(),
            'recording': self.recording_status(),
            'job': self.job.to_dict() if self.job else None,
            'disk': self.disk(),
        }
