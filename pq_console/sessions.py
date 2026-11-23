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

import bags as bags_mod

NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,80}$')
PROFILES_DIR = '/ws/install/pq_cuvslam/share/pq_cuvslam/profiles'
# Recording presets: streams of each chosen camera. The driver streams are switched to match
# (enable_* parameters); 'aligned' is the depth aligned to the RGB (align_depth.enable).
PRESETS = {
    'ir_imu': {'label': 'IR stereo + IMU', 'streams': ['infra1', 'infra2', 'imu']},
    'ir_depth_imu': {'label': 'IR stereo + depth + IMU',
                     'streams': ['infra1', 'infra2', 'depth', 'imu']},
    'all': {'label': 'IR stereo + RGB + depth + IMU',
            'streams': ['infra1', 'infra2', 'color', 'depth', 'imu']},
    'rgbd_imu': {'label': 'RGB + depth aligned to RGB + IMU', 'streams': ['color', 'aligned', 'imu']},
    'all_aligned': {'label': 'IR stereo + RGB + depth + depth aligned to RGB + IMU',
                    'streams': ['infra1', 'infra2', 'color', 'depth', 'aligned', 'imu']},
}
STREAM_TOPICS = {
    'infra1': ['infra1/image_rect_raw', 'infra1/camera_info', 'infra1/metadata'],
    'infra2': ['infra2/image_rect_raw', 'infra2/camera_info', 'infra2/metadata'],
    'color': ['color/image_raw', 'color/camera_info', 'color/metadata'],
    'depth': ['depth/image_rect_raw', 'depth/camera_info', 'depth/metadata'],
    'aligned': ['aligned_depth_to_color/image_raw', 'aligned_depth_to_color/camera_info'],
    'imu': ['imu', 'gyro/metadata', 'accel/metadata'],
}


def driver_streams(streams):
    """enable_* parameters of the RealSense driver for a list of recorded streams."""
    s = set(streams)
    return {'enable_infra1': 'infra1' in s, 'enable_infra2': 'infra2' in s,
            'enable_color': bool(s & {'color', 'aligned'}),
            'enable_depth': bool(s & {'depth', 'aligned'}),
            'align_depth.enable': 'aligned' in s}


def usb_cameras():
    """RealSense devices on the USB bus (sysfs): model, serial, link speed."""
    out = []
    for d in sorted(glob.glob('/sys/bus/usb/devices/*')):
        def read(name, d=d):
            try:
                with open(os.path.join(d, name)) as f:
                    return f.read().strip()
            except OSError:
                return ''
        if read('idVendor') != '8086':
            continue
        speed = read('speed')
        out.append({'port': os.path.basename(d), 'product_id': read('idProduct'),
                    'model': read('product').replace('Intel(R) RealSense(TM) ', ''),
                    'serial': read('serial'),
                    'speed_mbps': int(float(speed)) if speed else None})
    return out


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
        for d in (self.c.bags_dir, self.c.runs_dir, self.c.rerun_dir, self.c.slam_bags_dir):
            os.makedirs(d, exist_ok=True)
        if running('pq_rec'):
            self.recording = {'name': '(started outside the console)', 'started': None}

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
        # The emitter argument of the launch is not applied by the driver: set it at runtime and
        # read it back (ros2 param set exits with 0 also when the parameter is not declared yet).
        off = ' && '.join(f'ros2 param set /{cam.name} depth_module.emitter_enabled 0 > /dev/null 2>&1 && '
                          f'ros2 param get /{cam.name} depth_module.emitter_enabled 2>/dev/null | '
                          "grep -q 'value is: 0'" for cam in self.c.cameras)
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

    @staticmethod
    def record_topics(cameras, streams):
        topics = [f'/{cam}/{t}' for cam in cameras for s in streams for t in STREAM_TOPICS[s]]
        return topics + ['/tf_static']

    def start_recording(self, name, preset='all', cameras=None):
        with self.lock:
            if self.recording:
                raise ValueError('a recording is already running')
            if not NAME_RE.match(name or ''):
                raise ValueError('name: letters, digits, _ . - only')
            if os.path.exists(os.path.join(self.c.bags_dir, name)):
                raise ValueError(f'a recording named {name} exists')
            if self.job and self.job.state == 'running' and self.job.kind == 'replay':
                raise ValueError('a replay is running (same topics)')
            if preset not in PRESETS:
                raise ValueError(f'unknown preset {preset}')
            known = [c.name for c in self.c.cameras]
            cameras = [c for c in (cameras or known) if c in known]
            if not cameras:
                raise ValueError('choose at least one camera')
            topics = ' '.join(self.record_topics(cameras, PRESETS[preset]['streams']))
            docker('rm', '-f', 'pq_rec')
            code, out = docker(
                'run', '-d', '--name', 'pq_rec', *self.ros_args(), '-v', f'{self.c.bags_dir}:/out',
                '--entrypoint', 'bash', self.c.camera_image, '-c',
                'source /opt/ros/lyrical/setup.bash; exec ros2 bag record -s mcap '
                '--storage-preset-profile zstd_fast --include-unpublished-topics '
                f'-o /out/{name} --topics {topics} > /out/{name}.log 2>&1')
            if code:
                raise RuntimeError(out)
            self.recording = {'name': name, 'started': time.time(), 'preset': preset,
                              'label': PRESETS[preset]['label'], 'cameras': cameras}

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

    # ---- bags: view and SLAM replay ----

    def check_bag(self, path):
        if not bags_mod.inside(path, self.c.roots) or not os.path.exists(path):
            raise ValueError(f'{path}: not a bag of the console folders')

    def check_idle(self):
        if self.job and self.job.state == 'running':
            raise ValueError(f'{self.job.kind} {self.job.name} is running')
        if self.recording or self.cameras_running():
            raise ValueError('stop the cameras first: the bag uses the same topics')

    @staticmethod
    def bag_meta(path):
        """Name, start (s, receive time) and duration of a bag."""
        import rosbag2_py
        m = rosbag2_py.Info().read_metadata(path, '')
        return {'bag': bags_mod.bag_name(path), 'path': path,
                'start': m.starting_time.nanoseconds / 1e9, 'duration_s': m.duration.nanoseconds / 1e9}

    def play_args(self, path, rate, extra=''):
        """Player container (image of the cameras: it knows the RealSense metadata messages)."""
        paths = [path]
        aligned = bags_mod.sidecar(path, '_aligned')
        if bags_mod.is_bag_dir(aligned):
            paths.append(aligned)   # depth aligned to the RGB (tools/align_depth.py)
        mounts = []
        for d in sorted({os.path.dirname(x.rstrip('/')) for x in paths}):
            mounts += ['-v', f'{d}:{d}:ro']
        inputs = ' '.join(f'-i {shlex.quote(x)}' for x in paths)
        return ['run', '--rm', '--name', 'pq_play', *self.ros_args(), *mounts,
                '--entrypoint', 'bash', self.c.camera_image or self.c.slam_image, '-c',
                f'source /opt/ros/lyrical/setup.bash; ros2 bag play {inputs} --rate {rate} {extra} '
                '> /dev/null 2>&1']

    def start_view(self, path, rate=1.0, paused=False):
        """Play a bag without SLAM, to look at it (pause, rate and seek via the player)."""
        with self.lock:
            self.check_idle()
            self.check_bag(path)
            job = Job('view', bags_mod.bag_name(path))
            job.meta = self.bag_meta(path)
            job.meta.update(rate=float(rate), paused=bool(paused))
            self.job = job
        threading.Thread(target=self._view, args=(job, path, float(rate), paused), daemon=True).start()
        return job.name

    def _view(self, job, path, rate, paused):
        try:
            docker('rm', '-f', 'pq_play')
            job.info(f'play {path} at rate {rate}')
            # /clock at 10 Hz: position of the playback for the console.
            extra = '--clock 10' + (' --start-paused' if paused else '')
            docker(*self.play_args(path, rate, extra), timeout=24 * 3600)
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

    def versions(self):
        """cuVSLAM builds: {label: {'image', 'install_dir'}}; the first one is the default."""
        return self.c.cuvslam_versions or {
            'default': {'image': self.c.slam_image, 'install_dir': self.c.install_dir}}

    def version_args(self, version=None):
        """Image and docker arguments of a cuVSLAM build."""
        versions = self.versions()
        label = version or next(iter(versions))
        if label not in versions:
            raise ValueError(f'unknown cuVSLAM version {label}')
        v = versions[label]
        install = ['-v', f"{v['install_dir']}:/ws/install:ro"] if v.get('install_dir') else []
        return label, v.get('image') or self.c.slam_image, install + list(self.c.slam_extra_args)

    def start_replay(self, path, profile=None, custom=None, version=None, rate=1.0, rerun=True):
        """
        SLAM on a bag played in real time, as live.

        profile: name of an installed profile or 'dual'; custom: profile dict built by the console.
        """
        with self.lock:
            self.check_idle()
            self.check_bag(path)
            label, _, _ = self.version_args(version)
            if custom:
                profile = custom['name']
            elif profile not in self.profile_names(label):
                raise ValueError(f'no profile {profile} in cuVSLAM {label}')
            name = bags_mod.safe_name(f'{bags_mod.bag_name(path)}_{profile}' +
                                      ('' if label == next(iter(self.versions())) else f'_{label}'))
            job = Job('replay', name)
            job.meta = self.bag_meta(path)
            job.meta.update(rate=float(rate), profile=profile, version=label)
            self.job = job
        threading.Thread(target=self._replay, args=(job, path, profile, custom, label, float(rate),
                                                    rerun), daemon=True).start()
        return job.name

    def _replay(self, job, path, profile, custom, version, rate, rerun):
        c, name = self.c, job.name
        try:
            for n in ('pq_slam', 'pq_slamrec', 'pq_play'):
                docker('rm', '-f', n)
            _, image, args = self.version_args(version)
            if custom:
                os.makedirs(c.profiles_dir, exist_ok=True)
                with open(os.path.join(c.profiles_dir, f'{profile}.yaml'), 'w') as f:
                    f.write('# Profile built by pq_console for a bag.\n' + bags_mod.profile_yaml(custom))
                args += ['-v', f'{c.profiles_dir}:/console_profiles:ro']
                launch = (f'slam.launch.py profile:=/console_profiles/{profile}.yaml '
                          f'run_output_dir:=/out/{name} enable_vis:=true')
            elif profile == 'dual':
                launch = (f'dual_vio.launch.py cam1_profile:={c.dual_cam1_profile} '
                          f'run_output_dir:=/out/{name} enable_vis:=true')
            else:
                launch = f'slam.launch.py profile:={profile} run_output_dir:=/out/{name} enable_vis:=true'
            self.chown_rm(os.path.join(c.runs_dir, name))
            job.info(f'SLAM {profile} (cuVSLAM {version})')
            code, out = docker(
                'run', '-d', '--name', 'pq_slam', *c.gpu_args, *self.ros_args(), *args,
                '-v', f'{c.runs_dir}:/out', '--entrypoint', 'bash', image, '-c',
                'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; '
                f'exec ros2 launch pq_cuvslam {launch} > /out/{name}.log 2>&1')
            if code:
                raise RuntimeError(out)
            os.makedirs(c.slam_bags_dir, exist_ok=True)
            self.chown_rm(os.path.join(c.slam_bags_dir, f'{name}_slam'))
            code, out = docker(
                'run', '-d', '--name', 'pq_slamrec', *self.ros_args(), *args,
                '-v', f'{c.slam_bags_dir}:/out', '--entrypoint', 'bash', image, '-c',
                'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; '
                'exec ros2 bag record -s mcap --storage-preset-profile zstd_fast '
                f"--include-unpublished-topics -o /out/{name}_slam --regex '{SLAM_TOPICS_RE}' "
                f'> /out/{name}_slam.log 2>&1')
            if code:
                raise RuntimeError(out)
            time.sleep(8)   # nodes active and recorder subscribed before the first frame
            job.info(f'play {path} at rate {rate}')
            if not job.stop_event.is_set():
                # /clock: position of the playback for the console (the SLAM does not use it).
                docker(*self.play_args(path, rate, '--clock 10'), timeout=24 * 3600)
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
            else:
                job.info(f'no run output: see {c.runs_dir}/{name}.log')
            if rerun and not job.stop_event.is_set() and c.tools_dir:
                job.info('Rerun conversion')
                self._rerun(name, path)
            self.chown(c.runs_dir, c.slam_bags_dir, c.rerun_dir)
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

    def _rerun(self, name, path):
        c = self.c
        layout = ''
        if c.pylib_dir:
            layout = (f'python3 /tools/rerun_layout.py /rerun/{name}.mcap /rerun/{name}.rrd '
                      f'--slam-bag /slam/{name}_slam --run-dir /runs/{name} --pylib /pylib '
                      f'>> /rerun/{name}.log 2>&1')
        images = ','.join(f'/{cam}/{s}' for cam in ('cam2', 'cam1')
                          for s in ('infra1/image_rect_raw', 'color/image_raw'))
        pylib = ['-v', f'{c.pylib_dir}:/pylib:ro'] if c.pylib_dir else []
        bag_dir = os.path.dirname(path.rstrip('/'))
        _, image, args = self.version_args()   # pq_cuvslam messages (events) of the default build
        docker('run', '--rm', '--network', 'none', '-v', f'{c.rerun_dir}:/rerun',
               '-v', f'{c.slam_bags_dir}:/slam:ro', '-v', f'{c.runs_dir}:/runs:ro',
               '-v', f'{bag_dir}:{bag_dir}:ro',
               '-v', f'{c.tools_dir}:/tools:ro', *pylib, *args, '--entrypoint', 'bash', image, '-c',
               'source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; rm -rf /tmp/o; '
               f'python3 /tools/to_rerun.py /slam/{name}_slam {shlex.quote(path)} /tmp/o '
               f'--max-image-rate 15 --images {images} > /rerun/{name}.log 2>&1 '
               f'&& cp /tmp/o/*.mcap /rerun/{name}.mcap; {layout}', timeout=3 * 3600)

    # ---- lists ----

    def profiles(self, version=None, max_age=60.0):
        """Profiles installed in a cuVSLAM build (as its SLAM container sees them), plus dual."""
        label, image, args = self.version_args(version)
        self._profiles = getattr(self, '_profiles', {})
        cache = self._profiles.get(label)
        if cache and time.time() - cache[0] < max_age:
            return cache[1]
        script = ('import glob, json, os, yaml\n'
                  f'out = []\nfor p in sorted(glob.glob("{PROFILES_DIR}/*.yaml")):\n'
                  '    try:\n        y = yaml.safe_load(open(p))\n    except Exception:\n        y = {}\n'
                  '    out.append({"name": os.path.splitext(os.path.basename(p))[0], '
                  '"mode": y.get("mode", ""), "description": y.get("description", ""), '
                  '"profile": y})\n'
                  'print(json.dumps(out))')
        code, out = docker('run', '--rm', '--network', 'none', *args,
                           '--entrypoint', 'python3', image, '-c', script)
        found = []
        if code == 0:
            try:
                found = json.loads(out.splitlines()[-1])
            except (ValueError, IndexError):
                pass
        for p in found:
            p['topics'] = sorted(bags_mod.profile_topics(p.pop('profile') or {}))
        dual = {'name': 'dual', 'mode': 'dual VIO',
                'description': 'cam1 and cam2 VIO in parallel with the odometry selector',
                'topics': sorted({f'/{cam}/{s}' for cam in ('cam1', 'cam2') for s in (
                    'infra1/image_rect_raw', 'infra1/camera_info', 'infra2/image_rect_raw',
                    'infra2/camera_info', 'imu')})}
        result = [dual] + found
        self._profiles[label] = (time.time(), result)
        return result

    def profile_names(self, version=None):
        return [p['name'] for p in self.profiles(version)]

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
