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
pq_console: web console for the D455 cameras, recordings and pq_cuvslam replays.

Serves a page on listen (default 127.0.0.1:8080, reached through an ssh tunnel) with:
  - live: camera previews, IR emitter state, recording start/stop, SLAM trajectory and state;
  - sessions: recordings, SLAM replay of a recording with a profile, runs;
  - nodes and topics: ROS graph, rates on request.
Python standard library HTTP server, no other dependency than the pq_cuvslam image.

Usage (inside the container, see run.sh): console.py --config config.yaml
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
import threading
import time
import traceback
from urllib.parse import parse_qs, urlparse

import config as config_mod
import ros_bridge
import sessions as sessions_mod
import yaml

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
EMITTER = 'depth_module.emitter_enabled'

# Camera settings shown first, by group (only the parameters the driver has are shown). The
# other parameters of the driver are listed below them.
ROI = ('auto_exposure_roi.left', 'auto_exposure_roi.right', 'auto_exposure_roi.top',
       'auto_exposure_roi.bottom')
SETTING_GROUPS = [
    ('Streams', ['enable_infra1', 'enable_infra2', 'enable_color', 'enable_depth', 'enable_gyro',
                 'enable_accel', 'align_depth.enable']),
    ('Resolution and fps', ['depth_module.infra_profile', 'depth_module.depth_profile',
                            'rgb_camera.color_profile']),
    ('Infrared and depth', ['depth_module.emitter_enabled', 'depth_module.laser_power',
                            'depth_module.enable_auto_exposure', 'depth_module.exposure',
                            'depth_module.gain', 'depth_module.auto_exposure_limit_toggle',
                            'depth_module.auto_exposure_limit', 'depth_module.auto_gain_limit_toggle',
                            'depth_module.auto_gain_limit', 'depth_module.hdr_enabled'] +
     ['depth_module.' + r for r in ROI]),
    ('RGB', ['rgb_camera.enable_auto_exposure', 'rgb_camera.exposure', 'rgb_camera.gain',
             'rgb_camera.auto_exposure_priority', 'rgb_camera.enable_auto_white_balance',
             'rgb_camera.white_balance', 'rgb_camera.backlight_compensation'] +
     ['rgb_camera.' + r for r in ROI]),
]
SETTING_NAMES = [n for _, names in SETTING_GROUPS for n in names]
# Changing these restarts a stream of the driver: not during a recording.
RESTARTING = ('enable_', 'align_depth.', '_profile')


class Console:

    def __init__(self, config):
        self.c = config
        self.ros = ros_bridge.start()
        self.sessions = sessions_mod.Sessions(config)
        self.emitter_cache = {'time': 0.0, 'value': {}}
        self.settings_file = os.path.join(config.data_dir, 'camera_settings.yaml')
        self.settings_log = []    # messages of the startup settings

    def emitters(self, max_age=3.0):
        """IR emitter of each camera (parameter of the driver node); None if not reachable."""
        if time.time() - self.emitter_cache['time'] > max_age:
            values = {}
            nodes = set(self.ros.topics_state()['nodes'])
            for cam in self.c.cameras:
                node = f'/{cam.name}'
                if node not in nodes:
                    values[cam.name] = None
                    continue
                try:
                    values[cam.name] = self.ros.get_param(node, EMITTER)
                except Exception:   # noqa: B902 - driver starting or busy
                    values[cam.name] = None
            self.emitter_cache = {'time': time.time(), 'value': values}
        return self.emitter_cache['value']

    def state(self):
        s = self.sessions.state()
        s['cameras'] = [c.name for c in self.c.cameras]
        s['emitters'] = self.emitters() if s['cameras_running'] else {}
        s['previews'] = self.ros.previews_state()
        s['diagnostics'] = self.ros.diagnostics_state()
        clock = self.ros.clock
        s['clock'] = clock[0] if clock and time.time() - clock[1] < 2 else None
        s['time'] = time.time()
        return s

    # ---- camera settings ----

    def camera_settings(self):
        nodes = set(self.ros.topics_state()['nodes'])
        cameras = {}
        for cam in self.c.cameras:
            node = f'/{cam.name}'
            if node not in nodes:
                cameras[cam.name] = {'available': False, 'params': []}
                continue
            try:
                cameras[cam.name] = {'available': True, 'params': self.ros.node_parameters(node)}
            except Exception as e:   # noqa: B902 - driver starting or busy
                cameras[cam.name] = {'available': False, 'params': [], 'error': str(e)}
        saved = self.saved_settings()
        return {'groups': SETTING_GROUPS, 'cameras': cameras, 'saved': saved,
                'saved_time': os.path.getmtime(self.settings_file) if saved else None,
                'startup_log': self.settings_log[-10:],
                'recording': bool(self.sessions.recording)}

    def set_camera_param(self, cameras, name, value):
        if self.sessions.recording and any(k in name for k in RESTARTING):
            raise ValueError(f'{name} restarts the stream: not during a recording')
        results = {}
        for cam in cameras:
            node = f'/{cam}'
            type_, _ = self.ros.get_param_typed(node, name)
            self.ros.set_param(node, name, value, type_)
            results[cam] = self.ros.get_param(node, name)
        if name == EMITTER:
            self.emitter_cache['time'] = 0.0
        return results

    def snapshot(self, names=None, writable_only=False):
        """Current settings of the cameras ({camera: {name: value}})."""
        out = {}
        for cam in self.c.cameras:
            try:
                params = self.ros.node_parameters(f'/{cam.name}')
            except Exception:   # noqa: B902 - camera not running
                continue
            out[cam.name] = {p['name']: p['value'] for p in params
                             if (names is None or p['name'] in names) and
                             not (writable_only and p['read_only'])}
        return out

    def saved_settings(self):
        try:
            with open(self.settings_file) as f:
                return yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            return {}

    def save_settings(self):
        settings = self.snapshot(SETTING_NAMES, writable_only=True)
        if not settings:
            raise ValueError('no camera running')
        with open(self.settings_file, 'w') as f:
            f.write('# Camera settings applied by pq_console after the cameras start.\n')
            yaml.safe_dump(settings, f, sort_keys=True)
        self.sessions.chown(self.settings_file)
        return {k: len(v) for k, v in settings.items()}

    def apply_startup_settings(self):
        """After Start cameras: the saved settings, once the driver nodes answer."""
        saved = self.saved_settings()
        log = self.settings_log
        if not saved:
            return
        for cam, params in saved.items():
            node = f'/{cam}'
            for _ in range(60):
                if node in self.ros.topics_state()['nodes']:
                    break
                time.sleep(1)
            else:
                log.append(f'{cam}: driver not found, settings not applied')
                continue
            time.sleep(3)
            done, failed = 0, []
            for name, value in params.items():
                try:
                    type_, current = self.ros.get_param_typed(node, name)
                    if current != value:
                        self.ros.set_param(node, name, value, type_)
                        done += 1
                except Exception as e:   # noqa: B902 - reported in the page
                    failed.append(f'{name} ({e})')
            log.append(f'{time.strftime("%H:%M:%S")} {cam}: {done} settings applied' +
                       (f', failed: {"; ".join(failed)}' if failed else ''))
        self.emitter_cache['time'] = 0.0

    def write_recording_settings(self, name):
        """Settings of the cameras next to a recording (bags/<name>.camera_settings.yaml)."""
        try:
            settings = self.snapshot()
            path = os.path.join(self.c.bags_dir, f'{name}.camera_settings.yaml')
            with open(path, 'w') as f:
                f.write(f'# Parameters of the camera drivers at the start of {name}.\n')
                yaml.safe_dump(settings, f, sort_keys=True)
            self.sessions.chown(path)
        except Exception:   # noqa: B902 - the recording goes on without it
            traceback.print_exc()

    # ---- actions (POST /api/<name>) ----

    def act(self, name, body):
        if name == 'cameras/start':
            self.sessions.start_cameras()
            threading.Thread(target=self.apply_startup_settings, daemon=True).start()
        elif name == 'cameras/stop':
            self.sessions.stop_cameras()
        elif name == 'emitter':
            self.set_camera_param([body['camera']], EMITTER, int(body.get('value', 0)))
        elif name == 'param':
            cameras = body.get('cameras') or [body['camera']]
            known = {c.name for c in self.c.cameras}
            if not set(cameras) <= known:
                raise ValueError(f'unknown camera in {cameras}')
            return self.set_camera_param(cameras, body['name'], body['value'])
        elif name == 'settings/save':
            return self.save_settings()
        elif name == 'record/start':
            if self.c.cameras:
                emitters = self.emitters(max_age=0)
                on = [cam for cam, v in emitters.items() if v]
                unknown = [cam for cam, v in emitters.items() if v is None]
                if on and not body.get('force'):
                    raise ValueError(f'IR emitter on: {", ".join(on)}')
                if unknown and not body.get('force'):
                    raise ValueError(f'emitter state unknown (camera not running?): '
                                     f'{", ".join(unknown)}')
            self.sessions.start_recording(body.get('name', ''), bool(body.get('depth', True)),
                                          bool(body.get('aligned', False)))
            threading.Thread(target=self.write_recording_settings, args=(body['name'],),
                             daemon=True).start()
        elif name == 'record/stop':
            return {'name': self.sessions.stop_recording()}
        elif name == 'replay/start':
            self.ros.slam.reset()
            return {'name': self.sessions.start_replay(
                body['bag'], body['profile'], body.get('rate', 1.0), bool(body.get('rerun', True)))}
        elif name == 'view/start':
            return {'name': self.sessions.start_view(body['bag'], body.get('rate', 1.0),
                                                     bool(body.get('paused', False)))}
        elif name == 'player':
            job = self.sessions.job
            if not job or job.state != 'running':
                raise ValueError('no playback running')
            if job.kind != 'view' and body['command'] in ('seek', 'pause'):
                raise ValueError('pause and seek only when viewing a recording (not with SLAM)')
            self.ros.player(body['command'], body.get('value'))
            if body['command'] in ('pause', 'resume'):
                job.meta['paused'] = body['command'] == 'pause'
            elif body['command'] == 'rate':
                job.meta['rate'] = float(body['value'])
        elif name == 'job/stop':
            self.sessions.stop_job()
        elif name == 'slam/reset':
            self.ros.slam.reset()
        elif name == 'measure':
            self.ros.measure(body['topic'], float(body.get('seconds', 60)))
        else:
            raise KeyError(name)
        return {}


class Handler(BaseHTTPRequestHandler):
    console = None
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        pass

    def send_json(self, obj, code=200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        c = self.console
        try:
            if url.path == '/':
                return self.send_file(os.path.join(STATIC, 'index.html'))
            if url.path.startswith('/static/'):
                return self.send_file(os.path.join(STATIC, os.path.basename(url.path)))
            if url.path == '/stream':
                return self.stream(q['topic'])
            if url.path == '/api/state':
                return self.send_json(c.state())
            if url.path == '/api/topics':
                return self.send_json(c.ros.topics_state())
            if url.path == '/api/slam':
                return self.send_json(c.ros.slam.state(json.loads(q.get('since', '{}'))))
            if url.path == '/api/recordings':
                return self.send_json(c.sessions.recordings())
            if url.path == '/api/runs':
                return self.send_json(c.sessions.runs())
            if url.path == '/api/profiles':
                return self.send_json(c.sessions.profiles())
            if url.path == '/api/camera_settings':
                return self.send_json(c.camera_settings())
            self.send_json({'error': 'not found'}, 404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as e:   # noqa: B902 - returned to the page
            traceback.print_exc()
            self.send_json({'error': str(e)}, 500)

    def do_POST(self):
        url = urlparse(self.path)
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length) or b'{}')
            if not url.path.startswith('/api/'):
                raise KeyError(url.path)
            self.send_json(self.console.act(url.path[len('/api/'):], body) or {})
        except KeyError as e:
            self.send_json({'error': f'unknown request or missing field {e}'}, 400)
        except (ValueError, TimeoutError, RuntimeError) as e:
            self.send_json({'error': str(e)}, 400)
        except Exception as e:   # noqa: B902 - returned to the page
            traceback.print_exc()
            self.send_json({'error': str(e)}, 500)

    def send_file(self, path):
        if not os.path.isfile(path):
            return self.send_json({'error': 'not found'}, 404)
        with open(path, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type', mimetypes.guess_type(path)[0] or 'text/plain')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def stream(self, topic):
        """MJPEG stream of an image topic (multipart/x-mixed-replace)."""
        preview = self.console.ros.preview(topic)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True
        seq = -1
        while True:
            seq, jpeg = preview.wait(seq, timeout=5.0)
            if jpeg is None:
                continue
            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' +
                             str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
            self.wfile.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', required=True)
    args = ap.parse_args()
    config = config_mod.load(args.config)
    Handler.console = Console(config)
    server = ThreadingHTTPServer((config.listen_host, config.listen_port), Handler)
    server.daemon_threads = True
    print(f'pq_console on http://{config.listen_host}:{config.listen_port}', flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
