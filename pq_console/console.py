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
import time
import traceback
from urllib.parse import parse_qs, urlparse

import config as config_mod
import ros_bridge
import sessions as sessions_mod

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
EMITTER = 'depth_module.emitter_enabled'


class Console:

    def __init__(self, config):
        self.c = config
        self.ros = ros_bridge.start()
        self.sessions = sessions_mod.Sessions(config)
        self.emitter_cache = {'time': 0.0, 'value': {}}

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
        s['time'] = time.time()
        return s

    # ---- actions (POST /api/<name>) ----

    def act(self, name, body):
        if name == 'cameras/start':
            self.sessions.start_cameras()
        elif name == 'cameras/stop':
            self.sessions.stop_cameras()
        elif name == 'emitter':
            value = int(body.get('value', 0))
            self.ros.set_param(f"/{body['camera']}", EMITTER, value)
            self.emitter_cache['time'] = 0.0
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
        elif name == 'record/stop':
            return {'name': self.sessions.stop_recording()}
        elif name == 'replay/start':
            self.ros.slam.reset()
            return {'name': self.sessions.start_replay(
                body['bag'], body['profile'], body.get('rate', 1.0), bool(body.get('rerun', True)))}
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
