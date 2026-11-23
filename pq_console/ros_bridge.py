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
ROS side of pq_console: graph, topic rates, image previews, SLAM state, camera parameters.

Light by design, to run next to a recording on the Jetson:
  - topic rates are measured only on request (and for the previewed images);
  - previews subscribe to the serialized messages and decode at most preview_fps frames per
    second, downscaled and JPEG-compressed; a preview without viewers is dropped after 10 s;
  - the SLAM state comes from the odometry and diagnostics topics of pq_cuvslam (small messages).
"""

import collections
import math
import re
import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image

try:
    from pq_cuvslam_interfaces.msg import FrameDiagnostics, SlamEvent
except ImportError:   # console outside the pq_cuvslam image: no SLAM diagnostics
    FrameDiagnostics = SlamEvent = None

ODOM_RE = re.compile(r'^(/cam[12]_vio)?/visual_slam/tracking/odometry(_guarded|_fused)?$')
EVENT_NAMES = {1: 'tracking lost', 2: 'tracking recovered', 3: 'loop closure', 4: 'configured',
               5: 'tracker initialized', 6: 'pose rejected', 7: 'pose accepted',
               8: 'tracker reset', 9: 'camera switch'}
HEALTH_NAMES = {0: 'ok', 1: 'degraded', 2: 'rejected', 3: 'lost'}


class Meter:
    """Message rate, bandwidth and age over the last window_s seconds."""

    def __init__(self, window_s=2.0):
        self.window = window_s
        self.samples = collections.deque()
        self.last = None
        self.lock = threading.Lock()

    def add(self, size):
        now = time.time()
        with self.lock:
            self.samples.append((now, size))
            self.last = now
            while self.samples and now - self.samples[0][0] > self.window:
                self.samples.popleft()

    def stats(self):
        now = time.time()
        with self.lock:
            s = [x for x in self.samples if now - x[0] <= self.window]
            last = self.last
        hz = len(s) / self.window if s else 0.0
        return {'hz': round(hz, 1), 'kbps': round(sum(x[1] for x in s) * 8 / 1000 / self.window, 1),
                'age_s': round(now - last, 1) if last else None}


class Preview:
    """Latest JPEG of an image topic, decoded at most fps times per second."""

    def __init__(self, topic, compressed, fps, width):
        self.topic, self.compressed = topic, compressed
        self.period = 1.0 / fps
        self.width = width
        self.meter = Meter()
        self.jpeg = None
        self.seq = 0
        self.info = {}
        self.last_decode = 0.0
        self.last_viewer = time.time()
        self.cond = threading.Condition()
        self.sub = None

    def on_message(self, data):
        self.meter.add(len(data))
        now = time.time()
        if now - self.last_decode < self.period:
            return
        self.last_decode = now
        try:
            jpeg, info = self.encode(data)
        except Exception as e:   # noqa: B902 - shown in the console
            jpeg, info = None, {'error': str(e)}
        with self.cond:
            if jpeg is not None:
                self.jpeg = jpeg
                self.seq += 1
            self.info = info
            self.cond.notify_all()

    def encode(self, data):
        if self.compressed:
            msg = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
            enc = msg.format
        else:
            msg = deserialize_message(data, Image)
            enc = msg.encoding
            buf = np.frombuffer(bytes(msg.data), np.uint8)
            if enc in ('mono8', '8UC1'):
                img = buf.reshape(msg.height, msg.step)[:, :msg.width]
            elif enc in ('rgb8', 'bgr8'):
                img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(
                    msg.height, msg.width, 3)
                if enc == 'rgb8':
                    img = img[:, :, ::-1]
            elif enc in ('16UC1', 'mono16'):
                img = buf.view(np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
            else:
                raise ValueError(f'encoding {enc} not supported')
        info = {'encoding': enc, 'width': int(img.shape[1]), 'height': int(img.shape[0])}
        if img.dtype == np.uint16:   # depth in mm: colored 0-6 m, black = no depth
            valid = img > 0
            info['valid_depth_pct'] = round(100.0 * valid.mean(), 1)
            scaled = np.clip(img.astype(np.float32) * (255.0 / 6000.0), 0, 255).astype(np.uint8)
            img = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            img[~valid] = 0
        else:
            gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            info['saturated_pct'] = round(100.0 * (gray >= 250).mean(), 1)
            info['mean'] = round(float(gray.mean()), 1)
        if img.shape[1] > self.width:
            h = int(img.shape[0] * self.width / img.shape[1])
            img = cv2.resize(img, (self.width, h), interpolation=cv2.INTER_AREA)
        ok, out = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return (out.tobytes() if ok else None), info

    def wait(self, after_seq, timeout=5.0):
        with self.cond:
            self.last_viewer = time.time()
            self.cond.wait_for(lambda: self.seq != after_seq, timeout)
            return self.seq, self.jpeg


class Slam:
    """Trajectories, last diagnostics and events of the running SLAM."""

    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.paths = {}       # topic -> [[t, x, y, z], ...] subsampled
            self.last_point = {}
            self.diag = {}        # topic -> dict
            self.events = collections.deque(maxlen=50)
            self.generation = time.time()

    def odometry(self, topic, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        with self.lock:
            last = self.last_point.get(topic)
            if last is not None and t - last < 0.1:
                return
            self.last_point[topic] = t
            path = self.paths.setdefault(topic, [])
            path.append([round(t, 3), round(p.x, 3), round(p.y, 3), round(p.z, 3)])
            if len(path) > 40000:
                del path[::2]

    def diagnostics(self, topic, d):
        with self.lock:
            self.diag[topic] = {
                'time': time.time(), 'mode': d.mode, 'profile': d.profile, 'tracked': d.tracked,
                'track_ms': round(d.track_time_ms, 2), 'landmarks': d.landmarks,
                'observations': dict(zip(d.camera_names, [int(x) for x in d.observations])),
                'health': HEALTH_NAMES.get(d.health, str(d.health)), 'reasons': d.health_reasons,
                'speed_mps': None if math.isnan(d.speed_mps) else round(d.speed_mps, 2),
                'sync_offset_ms': None if math.isnan(d.sync_offset_ms) else round(d.sync_offset_ms, 1),
                'streams': [{'name': s.name, 'saturated_pct': round(s.saturated_pct, 1),
                             'mean': round(s.mean_intensity, 1),
                             'valid_depth_pct': None if math.isnan(s.valid_depth_pct)
                             else round(s.valid_depth_pct, 1)} for s in d.streams]}

    def event(self, topic, e):
        with self.lock:
            self.events.append({'time': time.time(), 'topic': topic,
                                'type': EVENT_NAMES.get(e.type, str(e.type)),
                                'description': e.description})

    def state(self, since):
        """Trajectory points after the given stamps (incremental polling)."""
        with self.lock:
            paths = {}
            for topic, pts in self.paths.items():
                after = since.get(topic, -1.0)
                paths[topic] = [p for p in pts if p[0] > after] if after >= 0 else list(pts)
            return {'generation': self.generation, 'paths': paths, 'diagnostics': dict(self.diag),
                    'events': list(self.events)}


class RosBridge(Node):

    def __init__(self, preview_fps=5.0, preview_width=424):
        super().__init__('pq_console')
        self.preview_fps, self.preview_width = preview_fps, preview_width
        self.lock = threading.Lock()
        self.graph = {'nodes': [], 'topics': []}
        self.measured = {}        # topic -> (Meter, subscription, until)
        self.previews = {}        # topic -> Preview
        self.slam = Slam()
        self.slam_subs = {}       # topic -> subscription
        self.diagnostics = {}     # status name -> dict (/diagnostics, e.g. RealSense driver)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 10)
        self.create_timer(2.0, self.refresh_graph)
        self.refresh_graph()

    # ---- graph ----

    def refresh_graph(self):
        try:
            nodes = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in self.get_node_names_and_namespaces())
            topics = []
            for name, types in sorted(self.get_topic_names_and_types()):
                # Lyrical rosidl::Buffer backends: hidden per-buffer topics (/<topic>/_buf/<id>).
                if '/_buf/' in name:
                    continue
                try:
                    pubs, subs = self.count_publishers(name), self.count_subscribers(name)
                except Exception:   # noqa: B902 - name rejected by rcl
                    pubs = subs = None
                topics.append({'name': name, 'type': types[0] if types else '',
                               'publishers': pubs, 'subscribers': subs})
        except Exception as e:   # noqa: B902 - graph queries can fail while shutting down
            self.get_logger().warning(f'graph: {e}')
            return
        with self.lock:
            self.graph = {'nodes': nodes, 'topics': topics}
        self.update_slam_subscriptions(topics)
        self.expire()

    def topics_state(self):
        with self.lock:
            graph = {'nodes': list(self.graph['nodes']),
                     'topics': [dict(t) for t in self.graph['topics']]}
            meters = {k: v[0] for k, v in self.measured.items()}
            meters.update({k: p.meter for k, p in self.previews.items()})
        for t in graph['topics']:
            if t['name'] in meters:
                t.update(meters[t['name']].stats())
                t['measuring'] = True
        return graph

    def topic_type(self, topic):
        with self.lock:
            for t in self.graph['topics']:
                if t['name'] == topic:
                    return t['type']
        return None

    def measure(self, topic, seconds=60.0):
        """Rate of a topic for the next seconds (serialized messages, not decoded)."""
        type_name = self.topic_type(topic)
        if not type_name:
            raise ValueError(f'unknown topic {topic}')
        with self.lock:
            if topic in self.measured:
                m, sub, _ = self.measured[topic]
                self.measured[topic] = (m, sub, time.time() + seconds)
                return
        meter = Meter()
        sub = self.create_subscription(get_message(type_name), topic,
                                       lambda data: meter.add(len(data)),
                                       qos_profile_sensor_data, raw=True)
        with self.lock:
            self.measured[topic] = (meter, sub, time.time() + seconds)

    def expire(self):
        now = time.time()
        drop = []
        with self.lock:
            for topic, (_, sub, until) in list(self.measured.items()):
                if now > until:
                    drop.append(sub)
                    del self.measured[topic]
            for topic, p in list(self.previews.items()):
                if now - p.last_viewer > 10.0:
                    drop.append(p.sub)
                    del self.previews[topic]
        for sub in drop:
            self.destroy_subscription(sub)

    # ---- previews ----

    def preview(self, topic):
        with self.lock:
            p = self.previews.get(topic)
            if p:
                return p
        type_name = self.topic_type(topic)
        if type_name not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            raise ValueError(f'{topic} is not an image topic ({type_name})')
        compressed = type_name.endswith('CompressedImage')
        p = Preview(topic, compressed, self.preview_fps, self.preview_width)
        p.sub = self.create_subscription(CompressedImage if compressed else Image, topic,
                                         p.on_message, qos_profile_sensor_data, raw=True)
        with self.lock:
            self.previews[topic] = p
        return p

    def previews_state(self):
        with self.lock:
            return {t: dict(p.info, **p.meter.stats()) for t, p in self.previews.items()}

    # ---- SLAM ----

    def update_slam_subscriptions(self, topics):
        for t in topics:
            name, type_name = t['name'], t['type']
            if name in self.slam_subs or not t['publishers']:
                continue
            if type_name == 'nav_msgs/msg/Odometry' and ODOM_RE.match(name):
                self.slam_subs[name] = self.create_subscription(
                    Odometry, name, lambda m, n=name: self.slam.odometry(n, m), 50)
            elif (FrameDiagnostics and type_name == 'pq_cuvslam_interfaces/msg/FrameDiagnostics'):
                self.slam_subs[name] = self.create_subscription(
                    FrameDiagnostics, name, lambda m, n=name: self.slam.diagnostics(n, m), 10)
            elif SlamEvent and type_name == 'pq_cuvslam_interfaces/msg/SlamEvent':
                self.slam_subs[name] = self.create_subscription(
                    SlamEvent, name, lambda m, n=name: self.slam.event(n, m), 50)

    # ---- /diagnostics ----

    def on_diagnostics(self, msg):
        now = time.time()
        with self.lock:
            for s in msg.status:
                self.diagnostics[s.name] = {
                    'time': now, 'level': int.from_bytes(s.level, 'little')
                    if isinstance(s.level, bytes) else int(s.level),
                    'message': s.message, 'values': {kv.key: kv.value for kv in s.values}}

    def diagnostics_state(self):
        now = time.time()
        with self.lock:
            return {k: dict(v, age_s=round(now - v['time'], 1)) for k, v in self.diagnostics.items()
                    if now - v['time'] < 30}

    # ---- parameters of other nodes ----

    def _call(self, srv_type, service, request, timeout=3.0):
        client = self.create_client(srv_type, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise TimeoutError(f'{service} not available')
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout):
                raise TimeoutError(f'{service}: no answer')
            return future.result()
        finally:
            self.destroy_client(client)

    def get_param(self, node, name):
        req = GetParameters.Request(names=[name])
        res = self._call(GetParameters, f'{node}/get_parameters', req)
        if not res.values:
            return None
        v = res.values[0]
        return {ParameterType.PARAMETER_BOOL: v.bool_value,
                ParameterType.PARAMETER_INTEGER: v.integer_value,
                ParameterType.PARAMETER_DOUBLE: v.double_value,
                ParameterType.PARAMETER_STRING: v.string_value}.get(v.type)

    def set_param(self, node, name, value):
        v = ParameterValue()
        if isinstance(value, bool):
            v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
        elif isinstance(value, int):
            v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
        elif isinstance(value, float):
            v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
        else:
            v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
        req = SetParameters.Request(parameters=[Parameter(name=name, value=v)])
        res = self._call(SetParameters, f'{node}/set_parameters', req)
        r = res.results[0]
        if not r.successful:
            raise RuntimeError(f'{node} {name}: {r.reason}')


def start(**kwargs):
    rclpy.init()
    bridge = RosBridge(**kwargs)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    return bridge
