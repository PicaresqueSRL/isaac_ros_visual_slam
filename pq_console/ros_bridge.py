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
  - previews and landmarks are not received continuously: their subscriptions (history 1) belong
    to a node that no executor spins, and the latest sample is taken only when the page asks for
    it (a few times per second), then decoded, downscaled and JPEG-compressed. The rate of the
    stream comes from the publication sequence numbers. A preview without viewers is dropped
    after 10 s;
  - topic rates of other topics are measured only on request;
  - the SLAM state comes from the odometry and diagnostics topics of pq_cuvslam (small messages).
"""

import base64
import collections
import math
import re
import struct
import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_interfaces.srv import Pause, Resume, Seek, SetRate
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Image, PointCloud2

try:
    from pq_cuvslam_interfaces.msg import FrameDiagnostics, SlamEvent
except ImportError:   # console outside the pq_cuvslam image: no SLAM diagnostics
    FrameDiagnostics = SlamEvent = None

ODOM_RE = re.compile(r'^(/cam[12]_vio)?/visual_slam/tracking/odometry(_guarded|_fused)?$')
LANDMARKS_RE = re.compile(r'visual_slam/vis/landmarks_cloud$')
LATEST_QOS = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.BEST_EFFORT)
LANDMARK_CELL_M = 0.05       # 3D grid of the landmarks sent to the page
LANDMARK_MAX = 60000
EVENT_NAMES = {1: 'tracking lost', 2: 'tracking recovered', 3: 'loop closure', 4: 'configured',
               5: 'tracker initialized', 6: 'pose rejected', 7: 'pose accepted',
               8: 'tracker reset', 9: 'camera switch'}
HEALTH_NAMES = {0: 'ok', 1: 'degraded', 2: 'rejected', 3: 'lost'}
PARAM_TYPES = {ParameterType.PARAMETER_BOOL: 'bool', ParameterType.PARAMETER_INTEGER: 'integer',
               ParameterType.PARAMETER_DOUBLE: 'double', ParameterType.PARAMETER_STRING: 'string',
               ParameterType.PARAMETER_BYTE_ARRAY: 'byte[]',
               ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
               ParameterType.PARAMETER_INTEGER_ARRAY: 'integer[]',
               ParameterType.PARAMETER_DOUBLE_ARRAY: 'double[]',
               ParameterType.PARAMETER_STRING_ARRAY: 'string[]'}
PLAYER = '/rosbag2_player'


def finite(x, digits=1):
    """Rounded float, None for NaN or infinity (NaN is not valid JSON: the page drops the answer)."""
    return round(float(x), digits) if math.isfinite(x) else None


def param_value(v):
    """Python value of a ParameterValue."""
    return {ParameterType.PARAMETER_BOOL: lambda: v.bool_value,
            ParameterType.PARAMETER_INTEGER: lambda: v.integer_value,
            ParameterType.PARAMETER_DOUBLE: lambda: v.double_value,
            ParameterType.PARAMETER_STRING: lambda: v.string_value,
            ParameterType.PARAMETER_BOOL_ARRAY: lambda: list(v.bool_array_value),
            ParameterType.PARAMETER_INTEGER_ARRAY: lambda: list(v.integer_array_value),
            ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(v.double_array_value),
            ParameterType.PARAMETER_STRING_ARRAY: lambda: list(v.string_array_value),
            }.get(v.type, lambda: None)()


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


def parse_image(data):
    """
    sensor_msgs/Image from its serialized form (little-endian CDR), without copying the pixels.

    deserialize_message builds a Python array of the pixels and is several times slower.
    Returns (height, width, encoding, step, memoryview of the pixels).
    """
    if data[1] != 1:
        raise ValueError('not little-endian CDR')
    buf = memoryview(data)
    pos = 4

    def u32():
        nonlocal pos
        pos += (-(pos - 4)) % 4          # alignment relative to the start of the payload
        v = struct.unpack_from('<I', buf, pos)[0]
        pos += 4
        return v

    def string():
        nonlocal pos
        n = u32()
        v = bytes(buf[pos:pos + n - 1]).decode()
        pos += n
        return v

    u32()                                # stamp.sec
    u32()                                # stamp.nanosec
    string()                             # frame_id
    height, width = u32(), u32()
    encoding = string()
    pos += 1                             # is_bigendian
    step = u32()
    n = u32()
    return height, width, encoding, step, buf[pos:pos + n]


class Latest:
    """Latest sample of a topic, taken on request (the node is not spun by an executor)."""

    def __init__(self, node, node_lock, msg_type, topic):
        self.node_lock = node_lock
        with node_lock:
            self.sub = node.create_subscription(msg_type, topic, lambda _: None, LATEST_QOS, raw=True)
        self.last_seq = None
        self.last_stamp = None
        self.last_new = None     # wall time of the last new sample
        self.hz = None

    def take(self):
        """Serialized latest sample, or None if nothing new arrived."""
        with self.node_lock, self.sub.handle:
            r = self.sub.handle.take_message(self.sub.msg_type, True)
        if not r:
            return None
        data, info = r
        seq, stamp = info.get('publication_sequence_number'), info.get('source_timestamp')
        if seq and self.last_seq and stamp and self.last_stamp and seq > self.last_seq and \
                stamp > self.last_stamp:
            hz = (seq - self.last_seq) / ((stamp - self.last_stamp) / 1e9)
            self.hz = hz if self.hz is None else 0.7 * self.hz + 0.3 * hz
        self.last_seq, self.last_stamp, self.last_new = seq, stamp, time.time()
        return data

    def rate(self):
        if self.last_new is None or time.time() - self.last_new > 3.0:
            return 0.0 if self.last_new else None
        return round(self.hz, 1) if self.hz else None


class Preview:
    """Latest JPEG of an image topic, decoded when the page asks for it."""

    def __init__(self, latest, compressed, width):
        self.latest, self.compressed = latest, compressed
        self.width = width
        self.jpeg = None
        self.seq = 0
        self.info = {}
        self.last_viewer = time.time()
        self.lock = threading.Lock()

    def get(self):
        """(frame number, JPEG, info) after taking the latest image, if a new one arrived."""
        with self.lock:
            self.last_viewer = time.time()
            data = self.latest.take()
            if data is not None:
                try:
                    jpeg, info = self.encode(data)
                except Exception as e:   # noqa: B902 - shown in the console
                    jpeg, info = None, {'error': str(e)}
                if jpeg is not None:
                    self.jpeg = jpeg
                    self.seq += 1
                self.info = info
            return self.seq, self.jpeg, dict(self.info, hz=self.latest.rate())

    def encode(self, data):
        if self.compressed:
            msg = deserialize_message(data, CompressedImage)
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_UNCHANGED)
            enc = msg.format
        else:
            height, width, enc, step, pixels = parse_image(data)
            buf = np.frombuffer(pixels, np.uint8)
            if enc in ('mono8', '8UC1'):
                img = buf.reshape(height, step)[:, :width]
            elif enc in ('rgb8', 'bgr8'):
                img = buf.reshape(height, step)[:, :width * 3].reshape(height, width, 3)
                if enc == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif enc in ('16UC1', 'mono16'):
                img = buf.view(np.uint16).reshape(height, step // 2)[:, :width]
            else:
                raise ValueError(f'encoding {enc} not supported')
        info = {'encoding': enc, 'width': int(img.shape[1]), 'height': int(img.shape[0])}
        # Subsampling (every n-th pixel) to about the preview width: no interpolation cost; the
        # statistics below are computed on the small image (close enough for a preview).
        n = max(1, int(img.shape[1] // self.width))
        if n > 1:
            img = img[::n, ::n]
        if img.dtype == np.uint16:   # depth in mm: colored 0-6 m, black = no depth
            valid = img > 0
            info['valid_depth_pct'] = round(100.0 * valid.mean(), 1)
            scaled = cv2.convertScaleAbs(img, alpha=255.0 / 6000.0)
            img = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            img[~valid] = 0
        else:
            gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            info['saturated_pct'] = round(100.0 * np.count_nonzero(gray >= 250) / gray.size, 1)
            info['mean'] = round(float(cv2.mean(gray)[0]), 1)
        img = np.ascontiguousarray(img)
        ok, out = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 65])
        return (out.tobytes() if ok else None), info


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
            self.landmarks = {}   # topic -> (number, base64 of int16 x y z cell indices)
            self.events = collections.deque(maxlen=50)
            self.loop_closures = []   # [{topic, position}] of the whole run
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
                'streams': [{'name': s.name, 'saturated_pct': finite(s.saturated_pct),
                             'mean': finite(s.mean_intensity),
                             'valid_depth_pct': finite(s.valid_depth_pct)} for s in d.streams]}

    def event(self, topic, e):
        with self.lock:
            ev = {'time': time.time(), 'topic': topic, 'type': EVENT_NAMES.get(e.type, str(e.type)),
                  'description': e.description}
            if e.type == 3:   # loop closure: where the tracker of the event is now
                ns = topic.split('/visual_slam')[0]
                for name in ('odometry_guarded', 'odometry', 'odometry_fused'):
                    path = self.paths.get(f'{ns}/visual_slam/tracking/{name}')
                    if path:
                        ev['position'] = path[-1][1:]
                        self.loop_closures.append({'topic': topic, 'position': ev['position']})
                        break
            self.events.append(ev)

    def set_landmarks(self, topic, msg):
        """Landmark cloud of pq_cuvslam reduced to the occupied cells of a 3D grid."""
        offsets = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        if n == 0 or not all(k in offsets for k in 'xyz'):
            xyz = np.zeros((0, 3))
        else:
            raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(n, msg.point_step)
            xyz = np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0]
                            for k in 'xyz'], axis=1)
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
        cells = np.unique(np.floor(xyz / LANDMARK_CELL_M).astype(np.int64), axis=0)
        # int16 cell indices (+-1600 m at 5 cm), little-endian, base64: about 8 bytes per point
        # instead of 20-25 as JSON text (the page may be behind an ssh tunnel).
        cells = cells[(np.abs(cells) < 32767).all(axis=1)][:LANDMARK_MAX]
        packed = base64.b64encode(cells.astype('<i2').tobytes()).decode()
        with self.lock:
            number = self.landmarks.get(topic, (0, None))[0] + 1
            self.landmarks[topic] = (number, packed)

    def state(self, since, have_landmarks):
        """Trajectory points after the given stamps, landmarks if changed (incremental polling)."""
        with self.lock:
            paths = {}
            for topic, pts in self.paths.items():
                after = since.get(topic, -1.0)
                paths[topic] = [p for p in pts if p[0] > after] if after >= 0 else list(pts)
            landmarks = {t: {'number': n, 'cell': LANDMARK_CELL_M, 'cells': packed}
                         for t, (n, packed) in self.landmarks.items() if have_landmarks.get(t) != n}
            return {'generation': self.generation, 'paths': paths, 'diagnostics': dict(self.diag),
                    'points': sum(len(p) for p in self.paths.values()),
                    'events': list(self.events), 'landmarks': landmarks,
                    'loop_closures': list(self.loop_closures)}


class RosBridge(Node):

    def __init__(self, preview_width=320):
        super().__init__('pq_console')
        self.preview_width = preview_width
        # Subscriptions read on request: this node is never spun by an executor.
        self.lazy = rclpy.create_node('pq_console_latest')
        self.lazy_lock = threading.Lock()
        self.landmark_subs = {}   # topic -> (Latest, last take time)
        self.lock = threading.Lock()
        self.graph = {'nodes': [], 'topics': []}
        self.measured = {}        # topic -> (Meter, subscription, until)
        self.previews = {}        # topic -> Preview
        self.slam = Slam()
        self.slam_subs = {}       # topic -> subscription
        self.diagnostics = {}     # status name -> dict (/diagnostics, e.g. RealSense driver)
        # Best effort: compatible with reliable and best-effort publishers (the player /clock).
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics,
                                 qos_profile_sensor_data)
        self.clock = None         # (bag time s, wall time) of the last /clock of the player
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
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
            previews = {k: p.latest for k, p in self.previews.items()}
        for t in graph['topics']:
            if t['name'] in meters:
                t.update(meters[t['name']].stats())
                t['measuring'] = True
            elif t['name'] in previews:
                t.update(hz=previews[t['name']].rate(), kbps=None, age_s=None, measuring=True)
        return graph

    def connections(self, max_age=2.0):
        """Publishers and subscriptions of every node: {node: {'pub': [...], 'sub': [...]}}."""
        cache = getattr(self, '_connections', None)
        if cache and time.time() - cache[0] < max_age:
            return cache[1]
        out = {}
        for name, ns in self.get_node_names_and_namespaces():
            full = f'{ns.rstrip("/")}/{name}'
            try:
                pub = self.get_publisher_names_and_types_by_node(name, ns)
                sub = self.get_subscriber_names_and_types_by_node(name, ns)
            except Exception:   # noqa: B902 - node gone meanwhile
                continue
            keep = lambda items: sorted({(t, ty[0] if ty else '') for t, ty in items  # noqa: E731
                                         if '/_buf/' not in t})
            entry = out.setdefault(full, {'pub': set(), 'sub': set()})
            entry['pub'].update(keep(pub))
            entry['sub'].update(keep(sub))
        out = {k: {'pub': sorted(v['pub']), 'sub': sorted(v['sub'])} for k, v in out.items()}
        self._connections = (time.time(), out)
        return out

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
            lazy_drop = []
            for topic, p in list(self.previews.items()):
                if now - p.last_viewer > 10.0:
                    lazy_drop.append(p.latest.sub)
                    del self.previews[topic]
        for sub in drop:
            self.destroy_subscription(sub)
        with self.lazy_lock:
            for sub in lazy_drop:
                self.lazy.destroy_subscription(sub)

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
        latest = Latest(self.lazy, self.lazy_lock, CompressedImage if compressed else Image, topic)
        p = Preview(latest, compressed, self.preview_width)
        with self.lock:
            if topic in self.previews:   # created meanwhile by another request
                with self.lazy_lock:
                    self.lazy.destroy_subscription(latest.sub)
                return self.previews[topic]
            self.previews[topic] = p
        return p

    def frames(self, topics, have):
        """Latest frame of each topic: {topic: {seq, info, jpeg (if newer than have)}}."""
        out = {}
        for topic, seen in zip(topics, have):
            try:
                seq, jpeg, info = self.preview(topic).get()
            except ValueError as e:
                out[topic] = {'seq': 0, 'info': {'error': str(e)}}
                continue
            f = {'seq': seq, 'info': info}
            if jpeg is not None and seq != seen:
                f['jpeg'] = base64.b64encode(jpeg).decode()
            out[topic] = f
        return out

    def slam_state(self, since, have_landmarks):
        """SLAM state; the landmark clouds are taken at most once per second."""
        now = time.time()
        for topic, (latest, last) in list(self.landmark_subs.items()):
            if now - last < 1.0:
                continue
            self.landmark_subs[topic] = (latest, now)
            data = latest.take()
            if data is not None:
                try:
                    self.slam.set_landmarks(topic, deserialize_message(data, PointCloud2))
                except Exception as e:   # noqa: B902 - malformed cloud: keep the last one
                    self.get_logger().warning(f'{topic}: {e}')
        return self.slam.state(since, have_landmarks)

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
            elif type_name == 'sensor_msgs/msg/PointCloud2' and LANDMARKS_RE.search(name):
                if name not in self.landmark_subs:
                    self.landmark_subs[name] = (Latest(self.lazy, self.lazy_lock, PointCloud2, name), 0.0)
                self.slam_subs[name] = None
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

    # ---- bag player (rosbag2 services) ----

    def on_clock(self, msg):
        self.clock = (msg.clock.sec + msg.clock.nanosec * 1e-9, time.time())

    def player(self, command, value=None):
        if command == 'pause':
            self._call(Pause, f'{PLAYER}/pause', Pause.Request())
        elif command == 'resume':
            self._call(Resume, f'{PLAYER}/resume', Resume.Request())
        elif command == 'rate':
            if not self._call(SetRate, f'{PLAYER}/set_rate', SetRate.Request(rate=float(value))).success:
                raise ValueError(f'rate {value} refused')
        elif command == 'seek':
            t = float(value)
            req = Seek.Request()
            req.time.sec, req.time.nanosec = int(t), int(round((t - int(t)) * 1e9))
            if not self._call(Seek, f'{PLAYER}/seek', req).success:
                raise ValueError('seek refused (time outside the bag)')
        else:
            raise ValueError(f'unknown player command {command}')

    # ---- parameters of other nodes ----

    def node_parameters(self, node):
        """All parameters of a node: name, type, value, read-only, description, range."""
        res = self._call(ListParameters, f'{node}/list_parameters', ListParameters.Request())
        names = sorted(res.result.names)
        descs = self._call(DescribeParameters, f'{node}/describe_parameters',
                           DescribeParameters.Request(names=names)).descriptors
        values = self._call(GetParameters, f'{node}/get_parameters',
                            GetParameters.Request(names=names)).values
        out = []
        for name, d, v in zip(names, descs, values):
            item = {'name': name, 'type': PARAM_TYPES.get(v.type, 'unset'), 'value': param_value(v),
                    'read_only': bool(d.read_only), 'description': d.description}
            if d.integer_range:
                r = d.integer_range[0]
                item.update(min=r.from_value, max=r.to_value, step=r.step)
            elif d.floating_point_range:
                r = d.floating_point_range[0]
                item.update(min=r.from_value, max=r.to_value, step=r.step)
            out.append(item)
        return out

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
        return param_value(res.values[0]) if res.values else None

    def get_param_typed(self, node, name):
        res = self._call(GetParameters, f'{node}/get_parameters', GetParameters.Request(names=[name]))
        if not res.values or res.values[0].type == ParameterType.PARAMETER_NOT_SET:
            raise ValueError(f'{node} has no parameter {name}')
        return res.values[0].type, param_value(res.values[0])

    def set_param(self, node, name, value, type_=None, timeout=3.0):
        """Set a parameter; type_ (ParameterType) converts the value (e.g. from JSON)."""
        v = ParameterValue()
        if type_ is None:
            type_ = (ParameterType.PARAMETER_BOOL if isinstance(value, bool) else
                     ParameterType.PARAMETER_INTEGER if isinstance(value, int) else
                     ParameterType.PARAMETER_DOUBLE if isinstance(value, float) else
                     ParameterType.PARAMETER_STRING)
        v.type = type_
        if type_ == ParameterType.PARAMETER_BOOL:
            v.bool_value = value if isinstance(value, bool) else str(value).lower() in ('1', 'true')
        elif type_ == ParameterType.PARAMETER_INTEGER:
            v.integer_value = int(round(float(value)))
        elif type_ == ParameterType.PARAMETER_DOUBLE:
            v.double_value = float(value)
        elif type_ == ParameterType.PARAMETER_STRING:
            v.string_value = str(value)
        else:
            raise ValueError(f'{name}: parameters of this type cannot be set from the console')
        req = SetParameters.Request(parameters=[Parameter(name=name, value=v)])
        res = self._call(SetParameters, f'{node}/set_parameters', req, timeout)
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
