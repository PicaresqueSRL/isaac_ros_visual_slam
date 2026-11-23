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
"""Configuration of pq_console (one YAML file per machine, see config.example.yaml)."""

from dataclasses import dataclass, field
import os

import yaml


@dataclass
class CameraConfig:
    name: str
    serial: str


@dataclass
class Config:
    listen_host: str = '127.0.0.1'
    listen_port: int = 8080
    data_dir: str = ''              # bags/, runs/, rerun/ (host path, mounted at the same path)
    bags_dir: str = ''              # default data_dir/bags
    ros_domain_id: int = 7
    dds_transports: str = 'LARGE_DATA'
    camera_image: str = ''          # RealSense driver, recorder and player
    slam_image: str = 'pq_cuvslam:dev'
    gpu_args: list = field(default_factory=lambda: ['--runtime', 'nvidia'])
    slam_extra_args: list = field(default_factory=list)
    install_dir: str = ''           # pq_cuvslam install mounted over /ws/install (local build)
    tools_dir: str = ''             # tools/ of the repo (to_rerun.py, rerun_layout.py)
    pylib_dir: str = ''             # rerun-sdk folder for rerun_layout.py; empty: no .rrd
    dual_cam1_profile: str = 'cam1_stereo_imu'
    cameras: list = field(default_factory=list)
    camera_params: dict = field(default_factory=dict)
    host_uid: int = 0
    host_gid: int = 0

    @property
    def runs_dir(self):
        return os.path.join(self.data_dir, 'runs')

    @property
    def rerun_dir(self):
        return os.path.join(self.data_dir, 'rerun')


def load(path):
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    c = Config()
    listen = str(raw.pop('listen', f'{c.listen_host}:{c.listen_port}'))
    host, _, port = listen.rpartition(':')
    c.listen_host, c.listen_port = host or c.listen_host, int(port)
    cameras = raw.pop('cameras', []) or []
    c.cameras = [CameraConfig(str(x['name']), str(x['serial'])) for x in cameras]
    for key, value in raw.items():
        if not hasattr(c, key):
            raise ValueError(f'{path}: unknown key {key}')
        setattr(c, key, value)
    if not c.data_dir:
        raise ValueError(f'{path}: data_dir is required')
    c.bags_dir = c.bags_dir or os.path.join(c.data_dir, 'bags')
    c.host_uid = int(os.environ.get('HOST_UID', c.host_uid or 0))
    c.host_gid = int(os.environ.get('HOST_GID', c.host_gid or 0))
    return c
