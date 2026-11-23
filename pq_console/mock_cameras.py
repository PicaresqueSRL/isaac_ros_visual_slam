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
Mock RealSense driver nodes (/cam1, /cam2) with a subset of the D455 parameters.

For developing pq_console without cameras: the camera settings page and the emitter check work
against these nodes. No images are published (play a bag for the previews).

Usage: mock_cameras.py [cam1 cam2 ...]
"""

import sys

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# name: (default, description, range (min, max, step) or None, read only)
PARAMS = {
    'enable_infra1': (True, 'Enable IR1 stream', None, False),
    'enable_infra2': (True, 'Enable IR2 stream', None, False),
    'enable_color': (True, 'Enable color stream', None, False),
    'enable_depth': (True, 'Enable depth stream', None, False),
    'enable_gyro': (True, 'Enable gyro', None, False),
    'enable_accel': (True, 'Enable accel', None, False),
    'align_depth.enable': (False, 'Publish depth aligned to color', None, False),
    'depth_module.infra_profile': ('848x480x30', 'Available: 1280x720x30, 848x480x30, 640x480x30',
                                   None, False),
    'depth_module.depth_profile': ('848x480x30', 'Available: 1280x720x30, 848x480x30', None, False),
    'rgb_camera.color_profile': ('640x480x30', 'Available: 1280x720x30, 848x480x30, 640x480x30',
                                 None, False),
    'depth_module.emitter_enabled': (0, 'Emitter: 0 off, 1 on, 2 auto', (0, 2, 1), False),
    'depth_module.laser_power': (150.0, 'Laser power', (0.0, 360.0, 30.0), False),
    'depth_module.enable_auto_exposure': (True, 'IR auto exposure', None, False),
    'depth_module.exposure': (8500, 'IR exposure (us)', (1, 200000, 1), False),
    'depth_module.gain': (16, 'IR gain', (16, 248, 1), False),
    'depth_module.hdr_enabled': (False, 'HDR', None, False),
    'rgb_camera.enable_auto_exposure': (True, 'RGB auto exposure', None, False),
    'rgb_camera.exposure': (166, 'RGB exposure', (1, 10000, 1), False),
    'rgb_camera.gain': (64, 'RGB gain', (0, 128, 1), False),
    'rgb_camera.auto_exposure_priority': (False, 'Keep the frame rate', None, False),
    'rgb_camera.enable_auto_white_balance': (True, 'Auto white balance', None, False),
    'rgb_camera.white_balance': (4600.0, 'White balance', (2800.0, 6500.0, 10.0), False),
    'serial_no': ('000000000000', 'Serial number', None, True),
    'diagnostics_period': (0.0, 'Diagnostics period (s)', None, True),
}


class MockCamera(Node):

    def __init__(self, name):
        super().__init__(name)
        for pname, (default, desc, rng, ro) in PARAMS.items():
            d = ParameterDescriptor(description=desc, read_only=ro)
            if rng and isinstance(default, int) and not isinstance(default, bool):
                d.integer_range = [IntegerRange(from_value=rng[0], to_value=rng[1], step=rng[2])]
            elif rng:
                d.floating_point_range = [FloatingPointRange(from_value=rng[0], to_value=rng[1],
                                                             step=rng[2])]
            self.declare_parameter(pname, default, d)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    for name in sys.argv[1:] or ['cam1', 'cam2']:
        executor.add_node(MockCamera(name))
    executor.spin()


if __name__ == '__main__':
    main()
