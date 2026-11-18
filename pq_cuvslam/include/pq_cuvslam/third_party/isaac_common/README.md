# isaac_common (vendored)

Header-only message buffering and synchronization utilities, copied from
the `isaac_common` package of
[NVIDIA-ISAAC-ROS/isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common),
branch `release-5.1`, commit `fddf2d2eb03aeeca1ea8149293249011987fd356`.
License: Apache 2.0 (see the repository `LICENSE`).

Vendored so that the node builds on a standard ROS 2 installation without
Isaac ROS packages.

Changes from upstream:

- include paths moved under `pq_cuvslam/third_party/`;
- include guards renamed to avoid clashes with an installed `isaac_common`.

The C++ namespace (`nvidia::isaac_common::messaging`) is unchanged. The upstream unit
tests are in `test/test_message_buffer.cpp` and
`test/test_message_stream_synchronizer.cpp`.
