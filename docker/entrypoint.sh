#!/bin/bash
# Source ROS 2 and the workspace, then run the given command.
set -e

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source /ws/install/setup.bash

exec "$@"
