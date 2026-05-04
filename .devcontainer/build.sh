#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [[ -f "install/setup.bash" ]]; then
    source "install/setup.bash"
fi

# Build workspace as non root user (prevent permission issues)
CCACHE_DIR=/home/orthopus/.ccache /ros_entrypoint.sh colcon build --symlink-install --mixin debug ccache compile-commands