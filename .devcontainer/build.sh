#!/bin/bash

# Immediately catch all errors
set -eo pipefail

# Delete install folder if already exist and was built in "isolated" layout
if [[ -f "install/.colcon_install_layout" && ! -z $(grep "isolated" "install/.colcon_install_layout") ]]; then rm -r install; fi

# Source ROS / colcon install files automatically in bashrc
if [[ -z $(grep "source /opt/ros/" ~/.bashrc) ]]; then echo 'source /opt/ros/${ROS_DISTRO}/setup.bash && source install/setup.bash' >> ~/.bashrc; fi

# Source ROS for remaining commands
source /opt/ros/${ROS_DISTRO}/setup.bash
[[ -f "${ROS_WS}/install/setup.bash" ]] && source "${ROS_WS}/install/setup.bash"

# Build workspace as non root user (prevent permission issues)
CCACHE_DIR=/root/.ccache colcon build --symlink-install --mixin debug ccache compile-commands