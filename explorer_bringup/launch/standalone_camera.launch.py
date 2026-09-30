# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from explorer_bringup.launch.optional import (
    declare_camera_node,
)
from explorer_bringup.launch.optional_parameters import (
    declare_parameter_list_camera_settings,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def _declare_arguments():
    return [
        # Override the parameter responsible for launching camera node.
        DeclareLaunchArgument(
            name="use_camera",
            default_value="True",
            description="If the camera node is launched",
        ),
        *declare_parameter_list_camera_settings(),
    ]


def generate_launch_description():
    # Initialize Arguments
    declared_arguments = _declare_arguments()

    camera_node = declare_camera_node()

    nodes = [camera_node]

    return LaunchDescription(declared_arguments + nodes)
