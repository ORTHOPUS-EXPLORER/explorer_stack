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


from typing import Literal

from click.core import Group
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from explorer_bringup.launch.optional_parameters import (
    get_parameter_input_device,
    get_parameter_spacenav,
)


def declare_joy_node(unique_device: Literal["movis", "xbox"] | None = None) -> Node:
    """Generate joy node

    Args:
        unique_device (str | None): If given, take this device config rather than launch 'input_device' parameter

    Returns:
        Node: Declared node
    """
    device_config_map = {
        "movis": "movis_joystick_settings.yaml",
        "xbox": "xbox_gamepad_settings.yaml",
    }

    # Select device config based on unique_device (if provided) (fallback to launch param 'input_device parameter')
    device_yaml_file_path = PathJoinSubstitution(
        [
            FindPackageShare("explorer_input_devices"),
            "config",
            PythonExpression(
                [
                    f'{device_config_map}["',
                    unique_device
                    if unique_device is not None
                    else get_parameter_input_device(),
                    '"]',
                ]
            ),
        ]
    )

    return Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[device_yaml_file_path],
    )


def declare_spacenav_node_group() -> GroupAction:
    spacenav_config = PathJoinSubstitution(
        [FindPackageShare("explorer_input_devices"), "config", "spacenav_settings.yaml"]
    )
    return GroupAction(
        actions=[
            Node(
                package="explorer_input_devices",
                executable="spacenav",
                parameters=[
                    spacenav_config,
                    {"static_rot_deadband": 0.5},
                    {"static_trans_deadband": 0.5},
                ],
            ),
            Node(
                package="spacenav",
                executable="spacenav_node",
                parameters=[
                    {"static_rot_deadband": 0.5},
                    {"static_trans_deadband": 0.5},
                ],
            ),
        ],
        condition=IfCondition(get_parameter_spacenav()),
    )
