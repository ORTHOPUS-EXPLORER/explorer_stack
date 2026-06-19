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

from typing import List

from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def declare_parameter_input_device() -> DeclareLaunchArgument:
    """Declare launch parameter 'input_device'

    Returns:
        DeclareLaunchArgument: Argument declared
    """
    return DeclareLaunchArgument(
        "input_device",
        default_value="movis",
        choices=["movis", "xbox"],
        description="Input device type",
    )


def declare_parameter_spacenav() -> DeclareLaunchArgument:
    """Declare launch parameter 'spacenav'

    Returns:
        DeclareLaunchArgument: Argument declared
    """
    return DeclareLaunchArgument(
        name="spacenav",
        default_value="True",
        description="If the spacenav 3D mouse is used",
    )


def declare_parameter_list_web_gui_settings() -> List[DeclareLaunchArgument]:
    """Declare launch parameters related to web gui settings: 'host', 'port', 'mode_config_path'

    Returns:
        List [ DeclareLaunchArgument ]: Argument list declared
    """
    return [
        DeclareLaunchArgument(
            "port", default_value="8080", description="Port for the web GUI server"
        ),
        DeclareLaunchArgument(
            "host",
            default_value="0.0.0.0",
            description="Host address for the web GUI server",
        ),
        DeclareLaunchArgument(
            "mode_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("explorer_user_interfaces_cpp"),
                    "config",
                    "config_mode_0.yaml",
                ]
            ),
            description="Path to the mode configuration YAML file",
        ),
    ]


## ---------- Parameters getter ----------


def get_parameter_input_device() -> LaunchConfiguration:
    """Get ros2 parameter "input_device".

    Returns:
        LaunchConfiguration: input_device
    """
    return LaunchConfiguration("input_device")


def get_parameter_spacenav() -> LaunchConfiguration:
    """Get ros2 parameter "spacenav".

    Returns:
        LaunchConfiguration: spacenav
    """
    return LaunchConfiguration("spacenav")


def get_parameter_web_gui_host() -> LaunchConfiguration:
    """Get ros2 parameter "host".

    Returns:
        LaunchConfiguration: host
    """
    return LaunchConfiguration("host")


def get_parameter_web_gui_port() -> LaunchConfiguration:
    """Get ros2 parameter "port".

    Returns:
        LaunchConfiguration: port
    """
    return LaunchConfiguration("port")


def get_parameter_web_gui_mode_config_path() -> LaunchConfiguration:
    """Get ros2 parameter "mode_config_path".

    Returns:
        LaunchConfiguration: mode_config_path
    """
    return LaunchConfiguration("mode_config_path")
