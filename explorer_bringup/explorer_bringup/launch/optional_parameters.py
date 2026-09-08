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


def declare_parameter_joy_backend() -> DeclareLaunchArgument:
    """Declare launch parameter 'joy_backend'

    Returns:
        DeclareLaunchArgument: Argument declared
    """
    return DeclareLaunchArgument(
        "joy_backend",
        default_value="joy",
        choices=["joy", "joy_linux"],
        description="Joystick driver backend used to publish sensor_msgs/Joy",
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


def declare_parameter_list_camera_settings() -> List[DeclareLaunchArgument]:
    """Declare launch parameters related to camera_ros node settings

    Returns:
        DeclareLaunchArgument: Argument declared
    """
    return [
        DeclareLaunchArgument(
            name="use_camera",
            default_value="False",
            description="Conditional camera node launched",
        ),
        # Camera settings that will be passed to camera_ros
        # https://github.com/christianrauch/camera_ros#static-camera-stream-configuration
        DeclareLaunchArgument(
            name="camera_device",
            default_value="0",
            description="Selects the camera by index or by name, defaults to 0.",
        ),
        DeclareLaunchArgument(
            name="camera_role",
            default_value="viewfinder",
            description="Configures the camera with a StreamRole (choices: raw, still, video, viewfinder), defaults to 'viewfinder'.",
        ),
        DeclareLaunchArgument(
            name="camera_format",
            default_value="",
            description="Selects a PixelFormat that is supported by the camera, defaults to 'auto'.",
        ),
        DeclareLaunchArgument(
            name="camera_width",
            default_value="",
            description="Desired image width, defaults to auto.",
        ),
        DeclareLaunchArgument(
            name="camera_height",
            default_value="",
            description="Desired image height, defaults to auto.",
        ),
        DeclareLaunchArgument(
            name="camera_orientation",
            default_value="0",
            description="Camera orientation in 90 degree steps (possible choices: 0, 90, 180, 270), defaults to '0'.",
        ),
        DeclareLaunchArgument(
            name="camera_info_url",
            default_value="",
            description="URL for a camera calibration YAML file (see Calibration), defaults to '~/.ros/camera_info/$NAME.yaml'.",
        ),
        DeclareLaunchArgument(
            name="camera_use_node_time",
            default_value="false",
            description="use node time instead of sensor timestamp for image message header, defaults to 'false'.",
        ),
    ]


## ---------- Parameters getter ----------


def get_parameter_input_device() -> LaunchConfiguration:
    """Get ros2 parameter "input_device".

    Returns:
        LaunchConfiguration: input_device
    """
    return LaunchConfiguration("input_device")


def get_parameter_joy_backend() -> LaunchConfiguration:
    """Get ros2 parameter "joy_backend".

    Returns:
        LaunchConfiguration: joy_backend
    """
    return LaunchConfiguration("joy_backend")


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


def get_parameter_use_camera() -> LaunchConfiguration:
    """Get ros2 parameter "use_camera".

    Returns:
        LaunchConfiguration: use_camera
    """
    return LaunchConfiguration("use_camera")


def get_parameter_camera_device() -> LaunchConfiguration:
    """Get ros2 parameter "camera_device".

    Returns:
        LaunchConfiguration: camera_device
    """
    return LaunchConfiguration("camera_device")


def get_parameter_camera_role() -> LaunchConfiguration:
    """Get ros2 parameter "camera_role".

    Returns:
        LaunchConfiguration: camera_role
    """
    return LaunchConfiguration("camera_role")


def get_parameter_camera_format() -> LaunchConfiguration:
    """Get ros2 parameter "camera_format".

    Returns:
        LaunchConfiguration: camera_format
    """
    return LaunchConfiguration("camera_format")



def get_parameter_camera_width() -> LaunchConfiguration:
    """Get ros2 parameter "camera_width".

    Returns:
        LaunchConfiguration: camera_width
    """
    return LaunchConfiguration("camera_width")



def get_parameter_camera_height() -> LaunchConfiguration:
    """Get ros2 parameter "camera_height".

    Returns:
        LaunchConfiguration: camera_height
    """
    return LaunchConfiguration("camera_height")



def get_parameter_camera_orientation() -> LaunchConfiguration:
    """Get ros2 parameter "camera_orientation".

    Returns:
        LaunchConfiguration: camera_orientation
    """
    return LaunchConfiguration("camera_orientation")


def get_parameter_camera_info_url() -> LaunchConfiguration:
    """Get ros2 parameter "camera_info_url".

    Returns:
        LaunchConfiguration: camera_info_url
    """
    return LaunchConfiguration("camera_info_url")


def get_parameter_camera_use_node_time() -> LaunchConfiguration:
    """Get ros2 parameter "camera_use_node_time".

    Returns:
        LaunchConfiguration: camera_use_node_time
    """
    return LaunchConfiguration("camera_use_node_time")
