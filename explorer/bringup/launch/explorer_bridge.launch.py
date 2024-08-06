# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Explorer bridge
    explorer_bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_explorer"),
            "config",
            "explorer_vesc.yaml",
        ]
    )

    explorer_bridge = Node(
        package="pyvesc_explorer",
        executable="ros_explorer_bridge",
        parameters=[explorer_bridge_params],
        output="both",
        remappings=[],
        arguments=['--non-interactive']#,'--ros-args'],#, '--log-level', 'DEBUG']
    )

    return LaunchDescription([explorer_bridge])
