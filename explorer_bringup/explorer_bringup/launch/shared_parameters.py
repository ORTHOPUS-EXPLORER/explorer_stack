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

from typing import List, Literal

from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

CONTROLLER_CONFIG_TYPE = Literal["controller", "custom_controller"]


def declare_shared_argument_list(
    use_actuator_interface: bool = True,
    robot_controller_config: CONTROLLER_CONFIG_TYPE = "controller",
) -> List[DeclareLaunchArgument]:
    """Declare all shared arguments to be used accross all launch files

    Returns:
        List[DeclareLaunchArgument]: List of arguments declared
    """
    return [
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="CAN Port for VESC Communication",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RVIZ2 and the Gazebo client automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "host_id",
            default_value="45",
            description="Host CAN ID for VESC Communication",
        ),
        DeclareLaunchArgument(
            "simulation",
            default_value="false",
            description="If true, use simulation (Gazebo), if false use real hardware",
        ),
        DeclareLaunchArgument(
            "use_actuator_interface",
            default_value=str(use_actuator_interface).lower(),
            description="Use VESCInterface to control the robot. Set to false for simulation",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock",
        ),
        DeclareLaunchArgument(
            "use_POC2",
            default_value="true",
            description="Use POC2 urdf",
        ),
        DeclareLaunchArgument(
            "robot_description_param",
            default_value=Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("explorer_description"),
                            "urdf",
                            "explorer.urdf.xacro",
                        ]
                    ),
                    " can_port:=",
                    get_parameter_can_port(),
                    " host_id:=",
                    get_parameter_host_id(),
                    " simulation:=",
                    get_parameter_simulation(),
                    " use_actuator_interface:=",
                    str(use_actuator_interface).lower(),
                    " use_POC2:=",
                    get_parameter_use_poc2(),
                    " robot_controller_config:=",
                    robot_controller_config,
                ]
            ),
            description="Robot description (URDF) evaluated from xacro",
        ),
    ]


def get_parameter_actuator_interface() -> LaunchConfiguration:
    """Get ros2 parameter "use_actuator_interface".

    Returns:
        LaunchConfiguration: use_actuator_interface
    """
    return LaunchConfiguration("use_actuator_interface")


def get_parameter_can_port() -> LaunchConfiguration:
    """Get ros2 parameter "can_port".

    Returns:
        LaunchConfiguration: can_port
    """
    return LaunchConfiguration("can_port")


def get_parameter_gui() -> LaunchConfiguration:
    """Get ros2 parameter "gui".

    Returns:
        LaunchConfiguration: gui
    """
    return LaunchConfiguration("gui")


def get_parameter_host_id() -> LaunchConfiguration:
    """Get ros2 parameter "host_id".

    Returns:
        LaunchConfiguration: host_id
    """
    return LaunchConfiguration("host_id")


def get_parameter_robot_description() -> LaunchConfiguration:
    """Get ros2 parameter "robot_description_param".

    Returns:
        LaunchConfiguration: robot_description_param
    """
    return LaunchConfiguration("robot_description_param")


def get_parameter_simulation() -> LaunchConfiguration:
    """Get ros2 parameter "simulation".

    Returns:
        LaunchConfiguration: simulation
    """
    return LaunchConfiguration("simulation")


def get_parameter_use_sim_time() -> LaunchConfiguration:
    """Get ros2 parameter "use_sim_time".

    Returns:
        LaunchConfiguration: use_sim_time
    """
    return LaunchConfiguration("use_sim_time")


def get_parameter_use_poc2() -> LaunchConfiguration:
    """Get ros2 parameter "use_POC2".

    Returns:
        LaunchConfiguration: use_POC2
    """
    return LaunchConfiguration("use_POC2")
