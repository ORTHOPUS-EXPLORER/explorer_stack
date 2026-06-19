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

import os

from ament_index_python.packages import get_package_share_directory
from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.optional import declare_joy_node
from explorer_bringup.launch.optional_parameters import (
    declare_parameter_input_device,
    declare_parameter_list_web_gui_settings,
    get_parameter_web_gui_host,
    get_parameter_web_gui_mode_config_path,
    get_parameter_web_gui_port,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_use_sim_time,
)
from explorer_bringup.launch.simulation import (
    declare_simulation_node_group,
)
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _declare_arguments():
    return [
        *declare_simulation_argument_list(),
        *declare_hardware_argument_list(),
        declare_parameter_input_device(),
        # TODO unused ?
        DeclareLaunchArgument(
            "force_deploy",
            default_value="true",
            description="Force robot deployment to rest position before enabling any other control",
        ),
    *declare_parameter_list_web_gui_settings()
    ]


def generate_launch_description():
    # Initialize Arguments
    trajectory = LaunchConfiguration("force_deploy")
    declared_arguments = _declare_arguments()

    input_integrator_node = Node(
        package="explorer_controllers",
        executable="input_integrator",
        name="input_integrator",
        parameters=[{"use_sim_time": get_parameter_use_sim_time()}],
    )

    output_integrator_node = Node(
        package="explorer_controllers",
        executable="output_integrator",
        name="output_integrator",
        parameters=[{"use_sim_time": get_parameter_use_sim_time()}],
    )

    pkg_share = get_package_share_directory("explorer_user_interfaces_cpp")
    yaml_file_path = os.path.join(pkg_share, "config", "config_mode_0.yaml")
    trajectory_yaml_file_path = os.path.join(
        pkg_share, "config", "config_trajectory.yaml"
    )

    command_node = Node(
        package="explorer_user_interfaces_cpp",
        executable="command_node",
        output="screen",
        parameters=[
            {
                "mode_file": yaml_file_path,
                "trajectory_file": trajectory_yaml_file_path,
                "active_trajectory": trajectory,
            }
        ],
        remappings=[
            (
                "/command_node/cartesian_velocity_command",
                "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity",
            ),
            (
                "/command_node/gripper_velocity_command",
                "/explorer_user_interfaces/rqt_armcontrol/input_gripper_velocity",
            ),
        ],
    )

    robot_simulation = declare_simulation_node_group(
        launch_qp_solving=True,
        qp_solving_post_start_list=[
            input_integrator_node,
            output_integrator_node,
            command_node,
        ],
    )

    robot_hardware = declare_hardware_node_group(
        launch_qp_solving=True,
        qp_solving_post_start_list=[
            input_integrator_node,
            output_integrator_node,
            command_node,
        ],
    )

    joy_node = declare_joy_node()

    web_gui_node = Node(
        package="explorer_user_interfaces_web",
        executable="web_gui_node",
        name="web_gui_node",
        parameters=[
            {
                "port": get_parameter_web_gui_port(),
                "host": get_parameter_web_gui_host(),
                "mode_config_path": get_parameter_web_gui_mode_config_path(),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        joy_node,
        web_gui_node,
    ]

    return LaunchDescription([*declared_arguments, *nodes])
