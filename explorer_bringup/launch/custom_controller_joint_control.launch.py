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

from explorer_bringup.launch.controller_manager_spawner import (
    declare_custom_controller_spawner,
)
from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.shared_parameters import (
    CONTROLLER_CONFIG_TYPE,
    get_parameter_use_sim_time,
)
from explorer_bringup.launch.simulation import declare_simulation_node_group
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _declare_arguments(robot_controller_config: CONTROLLER_CONFIG_TYPE):
    return [
        *declare_simulation_argument_list(
            robot_controller_config=robot_controller_config
        ),
        *declare_hardware_argument_list(
            robot_controller_config=robot_controller_config
        ),
    ]


def generate_launch_description():
    robot_controller_config = "custom_controller"

    # Initialize Arguments
    declared_arguments = _declare_arguments(
        robot_controller_config=robot_controller_config
    )

    joint_output_integrator_node = Node(
        package="explorer_controllers",
        executable="joint_output_integrator",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("explorer_bringup"),
                    "config",
                    "settings_joint_POC2.yaml",
                ]
            ),
            {"use_sim_time": get_parameter_use_sim_time()},
        ],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/explorer_custom_controller/position_commands",
            )
        ],
    )

    robot_controller_list = [
        declare_custom_controller_spawner()
    ]

    robot_simulation = declare_simulation_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
    )

    robot_hardware = declare_hardware_node_group(
        robot_controller_config_type=robot_controller_config,
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
    )

    # Declare GUI controller node
    gui_control_node = Node(
        package="explorer_user_interfaces",
        executable="rqt_jointcontrol",
        remappings=[
            (
                "/explorer_user_interfaces/rqt_jointcontrol/dq_output",
                "/explorer_controllers/qp_solving/dq_output",
            ),
        ],
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        joint_output_integrator_node,
        gui_control_node,
    ]

    return LaunchDescription([*declared_arguments, *nodes])
