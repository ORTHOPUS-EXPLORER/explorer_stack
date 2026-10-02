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
    declare_node_forward_position_controller_spawner,
    declare_node_gripper_controller_spawner,
    declare_node_qcontrol_controller_spawner,
    declare_node_trajectory_controller_spawner,
)
from explorer_bringup.launch.hardware import (
    declare_hardware_node_group,
)
from explorer_bringup.launch.hardware_parameters import (
    declare_hardware_argument_list,
)
from explorer_bringup.launch.optional import (
    declare_spacenav_node_group,
)
from explorer_bringup.launch.optional_parameters import (
    declare_parameter_spacenav,
)
from explorer_bringup.launch.shared import (
    declare_input_integrator_node,
    declare_output_integrator_node,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_gui,
)
from explorer_bringup.launch.simulation import (
    declare_simulation_node_group,
)
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node


def _declare_arguments(robot_controller_config: str):
    return [
        *declare_simulation_argument_list(
            robot_controller_config=robot_controller_config
        ),
        *declare_hardware_argument_list(
            robot_controller_config=robot_controller_config
        ),
        declare_parameter_spacenav(),
    ]


def generate_launch_description():
    robot_controller_config = "explorer_controller"
    # Used only if use_qp_inria=false
    controller_position_topic_name = "/forward_position_controller/commands"

    # Initialize Arguments
    declared_arguments = _declare_arguments(
        robot_controller_config=robot_controller_config
    )

    robot_controller_list = [
        declare_node_forward_position_controller_spawner(),
        declare_node_trajectory_controller_spawner(),
        declare_node_gripper_controller_spawner(),
        declare_node_qcontrol_controller_spawner(),
    ]
    robot_simulation = declare_simulation_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=True,
        controller_position_topic_name=controller_position_topic_name,
    )

    robot_hardware = declare_hardware_node_group(
        robot_controller_config_file=robot_controller_config,
        robot_controller_list=robot_controller_list,
        launch_qp_solving=True,
        controller_position_topic_name=controller_position_topic_name,
    )

    spacenav_node_group = declare_spacenav_node_group()

    input_integrator_node = declare_input_integrator_node()
    output_integrator_node = declare_output_integrator_node(
        controller_position_topic_name=controller_position_topic_name
    )

    gui_control_node = Node(
        package="explorer_user_interfaces",
        executable="rqt_armcontrol",
        condition=IfCondition(get_parameter_gui()),
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        spacenav_node_group,
        gui_control_node,
        input_integrator_node,
        output_integrator_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
