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

from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.optional import (
    declare_spacenav_node_group,
)
from explorer_bringup.launch.optional_parameters import (
    declare_parameter_spacenav,
)
from explorer_bringup.launch.shared_parameters import get_parameter_use_sim_time
from explorer_bringup.launch.simulation import declare_simulation_node_group
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch_ros.actions import Node


def _declare_arguments():
    return [
        *declare_simulation_argument_list(),
        *declare_hardware_argument_list(),
        declare_parameter_spacenav()
    ]

def generate_launch_description():
    # Initialize Arguments
    declared_arguments = _declare_arguments()

    # Nodes
    input_integrator_node = Node(
        package="explorer_controllers",
        executable="input_integrator",
        name="input_integrator",
        parameters=[
            {'use_sim_time': get_parameter_use_sim_time()}
        ],
    )

    output_integrator_node = Node(
        package="explorer_controllers",
        executable="output_integrator",
        name="output_integrator",
        parameters=[
            {'use_sim_time': get_parameter_use_sim_time()}
        ],
    )

    robot_simulation = declare_simulation_node_group(
        launch_qp_solving=True,
        qp_solving_post_start_list=[input_integrator_node, output_integrator_node]
    )

    robot_hardware = declare_hardware_node_group(
        launch_qp_solving=True,
        qp_solving_post_start_list=[input_integrator_node, output_integrator_node]
    )

    spacenav_node_group = declare_spacenav_node_group()

    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_armcontrol',
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        spacenav_node_group,
        gui_control_node
    ]

    return LaunchDescription(declared_arguments + nodes)
