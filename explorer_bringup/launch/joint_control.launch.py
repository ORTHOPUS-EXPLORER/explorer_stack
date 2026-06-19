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
from explorer_bringup.launch.optional import declare_joy_node
from explorer_bringup.launch.shared_parameters import (
    get_parameter_use_poc2,
    get_parameter_use_sim_time,
)
from explorer_bringup.launch.simulation import declare_simulation_node_group
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _declare_arguments():
    return [
        *declare_simulation_argument_list(),
        *declare_hardware_argument_list(),
    ]


def generate_launch_description():
    # Initialize Arguments
    declared_arguments = _declare_arguments()

    output_integrator_node = Node(
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
    )

    robot_simulation = declare_simulation_node_group(
        launch_qp_solving=False
    )

    robot_hardware = declare_hardware_node_group(
        launch_qp_solving=False
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

    joy_node = declare_joy_node(unique_device="xbox")

    xbox_gamepad_joint_node = Node(
        package="explorer_input_devices",
        executable="xbox_gamepad_joint",
        remappings=[
            (
                "/explorer_input_devices/xbox_gamepad_joint/dq_output",
                "/explorer_controllers/qp_solving/dq_output",
            ),
        ],
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        output_integrator_node,
        gui_control_node,
        joy_node,
        xbox_gamepad_joint_node,
    ]

    return LaunchDescription([*declared_arguments, *nodes])
