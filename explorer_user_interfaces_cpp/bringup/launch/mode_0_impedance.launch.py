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
    declare_node_trajectory_controller_spawner,
)
from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.optional import declare_joy_node
from explorer_bringup.launch.optional_parameters import (
    declare_parameter_input_device,
    declare_parameter_list_web_gui_settings,
)
from explorer_bringup.launch.shared import (
    declare_command_node,
    declare_input_integrator_node,
    declare_output_integrator_node,
    declare_web_gui_node,
)
from explorer_bringup.launch.shared_parameters import (
    CONTROLLER_CONFIG_TYPE,
)
from explorer_bringup.launch.simulation import (
    declare_simulation_node_group,
)
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def _declare_arguments(robot_controller_config: CONTROLLER_CONFIG_TYPE):
    return [
        *declare_simulation_argument_list(
            robot_controller_config=robot_controller_config
        ),
        *declare_hardware_argument_list(
            robot_controller_config=robot_controller_config
        ),
        declare_parameter_input_device(),
        # TODO unused ?
        DeclareLaunchArgument(
            "force_deploy",
            default_value="true",
            description="Force robot deployment to rest position before enabling any other control",
        ),
        *declare_parameter_list_web_gui_settings(),
    ]


def generate_launch_description():
    # Use default robot controller config
    robot_controller_config = "custom_controller"
    controller_position_topic_name = "/explorer_custom_controller/position/commands"

    # Initialize Arguments
    declared_arguments = _declare_arguments(
        robot_controller_config=robot_controller_config
    )

    input_integrator_node = declare_input_integrator_node()
    output_integrator_node = declare_output_integrator_node(
        controller_position_topic_name=controller_position_topic_name
    )
    command_node = declare_command_node(
        default_controller_name="explorer_custom_controller",
        default_controller_position_topic_name=controller_position_topic_name,
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

    robot_controller_list = [
        *declare_custom_controller_spawner(
            robot_controller_config=robot_controller_config
        ),
        declare_node_trajectory_controller_spawner(),
    ]

    robot_simulation = declare_simulation_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=True,
        controller_position_topic_name=controller_position_topic_name,
        qp_solving_post_start_list=[
            input_integrator_node,
            output_integrator_node,
            command_node,
        ],
    )

    robot_hardware = declare_hardware_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=True,
        controller_position_topic_name=controller_position_topic_name,
        qp_solving_post_start_list=[
            input_integrator_node,
            output_integrator_node,
            command_node,
        ],
        robot_controller_config_type=robot_controller_config,
    )

    joy_node = declare_joy_node()
    web_gui_node = declare_web_gui_node()

    nodes = [
        robot_simulation,
        robot_hardware,
        joy_node,
        web_gui_node,
    ]

    return LaunchDescription([*declared_arguments, *nodes])
