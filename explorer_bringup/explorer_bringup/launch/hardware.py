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

from typing import Callable, Dict, List

from launch import Action
from launch.actions import GroupAction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

from explorer_bringup.launch.controller_manager_spawner import (
    declare_node_forward_position_controller_spawner,
    declare_node_joint_state_broadcaster_spawner,
    declare_node_trajectory_controller_spawner,
)
from explorer_bringup.launch.shared import (
    declare_node_robot_state_publisher,
    declare_qp_solving_node_list,
    declare_rviz_node,
    get_robot_controller_config_path,
)
from explorer_bringup.launch.shared_parameters import (
    CONTROLLER_CONFIG_TYPE,
    get_parameter_simulation,
)


def _declare_node_controller_manager_control_node(
    controller_config_type: CONTROLLER_CONFIG_TYPE,
) -> Node:
    """Declare controller manager control node (only needed in HW, gazebo launch it automatically in simulation)

    Returns:
        Node:
    """
    controller_config_path = get_robot_controller_config_path(controller_config_type)

    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_path],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )


def _declare_command_controller_list(
    controller_type: CONTROLLER_CONFIG_TYPE,
) -> List[Node]:
    command_controller_generator_map: Dict[CONTROLLER_CONFIG_TYPE, Callable] = {
        "controller": lambda: [
            declare_node_trajectory_controller_spawner(),
            declare_node_forward_position_controller_spawner(),
        ],
        "custom_controller": lambda: [],
    }

    return command_controller_generator_map[controller_type]()


def declare_hardware_node_group(
    robot_controller_list: List,
    launch_qp_solving: bool,
    qp_solving_post_start_list: List[Action] = [],
    robot_controller_config_type: CONTROLLER_CONFIG_TYPE = "controller",
) -> GroupAction:
    """Declare nodes needed when using hardware

    Returns:
        GroupAction: hardware node list object
    """
    # Nodes
    controller_manager_node = _declare_node_controller_manager_control_node(
        controller_config_type=robot_controller_config_type
    )
    joint_state_broadcaster_spawner = declare_node_joint_state_broadcaster_spawner()

    robot_state_publisher = declare_node_robot_state_publisher()
    rviz_node = declare_rviz_node()
    nodes = [
        controller_manager_node,
    ]

    # Event handlers
    event_handler_list = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    joint_state_broadcaster_spawner,
                    robot_state_publisher,
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    *robot_controller_list,
                    rviz_node,
                ],
            )
        ),
    ]

    if launch_qp_solving:
        qp_solving_node_list = declare_qp_solving_node_list(
            qp_solving_post_start_list=qp_solving_post_start_list
        )
        event_handler_list.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=qp_solving_node_list,
                )
            ),
        )

    return GroupAction(
        condition=UnlessCondition(get_parameter_simulation()),
        actions=[*nodes, *event_handler_list],
    )
