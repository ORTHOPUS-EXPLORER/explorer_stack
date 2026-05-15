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

from typing import Callable, Dict, List, Literal

from launch import Action
from launch.actions import GroupAction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from explorer_bringup.launch.controller_manager_spawner import (
    declare_node_forward_position_controller_spawner,
    declare_node_joint_state_broadcaster_spawner,
    declare_node_trajectory_controller_spawner,
)
from explorer_bringup.launch.shared import (
    declare_node_robot_state_publisher,
    declare_qp_solving_node_list,
    declare_rviz_node,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_simulation,
)

ALLOWED_CONTROLLER_TYPE = Literal["default", "multi_intf"]


def _declare_node_controller_manager_control_node(
    controller_type: ALLOWED_CONTROLLER_TYPE,
) -> Node:
    """Declare controller manager control node (only needed in HW, gazebo launch it automatically in simulation)

    Returns:
        Node:
    """
    robot_controller_config_map = {
        "default": "controller",
        "multi_intf": "multi_intf_controller",
    }
    controller_config = PathJoinSubstitution(
        [
            FindPackageShare("explorer_bringup"),
            "config",
            "explorer_" + robot_controller_config_map[controller_type] + ".yaml",
        ]
    )

    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )


def _declare_command_controller_list(
    controller_type: ALLOWED_CONTROLLER_TYPE,
) -> List[Node]:
    command_controller_generator_map: Dict[ALLOWED_CONTROLLER_TYPE, Callable] = {
        "default": lambda: [
            declare_node_trajectory_controller_spawner(),
            declare_node_forward_position_controller_spawner(),
        ],
        "multi_intf": lambda: [],
    }

    return command_controller_generator_map[controller_type]()


def declare_hardware_node_group(
    launch_qp_solving: bool,
    qp_solving_post_start_list: List[Action] = [],
    controller_manager_type: ALLOWED_CONTROLLER_TYPE = "default",
) -> GroupAction:
    """Declare nodes needed when using hardware

    Returns:
        GroupAction: hardware node list object
    """
    # Nodes
    controller_manager_node = _declare_node_controller_manager_control_node(
        controller_type=controller_manager_type
    )
    joint_state_broadcaster_spawner = declare_node_joint_state_broadcaster_spawner()

    command_controller_list = _declare_command_controller_list(
        controller_type=controller_manager_type
    )

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
                    *command_controller_list,
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
