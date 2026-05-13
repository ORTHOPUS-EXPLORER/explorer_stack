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

from typing import List

from launch import Action
from launch.actions import GroupAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from explorer_bringup.launch.shared import (
    declare_node_forward_position_controller_spawner,
    declare_node_joint_state_broadcaster_spawner,
    declare_node_robot_state_publisher,
    declare_node_trajectory_controller_spawner,
    declare_qp_solving_node_list,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_gui,
    get_parameter_simulation,
    get_parameter_use_sim_time,
)


def declare_node_controller_manager_control_node() -> Node:
    """Generate controller manager control node (only needed in HW, gazebo launch it automatically in simulation)

    Returns:
        Node:
    """
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("explorer_bringup"),
                    "config",
                    "explorer_controller.yaml",
                ]
            )
        ],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )


def declare_hardware_node_list(
    launch_qp_solving: bool, qp_solving_post_start_list: List[Action]
) -> GroupAction:
    # Nodes
    controller_manager_node = declare_node_controller_manager_control_node()
    joint_state_broadcaster_spawner = declare_node_joint_state_broadcaster_spawner()
    trajectory_controller_spawner = declare_node_trajectory_controller_spawner()
    forward_position_robot_controller_spawner = (
        declare_node_forward_position_controller_spawner()
    )
    robot_state_publisher = declare_node_robot_state_publisher()
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
                    forward_position_robot_controller_spawner,
                    trajectory_controller_spawner,
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="log",
                        arguments=[
                            "-d",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("explorer_description"),
                                    "rviz",
                                    "view_robot.rviz",
                                ]
                            ),
                        ],
                        condition=IfCondition(get_parameter_gui()),
                        parameters=[{"use_sim_time": get_parameter_use_sim_time()}],
                    ),
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
