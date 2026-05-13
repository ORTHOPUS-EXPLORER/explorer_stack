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
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, PythonExpression
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
from explorer_bringup.launch.simulation_parameters import get_parameter_world_file


def declare_node_list_gazebo(gazebo_on_exit_node_list: List) -> List[Action]:
    """Generate node list related to gazebo.

    Args:
        gazebo_on_exit_node_list (List): List of nodes to launch only after Gazebo is up

    Returns:
        List [ Action ]:
    """
    gazebo_sim_init = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "explorer",
            "-allow_renaming",
            "true",
        ],
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={
                "gz_args": [
                    "-r -v4 ",
                    ## Start in headless mode if GUI disabled
                    PythonExpression([
                        '"" if "',
                        get_parameter_gui(),
                        '" == "true" else " -s "'
                    ]),
                    PathJoinSubstitution(
                        [
                            FindPackageShare("explorer_gazebo"),
                            "worlds",
                            get_parameter_world_file(),
                        ]
                    ),
                ],
                "on_exit_shutdown": "true",
            }.items(),
        ),
        gazebo_sim_init,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_sim_init,
                on_exit=gazebo_on_exit_node_list,
            )
        )
    ]


def declare_simulation_node_list(
    launch_qp_solving: bool, qp_solving_post_start_list: List[Action]
) -> GroupAction:
    """Generate all nodes related to simulation

    Args:
        launch_qp_solving: Enable qp solving node if True
        qp_solving_post_start_list: List of node to start after ProcessStart on qp_solving node (ignored if qp_solving is False)

    Returns:
        GroupAction: GroupAction to add to LaunchDescription
    """
    # Nodes
    joint_state_broadcaster_spawner = declare_node_joint_state_broadcaster_spawner()
    trajectory_controller_spawner = declare_node_trajectory_controller_spawner()
    forward_position_controller_spawner = declare_node_forward_position_controller_spawner()
    robot_state_publisher = declare_node_robot_state_publisher()

    gazebo_on_exit_node_list = [
        joint_state_broadcaster_spawner,
        trajectory_controller_spawner,
        forward_position_controller_spawner,
    ]
    gazebo_node_list = declare_node_list_gazebo(
        gazebo_on_exit_node_list=gazebo_on_exit_node_list
    )

    # Event handlers
    event_handler_list = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
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
                    )
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
        condition=IfCondition(get_parameter_simulation()),
        actions=[robot_state_publisher, *gazebo_node_list, *event_handler_list],
    )
