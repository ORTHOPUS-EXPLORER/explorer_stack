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
from typing import List

from ament_index_python.packages import get_package_share_directory
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

from explorer_bringup.launch.controller_manager_spawner import (
    declare_node_joint_state_broadcaster_spawner,
)
from explorer_bringup.launch.shared import (
    declare_node_robot_state_publisher,
    declare_qp_solving_node_list,
    declare_rviz_node,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_gui,
    get_parameter_simulation,
)
from explorer_bringup.launch.simulation_parameters import get_parameter_world_file


def _declare_gazebo_node_list(gazebo_on_exit_node_list: List) -> List[Action]:
    """Generate gazebo node(s).

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

    bridge_params = os.path.join(
        get_package_share_directory("explorer_bringup"),
        "config",
        "bridge.yaml",
    )

    return [
        gazebo_sim_init,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_sim_init,
                on_exit=gazebo_on_exit_node_list,
            )
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "--ros-args",
                "-p",
                "config_file:=" + bridge_params,
            ],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={
                "gz_args": [
                    "-r -v4 ",
                    ## Start in headless mode if GUI disabled
                    PythonExpression(
                        ['"" if "', get_parameter_gui(), '" == "true" else " -s "']
                    ),
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
    ]


def declare_simulation_node_group(
    robot_controller_list: List,
    launch_qp_solving: bool,
    qp_solving_post_start_list: List[Action] = [],
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
    robot_state_publisher = declare_node_robot_state_publisher()
    rviz_node = declare_rviz_node()
    gazebo_on_exit_node_list = [joint_state_broadcaster_spawner, *robot_controller_list]
    gazebo_node_list = _declare_gazebo_node_list(
        gazebo_on_exit_node_list=gazebo_on_exit_node_list,
    )

    # Event handlers
    event_handler_list = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
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
