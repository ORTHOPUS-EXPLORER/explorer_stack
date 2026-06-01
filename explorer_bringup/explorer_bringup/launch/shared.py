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


from pathlib import Path
from typing import List

from launch import Action
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from explorer_bringup.launch.shared_parameters import (
    CONTROLLER_CONFIG_TYPE,
    get_parameter_gui,
    get_parameter_robot_description,
    get_parameter_use_poc2,
    get_parameter_use_sim_time,
)


def declare_node_robot_state_publisher() -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[
            {
                "robot_description": ParameterValue(
                    get_parameter_robot_description(), value_type=str
                )
            },
            {"use_sim_time": get_parameter_use_sim_time()},
        ],
    )


def declare_qp_solving_node_list(qp_solving_post_start_list: List[Action]):
    robot_description = {
        "robot_description": ParameterValue(
            get_parameter_robot_description(), value_type=str
        )
    }

    # Get SRDF via xacro
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("explorer_description"), "urdf", "explorer.srdf"]
            ),
            " ",
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(semantic_content, value_type=str)
    }

    config_POC1 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC1.yaml"]
    )

    config_POC2 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC2.yaml"]
    )

    qp_solving_POC1_node = Node(
        package="explorer_controllers",
        executable="qp_solving",
        parameters=[
            config_POC1,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": get_parameter_use_sim_time()},
        ],
        condition=UnlessCondition(get_parameter_use_poc2()),
    )

    qp_solving_POC2_node = Node(
        package="explorer_controllers",
        executable="qp_solving",
        parameters=[
            config_POC2,
            robot_description,
            robot_description_semantic,
            {"use_sim_time": get_parameter_use_sim_time()},
        ],
        condition=IfCondition(get_parameter_use_poc2()),
    )

    register_event_handlers = []
    if len(qp_solving_post_start_list) > 0:
        register_event_handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=qp_solving_POC1_node,
                    on_start=qp_solving_post_start_list,
                )
            )
        )

        register_event_handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=qp_solving_POC2_node,
                    on_start=qp_solving_post_start_list,
                )
            )
        )

    return [qp_solving_POC1_node, qp_solving_POC2_node, *register_event_handlers]


def declare_rviz_node() -> Node:
    return Node(
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


def get_robot_controller_config_path(controller_type: CONTROLLER_CONFIG_TYPE) -> Path:
    robot_controller_config_map = {
        "controller": "controller",
        "custom_controller": "custom_controller",
    }
    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare("explorer_bringup"),
            "config",
            "explorer_" + robot_controller_config_map[controller_type] + ".yaml",
        ]
    )
    return controller_config_path
