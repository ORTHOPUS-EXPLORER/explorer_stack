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
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import Action
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from explorer_bringup.launch.optional_parameters import (
    get_parameter_web_gui_host,
    get_parameter_web_gui_mode_config_path,
    get_parameter_web_gui_port,
)
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


def declare_qp_solving_node_list(controller_position_topic_name: str, qp_solving_post_start_list: List[Action]):
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
            {
                "use_sim_time": get_parameter_use_sim_time(),
                "controller_position_topic_name": controller_position_topic_name,
            },
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
            {
                "use_sim_time": get_parameter_use_sim_time(),
                "controller_position_topic_name": controller_position_topic_name,
            },
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


def declare_input_integrator_node(output: str = "log") -> Node:
    return Node(
        package="explorer_controllers",
        executable="input_integrator",
        name="input_integrator",
        output=output,
        parameters=[{"use_sim_time": get_parameter_use_sim_time()}],
    )


def declare_output_integrator_node(
    controller_position_topic_name: str, output: str = "log"
) -> Node:
    return Node(
        package="explorer_controllers",
        executable="output_integrator",
        name="output_integrator",
        output=output,
        parameters=[
            {
                "use_sim_time": get_parameter_use_sim_time(),
                "controller_position_topic_name": controller_position_topic_name,
            }
        ],
    )


def declare_command_node(
    default_controller_name: str,
    default_controller_position_topic_name: str,
    output: str = "screen",
    remappings: List = [],
) -> Node:
    ## It was like this before refactor but it's weird mapping trajectory / force_deploy
    trajectory = LaunchConfiguration("force_deploy")
    pkg_share = get_package_share_directory("explorer_user_interfaces_cpp")
    yaml_file_path = os.path.join(pkg_share, "config", "config_mode_0.yaml")
    trajectory_yaml_file_path = os.path.join(
        pkg_share, "config", "config_trajectory.yaml"
    )

    return Node(
        package="explorer_user_interfaces_cpp",
        executable="command_node",
        output=output,
        parameters=[
            {
                "mode_file": yaml_file_path,
                "trajectory_file": trajectory_yaml_file_path,
                "active_trajectory": trajectory,
                "default_controller_name": default_controller_name,
                "default_controller_position_topic_name": default_controller_position_topic_name,
            }
        ],
        remappings=remappings,
    )


def declare_web_gui_node(output: str = "screen") -> Node:
    return Node(
        package="explorer_user_interfaces_web",
        executable="web_gui_node",
        name="web_gui_node",
        parameters=[
            {
                "port": get_parameter_web_gui_port(),
                "host": get_parameter_web_gui_host(),
                "mode_config_path": get_parameter_web_gui_mode_config_path(),
            }
        ],
        output=output,
        emulate_tty=True,
    )
