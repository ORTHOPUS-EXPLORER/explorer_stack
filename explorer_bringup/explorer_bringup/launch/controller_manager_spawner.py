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


import copy
import os
import tempfile
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from explorer_bringup.launch.shared_parameters import get_parameter_simulation


def declare_node_joint_state_broadcaster_spawner(output: str = "log") -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        output=output,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )


def declare_node_trajectory_controller_spawner(output: str = "log") -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        output=output,
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )


def declare_node_forward_position_controller_spawner(output: str = "log") -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        output=output,
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )


def declare_custom_controller_spawner(
    robot_controller_config: str, output: str = "log"
) -> List[Node]:
    # Python method called at launch time that create a temporary duplicate controller config file with simulation parameter properly set
    def inner_opaque_function(context, robot_controller_config: str):
        config_path = os.path.join(
            get_package_share_directory("explorer_bringup"),
            "config",
            "explorer_" + robot_controller_config + ".yaml",
        )

        # Load existing config
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

        # Modify simulation parameter
        config["explorer_custom_controller"]["ros__parameters"]["simulation"] = bool(
            get_parameter_simulation().perform(context)
        )

        # Write temporary config
        tmp_file = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".yaml",
            delete=False,
        )

        yaml.safe_dump(config, tmp_file)

        # Spawn controller
        spawn_controller = Node(
            package="controller_manager",
            executable="spawner",
            output=output,
            arguments=[
                "explorer_custom_controller",
                "--controller-manager",
                "/controller_manager",
                "-p",
                tmp_file.name,
            ],
        )

        return [
            spawn_controller,
            # Cleanup temporary file on exit
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_controller,
                    on_exit=[
                        OpaqueFunction(
                            function=lambda context: (
                                os.path.exists(tmp_file.name)
                                and os.remove(tmp_file.name),
                                [],
                            )[1]
                        )
                    ],
                )
            ),
        ]

    return [
        OpaqueFunction(
            function=inner_opaque_function,
            kwargs={"robot_controller_config": robot_controller_config},
        )
    ]
