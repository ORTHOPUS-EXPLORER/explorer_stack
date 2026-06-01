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


from launch_ros.actions import Node


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


def declare_custom_controller_spawner(output: str = "log") -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        output=output,
        arguments=[
            "explorer_custom_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
