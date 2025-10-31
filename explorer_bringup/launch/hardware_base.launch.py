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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    poc2 = LaunchConfiguration("use_POC2")
    gui = LaunchConfiguration("gui")
    rviz_delay = LaunchConfiguration("rviz_delay")

    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_POC2",
            default_value="true",
            description="Use POC2 urdf",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_delay",
            default_value="5.0",
            description="Delay before starting RViz2 (seconds)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="CAN Port for VESC Communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "host_id",
            default_value="45",
            description="Host CAN ID for VESC Communication",
        )
    )

    # Get URDF via xacro
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("explorer_description"), "urdf", "explorer.urdf.xacro"]
                ),
                " ",
                "use_POC2:=", poc2,
                " ",
                "simulation:=false",
                " ",
                "can_port:=", can_port,
                " ",
                "host_id:=", host_id,

            ]
            )
    }

    delayed_control_node = TimerAction(
        period = 1.0,
        actions = [
            Node(package="controller_manager",
                    executable="ros2_control_node",
                    parameters=[
                        PathJoinSubstitution([FindPackageShare("explorer_bringup"), "config", "explorer_controller.yaml"]), 
                        robot_description
                    ],
                    output="both",
                    remappings=[
                        ("~/robot_description", "/robot_description"),
                    ],
                ),
        ]
    )

    joint_state_broadcaster_spawner = Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="log",
            )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions = [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="log",
                parameters=[
                    robot_description, 
                    {'use_sim_time': False}
                ],
            ),
            joint_state_broadcaster_spawner,            
        ]
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="log",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments = [
            "-d", PathJoinSubstitution([FindPackageShare("explorer_description"), "rviz", "view_robot.rviz"]
        )],
        condition=IfCondition(gui),
    )

    delayed_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit = [
                robot_controller_spawner
            ]
        )
    )

    delayed_rviz_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit = [
                TimerAction(period=rviz_delay, actions=[rviz_node])
            ]
        )
    )

    nodes = [
        delayed_control_node,
        delayed_joint_state_broadcaster,
        delayed_robot_controller,
        delayed_rviz_handler,
    ]

    return LaunchDescription(declared_arguments + nodes)
