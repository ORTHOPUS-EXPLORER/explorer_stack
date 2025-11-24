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
import xacro
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    poc2 = LaunchConfiguration("use_POC2")
    rviz_delay = LaunchConfiguration("rviz_delay")
    world_file = LaunchConfiguration("world_file")
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_POC2",
            default_value="true",
            description="Use POC2 urdf",
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
            "world_file",
            default_value="empty_world.world",
            description="Gazebo world file to load",
        )
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            # Use xacro to get URDF
            {"robot_description": ParameterValue(
                Command([
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("explorer_description"), "urdf", "explorer.urdf.xacro"]
                    ),
                    " ", "simulation:=true",
                    " ", "use_ignition:=true",
                    " ", "use_POC2:=", poc2,
                ]),
                value_type=str
            )}, 
            {'use_sim_time': use_sim_time}
        ],
    )

    gz_spawn_entity = Node(
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    
    controllers_control_node = Node(package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("explorer_bringup"), "config", "explorer_controller.yaml"]),
        ],
        output="both",
    )
    
    register_event_handler = []
    register_event_handler.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    robot_controller_spawner,
                    controllers_control_node,
                ],
            )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    TimerAction(period=rviz_delay, actions=[
                        Node(
                            package="rviz2",
                            executable="rviz2",
                            name="rviz2",
                            output="log",
                            arguments=[
                                "-d", 
                                PathJoinSubstitution(
                                    [FindPackageShare("explorer_description"), "rviz", "view_robot.rviz"]
                                )
                            ],
                            condition=IfCondition(gui),
                            parameters=[{'use_sim_time': use_sim_time}]
                        )
                    ])
                ],
            )
        )
    )

    nodes = [
        # Ignition Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={
                'gz_args': [
                    '-r -s -v4 ', 
                    PathJoinSubstitution([
                        FindPackageShare('explorer_gazebo'),
                        'worlds',
                        world_file
                    ])
                ], 
                'on_exit_shutdown': 'true'
            }.items()
        ),
        # Ignition Client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={'gz_args': '-g -v4 '}.items()
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
