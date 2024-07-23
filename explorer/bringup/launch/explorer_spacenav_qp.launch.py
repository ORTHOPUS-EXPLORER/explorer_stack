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
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    spacenav = LaunchConfiguration('spacenav')

    spacenav_arg = DeclareLaunchArgument(
        name='spacenav',
        default_value='True',
        description='If the spacenav 3D mouse is used')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_explorer"), "description/urdf", "explorer.urdf.xacro"]
            ),
            " ",
            "use_gazebo_classic:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Get SRDF via xacro
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_explorer"), "description/urdf", "explorer.srdf"]
            ),
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": semantic_content}
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "description/rviz", "view_robot.rviz"]
    )

    ## Declare SpaceNav nodes (driver & input_device)
    spacenav_config =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "spacenav_settings.yaml"]
    )

    config =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "settings.yaml"]
    )
    
    spacenav_node = Node(
        package='ros2_control_explorer',
        executable='spacenav',
        parameters=[
            spacenav_config,
            {'static_rot_deadband':0.5},
            {'static_trans_deadband':0.5}
        ],
        condition=IfCondition(spacenav)
    )

    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        parameters=[
            {'static_rot_deadband':0.5},
            {'static_trans_deadband':0.5}
        ],
        condition=IfCondition(spacenav),
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    
    spacenav_trajectory_qp_node = Node(
        package="ros2_control_explorer",
        executable="spacenav_trajectory_qp",
        parameters=[
            config,
            robot_description,
            robot_description_semantic
            ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "explorer_system_position"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["explorer_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    # Declare GUI controller node
    gui_control_node = Node(
        package='rqt_armcontrol',
        executable='rqt_armcontrol',
        condition=IfCondition(gui)
        )

    nodes = [
        spacenav_arg,
        spacenav_node,
        gui_control_node,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        spacenav_driver_node,
        spacenav_trajectory_qp_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
