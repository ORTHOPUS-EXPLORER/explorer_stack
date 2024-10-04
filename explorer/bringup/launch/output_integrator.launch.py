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
            default_value="true",
            description="Start GUI automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    spacenav = LaunchConfiguration('spacenav')

    spacenav_arg = DeclareLaunchArgument(
        name='spacenav',
        default_value='True',
        description='If the spacenav 3D mouse is used')


    ## Declare SpaceNav nodes (driver & input_device)
    spacenav_config =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "spacenav_settings.yaml"]
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

    output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="output_integrator",
        name="output_integrator_node",
    )

    test_output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="test_output_integrator",
        name="test_output_integrator_node",
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
        output_integrator_node,
        test_output_integrator_node,
        spacenav_driver_node,
        gui_control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
