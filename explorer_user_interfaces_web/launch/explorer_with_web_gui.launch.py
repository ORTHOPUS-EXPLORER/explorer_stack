#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for the web GUI server'
    )
    
    mode_config_path_arg = DeclareLaunchArgument(
        'mode_config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('explorer_user_interfaces_cpp'),
            'config',
            'config_mode_0.yaml'
        ]),
        description='Path to the mode configuration YAML file'
    )
    
    # Include the command node launch file (if it exists)
    # command_node_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('explorer_user_interfaces_cpp'),
    #             'launch',
    #             'command_node.launch.py'  # Adjust filename as needed
    #         ])
    #     ])
    # )
    
    # Alternatively, launch the command node directly
    command_node = Node(
        package='explorer_user_interfaces_cpp',
        executable='command_node',
        name='command_node',
        parameters=[{
            'mode_file': LaunchConfiguration('mode_config_path'),
        }],
        output='screen',
    )
    
    # Web GUI Node  
    web_gui_node = Node(
        package='explorer_user_interfaces_web',
        executable='web_gui_node',
        name='web_gui_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': '0.0.0.0',
            'mode_config_path': LaunchConfiguration('mode_config_path'),
        }],
        output='screen',
    )
    
    return LaunchDescription([
        port_arg,
        mode_config_path_arg,
        LogInfo(msg=['Starting Explorer Robot Control System with Web GUI on port ', LaunchConfiguration('port')]),
        command_node,
        web_gui_node,
    ])
