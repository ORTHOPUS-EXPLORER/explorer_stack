#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for the web GUI server'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address for the web GUI server'
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
    
    # Web GUI Node
    web_gui_node = Node(
        package='explorer_user_interfaces_web',
        executable='web_gui_node',
        name='web_gui_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
            'mode_config_path': LaunchConfiguration('mode_config_path'),
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        port_arg,
        host_arg,
        mode_config_path_arg,
        LogInfo(msg=['Starting Explorer Robot Web GUI on port ', LaunchConfiguration('port')]),
        web_gui_node,
    ])
