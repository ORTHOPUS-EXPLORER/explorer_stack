import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('explorer_user_interfaces_cpp')
    input_devices_share = get_package_share_directory('explorer_input_devices')

    config_yaml_file_path = os.path.join(input_devices_share, 'config', 'config_mode_0.yaml')
    trajectory_yaml_file_path = os.path.join(pkg_share, 'config', 'config_trajectory.yaml')
    led_status_config_arg = DeclareLaunchArgument(
        'led_status_config',
        default_value='led_status_mode_0.yaml',
        description='LED status config file name, from explorer_user_interfaces_cpp/config',
    )
    led_status_yaml_file_path = PathJoinSubstitution(
        [FindPackageShare('explorer_user_interfaces_cpp'), 'config', LaunchConfiguration('led_status_config')]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    joystick_device_node = Node(
        package="explorer_input_devices",
        executable="device_joystick",
        output="screen",
        parameters=[{
            "mode_file": config_yaml_file_path,
            # No joystick_selector here, feed command_node directly
            "cartesian_command_topic": "/command_node/robot/velocity/commands",
        }],
    )

    command_node = Node(
    package="explorer_user_interfaces_cpp",
    executable="command_node",
    output="screen",
    parameters=[{
        "trajectory_file": trajectory_yaml_file_path,
        "led_status_file": led_status_yaml_file_path,
        }],
)
    nodes = [
        led_status_config_arg,
        command_node,
        joy_node,
        joystick_device_node,
    ]

    return LaunchDescription(nodes)
