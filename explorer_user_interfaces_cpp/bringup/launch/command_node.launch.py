import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('explorer_user_interfaces_cpp')
    input_devices_share = get_package_share_directory('explorer_input_devices')

    config_yaml_file_path = os.path.join(input_devices_share, 'config', 'config_mode_0.yaml')
    trajectory_yaml_file_path = os.path.join(pkg_share, 'config', 'config_trajectory.yaml')

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
        }],
    )

    command_node = Node(
    package="explorer_user_interfaces_cpp",
    executable="command_node",
    output="screen",
    parameters=[{
        "trajectory_file": trajectory_yaml_file_path
        }],
    remappings=[
        (
            "/command_node/cartesian_velocity_command",
            "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity",
        ),
    ],
)
    nodes = [
        command_node,
        joy_node,
        joystick_device_node,
    ]

    return LaunchDescription(nodes)
