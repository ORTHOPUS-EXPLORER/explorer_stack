from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('explorer_user_interfaces_cpp')

    yaml_file_path = os.path.join(pkg_share, 'config', 'config_mode_0.yaml')

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    command_node = Node(
    package="explorer_user_interfaces_cpp",
    executable="command_node",
    output="screen",
    parameters=[{
        "mode_file": yaml_file_path
        }],
)
    nodes = [
        command_node,
        joy_node,
    ]

    return LaunchDescription(nodes)
