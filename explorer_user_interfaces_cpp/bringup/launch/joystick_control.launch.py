from launch import LaunchDescription
from launch_ros.actions import Node
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
        remappings=[
            ('/command_node/cartesian_velocity_command', '/explorer_user_interfaces/rqt_armcontrol/input_device_velocity'),
        ],
    )

    nodes = [
        joy_node,
        command_node,
    ]

    return LaunchDescription(nodes)
