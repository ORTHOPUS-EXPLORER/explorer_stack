from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    joint_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("explorer_bringup"),
                    "launch",
                    "joint_control.launch.py",
                ]
            )
        ),
        launch_arguments=[
            ("simulation", "true"),
        ],
    )

    gravity_compensation_node = Node(
        package="explorer_controllers",
        executable="gravity_compensation_node",
        name="gravity_compensation_node",
        parameters=[
            {
                "urdf_path": "/root/explorer_ws/explorer_stack/explorer_description/urdf/explorer.urdf.xacro",
                "joint_state_topic": "/joint_states",
            }
        ],
    )

    return LaunchDescription([
        joint_control_launch,
        gravity_compensation_node,
    ])
