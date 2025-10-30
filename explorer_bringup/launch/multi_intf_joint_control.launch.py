from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os, sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from common import *


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation = LaunchConfiguration('simulation')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")

    # Declare arguments
    declared_arguments = []

    set_common_args()

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'simulation',
            default_value='true',
            description='If true, use simulation (Gazebo), if false use real hardware')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock. Auto-set based on simulation mode if not specified')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_actuator_interface",
            default_value="true",
            description="Use VESCInterface to control the robot. Set to false for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="CAN Port for VESC Communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "host_id",
            default_value="45",
            description="Host CAN ID for VESC Communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_POC2",
            default_value="true",
            description="Use POC2 urdf",
        )
    )

    # Include robot simulation (when simulation=true)
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"), 
            "/launch/simulation_base.launch.py"
        ]),
        launch_arguments={
            'use_POC2': poc2,
            'gui': gui,
            'use_sim_time': use_sim_time,
            'rviz_delay': '5.0'
        }.items(),
        condition=IfCondition(simulation)
    )

    # Include robot hardware (when simulation=false)
    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"), 
            "/launch/hardware_multi_intf_base.launch.py"
        ]),
        launch_arguments={
            'gui': gui,
            'use_sim_time': use_sim_time,
            'use_actuator_interface': use_actuator_interface,
            'can_port': can_port,
            'host_id': host_id,
            'use_POC2': poc2,
            'rviz_delay': '5.0'
        }.items(),
        condition=UnlessCondition(simulation)
    )

    # Declare GUI controller node
    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_jointcontrol',
        remappings=[
            ( '/explorer_user_interfaces/rqt_jointcontrol/dq_output', '/explorer_multi_intf_controller/refs'),
        ]
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        gui_control_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
