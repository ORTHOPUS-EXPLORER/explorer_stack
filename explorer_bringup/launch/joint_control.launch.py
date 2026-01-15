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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
        # Common
    gui = LaunchConfiguration("gui")
    poc2 = LaunchConfiguration("use_POC2")
        # Simulation
    simulation = LaunchConfiguration('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
        # Real hardware
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_POC2",
            default_value="true",
            description="Use POC2 urdf",
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


    # Include robot simulation (when simulation=true)
    robot_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"), 
            "/launch/simulation_base.launch.py"
        ]),
        launch_arguments={
            'use_POC2': poc2,
            'gui': gui,
            'rviz_delay': '5.0',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(simulation)
    )

    # Include robot hardware (when simulation=false)
    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("explorer_bringup"), 
            "/launch/hardware_base.launch.py"
        ]),
        launch_arguments={
            'use_POC2': poc2,
            'gui': gui,
            'rviz_delay': '5.0',
            'can_port': can_port,
            'host_id': host_id,
        }.items(),
        condition=UnlessCondition(simulation)
    )

    config_POC1 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_joint_POC1.yaml"]
    )

    config_POC2 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_joint_POC2.yaml"]
    )

    output_integrator_POC1_node = Node(
        package="explorer_controllers",
        executable="joint_output_integrator",
        parameters=[
            config_POC1,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(poc2),
    )

    output_integrator_POC2_node = Node(
        package="explorer_controllers",
        executable="joint_output_integrator",
        parameters=[
            config_POC2,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(poc2),
    )

    # Declare GUI controller node
    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_jointcontrol',
        remappings=[
            ( '/explorer_user_interfaces/rqt_jointcontrol/dq_output', '/explorer_controllers/qp_solving/dq_output'),
        ]
    )

    joystick_yaml_file_path = PathJoinSubstitution([
        FindPackageShare("explorer_input_devices"),
        "config",
        "xbox_gamepad_settings.yaml",
    ])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[joystick_yaml_file_path],
    )

    xbox_gamepad_joint_node = Node(
        package="explorer_input_devices",
        executable="xbox_gamepad_joint",
        remappings=[
            ( '/explorer_input_devices/xbox_gamepad_joint/dq_output', '/explorer_controllers/qp_solving/dq_output'),
        ]
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        gui_control_node,
        joy_node,
        xbox_gamepad_joint_node,
        output_integrator_POC1_node,
        output_integrator_POC2_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
