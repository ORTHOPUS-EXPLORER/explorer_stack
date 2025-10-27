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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation = LaunchConfiguration('simulation')
    spacenav = LaunchConfiguration('spacenav')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")
    robot_description_param = LaunchConfiguration("robot_description_param")

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
            default_value="vxcan1",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_param",
            default_value=Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([
                    FindPackageShare("explorer_description"),
                    "urdf",
                    "explorer.urdf.xacro"
                ]),
                " ",
                "use_ignition:=", simulation,
                " ",
                "use_actuator_interface:=", use_actuator_interface,
                " can_port:=", can_port,
                " host_id:=", host_id,
                " use_POC2:=", poc2
            ]),
            description="Robot description (URDF) evaluated from xacro"
        )
    )

    spacenav_arg = DeclareLaunchArgument(
        name='spacenav',
        default_value='True',
        description='If the spacenav 3D mouse is used')

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
            'rviz_delay': '0.0'
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

    robot_description = {"robot_description": robot_description_param}

    # Get SRDF via xacro
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("explorer_description"), "urdf", "explorer.srdf"]
            ),
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": semantic_content}

    config_POC1 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC1.yaml"])

    config_POC2 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC2.yaml"])

    spacenav_config = PathJoinSubstitution(
        [FindPackageShare("explorer_input_devices"),
         "config",
         "spacenav_settings.yaml"
    ])

    spacenav_node = Node(
        package='explorer_input_devices',
        executable='spacenav',
        parameters=[
            spacenav_config,
            {'static_rot_deadband': 0.5},
            {'static_trans_deadband': 0.5}
        ],
        condition=IfCondition(spacenav),
    )

    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        parameters=[
            {'static_rot_deadband': 0.5},
            {'static_trans_deadband': 0.5}
        ],
        condition=IfCondition(spacenav),
    )

    input_integrator_node = Node(
        package="explorer_controllers",
        executable="input_integrator",
        name="input_integrator",
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    output_integrator_node = Node(
        package="explorer_controllers",
        executable="output_integrator",
        name="output_integrator",
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    qp_solving_POC1_node = Node(
        package="explorer_controllers",
        executable="qp_solving",
        parameters=[config_POC1, robot_description, robot_description_semantic, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(poc2),
    )

    qp_solving_POC2_node = Node(
        package="explorer_controllers",
        executable="qp_solving",
        parameters=[config_POC2, robot_description, robot_description_semantic, {'use_sim_time': use_sim_time}],
        condition=IfCondition(poc2),
    )

    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_armcontrol',
    )

    nodes = [
        spacenav_arg,
        robot_simulation,
        robot_hardware,
        spacenav_node,
        spacenav_driver_node,
        gui_control_node,
        input_integrator_node,
        output_integrator_node,
        qp_solving_POC1_node,
        qp_solving_POC2_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
