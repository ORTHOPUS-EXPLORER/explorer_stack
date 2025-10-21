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
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")

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
            'use_sim_time',
            default_value= use_sim_time,
            description='If true, use simulated clock')
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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("explorer_description"), "urdf", "explorer.urdf.xacro"]
            ),
            " ",
            "use_ignition:=false",
            " ",
            " use_actuator_interface:=",use_actuator_interface,
            " ",
            " can_port:=",can_port,
            " ",
            " host_id:=",host_id,
            " ",
            "use_POC2:=",poc2
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("explorer_bringup"),
            "config",
            "explorer_controller.yaml",
        ]
    )

    config_POC1 =  PathJoinSubstitution(
            [FindPackageShare("explorer_bringup"), "config", "settings_joint_POC1.yaml"]
    )

    config_POC2 =  PathJoinSubstitution(
            [FindPackageShare("explorer_bringup"), "config", "settings_joint_POC2.yaml"]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "explorer_description/rviz", "view_robot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
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
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delayed_rviz = TimerAction(period=5.0,actions=[rviz_node])
    
    # Declare GUI controller node
    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_jointcontrol',
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    xbox_gamepad_joint_node = Node(
        package="explorer_input_devices",
        executable="xbox_gamepad_joint",
    )

    register_event_handler = []
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[robot_controller_spawner],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[output_integrator_POC1_node],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[output_integrator_POC2_node],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[delayed_rviz],
                )
        )
    )

    nodes = [
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        gui_control_node,
        joy_node,
        xbox_gamepad_joint_node,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
