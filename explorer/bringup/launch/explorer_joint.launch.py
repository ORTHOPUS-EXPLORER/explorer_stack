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
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    run_bridge = LaunchConfiguration("use_bridge")
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
            "use_bridge",
            default_value="true",
            description="Start Explorer PyVESC Bridge (and use Actuators HW Interfaces)",
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_explorer"), "description/urdf", "explorer.urdf.xacro"]
            ),
            " ",
            "use_ignition:=false",
            " ",
            "use_actuator_interface:=",run_bridge
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_explorer"),
            "config",
            "explorer_controller.yaml",
        ]
    )

    config =  PathJoinSubstitution(
            [FindPackageShare("ros2_control_explorer"), "config", "settings_joint.yaml"]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "description/rviz", "view_robot.rviz"]
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

    output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="joint_output_integrator",
        parameters=[
            config,
            {'use_sim_time': use_sim_time}
        ],
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
        package='rqt_armcontrol',
        executable='rqt_jointcontrol',
        condition=IfCondition(gui)
    )

    explorer_bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_explorer"),
            "config",
            "explorer_vesc.yaml",
        ]
    )

    explorer_bridge = Node(
        package="pyvesc_explorer",
        executable="ros_explorer_bridge",
        parameters=[explorer_bridge_params],
        output="both",
        remappings=[],
        arguments=['--non-interactive','--ros-args'],#, '--log-level', 'DEBUG']
        #prefix=['xterm -e gdb -ex run --args'],
        condition=IfCondition(run_bridge),
    )


    register_event_handler = []
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=control_node,
                    on_exit=[joint_state_broadcaster_spawner],
                )
        )
    )
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
                    on_exit=[output_integrator_node],
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
        explorer_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
