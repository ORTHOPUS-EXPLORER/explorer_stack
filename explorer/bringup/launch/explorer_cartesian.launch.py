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
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    spacenav = LaunchConfiguration('spacenav')
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
            default_value=use_sim_time,
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

    spacenav_arg = DeclareLaunchArgument(
        name='spacenav',
        default_value='True',
        description='If the spacenav 3D mouse is used')
    

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_explorer"),
                    "description/urdf",
                    "explorer.urdf.xacro",
                ]
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

    # Get SRDF via xacro
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_explorer"), "description/urdf", "explorer.srdf"]
            ),
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": semantic_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_explorer"),
            "config",
            "explorer_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "description/rviz", "view_robot.rviz"]
    )

    ## Declare SpaceNav nodes (driver & input_device)
    spacenav_config =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "spacenav_settings.yaml"]
    )

    config_POC1 =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "settings_POC1.yaml"]
    )

    config_POC2 =  PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "config", "settings_POC2.yaml"]
    )
    
    spacenav_node = Node(
        package='ros2_control_explorer',
        executable='spacenav',
        parameters=[
            spacenav_config,
            {'static_rot_deadband':0.5},
            {'static_trans_deadband':0.5}
        ],
        condition=IfCondition(spacenav)
    )

    spacenav_driver_node = Node(
        package='spacenav',
        executable='spacenav_node',
        parameters=[
            {'static_rot_deadband':0.5},
            {'static_trans_deadband':0.5}
        ],
        condition=IfCondition(spacenav),
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
        parameters=[robot_description],
    )

    input_integrator_node = Node(
        package="ros2_control_explorer",
        executable="input_integrator",
        name="input_integrator_node",
    )

    qp_solving_POC1_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[
            config_POC1,
            robot_description,
            robot_description_semantic
            ],
        condition=UnlessCondition(poc2),
    )

    qp_solving_POC2_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[
            config_POC2,
            robot_description,
            robot_description_semantic
            ],
        condition=IfCondition(poc2),
    )

    output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="output_integrator",
        name="output_integrator_node",
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delayed_rviz = TimerAction(period=0.0,actions=[rviz_node])
    
    # Declare GUI controller node
    gui_control_node = Node(
        package='rqt_armcontrol',
        executable='rqt_armcontrol',
    )


    explorer_bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_explorer"),
            "config",
            "explorer_vesc_hw.yaml",
        ]
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
                    on_exit=[qp_solving_POC1_node],
                )
        )
    )
    
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[qp_solving_POC2_node],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawner,
                    on_exit=[input_integrator_node],
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

    register_event_handler.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager", "/controller_manager",
                            "--activate",
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "forward_position_controller",
                            "--controller-manager", "/controller_manager",
                            "--activate",
                        ],
                    ),
                ],
            )
        )
    )

    register_event_handler.append(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[joint_state_broadcaster_spawner],
            )
        )
    )

    nodes = [
        spacenav_arg,
        spacenav_node,
        spacenav_driver_node,
        control_node,
        node_robot_state_publisher,
        #joint_state_broadcaster_spawner, removed, set in an event handler
        gui_control_node,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
