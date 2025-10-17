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

import os
import xacro
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    spacenav = LaunchConfiguration('spacenav')
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
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock')
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
                    FindPackageShare("ros2_control_explorer"),
                    "explorer_description/urdf",
                    "explorer.urdf.xacro"
                ]),
                " ",
                "use_ignition:=true",
                " ",
                "use_POC2:=", poc2
            ]),
            description="Robot description (URDF) evaluated from xacro"
        )
    )

    spacenav_arg = DeclareLaunchArgument(
        name='spacenav',
        default_value='True',
        description='If the spacenav 3D mouse is used')
    
    world = os.path.join(
        get_package_share_directory('ros2_control_explorer'),
        'explorer_description/worlds',
        'empty_world.world'
    )

    ignition_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    ignition_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items() #Rajouter -s pour que Gazebo s'affiche pas
    )

    robot_description = {"robot_description": robot_description_param}

    # Get SRDF via xacro
    semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_explorer"), "explorer_description/urdf", "explorer.srdf"]
            ),
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": semantic_content}
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_explorer"), "explorer_description/rviz", "view_robot.rviz"]
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

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "explorer",
            "-allow_renaming",
            "true",
        ],
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

    input_integrator_node = Node(
        package="ros2_control_explorer",
        executable="input_integrator",
        name="input_integrator",
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )

    qp_solving_POC1_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[
            config_POC1,
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(poc2),
    )

    qp_solving_POC2_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[
            config_POC2,
            robot_description,
            robot_description_semantic,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(poc2),
    )

    output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="output_integrator",
        name="output_integrator",
        parameters=[
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

    delayed_rviz = TimerAction(period=0.0,actions=[rviz_node])
    
    # Declare GUI controller node
    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_armcontrol',
    )
    
    bridge_config = os.path.join(
        get_package_share_directory('ros2_control_explorer'),
        'config',
        'bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
        output='screen',
    )

 # Bridge
    bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['camera', 'depth_camera', 'rgbd_camera/image', 'rgbd_camera/depth_image'],
        output='screen'
    )

    register_event_handler = []
    register_event_handler.append(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_robot_state_publisher,
                on_start=[joint_state_broadcaster_spawner],
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
    

    nodes = [
        spacenav_arg,
        ignition_server,
        ignition_client,
        spacenav_node,
        spacenav_driver_node,
        node_robot_state_publisher,
        gz_spawn_entity,
        gui_control_node,
        start_gazebo_ros_bridge_cmd,
        bridge,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
