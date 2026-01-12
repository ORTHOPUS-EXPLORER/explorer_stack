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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation = LaunchConfiguration('simulation')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")
    robot_description_param = LaunchConfiguration("robot_description_param")
    trajectory = LaunchConfiguration("force_deploy")
    port_arg = LaunchConfiguration('port')
    host_arg = LaunchConfiguration('host')
    mode_config_path_arg = LaunchConfiguration('mode_config_path')

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
    declared_arguments.append(
        DeclareLaunchArgument(
            "force_deploy",
            default_value="true",
            description="Force robot deployment to rest position before enabling any other control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="8080",
            description="Port for the web GUI server"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "host",
            default_value="0.0.0.0",
            description="Host address for the web GUI server"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode_config_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("explorer_user_interfaces_cpp"),
                "config",
                "config_mode_0.yaml"
            ]),
            description="Path to the mode configuration YAML file"
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

    robot_description = {"robot_description": ParameterValue(robot_description_param, value_type=str)}

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

    robot_description_semantic = {"robot_description_semantic": ParameterValue(semantic_content, value_type=str)}

    config_POC1 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC1.yaml"])

    config_POC2 = PathJoinSubstitution(
        [FindPackageShare("explorer_bringup"), "config", "settings_POC2.yaml"])


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

    pkg_share = get_package_share_directory('explorer_user_interfaces_cpp')
    yaml_file_path = os.path.join(pkg_share, 'config', 'config_mode_0.yaml')
    trajectory_yaml_file_path = os.path.join(pkg_share, 'config', 'config_trajectory.yaml')

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
            "mode_file": yaml_file_path,
            "trajectory_file": trajectory_yaml_file_path,
            "active_trajectory": trajectory,
        }],
        remappings=[
            ('/command_node/cartesian_velocity_command', '/explorer_user_interfaces/rqt_armcontrol/input_device_velocity'),
            ('/command_node/gripper_velocity_command', '/explorer_user_interfaces/rqt_armcontrol/input_gripper_velocity'),
        ],
    )

    web_gui_node = Node(
        package='explorer_user_interfaces_web',
        executable='web_gui_node',
        name='web_gui_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
            'mode_config_path': LaunchConfiguration('mode_config_path'),
        }],
        output='screen',
        emulate_tty=True,
    )

    nodes = [
        robot_simulation,
        robot_hardware,
        input_integrator_node,
        output_integrator_node,
        qp_solving_POC1_node,
        qp_solving_POC2_node,
        joy_node,
        command_node,
        web_gui_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
