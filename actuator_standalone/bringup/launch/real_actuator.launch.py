import os
import xacro
from ament_index_python.packages import get_package_share_path, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions


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
            default_value=use_sim_time,
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
                [FindPackageShare("ros2_control_actuator"), "description/urdf", "actuator.urdf.xacro"]
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
            FindPackageShare("ros2_control_actuator"),
            "config",
            "actuator_controller.yaml",
        ]
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_actuator"), "description/rviz", "view_actuator.rviz"]
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    actuator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    joint_controller_node = Node(
        package="ros2_control_actuator",
        executable="joint_controller",
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
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

    actuator_bridge_params = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_actuator"),
            "config",
            "actuator_vesc.yaml",
        ]
    )

    actuator_bridge = Node(
        package="pyvesc_explorer",
        executable="ros_actuator_bridge",
        parameters=[actuator_bridge_params],
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
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[actuator_controller_spawner],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=actuator_controller_spawner,
                    on_exit=[joint_controller_node],
                )
        )
    )
    register_event_handler.append(
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=actuator_controller_spawner,
                    on_exit=[delayed_rviz],
                )
        )
    )
    
    nodes = [
        control_node,
        node_robot_state_publisher,
        joy_node,
        actuator_bridge,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
