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
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")
    rviz_delay = LaunchConfiguration("rviz_delay")

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
            "rviz_delay",
            default_value="5.0",
            description="Delay before starting RViz2 (seconds)",
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
            "use_actuator_interface:=", use_actuator_interface,
            " ",
            "can_port:=", can_port,
            " ",
            "host_id:=", host_id,
            " ",
            "use_POC2:=", poc2
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("explorer_bringup"),
            "config",
            "explorer_multi_intf_controller.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("explorer_description"), "rviz", "view_robot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        #arguments=['--ros-args', '--log-level', 'debug']
    )

    delayed_control_node = TimerAction(
        period=1.0,
        actions=[control_node]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="log",
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    robot_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "explorer_multi_intf_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delayed_rviz = TimerAction(period=rviz_delay, actions=[rviz_node])

    delayed_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                robot_command_controller_spawner,
            ]
        )
    )

    delayed_rviz_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_command_controller_spawner,
            on_exit=[delayed_rviz]
        )
    )

    nodes = [
        delayed_control_node,
        node_robot_state_publisher,
        delayed_joint_state_broadcaster,
    ]

    register_event_handler = [
        delayed_robot_controller,
        delayed_rviz_handler,
    ]

    return LaunchDescription(declared_arguments + nodes + register_event_handler)
