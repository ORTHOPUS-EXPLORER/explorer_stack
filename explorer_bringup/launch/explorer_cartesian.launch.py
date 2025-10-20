from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    spacenav = LaunchConfiguration('spacenav')
    use_actuator_interface = LaunchConfiguration("use_actuator_interface")
    can_port = LaunchConfiguration("can_port")
    host_id = LaunchConfiguration("host_id")
    poc2 = LaunchConfiguration("use_POC2")

    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 automatically with this launch file."),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='If true, use simulated clock'),
        DeclareLaunchArgument("use_actuator_interface", default_value="true", description="Use VESCInterface to control the robot. Set to false for simulation"),
        DeclareLaunchArgument("can_port", default_value="vxcan1", description="CAN Port for VESC Communication"),
        DeclareLaunchArgument("host_id", default_value="45", description="Host CAN ID for VESC Communication"),
        DeclareLaunchArgument("use_POC2", default_value="true", description="Use POC2 urdf"),
        DeclareLaunchArgument('spacenav', default_value='True', description='If the spacenav 3D mouse is used')
    ]

    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("explorer_description"), "urdf", "explorer.urdf.xacro"]),
        " ",
        "use_ignition:=false ",
        "use_actuator_interface:=", use_actuator_interface,
        " can_port:=", can_port,
        " host_id:=", host_id,
        " use_POC2:=", poc2
    ])
    robot_description = {"robot_description": robot_description_command}

    semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("explorer_description"), "urdf", "explorer.srdf"]),
        " "
    ])
    robot_description_semantic = {"robot_description_semantic": semantic_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("explorer_bringup"),
        "config", "explorer_controller.yaml"
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("explorer_description"), "rviz", "view_robot.rviz"
    ])

    config_POC1 = PathJoinSubstitution([
        FindPackageShare("explorer_bringup"), "config", "settings_POC1.yaml"])

    config_POC2 = PathJoinSubstitution([
        FindPackageShare("explorer_bringup"), "config", "settings_POC2.yaml"])

    spacenav_config = PathJoinSubstitution([
        FindPackageShare("explorer_input_devices"),
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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, robot_description],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
    )

    delayed_control_node = TimerAction(
        period=1.0,
        actions=[control_node]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="log",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
        output="log",
    )

    input_integrator_node = Node(
        package="ros2_control_explorer",
        executable="input_integrator",
        name="input_integrator_node",
    )

    output_integrator_node = Node(
        package="ros2_control_explorer",
        executable="output_integrator",
        name="output_integrator_node",
    )

    qp_solving_POC1_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[config_POC1, robot_description, robot_description_semantic],
        condition=UnlessCondition(poc2),
    )

    qp_solving_POC2_node = Node(
        package="ros2_control_explorer",
        executable="qp_solving",
        parameters=[config_POC2, robot_description, robot_description_semantic],
        condition=IfCondition(poc2),
    )

    gui_control_node = Node(
        package='explorer_user_interfaces',
        executable='rqt_armcontrol',
    )

    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_robot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
    )

    delayed_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[input_integrator_node, output_integrator_node, qp_solving_POC1_node, qp_solving_POC2_node]
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    return LaunchDescription(declared_arguments + [
        delayed_control_node,
        node_robot_state_publisher,
        gui_control_node,
        delayed_joint_state_broadcaster,
        delayed_robot_controller,
        delayed_nodes,
        delayed_rviz,
        spacenav_node,
        spacenav_driver_node,
    ])
