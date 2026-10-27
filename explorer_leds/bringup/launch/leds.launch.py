from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    simulation = LaunchConfiguration("simulation")
    strip_config = LaunchConfiguration("strip_config")

    declared_arguments = [
        DeclareLaunchArgument(
            "simulation",
            default_value="false",
            description="Receive and play LED commands without sending them to any hardware",
        ),
        DeclareLaunchArgument(
            "strip_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("explorer_leds"), "config", "led_strip.yaml"]
            ),
            description="LED strip config file (SPI device, LED count, RGBW, brightness cap)",
        ),
    ]

    led_node = Node(
        package="explorer_leds",
        executable="led_node",
        output="screen",
        parameters=[{
            "simulation": ParameterValue(simulation, value_type=bool),
            "command_topic": "/explorer_leds/command",
            "flash_topic": "/explorer_leds/flash",
            "heartbeat_topic": "/command_node/ready",
            "update_rate_hz": 50.0,
            "watchdog_timeout_seconds": 0.5,
            "strip_config_file": strip_config,
        }],
    )

    return LaunchDescription(declared_arguments + [led_node])
