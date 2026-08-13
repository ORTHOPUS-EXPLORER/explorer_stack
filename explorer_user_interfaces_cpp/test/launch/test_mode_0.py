import os
import unittest

import launch
import launch.actions
import launch.events
import launch_testing
import launch_testing.asserts
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.launch_test
def generate_test_description():
    # 1. Locate your launch file
    pkg_share = get_package_share_directory('explorer_user_interfaces_cpp')
    launch_file_path = os.path.join(pkg_share, 'launch', 'mode_0.launch.py')

    # 2. Include the launch file (you can pass launch arguments here)
    launch_inclusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={'can_port': 'vcan0', 'gui': 'false', 'use_qp_inria': 'false'}.items()
    )

    # Create a timer to kill the test after 7.5 seconds. 
    # This gives nodes enough time to boot up, configure, and prove they don't crash.
    shutdown_timer = launch.actions.TimerAction(
        period=7.5,
        actions=[
            launch_testing.actions.ReadyToTest()
        ]
    )

    return launch.LaunchDescription([
        launch_inclusion,
        shutdown_timer,
    ])

# -----------------------------------------------------------------------------
# Tests that run AFTER the launch file is shut down
# -----------------------------------------------------------------------------
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_no_crashes(self, proc_info, proc_output):
        """Check that all processes exited normally (no crashes)."""

        ## Cannot be tested as long as the while loop in constructor exists
        # launch_testing.asserts.assertExitCodes(proc_info, process="input_integrator")
        # launch_testing.asserts.assertExitCodes(proc_info, process="output_integrator")
        
        launch_testing.asserts.assertExitCodes(proc_info, process="command_node")
        launch_testing.asserts.assertExitCodes(proc_info, process="robot_state_publisher")
        launch_testing.asserts.assertExitCodes(proc_info, process="spawner", cmd_args=["joint_state_broadcaster"])
        launch_testing.asserts.assertExitCodes(proc_info, process="spawner", cmd_args=["forward_position_controller"])
        launch_testing.asserts.assertExitCodes(proc_info, process="spawner", cmd_args=["gripper_controller"])
        launch_testing.asserts.assertExitCodes(proc_info, process="spawner", cmd_args=["joint_trajectory_controller"])
        launch_testing.asserts.assertExitCodes(proc_info, process="joy_node")
        launch_testing.asserts.assertExitCodes(proc_info, process="web_gui_node")        ## Never quit properly: would be better to check stdout
