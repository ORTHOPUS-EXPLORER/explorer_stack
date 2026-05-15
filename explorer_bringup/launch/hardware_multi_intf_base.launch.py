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

from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import declare_hardware_argument_list
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from launch import LaunchDescription


def _declare_arguments():
    return [
        *declare_simulation_argument_list(),
        *declare_hardware_argument_list(),
    ]


def generate_launch_description():
    # Initialize Arguments
    declared_arguments = _declare_arguments()

    # Nodes

    robot_hardware = declare_hardware_node_group(
        launch_qp_solving=True,
        controller_manager_type = "multi_intf"
    )


    nodes = [robot_hardware]

    return LaunchDescription([*declared_arguments, *nodes])
