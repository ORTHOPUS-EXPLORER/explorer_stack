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

from typing import List

from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
)

from explorer_bringup.launch.shared_parameters import (
    declare_shared_argument_list,
)


def declare_simulation_argument_list() -> List[DeclareLaunchArgument]:
    return [
        *declare_shared_argument_list(use_actuator_interface=False),
        DeclareLaunchArgument(
            "world_file",
            default_value="empty_world.world",
            description="(Only used when simulation=true) Gazebo world file to load",
        ),
    ]


def get_parameter_world_file() -> LaunchConfiguration:
    """Get ros2 parameter "world_file".

    Returns:
        LaunchConfiguration: world_file
    """
    return LaunchConfiguration("world_file")
