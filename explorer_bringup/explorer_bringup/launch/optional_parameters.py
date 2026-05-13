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


from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import (
    LaunchConfiguration,
)


def declare_parameter_input_device() -> DeclareLaunchArgument:
    """Declare launch parameter 'input_device'

    Returns:
        DeclareLaunchArgument: Argument declared
    """
    return DeclareLaunchArgument(
        "input_device",
        default_value="movis",
        choices=["movis", "xbox"],
        description="Input device type",
    )


def get_parameter_input_device():
    return LaunchConfiguration("input_device")
