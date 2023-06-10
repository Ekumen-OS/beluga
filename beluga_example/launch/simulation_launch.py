# Copyright 2022-2023 Ekumen, Inc.
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

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    example_dir = Path(get_package_share_directory('beluga_example'))

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(example_dir / 'launch' / 'utils' / 'localization_launch.py'),
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(example_dir / 'launch' / 'utils' / 'rviz_launch.py'),
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(example_dir / 'launch' / 'utils' / 'flatland_launch.py'),
                )
            ),
        ]
    )

    return LaunchDescription([load_nodes])
