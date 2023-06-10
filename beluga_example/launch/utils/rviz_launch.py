# Copyright 2023 Ekumen, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = get_package_share_directory('beluga_example')
    example_dir_path = Path(example_dir)

    declare_display_config = DeclareLaunchArgument(
        'display_config',
        default_value=str(example_dir_path / 'rviz' / 'rviz.rviz'),
        description='A display config file (.rviz) to load.',
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='own_log',
                arguments=[
                    '--display-config',
                    LaunchConfiguration('display_config'),
                ],
            ),
        ]
    )

    return LaunchDescription([declare_display_config, load_nodes])
