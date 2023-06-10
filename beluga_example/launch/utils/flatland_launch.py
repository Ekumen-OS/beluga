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
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = Path(get_package_share_directory('beluga_example'))

    load_nodes = GroupAction(
        actions=[
            Node(
                package='flatland_server',
                executable='flatland_server',
                output='screen',
                parameters=[
                    {
                        'world_path': str(
                            example_dir / 'worlds' / 'turtlebot3_world.yaml'
                        )
                    },
                    {'update_rate': 200.0},
                    {'step_size': 0.005},
                    {'show_viz': False},
                    {'viz_pub_rate': 30.0},
                    {'use_sim_time': True},
                ],
            ),
        ]
    )

    return LaunchDescription([load_nodes])
