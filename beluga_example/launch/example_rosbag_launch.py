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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    example_dir = get_package_share_directory('beluga_example')

    package = LaunchConfiguration('package')
    node = LaunchConfiguration('node')
    prefix = LaunchConfiguration('prefix')

    package_launch_arg = DeclareLaunchArgument(
        name='package',
        default_value='beluga_amcl',
        description='Package that provides the localization node to launch.',
        choices=['beluga_amcl', 'nav2_amcl'],
    )

    node_launch_arg = DeclareLaunchArgument(
        name='node',
        default_value='amcl_node',
        description='Localization node to launch.',
    )

    prefix_launch_arg = DeclareLaunchArgument(
        name='prefix',
        default_value='',
        description='Set of commands/arguments to preceed the node command (e.g. "timem --").',
    )

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                example_dir,
                                'launch',
                                'utils',
                                'common_nodes_launch.py',
                            ]
                        )
                    ]
                )
            ),
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'bag',
                    'play',
                    os.path.join(example_dir, 'bags', 'perfect_odometry'),
                    '--rate',
                    '3',
                ],
                output='own_log',
                on_exit=[Shutdown()],
            ),
            Node(
                package=package,
                executable=node,
                name='amcl',
                output='screen',
                parameters=[os.path.join(example_dir, 'config', 'params.yaml')],
                arguments=['--ros-args', '--log-level', 'info'],
                prefix=prefix,
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(package_launch_arg)
    ld.add_action(node_launch_arg)
    ld.add_action(prefix_launch_arg)
    ld.add_action(load_nodes)

    return ld
