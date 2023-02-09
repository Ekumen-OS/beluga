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
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.utilities.type_utils import get_typed_value

from launch_ros.actions import SetParameter

from beluga_example.launch_utils import with_launch_arguments
from beluga_example.launch_utils import (
    get_node_with_arguments_declared_from_params_file,
)


def get_launch_arguments():
    example_dir_path = Path(get_package_share_directory('beluga_example'))
    params_file_path = example_dir_path / 'config' / 'params.yaml'
    return [
        DeclareLaunchArgument(
            name='package',
            default_value='beluga_amcl',
            description='Package that provides the localization node to launch.',
            choices=['beluga_amcl', 'nav2_amcl'],
        ),
        DeclareLaunchArgument(
            name='node',
            default_value='amcl_node',
            description='Localization node to launch.',
        ),
        DeclareLaunchArgument(
            name='prefix',
            default_value='',
            description='Set of commands/arguments to preceed the node command (e.g. "timem --").',
        ),
        DeclareLaunchArgument(
            name='start_paused',
            default_value='False',
            description='Start the rosbag player in a paused state.',
        ),
        DeclareLaunchArgument(
            name='amcl_parameters_file',
            default_value=str(params_file_path),
            description='Parameters file to be used to run amcl',
        ),
        DeclareLaunchArgument(
            name='record_bag',
            default_value='False',
            description='If to record a bagfile or not',
        ),
        DeclareLaunchArgument(
            name='topics_to_record',
            default_value='[/tf, /pose]',
            description='If to record a bagfile or not',
        ),
    ]


@with_launch_arguments(get_launch_arguments())
def generate_launch_description(
    package,
    node,
    prefix,
    start_paused,
    amcl_parameters_file,
    record_bag,
    topics_to_record,
):
    example_dir_path = Path(get_package_share_directory('beluga_example'))

    start_paused = get_typed_value(start_paused, bool)
    record_bag = get_typed_value(record_bag, bool)
    topics_to_record = get_typed_value(topics_to_record, List[str])
    bag_play_cmd = [
        'ros2',
        'bag',
        'play',
        str(example_dir_path / 'bags' / 'perfect_odometry'),
        '--rate',
        '3',
    ]
    if start_paused:
        bag_play_cmd.append('--start-paused')

    other_nodes = []
    if record_bag:
        other_nodes.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', *topics_to_record],
                output='own_log',
            )
        )
    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        example_dir_path / 'launch' / 'utils' / 'common_nodes_launch.py'
                    )
                )
            ),
            ExecuteProcess(
                cmd=bag_play_cmd,
                output='own_log',
                on_exit=[Shutdown()],
            ),
            *get_node_with_arguments_declared_from_params_file(
                package=package,
                executable=node,
                namespace='',
                name='amcl',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                prefix=prefix,
                params_file=amcl_parameters_file,
            ),
            *other_nodes,
        ]
    )

    return LaunchDescription([load_nodes])
