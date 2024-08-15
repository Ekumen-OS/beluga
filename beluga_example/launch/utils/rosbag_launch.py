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
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.utilities.type_utils import get_typed_value

from launch_ros.actions import SetParameter

from beluga_example.launch_utils import with_launch_arguments


def get_launch_arguments():
    example_dir_path = Path(get_package_share_directory('beluga_example'))
    return [
        DeclareLaunchArgument(
            name='start_paused',
            default_value='False',
            description='Start the rosbag player in a paused state.',
        ),
        DeclareLaunchArgument(
            name='playback_rate',
            default_value='1.',
            description='Rate used to playback the bag.',
        ),
        DeclareLaunchArgument(
            name='rosbag_path',
            default_value=str(example_dir_path / 'bags' / 'perfect_odometry'),
            description='Path of the rosbag to playback.',
        ),
        DeclareLaunchArgument(
            name='record_bag',
            default_value='False',
            description='Whether to record a bagfile or not.',
        ),
        DeclareLaunchArgument(
            name='topics_to_record',
            default_value='[/tf, /amcl_pose, /pose, /odometry/ground_truth]',
            description='Topics to record in a new bagfile.',
        ),
        DeclareLaunchArgument(
            name='bagfile_output',
            default_value='',
            description='Destination bagfile to create, defaults to timestamped folder',
        ),
        DeclareLaunchArgument(
            name='clock',
            default_value='True',
            description='Publish to /clock. Defaults to True.',
        ),
    ]


@with_launch_arguments(get_launch_arguments())
def generate_launch_description(
    start_paused,
    rosbag_path,
    playback_rate,
    record_bag,
    topics_to_record,
    bagfile_output,
    clock,
):
    start_paused = get_typed_value(start_paused, bool)
    record_bag = get_typed_value(record_bag, bool)
    topics_to_record = get_typed_value(topics_to_record, List[str])
    clock = get_typed_value(clock, bool)

    bag_play_cmd = [
        'ros2',
        'bag',
        'play',
        rosbag_path,
        '--rate',
        playback_rate,
    ]
    if clock:
        bag_play_cmd.append('--clock')
    if start_paused:
        bag_play_cmd.append('--start-paused')

    other_nodes = []
    if record_bag:
        other_args = topics_to_record
        if bagfile_output != '':
            other_args.extend(('-o', bagfile_output))
        other_nodes.append(
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', *other_args],
                output='own_log',
            )
        )

    load_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', True),
            ExecuteProcess(
                cmd=bag_play_cmd,
                output='own_log',
                on_exit=[Shutdown()],
            ),
            *other_nodes,
        ]
    )

    return LaunchDescription([load_nodes])
