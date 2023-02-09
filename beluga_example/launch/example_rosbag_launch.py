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

from launch import Action
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
import launch.logging
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import Parameter

import yaml


class PlayBag(Action):
    def __init__(self, *, start_paused, example_dir_path, condition=None, **kwargs):
        self._start_paused = normalize_typed_substitution(start_paused, bool)
        self._example_dir_path = example_dir_path
        self._kwargs = kwargs
        super().__init__(condition=condition)

    def execute(self, context):
        start_paused = perform_typed_substitution(context, self._start_paused, bool)
        cmd = [
            'ros2',
            'bag',
            'play',
            str(self._example_dir_path / 'bags' / 'perfect_odometry'),
            '--rate',
            '3',
        ]
        if start_paused:
            cmd.append('--start-paused')
        return [
            ExecuteProcess(
                cmd=cmd, output='own_log', on_exit=[Shutdown()], **self._kwargs
            )
        ]


def get_declare_arguments_and_parameters_from_yaml_file(params_file, node_name):
    """
    Declare arguments from param file.

    Currently ROS 2 argument handling is a bit broken ...
    If you specify a parameter file, there's no way to override the parameter value, except with other parameter file.

    Some magic to be able to easily override parameter values specified in a file from launch ...
    """
    node_name_without_starting_slash = node_name.lstrip('/')
    node_name_with_slash = f'/{node_name_without_starting_slash}'
    params_file_dict = yaml.safe_load(params_file)
    node_name_key = None
    if node_name_with_slash in params_file_dict:
        node_name_key = node_name_with_slash
    if node_name_without_starting_slash in params_file_dict:
        node_name_key = node_name_without_starting_slash
    if node_name_key is None:
        launch.logging.get_logger().warn(
            f"parameters file has no parameters for node '{node_name}'"
        )
        return [], []
    entries_for_node = params_file_dict[node_name_key]
    if 'ros__parameters' not in entries_for_node:
        launch.logging.get_logger().warn(
            f"parameters file does not have a 'ros__parameters' entry for node '{node_name}'"
        )
        return [], []
    params_dict = entries_for_node['ros__parameters']
    declared_launch_arguments = []
    parameters = []
    for name, value in params_dict.items():
        declared_launch_arguments.append(
            DeclareLaunchArgument(
                name=f'{node_name}.{name}',
                default_value=str(value),
            )
        )
        parameters.append(
            Parameter(
                name=name,
                value=LaunchConfiguration(f'{node_name}.{name}'),
            )
        )
    return declared_launch_arguments, parameters


def generate_launch_description():
    example_dir_path = Path(get_package_share_directory('beluga_example'))

    package = LaunchConfiguration('package')
    node = LaunchConfiguration('node')
    prefix = LaunchConfiguration('prefix')
    start_paused = LaunchConfiguration('start_paused')

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

    start_paused_launch_arg = DeclareLaunchArgument(
        name='start_paused',
        default_value='False',
        description='Start the rosbag player in a paused state.',
    )

    node_name = 'amcl'
    params_file_path = example_dir_path / 'config' / 'params.yaml'
    with params_file_path.open() as params_file:
        (
            amcl_params_args,
            amcl_params,
        ) = get_declare_arguments_and_parameters_from_yaml_file(params_file, node_name)

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
            PlayBag(start_paused=start_paused, example_dir_path=example_dir_path),
            Node(
                package=package,
                executable=node,
                namespace='',
                name='amcl',
                output='screen',
                parameters=amcl_params,
                arguments=['--ros-args', '--log-level', 'info'],
                prefix=prefix,
            ),
        ]
    )

    return LaunchDescription(
        [
            package_launch_arg,
            node_launch_arg,
            prefix_launch_arg,
            start_paused_launch_arg,
            *amcl_params_args,
            load_nodes,
        ]
    )
