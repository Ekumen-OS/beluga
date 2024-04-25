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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions.composable_node import ComposableNode
from launch.utilities.type_utils import get_typed_value

from launch_ros.actions import SetParameter

from beluga_example.launch_utils import with_launch_arguments
from beluga_example.launch_utils import (
    get_node_with_arguments_declared_from_params_file,
)


def get_launch_arguments():
    example_dir_path = Path(get_package_share_directory('beluga_example'))
    return [
        DeclareLaunchArgument(
            name='localization_package',
            default_value='beluga_amcl',
            description='Package that provides the localization node to launch.',
            choices=['beluga_amcl'],
        ),
        DeclareLaunchArgument(
            name='localization_node',
            default_value='ndt_amcl_node',
            description='Localization node to launch.',
        ),
        DeclareLaunchArgument(
            name='localization_plugin',
            default_value='beluga_amcl::AmclNode',
            description='Localization node plugin to use if composition is enabled. ',
        ),
        DeclareLaunchArgument(
            name='localization_prefix',
            default_value='',
            description='Set of commands/arguments to precede the node command (e.g. "timem --").',
        ),
        DeclareLaunchArgument(
            name='localization_params_file',
            default_value=str(example_dir_path / 'params' / 'default.ros2.yaml'),
            description='Parameters file to be used to run the localization node.',
        ),
        DeclareLaunchArgument(
            name='localization_map',
            default_value=str(example_dir_path / 'maps' / 'turtlebot3_world.yaml'),
            description='Map YAML file to be used by the localization node.',
        ),
        DeclareLaunchArgument(
            name='use_composition',
            default_value='False',
            description='Whether to use composed node bringup.',
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Whether to use simulation clock.',
        ),
    ]


@with_launch_arguments(get_launch_arguments())
def generate_launch_description(
    localization_package,
    localization_node,
    localization_plugin,
    localization_prefix,
    localization_params_file,
    localization_map,
    use_composition,
    use_sim_time,
):
    use_composition = get_typed_value(use_composition, bool)
    use_sim_time = get_typed_value(use_sim_time, bool)
    print(localization_params_file, flush=True)

    load_nodes = GroupAction(
        actions=[
            *get_node_with_arguments_declared_from_params_file(
                package=localization_package,
                executable=localization_node,
                namespace='',
                name='ndt_amcl_node',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                prefix=localization_prefix,
                params_file=localization_params_file
            ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                namespace='',
                name='map_server',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[
                    {'yaml_filename': localization_map},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace='',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                sigterm_timeout='20',
                sigkill_timeout='20',
                parameters=[
                    {'autostart': True},
                    {'node_names': ['map_server', 'ndt_amcl_node']},
                ],
            ),
        ]
    )

    load_composable_nodes = GroupAction(
        actions=[
            ComposableNodeContainer(
                package='rclcpp_components',
                executable='component_container',
                namespace='',
                name='amcl_component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package=localization_package,
                        plugin=localization_plugin,
                        name='amcl',
                        parameters=[localization_params_file],
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[
                            {'yaml_filename': localization_map},
                        ],
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_localization',
                        parameters=[
                            {'autostart': True},
                            {'node_names': ['map_server', 'ndt_amcl_node']},
                        ],
                    ),
                ],
            ),
        ]
    )

    return LaunchDescription(
        [
            SetParameter('use_sim_time', use_sim_time),
            load_composable_nodes if use_composition else load_nodes,
        ]
    )
