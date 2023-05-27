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
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import SetParameter
from launch_ros.descriptions.composable_node import ComposableNode


def generate_launch_description():
    example_dir = get_package_share_directory('beluga_example')

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                example_dir,
                                'launch',
                                'utils',
                                'flatland_launch.py',
                            ]
                        )
                    ]
                )
            ),
            ComposableNodeContainer(
                name='component_container',
                package='rclcpp_components',
                namespace='',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='beluga_amcl',
                        plugin='beluga_amcl::AmclNode',
                        name='amcl',
                        parameters=[os.path.join(example_dir, 'params', 'node.yaml')],
                    )
                ],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)

    return ld
