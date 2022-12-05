# Copyright 2022 Ekumen, Inc.
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
from launch_ros.actions import SetParameter
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = get_package_share_directory("beluga_example")

    load_nodes = GroupAction(
        actions=[
            SetParameter("use_sim_time", True),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "--display-config",
                    os.path.join(example_dir, "rviz", "rviz.rviz"),
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    {
                        "yaml_filename": os.path.join(
                            example_dir, "maps", "turtlebot3_world.yaml"
                        )
                    }
                ],
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="beluga_amcl",
                executable="amcl_node",
                name="amcl",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[os.path.join(example_dir, "config", "params.yaml")],
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {"autostart": True},
                    {"node_names": ["map_server", "amcl"]},
                ],
            ),
            Node(
                package="flatland_server",
                executable="flatland_server",
                output="screen",
                parameters=[
                    {
                        "world_path": os.path.join(
                            example_dir, "worlds", "turtlebot3_world.yaml"
                        )
                    },
                    {"update_rate": 200.0},
                    {"step_size": 0.005},
                    {"show_viz": False},
                    {"viz_pub_rate": 30.0},
                    {"use_sim_time": True},
                ],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)

    return ld
