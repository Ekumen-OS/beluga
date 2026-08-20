# Copyright 2026 Ekumen, Inc.
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

"""Launch file for the Beluga AMCL benchmarking environment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

pkg_dir = get_package_share_directory("beluga_ros2")

default_yaml_path = os.path.join(pkg_dir, "params", "default.ros2.yaml")


def generate_launch_description():
    """Generates the launch description for the Beluga AMCL benchmarking environment.

    Declares launch arguments for the map path, laser model type,
    and maximum particles, and configures the required ROS 2 nodes:
    beluga_amcl, map_server, and lifecycle_manager.

    Returns:
        LaunchDescription: The complete ROS 2 launch description object.
    """
    config_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_yaml_path,
        description="Absolute YAML file path",
    )

    map_path_arg = DeclareLaunchArgument("map_path", description="Absolute map path")

    laser_model_arg = DeclareLaunchArgument(
        "laser_model_type",
        default_value="likelihood_field",
        description="Sensor model",
        choices=["likelihood_field", "beam"],
    )

    max_particles_arg = DeclareLaunchArgument(
        "max_particles", default_value="2000", description="Max number of particles"
    )

    beluga_node = Node(
        package="beluga_amcl",
        executable="amcl_node",
        name="beluga_amcl",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "laser_model_type": LaunchConfiguration("laser_model_type"),
                "max_particles": LaunchConfiguration("max_particles"),
            },
        ],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map_path")}],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server", "beluga_amcl"]}],
    )

    return LaunchDescription([
        config_file_arg,
        map_path_arg,
        laser_model_arg,
        max_particles_arg,
        beluga_node,
        map_server_node,
        lifecycle_manager_node,
    ])
