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

"""Launch file for the beluga_estimation_ekf variant of the EKF odometry benchmark."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the Beluga node."""
    pkg_share = get_package_share_directory("beluga_estimation_ekf_benchmark")
    default_params = os.path.join(
        pkg_share, "params", "beluga_estimation_ekf.ros2.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Full path to the ROS 2 parameters file for the Beluga node",
            ),
            Node(
                package="beluga_estimation_ros",
                executable="node",
                name="beluga",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
        ]
    )
