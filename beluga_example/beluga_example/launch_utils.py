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

from typing import List

from launch import LaunchDescription
from launch import LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
import launch.logging
from launch.utilities import perform_substitutions
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import Parameter

import yaml


def with_launch_arguments(arguments_to_declare: List[DeclareLaunchArgument]):
    """Decorate generate_launch_description function to resolve launch arguments early."""

    def decorator(get_launch_description_f):
        def wrapper():
            launch_configurations_for_arguments = [
                LaunchConfiguration(arg.name) for arg in arguments_to_declare
            ]

            def action(context):
                resolved_args = {
                    perform_substitutions(
                        context, config.variable_name
                    ): config.perform(context)
                    for config in launch_configurations_for_arguments
                }
                return [
                    IncludeLaunchDescription(
                        LaunchDescriptionSource(
                            get_launch_description_f(**resolved_args)
                        )
                    )
                ]

            return LaunchDescription(
                [
                    *arguments_to_declare,
                    OpaqueFunction(function=action),
                ]
            )

        return wrapper

    return decorator


def get_node_with_arguments_declared_from_params_file(
    *, params_file, name, namespace='/', **kwargs
):
    """
    Declare arguments from param file.

    Currently ROS 2 argument handling is a bit broken ...
    If you specify a parameter file, there's no way to override the parameter value, except with
    other parameter file.

    Some magic to be able to easily override parameter values specified in a file from launch ...
    """
    full_name = f'{namespace}/{name}'
    full_name_without_starting_slash = full_name.lstrip('/')
    full_name_with_slash = f'/{full_name_without_starting_slash}'
    with open(params_file) as params_file:
        params_file_dict = yaml.safe_load(params_file)
    node_name_key = None
    if full_name_with_slash in params_file_dict:
        node_name_key = full_name_with_slash
    if full_name_without_starting_slash in params_file_dict:
        node_name_key = full_name_without_starting_slash
    if node_name_key is None:
        launch.logging.get_logger().warning(
            f"parameters file has no parameters for node '{full_name_without_starting_slash}'"
        )
        return [Node(name=name, namespace=namespace, **kwargs)]
    entries_for_node = params_file_dict[node_name_key]
    if 'ros__parameters' not in entries_for_node:
        launch.logging.get_logger().warning(
            f"parameters file does not have a 'ros__parameters' entry for node "
            f"'{full_name_without_starting_slash}'"
        )
        return [Node(name=name, namespace=namespace, **kwargs)]
    params_dict = entries_for_node['ros__parameters']
    actions = []
    parameters = []
    for param_name, param_value in params_dict.items():
        actions.append(
            DeclareLaunchArgument(
                name=f'{full_name_without_starting_slash}.{param_name}',
                default_value=str(param_value),
            )
        )
        parameters.append(
            Parameter(
                name=param_name,
                value=LaunchConfiguration(
                    f'{full_name_without_starting_slash}.{param_name}'
                ),
            )
        )
    actions.append(
        Node(name=name, namespace=namespace, parameters=parameters, **kwargs)
    )
    return actions
