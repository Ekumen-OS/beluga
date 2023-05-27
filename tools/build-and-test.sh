#!/bin/bash

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

# Run tests and generate code coverage report.

[[ -z "${WITHIN_DEV}" ]] && echo -e "\033[1;33mWARNING: Try running this script inside the development container if you experience any issues.\033[0m"

set -o errexit

ROS_PACKAGES="beluga beluga_amcl beluga_benchmark beluga_example beluga_system_tests"

source /opt/ros/${ROS_DISTRO}/setup.sh
# Do a build without coverage flags first to avoid generating .gcno files
# that prevent the html output from lcov from being generated correctly.
colcon build --packages-up-to ${ROS_PACKAGES} --event-handlers=console_cohesion+ --symlink-install --mixin ccache
colcon build --packages-up-to ${ROS_PACKAGES} --event-handlers=console_cohesion+ --symlink-install --mixin ccache coverage-gcc coverage-pytest
colcon lcov-result --initial
colcon test --packages-select ${ROS_PACKAGES} --event-handlers=console_cohesion+ --return-code-on-test-failure --mixin coverage-pytest
colcon lcov-result --packages-select ${ROS_PACKAGES} --verbose
colcon coveragepy-result --packages-select ${ROS_PACKAGES} --verbose --coverage-report-args -m
