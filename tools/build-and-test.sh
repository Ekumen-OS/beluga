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

SCRIPT_PATH=$(dirname "$(readlink -f "$0")")

set -o errexit

CMAKE_EXTRA_ARGS=""
COLCON_EXTRA_ARGS=""

if [ "${ROS_DISTRO}" != "noetic" ]; then
    if [ "${ROS_DISTRO}" != "jazzy" ] && [ "${ROS_DISTRO}" != "rolling" ]; then
        ROS_PACKAGES="beluga beluga_ros beluga_amcl beluga_benchmark beluga_example beluga_system_tests beluga_tools"
    else
        ROS_PACKAGES="beluga beluga_ros beluga_amcl beluga_system_tests beluga_tools"
        if [ "${ROS_DISTRO}" != "humble" ] && [ "${ROS_DISTRO}" != "iron" ]; then
            ROS_PACKAGES="beluga beluga_ros beluga_amcl beluga_system_tests beluga_tools beluga_vdb"
        fi
    fi
else
    ROS_PACKAGES="beluga beluga_ros beluga_amcl beluga_example"
fi


if [ "${CMAKE_EXTRA_ARGS}" != "" ]; then
    COLCON_EXTRA_ARGS="${COLCON_EXTRA_ARGS} --cmake-args ${CMAKE_EXTRA_ARGS}"
fi

source /opt/ros/${ROS_DISTRO}/setup.sh

echo ::group::Release Build
colcon build \
    --event-handlers console_cohesion+ \
    --packages-up-to ${ROS_PACKAGES} \
    --symlink-install \
    --mixin \
        build-testing-on \
        ccache \
        release \
    --cmake-force-configure \
    ${COLCON_EXTRA_ARGS}
echo ::endgroup::


echo ::group::Debug Build
colcon build \
    --packages-up-to ${ROS_PACKAGES} \
    --event-handlers console_cohesion+ \
    --symlink-install \
    --mixin \
        build-testing-on \
        ccache \
        coverage-gcc \
        coverage-pytest \
        debug \
    --cmake-force-configure \
    ${COLCON_EXTRA_ARGS}
echo ::endgroup::

LCOV_CONFIG_PATH=${SCRIPT_PATH}/../.lcovrc

echo ::group::Test
colcon lcov-result \
    --initial \
    --lcov-config-file ${LCOV_CONFIG_PATH} \
    --packages-select ${ROS_PACKAGES}
colcon test \
    --packages-select ${ROS_PACKAGES} \
    --event-handlers console_cohesion+ \
    --return-code-on-test-failure \
    --mixin coverage-pytest
echo ::endgroup::

echo ::group::Generate code coverage results
colcon lcov-result \
    --packages-select ${ROS_PACKAGES} \
    --lcov-config-file ${LCOV_CONFIG_PATH}
colcon coveragepy-result \
    --packages-select ${ROS_PACKAGES} \
    --coverage-report-args -m
echo ::endgroup::
