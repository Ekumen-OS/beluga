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

# Run clang-tidy for static analysis in the Beluga repository.

[[ -z "${WITHIN_DEV}" ]] && echo -e "\033[1;33mWARNING: Try running this script inside the development container if you experience any issues.\033[0m"

set -o errexit -o xtrace

OPEN_VDB="Off"

if [ "${ROS_DISTRO}" != "noetic" ]; then
    ROS_PACKAGES="beluga beluga_ros beluga_amcl beluga_system_tests"
    if [ "${ROS_DISTRO}" != "humble" ]; then
        OPEN_VDB="On"
    fi
else
    ROS_PACKAGES="beluga beluga_ros beluga_amcl"
fi

source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --packages-up-to ${ROS_PACKAGES} --event-handlers=console_cohesion+ --symlink-install --mixin ccache --cmake-args -DUSE_OPENVDB=${OPEN_VDB}
echo ${ROS_PACKAGES} |
    xargs -n1 echo |
    # NOTE: `-Wno-gnu-zero-variadic-macro-arguments` is needed due to
    # https://github.com/google/googletest/issues/2650, fixed in 1.11 but not backported to 1.10.
    xargs -I{} run-clang-tidy \
        -j=4 \
        -fix \
        -extra-arg='-Wno-gnu-zero-variadic-macro-arguments' \
        -p=./build/{} \
        ${PWD}/src/.*
