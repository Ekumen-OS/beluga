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

if [ "${ROS_DISTRO}" != "noetic" ]; then
    ROS_PACKAGES="beluga beluga_system_tests"
else
    ROS_PACKAGES="beluga"
fi

source /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --packages-up-to ${ROS_PACKAGES} --event-handlers=console_cohesion+ --symlink-install --mixin ccache
echo ${ROS_PACKAGES} | xargs -n1 echo | xargs -I{} run-clang-tidy -quiet -j 4 -p ./build/{}
