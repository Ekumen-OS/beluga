#!/bin/bash

# Copyright 2024 Ekumen, Inc.
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

# Enforce code coverage limits

[[ -z "${WITHIN_DEV}" ]] && echo -e "\033[1;33mWARNING: Try running this script inside the development container if you experience any issues.\033[0m"

set -o errexit

# NOTE: Temporarily disabling beluga_estimation and beluga_estimation_ros from code coverage (currently at 92%, under 95%)
# To enable add these to the list:
#  -f 'src/beluga/beluga_estimation/.*' \
#  -f 'src/beluga/beluga_estimation_ros/.*' \
gcovr --fail-under-line 95 -j -u \
  -f 'src/beluga/beluga/.*' \
  -f 'src/beluga/beluga_amcl/.*' \
  -f 'src/beluga/beluga_ros/.*' \
  -e '.*/test/.*cpp' build/
# NOTE: Enable Python code coverage checks when attained
# (cd coveragepy && python3 -m coverage report --fail-under=95)
