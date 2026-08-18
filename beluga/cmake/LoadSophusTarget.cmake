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

# This file can be removed after https://github.com/strasdat/Sophus/pull/558

if(NOT TARGET Sophus::Sophus)
  find_path(
    Sophus_CMAKE_DIR
    NAMES SophusTargets.cmake
    PATH_SUFFIXES share/sophus/cmake REQUIRED)
  find_package(fmt REQUIRED)
  include("${Sophus_CMAKE_DIR}/SophusTargets.cmake")
endif()
