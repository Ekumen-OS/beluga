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

cmake_minimum_required(VERSION 3.16)
project(beluga_benchmark)

set(ROS_VERSION $ENV{ROS_VERSION})
if(NOT ROS_VERSION)
  if(CATKIN_DEVEL_PREFIX OR CATKIN_BUILD_BINARY_PACKAGE)
    set(ROS_VERSION 1)
  else()
    find_package(ament_cmake QUIET)
    if(ament_cmake_FOUND)
      set(ROS_VERSION 2)
    endif()
  endif()
endif()
if(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(beluga_amcl REQUIRED)

  ament_python_install_package(${PROJECT_NAME})

  install(DIRECTORY scripts/profiling scripts/benchmarking
                    USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})

  ament_package()
elseif(ROS_VERSION EQUAL 1)
  install(CODE "message(STATUS 'Nothing to be done')")
else()
  message(FATAL_ERROR "Unknown ROS distribution version: ${ROS_VERSION}")
endif()
