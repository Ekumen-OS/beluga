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

#
# Register a beluga plugin library with the ament resource index.
#
# The passed library can contain multiple plugins.
#
# :param target: the shared library target
# :type target: string
#
function(beluga_estimation_ros_register_plugins target)
  if(NOT TARGET ${target})
    message(
      FATAL_ERROR "beluga_estimation_ros_register_plugins() first argument "
                  "'${target}' is not a target")
  endif()
  get_target_property(target_type ${target} TYPE)
  if(NOT
     target_type
     STREQUAL
     "SHARED_LIBRARY")
    message(
      FATAL_ERROR "beluga_estimation_ros_register_plugins() first argument "
                  "'${target}' is not a shared library target")
  endif()

  if(WIN32)
    set(path "bin")
  else()
    set(path "lib")
  endif()
  set_property(
    DIRECTORY "${PROJECT_SOURCE_DIR}"
    APPEND
    PROPERTY BELUGA_ESTIMATION_ROS_REGISTERED_PLUGIN_LIBRARIES
             "${path}/$<TARGET_FILE_NAME:${target}>\n")
endfunction()
