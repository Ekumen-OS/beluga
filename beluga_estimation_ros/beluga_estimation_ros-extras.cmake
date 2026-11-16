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

# copied from beluga_estimation_ros/beluga_estimation_ros-extras.cmake

# register ament_package() hook for node plugins once.
macro(_beluga_estimation_ros_register_package_hook)
  if(NOT DEFINED _beluga_estimation_ros_package_hook_registered)
    set(_beluga_estimation_ros_package_hook_registered TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    ament_register_extension("ament_package" "beluga_estimation_ros"
                             "beluga_estimation_ros_package_hook.cmake")
  endif()
endmacro()

_beluga_estimation_ros_register_package_hook()

include(
  "${beluga_estimation_ros_DIR}/beluga_estimation_ros_register_plugins.cmake")
