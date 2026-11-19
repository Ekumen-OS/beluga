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

add_library(beluga_ros_test_method_plugins SHARED test_method_plugins.cpp)
target_include_directories(beluga_ros_test_method_plugins
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(beluga_ros_test_method_plugins PUBLIC ${PROJECT_NAME})

set(LOCAL_AMENT_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/test_method")
set(METHOD_PLUGINS_RESOURCE_RELPATH
    "share/ament_index/resource_index/beluga_plugins/beluga_ros")
file(
  GENERATE
  OUTPUT "${LOCAL_AMENT_PREFIX}/${METHOD_PLUGINS_RESOURCE_RELPATH}"
  CONTENT "$<TARGET_FILE:beluga_ros_test_method_plugins>\n")

ament_add_ros_isolated_gmock(
  node_test
  node_test.cpp
  ENV
  "AMENT_PREFIX_PATH=${LOCAL_AMENT_PREFIX}")
target_include_directories(node_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set(_test_config_path "${PROJECT_SOURCE_DIR}/config/beluga_filter_node.yaml")
target_compile_definitions(
  node_test
  PRIVATE BELUGA_ESTIMATION_ROS_TEST_CONFIG_PATH="${_test_config_path}")
target_link_libraries(node_test ${PROJECT_NAME} ${PROJECT_NAME}_component)
add_dependencies(node_test beluga_ros_test_method_plugins)
