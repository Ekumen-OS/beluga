# Copyright 2023-2024 Ekumen, Inc.
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

find_package(ament_cmake_ros REQUIRED)

file(COPY test_data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

ament_add_ros_isolated_gmock(test_ros2_common test_ros2_common.cpp)
target_compile_options(test_ros2_common PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_ros2_common beluga_amcl_ros2_common)
target_include_directories(test_ros2_common
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_utils)

ament_add_ros_isolated_gmock(test_amcl_node test_amcl_node.cpp)
target_compile_options(test_amcl_node PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_amcl_node amcl_node_component)
target_include_directories(test_amcl_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_utils)

ament_add_ros_isolated_gmock(
  test_ndt_amcl_node
  test_ndt_amcl_node.cpp
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(test_ndt_amcl_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_utils)
target_compile_options(test_ndt_amcl_node PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_ndt_amcl_node ndt_amcl_node_component)

ament_add_ros_isolated_gmock(
  test_ndt_amcl_3d_node
  test_ndt_amcl_3d_node.cpp
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(test_ndt_amcl_3d_node
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test_utils)
target_compile_options(test_ndt_amcl_3d_node PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_ndt_amcl_3d_node ndt_amcl_node_3d_component)
