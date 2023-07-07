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

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(bondcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(amcl_node_utils SHARED)
target_sources(amcl_node_utils PRIVATE src/amcl_node_utils.cpp
                                       src/particle_filtering.cpp)
target_include_directories(
  amcl_node_utils PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(amcl_node_utils PUBLIC beluga::beluga)
ament_target_dependencies(
  amcl_node_utils
  PUBLIC
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs)
target_compile_definitions(amcl_node_utils PUBLIC BELUGA_AMCL_ROS_VERSION=2)
target_compile_features(amcl_node_utils PUBLIC cxx_std_17)

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PRIVATE src/amcl_node.cpp
                                           src/amcl_node_utils.cpp)
target_compile_features(amcl_node_component PUBLIC cxx_std_17)
target_link_libraries(amcl_node_component PUBLIC beluga::beluga amcl_node_utils)
ament_target_dependencies(
  amcl_node_component
  PUBLIC
  bondcpp
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_srvs)
rclcpp_components_register_node(
  amcl_node_component
  PLUGIN
  "beluga_amcl::AmclNode"
  EXECUTABLE
  amcl_node)

ament_export_dependencies(
  beluga
  bondcpp
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_definitions(BELUGA_AMCL_ROS_VERSION=2)
ament_export_targets(amcl_node_utils HAS_LIBRARY_TARGET)

install(
  TARGETS amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS amcl_node_utils
  EXPORT amcl_node_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "beluga_amcl/private" EXCLUDE)

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
