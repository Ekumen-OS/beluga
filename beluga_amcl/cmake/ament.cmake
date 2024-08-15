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
find_package(beluga_ros REQUIRED)
find_package(bondcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(beluga_amcl_ros2_common SHARED)
target_sources(beluga_amcl_ros2_common PRIVATE src/ros2_common.cpp)

target_include_directories(
  beluga_amcl_ros2_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(
  beluga_amcl_ros2_common
  PUBLIC beluga_ros
         bondcpp
         rclcpp
         rclcpp_components
         rclcpp_lifecycle
         std_srvs)

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PRIVATE src/amcl_node.cpp)

install(
  TARGETS beluga_amcl_ros2_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

target_include_directories(
  amcl_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(amcl_node_component PUBLIC beluga_amcl_ros2_common)

target_compile_features(amcl_node_component PUBLIC cxx_std_17)

ament_target_dependencies(
  amcl_node_component
  PUBLIC beluga
         beluga_ros
         bondcpp
         rclcpp
         rclcpp_components
         rclcpp_lifecycle
         std_srvs)

rclcpp_components_register_node(
  amcl_node_component
  PLUGIN "beluga_amcl::AmclNode"
  EXECUTABLE amcl_node)

ament_export_dependencies(
  beluga
  beluga_ros
  bondcpp
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs)
ament_export_include_directories("include/${PROJECT_NAME}")

install(
  TARGETS amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_library(ndt_amcl_node_component SHARED)

target_sources(ndt_amcl_node_component PRIVATE src/ndt_amcl_node.cpp)

target_include_directories(
  ndt_amcl_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(ndt_amcl_node_component PUBLIC cxx_std_17)

target_link_libraries(ndt_amcl_node_component PUBLIC beluga_amcl_ros2_common)

ament_target_dependencies(
  ndt_amcl_node_component
  PUBLIC beluga
         beluga_ros
         bondcpp
         rclcpp
         rclcpp_components
         rclcpp_lifecycle
         std_srvs)

rclcpp_components_register_node(
  ndt_amcl_node_component
  PLUGIN "beluga_amcl::NdtAmclNode"
  EXECUTABLE ndt_amcl_node)

ament_export_dependencies(
  beluga
  beluga_ros
  bondcpp
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs)
ament_export_include_directories("include/${PROJECT_NAME}")

install(
  TARGETS ndt_amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ndt_amcl_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

add_library(ndt_amcl_node_3d_component SHARED)
target_sources(ndt_amcl_node_3d_component PRIVATE src/ndt_amcl_node_3d.cpp)

target_include_directories(
  ndt_amcl_node_3d_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(ndt_amcl_node_3d_component PUBLIC cxx_std_17)

target_link_libraries(ndt_amcl_node_3d_component PUBLIC beluga_amcl_ros2_common)

ament_target_dependencies(
  ndt_amcl_node_3d_component
  PUBLIC beluga
         beluga_ros
         bondcpp
         rclcpp
         rclcpp_components
         rclcpp_lifecycle
         std_srvs)

rclcpp_components_register_node(
  ndt_amcl_node_3d_component
  PLUGIN "beluga_amcl::NdtAmclNode3D"
  EXECUTABLE ndt_amcl_node_3d)

install(
  TARGETS ndt_amcl_node_3d_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ndt_amcl_node_3d DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(
  beluga
  beluga_ros
  bondcpp
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_srvs)
ament_export_include_directories("include/${PROJECT_NAME}")

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
