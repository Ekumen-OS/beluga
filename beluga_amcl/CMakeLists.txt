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

project(beluga_amcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wconversion
    -Wextra
    -Werror
    -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-fno-inline)
endif()

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(beluga_ros REQUIRED)
find_package(bondcpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)

add_library(beluga_amcl_ros2_common SHARED)
target_sources(beluga_amcl_ros2_common PRIVATE src/ros2_common.cpp)

target_include_directories(
  beluga_amcl_ros2_common
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${std_srvs_INCLUDE_DIRS})

target_link_libraries(
  beluga_amcl_ros2_common
  PUBLIC beluga_ros::beluga_ros
         bondcpp::bondcpp
         rclcpp::rclcpp
         rclcpp_components::component
         rclcpp_lifecycle::rclcpp_lifecycle
         ${std_srvs_TARGETS})

target_compile_definitions(
  beluga_amcl_ros2_common
  PUBLIC
    BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MAJOR=${message_filters_VERSION_MAJOR}
    BELUGA_AMCL_MESSAGE_FILTERS_VERSION_MINOR=${message_filters_VERSION_MINOR}
    BELUGA_AMCL_MESSAGE_FILTERS_VERSION_PATCH=${message_filters_VERSION_PATCH})

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PRIVATE src/amcl_node.cpp)

install(
  TARGETS beluga_amcl_ros2_common
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

target_include_directories(
  amcl_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${std_srvs_INCLUDE_DIRS})

target_link_libraries(amcl_node_component PUBLIC beluga_amcl_ros2_common)

target_compile_features(amcl_node_component PUBLIC cxx_std_17)

target_link_libraries(
  amcl_node_component
  PUBLIC beluga::beluga
         beluga_ros::beluga_ros
         bondcpp::bondcpp
         rclcpp::rclcpp
         rclcpp_components::component
         rclcpp_lifecycle::rclcpp_lifecycle
         ${std_srvs_TARGETS})

rclcpp_components_register_node(
  amcl_node_component
  PLUGIN "beluga_amcl::AmclNode"
  EXECUTABLE amcl_node)

install(
  TARGETS amcl_node_component
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node DESTINATION lib/${PROJECT_NAME})

add_library(ndt_amcl_node_component SHARED)

target_sources(ndt_amcl_node_component PRIVATE src/ndt_amcl_node.cpp)

target_include_directories(
  ndt_amcl_node_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${std_srvs_INCLUDE_DIRS})

target_compile_features(ndt_amcl_node_component PUBLIC cxx_std_17)

target_link_libraries(ndt_amcl_node_component PUBLIC beluga_amcl_ros2_common)

target_link_libraries(
  ndt_amcl_node_component
  PUBLIC beluga::beluga
         beluga_ros::beluga_ros
         bondcpp::bondcpp
         rclcpp::rclcpp
         rclcpp_components::component
         rclcpp_lifecycle::rclcpp_lifecycle
         ${std_srvs_TARGETS})

rclcpp_components_register_node(
  ndt_amcl_node_component
  PLUGIN "beluga_amcl::NdtAmclNode"
  EXECUTABLE ndt_amcl_node)

install(
  TARGETS ndt_amcl_node_component
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ndt_amcl_node DESTINATION lib/${PROJECT_NAME})

add_library(ndt_amcl_node_3d_component SHARED)
target_sources(ndt_amcl_node_3d_component PRIVATE src/ndt_amcl_node_3d.cpp)

target_include_directories(
  ndt_amcl_node_3d_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${std_srvs_INCLUDE_DIRS})

target_compile_features(ndt_amcl_node_3d_component PUBLIC cxx_std_17)

target_link_libraries(ndt_amcl_node_3d_component PUBLIC beluga_amcl_ros2_common)

target_link_libraries(
  ndt_amcl_node_3d_component
  PUBLIC beluga::beluga
         beluga_ros::beluga_ros
         bondcpp::bondcpp
         rclcpp::rclcpp
         rclcpp_components::component
         rclcpp_lifecycle::rclcpp_lifecycle
         ${std_srvs_TARGETS})

rclcpp_components_register_node(
  ndt_amcl_node_3d_component
  PLUGIN "beluga_amcl::NdtAmclNode3D"
  EXECUTABLE ndt_amcl_node_3d)

install(
  TARGETS ndt_amcl_node_3d_component
  EXPORT ${PROJECT_NAME}
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
ament_export_targets(${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
