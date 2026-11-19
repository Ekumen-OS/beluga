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

cmake_minimum_required(VERSION 3.16)

project(beluga_estimation_ros VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

set(DEFAULT_PARANOID OFF)
if(DEFINED ENV{BELUGA_PARANOID})
  set(DEFAULT_PARANOID $ENV{BELUGA_PARANOID})
endif()
option(PARANOID "Enable paranoid mode" ${DEFAULT_PARANOID})
unset(DEFAULT_PARANOID)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wconversion
    -Wextra
    -Wpedantic
    $<$<BOOL:${PARANOID}>:-Werror>)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-fno-inline)
endif()

find_package(ament_cmake REQUIRED)
find_package(beluga_estimation REQUIRED)
find_package(bondcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME})
target_sources(
  ${PROJECT_NAME}
  PRIVATE src/converters/parameter_value.cpp
          src/interfaces/clock.cpp
          src/interfaces/middleware.cpp
          src/interfaces/parameters.cpp
          src/interfaces/rate.cpp
          src/interfaces/topics.cpp
          src/interfaces/transform.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ament_index_cpp::ament_index_cpp
         beluga_estimation::beluga_estimation
         class_loader::class_loader
         rclcpp::rclcpp
         ${nav_msgs_TARGETS}
         ${sensor_msgs_TARGETS}
         ${std_msgs_TARGETS}
         tf2_ros::tf2_ros)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_executable(clang_tidy_findable)
target_sources(clang_tidy_findable PRIVATE src/clang_tidy_findable.cpp)
target_link_libraries(clang_tidy_findable PRIVATE ${PROJECT_NAME})

add_library(${PROJECT_NAME}_component SHARED)
target_sources(${PROJECT_NAME}_component PRIVATE src/node.cpp)
target_compile_definitions(${PROJECT_NAME}_component
                           PRIVATE BELUGA_ESTIMATION_LINK_STATIC_REGISTRY)

target_include_directories(
  ${PROJECT_NAME}_component
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/private>)

target_link_libraries(
  ${PROJECT_NAME}_component
  PUBLIC ${PROJECT_NAME}
         beluga_estimation::beluga_estimation
         bondcpp::bondcpp
         ${lifecycle_msgs_TARGETS}
         rclcpp::rclcpp
         rclcpp_components::component
         rclcpp_lifecycle::rclcpp_lifecycle)

rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "beluga_estimation_ros::Node"
  EXECUTABLE node)

option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")
  enable_testing()
  add_subdirectory(test)
endif()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_component
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  beluga_estimation
  class_loader
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package(CONFIG_EXTRAS beluga_estimation_ros-extras.cmake)
