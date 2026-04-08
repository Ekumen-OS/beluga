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

project(beluga_ros)

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
find_package(beluga REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(_deps
    ${geometry_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs)

if(NOT PARANOID)
  foreach(target IN ITEMS ${_deps})
    if(TARGET ${target})
      set_target_properties(${target} PROPERTIES SYSTEM ON)
    endif()
  endforeach()
endif()

add_library(beluga_ros)
target_sources(beluga_ros PRIVATE src/amcl.cpp src/ndt_ellipsoid.cpp)
target_include_directories(
  beluga_ros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(beluga_ros PUBLIC beluga::beluga ${_deps})
target_compile_features(beluga_ros PUBLIC cxx_std_17)

set_target_properties(beluga_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_export_dependencies(
  beluga
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  visualization_msgs)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(beluga_ros HAS_LIBRARY_TARGET)

install(
  TARGETS beluga_ros
  EXPORT beluga_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
