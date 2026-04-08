# Copyright 2024 Ekumen, Inc.
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

project(beluga_tutorial)

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

find_package(beluga REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${yaml_cpp_vendor_INCLUDE_DIRS})

if(NOT PARANOID)
  set_target_properties(yaml-cpp PROPERTIES SYSTEM ON)
endif()
target_link_libraries(${PROJECT_NAME} beluga::beluga yaml-cpp)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
