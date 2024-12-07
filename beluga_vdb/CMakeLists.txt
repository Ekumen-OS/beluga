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

project(beluga_vdb VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OPENVDB_CMAKE_MODULE_PATH
    ${OPENVDB_CMAKE_MODULE_PATH}
    CACHE PATH "Path to OpenVDB CMake module")

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
  target_compile_options(-fno-inline)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(range-v3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(beluga REQUIRED)

if(NOT OPENVDB_CMAKE_MODULE_PATH)
  file(
    GLOB_RECURSE
    OPENVDB_MODULES
    /usr/lib/*/FindOpenVDB.cmake
    /usr/local/lib/*/FindOpenVDB.cmake)
  list(LENGTH OPENVDB_MODULES NUM_OPENVDB_MODULES)
  if(NUM_OPENVDB_MODULES EQUAL 1)
    list(
      GET
      OPENVDB_MODULES
      0
      OPENVDB_MODULE)
    get_filename_component(OPENVDB_CMAKE_MODULE_PATH ${OPENVDB_MODULE}
                           DIRECTORY)
  endif()
  unset(NUM_OPENVDB_MODULES)
  unset(OPENVDB_MODULES)
endif()

if(OPENVDB_CMAKE_MODULE_PATH)
  list(APPEND CMAKE_MODULE_PATH ${OPENVDB_CMAKE_MODULE_PATH})
endif()

find_package(OpenVDB REQUIRED)

if(OPENVDB_CMAKE_MODULE_PATH)
  list(POP_BACK CMAKE_MODULE_PATH)
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE beluga::beluga OpenVDB::openvdb)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")
  enable_testing()
  add_subdirectory(test)
endif()
