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

project(beluga_system_tests VERSION 1.0)

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

option(BUILD_TESTING "Build the testing tree." ON)

if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")

  enable_testing()
  add_subdirectory(test)
endif()

install(CODE "message(STATUS 'Nothing to be done')")
