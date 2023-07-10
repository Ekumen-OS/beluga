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

find_package(GTest MODULE)
if(NOT TARGET GTest::gmock_main)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG 03597a01ee50ed33e9dfd640b249b4be3799d395)
  # For Windows: Prevent overriding the parent project's compiler/linker
  # settings
  set(GTEST_FORCE_SHARED_CRT
      ON
      CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

find_package(benchmark REQUIRED)
if(NOT TARGET benchmark::benchmark_main)
  include(FetchContent)
  # cmake-lint: disable=C0103
  # Invalid INTERNAL variable name doesn't match `_[A-Z][0-9A-Z_]+`
  set(BENCHMARK_ENABLE_GTEST_TESTS
      OFF
      CACHE INTERNAL "")
  FetchContent_Declare(
    benchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.7.1.zip)
  # For Windows: Prevent overriding the parent project's compiler/linker
  # settings
  set(GTEST_FORCE_SHARED_CRT
      ON
      CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(benchmark)
endif()

include(CTest)
add_subdirectory(benchmark)
add_subdirectory(beluga)
