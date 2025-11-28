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

option(BELUGA_RUN_PERFORMANCE_TESTS
       "Enable performance tests instead of unconditionally skipping them" OFF)

add_executable(
  benchmark_beluga
  benchmark_likelihood_field_model.cpp
  benchmark_low_variance_sample.cpp
  benchmark_main.cpp
  benchmark_raycasting.cpp
  benchmark_spatial_hash.cpp
  benchmark_take_while_kld.cpp
  benchmark_tuple_vector.cpp)
target_include_directories(benchmark_beluga PRIVATE ../beluga/include)
target_link_libraries(
  benchmark_beluga
  PUBLIC benchmark::benchmark
  PRIVATE ${PROJECT_NAME} beluga_compile_options)
set(TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}")
file(MAKE_DIRECTORY ${TEST_RESULTS_DIR})

set(BENCHMARK_OUT "${TEST_RESULTS_DIR}/benchmark_beluga.google_benchmark.json")
if(BELUGA_RUN_PERFORMANCE_TESTS)
  add_test(NAME benchmark_beluga
           COMMAND benchmark_beluga "--benchmark_out_format=json"
                   "--benchmark_out=${BENCHMARK_OUT}")
endif()
