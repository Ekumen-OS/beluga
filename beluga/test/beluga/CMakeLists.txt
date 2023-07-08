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

add_executable(
  test_beluga
  algorithm/raycasting/test_bresenham.cpp
  algorithm/test_distance_map.cpp
  algorithm/test_estimation.cpp
  algorithm/test_exponential_filter.cpp
  algorithm/test_particle_filter.cpp
  algorithm/test_raycasting.cpp
  algorithm/test_sampling.cpp
  mixin/test_descriptor.cpp
  mixin/test_utility.cpp
  motion/test_differential_drive_model.cpp
  motion/test_omnidirectional_drive_model.cpp
  random/test_multivariate_normal_distribution.cpp
  sensor/data/test_cache_friendly_grid_storage.cpp
  sensor/data/test_dense_grid.cpp
  sensor/data/test_grid_storage_types.cpp
  sensor/data/test_occupancy_grid.cpp
  sensor/data/test_regular_grid.cpp
  sensor/test_beam_model.cpp
  sensor/test_likelihood_field_model.cpp
  test_mixin.cpp
  test_spatial_hash.cpp
  test_storage.cpp
  test_tuple_vector.cpp
  type_traits/test_strongly_typed_numeric.cpp)

target_link_libraries(
  test_beluga PRIVATE ${PROJECT_NAME} beluga_compile_options GTest::gmock_main)
target_include_directories(test_beluga PRIVATE include)
target_compile_options(test_beluga PRIVATE -Wno-sign-compare)

include(GoogleTest)
gtest_discover_tests(test_beluga)
