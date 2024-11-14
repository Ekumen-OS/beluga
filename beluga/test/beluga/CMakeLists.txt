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

file(COPY test_data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(
  test_beluga
  $<IF:$<BOOL:${USE_OPENVDB}>,sensor/test_likelihood_3d_field_model.cpp,>
  actions/test_assign.cpp
  actions/test_normalize.cpp
  actions/test_overlay.cpp
  actions/test_propagate.cpp
  actions/test_reweight.cpp
  algorithm/raycasting/test_bresenham.cpp
  algorithm/test_amcl_core.cpp
  algorithm/test_cluster_based_estimation.cpp
  algorithm/test_distance_map.cpp
  algorithm/test_effective_sample_size.cpp
  algorithm/test_estimation.cpp
  algorithm/test_exponential_filter.cpp
  algorithm/test_raycasting.cpp
  algorithm/test_thrun_recovery_probability_estimator.cpp
  algorithm/test_unscented_transform.cpp
  containers/test_circular_array.cpp
  containers/test_tuple_vector.cpp
  motion/test_differential_drive_model.cpp
  motion/test_omnidirectional_drive_model.cpp
  policies/test_every_n.cpp
  policies/test_on_effective_size_drop.cpp
  policies/test_on_motion.cpp
  policies/test_policy.cpp
  random/test_multivariate_normal_distribution.cpp
  random/test_multivariate_uniform_distribution.cpp
  sensor/data/test_dense_grid.cpp
  sensor/data/test_landmark_map.cpp
  sensor/data/test_laser_scan.cpp
  sensor/data/test_linear_grid.cpp
  sensor/data/test_ndt_cell.cpp
  sensor/data/test_occupancy_grid.cpp
  sensor/data/test_regular_grid.cpp
  sensor/data/test_sparse_value_grid.cpp
  sensor/test_beam_model.cpp
  sensor/test_bearing_sensor_model.cpp
  sensor/test_landmark_sensor_model.cpp
  sensor/test_lfm_with_unknown_space.cpp
  sensor/test_likelihood_field_model.cpp
  sensor/test_ndt_model.cpp
  test_3d_embedding.cpp
  test_primitives.cpp
  test_spatial_hash.cpp
  testing/test_sophus_matchers.cpp
  testing/test_sophus_printers.cpp
  type_traits/test_particle_traits.cpp
  type_traits/test_strongly_typed_numeric.cpp
  type_traits/test_tuple_traits.cpp
  utility/test_forward_like.cpp
  utility/test_indexing_iterator.cpp
  views/test_random_intersperse.cpp
  views/test_sample.cpp
  views/test_take_evenly.cpp
  views/test_take_while_kld.cpp)

target_link_libraries(
  test_beluga PRIVATE ${PROJECT_NAME} beluga_compile_options GTest::gmock_main)
target_include_directories(test_beluga PRIVATE include)
target_compile_options(test_beluga PRIVATE -Wno-sign-compare)

include(GoogleTest)
gtest_discover_tests(test_beluga)
