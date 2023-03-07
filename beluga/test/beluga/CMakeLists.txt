find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(test_beluga
  algorithm/test_distance_map.cpp
  algorithm/test_estimation.cpp
  algorithm/test_exponential_filter.cpp
  algorithm/test_particle_filter.cpp
  algorithm/test_sampling.cpp
  mixin/test_utility.cpp
  motion/test_differential_drive_model.cpp
  motion/test_omnidirectional_drive_model.cpp
  random/test_multivariate_normal_distribution.cpp
  resampling_policies/test_resample_interval_policy.cpp
  resampling_policies/test_resample_on_motion_policy.cpp
  resampling_policies/test_resampling_policies_poller.cpp
  resampling_policies/test_selective_resampling_policy.cpp
  sensor/test_likelihood_field_model.cpp
  test_tuple_vector.cpp
  test_spatial_hash.cpp)
target_link_libraries(test_beluga ${PROJECT_NAME})
target_compile_options(test_beluga PRIVATE -Wno-sign-compare)
