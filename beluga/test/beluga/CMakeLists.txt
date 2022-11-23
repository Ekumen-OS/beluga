find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(test_beluga
  algorithm/test_exponential_filter.cpp
  algorithm/test_particle_filter.cpp
  algorithm/test_sampling.cpp
  test_tuple_vector.cpp
  test_spatial_hash.cpp)
target_link_libraries(test_beluga ${PROJECT_NAME})
target_compile_options(test_beluga PRIVATE -Wno-sign-compare)
