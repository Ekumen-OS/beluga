find_package(ament_cmake_google_benchmark REQUIRED)

ament_add_google_benchmark_executable(benchmark_beluga
  benchmark_likelihood_field_model.cpp
  benchmark_sampling.cpp
  benchmark_spatial_hash.cpp
  benchmark_tuple_vector.cpp)
target_link_libraries(benchmark_beluga ${PROJECT_NAME})
