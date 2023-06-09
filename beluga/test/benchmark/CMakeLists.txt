option(BELUGA_RUN_PERFORMANCE_TESTS
  "Enable performance tests instead of unconditionally skipping them" OFF)

add_executable(benchmark_beluga
  benchmark_likelihood_field_model.cpp
  benchmark_raycasting.cpp
  benchmark_sampling.cpp
  benchmark_spatial_hash.cpp
  benchmark_tuple_vector.cpp
  benchmark_main.cpp)
target_include_directories(benchmark_beluga PRIVATE ../beluga/include)
target_link_libraries(benchmark_beluga
  PUBLIC benchmark::benchmark
  PRIVATE ${PROJECT_NAME} beluga_compile_options)
set(TEST_RESULTS_DIR "${CMAKE_BINARY_DIR}/test_results/${PROJECT_NAME}")
file(MAKE_DIRECTORY ${TEST_RESULTS_DIR})

set(benchmark_out "${TEST_RESULTS_DIR}/benchmark_beluga.google_benchmark.json")
if(BELUGA_RUN_PERFORMANCE_TESTS)
  add_test(
    NAME benchmark_beluga
    COMMAND benchmark_beluga "--benchmark_out_format=json" "--benchmark_out=${benchmark_out}")
endif()
