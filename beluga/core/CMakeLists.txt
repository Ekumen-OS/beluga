find_package(range-v3)

add_library(core INTERFACE)
target_include_directories(core INTERFACE include)
target_link_libraries(core INTERFACE range-v3::range-v3)
target_compile_features(core INTERFACE cxx_std_17)

add_executable(core_benchmark)
target_sources(core_benchmark PRIVATE
  benchmark/tuple_vector_benchmark.cpp
  benchmark/spatial_hash_benchmark.cpp)
target_link_libraries(core_benchmark PRIVATE benchmark_main core)
target_compile_options(core_benchmark PRIVATE -Wall -Werror)

add_executable(core_test)
target_sources(core_test PRIVATE
  test/particle_filter_test.cpp
  test/tuple_vector_test.cpp
  test/spatial_hash_test.cpp)
target_link_libraries(core_test PRIVATE gmock_main core)
target_compile_options(core_test PRIVATE -Wall -Werror)

include(GoogleTest)
gtest_discover_tests(core_test)
