find_package(GTest MODULE)
if(NOT TARGET GTest::gmock_main)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

find_package(benchmark REQUIRED)
if(NOT TARGET benchmark::benchmark_main)
  include(FetchContent)
  set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE INTERNAL "")
  FetchContent_Declare(
    benchmark
    URL https://github.com/google/benchmark/archive/refs/tags/v1.7.1.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(benchmark)
endif()

include(CTest)
add_subdirectory(benchmark)
add_subdirectory(beluga)
