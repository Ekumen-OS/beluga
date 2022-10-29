include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)

conan_basic_setup(TARGETS)

include(FetchContent)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.11.0)

set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "Disable testing of the benchmark library.")

FetchContent_Declare(
  googlebenchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.7.0)

FetchContent_MakeAvailable(
  googletest
  googlebenchmark)

add_subdirectory(core)
