include(cmake/conan.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_BINARY_DIR})

conan_cmake_configure(
  REQUIRES   abseil/20220623.0 range-v3/0.12.0
  GENERATORS cmake_find_package)

conan_cmake_autodetect(settings)

conan_cmake_install(
  PATH_OR_REFERENCE .
  BUILD missing
  REMOTE conancenter
  SETTINGS ${settings})

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
