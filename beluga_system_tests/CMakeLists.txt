cmake_minimum_required(VERSION 3.16)

project(beluga_system_tests VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Werror -Wextra -Wpedantic)
endif()

option(BUILD_TESTING "Build the testing tree." ON)

if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")

  find_package(ament_cmake_clang_format QUIET)
  if(ament_cmake_clang_format_FOUND)
    ament_clang_format(CONFIG_FILE .clang-format)
  endif()

  find_package(ament_cmake_clang_tidy QUIET)
  if (ament_cmake_clang_tidy_FOUND)
    ament_clang_tidy(${CMAKE_BINARY_DIR} CONFIG_FILE .clang-tidy)
  endif()

  find_package(ament_cmake_copyright QUIET)
  if(ament_cmake_copyright_FOUND)
    ament_copyright()
  endif()

  enable_testing()
  add_subdirectory(test)
endif()
