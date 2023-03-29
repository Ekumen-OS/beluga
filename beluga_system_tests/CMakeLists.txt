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

  enable_testing()
  add_subdirectory(test)
endif()
