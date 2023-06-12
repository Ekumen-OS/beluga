cmake_minimum_required(VERSION 3.16)

project(beluga_amcl)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Wextra -Werror -Wpedantic)
endif()

set(ROS_VERSION $ENV{ROS_VERSION})
if(ROS_VERSION EQUAL 2)
  include(cmake/ament.cmake)
elseif(ROS_VERSION EQUAL 1)
  include(cmake/catkin.cmake)
else()
  message(FATAL_ERROR "Unknown ROS distribution version: ${ROS_VERSION}")
endif()
