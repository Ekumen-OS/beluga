cmake_minimum_required(VERSION 3.16)
project(beluga_benchmark)

set(ROS_VERSION $ENV{ROS_VERSION})

if(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(beluga_amcl REQUIRED)

  ament_python_install_package(${PROJECT_NAME})

  install(DIRECTORY
    scripts/profiling
    scripts/benchmarking
    USE_SOURCE_PERMISSIONS
    DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()
elseif(ROS_VERSION EQUAL 1)
  install(CODE "message(STATUS 'Nothing to be done')")
else()
  message(FATAL_ERROR "Unknown ROS distribution version: ${ROS_VERSION}")
endif()
