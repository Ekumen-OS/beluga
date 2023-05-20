cmake_minimum_required(VERSION 3.16)
project(beluga_benchmark)

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
