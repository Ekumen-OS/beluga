cmake_minimum_required(VERSION 3.16)
project(beluga_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga_amcl REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  bags
  config
  launch
  maps
  models
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
