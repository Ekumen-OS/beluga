find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga_amcl REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_SOURCE_DIR}/${PROJECT_NAME})

install(DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.bag" EXCLUDE)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.rviz")

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.yaml")

install(DIRECTORY maps models worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
