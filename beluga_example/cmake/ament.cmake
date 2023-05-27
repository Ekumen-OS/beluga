find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga_amcl REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE)

install(DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.bag" EXCLUDE)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.rviz2")

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "nodelet*" EXCLUDE)

install(DIRECTORY maps models worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
