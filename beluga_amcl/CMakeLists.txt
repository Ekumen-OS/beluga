cmake_minimum_required(VERSION 3.16)

project(beluga_amcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Wextra -Werror -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(beluga REQUIRED)
find_package(bondcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(amcl_node)
target_sources(amcl_node PUBLIC src/main.cpp src/amcl_node.cpp)
target_include_directories(amcl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(amcl_node PUBLIC cxx_std_17)
ament_target_dependencies(amcl_node PUBLIC
  beluga
  bondcpp
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle)

install(TARGETS amcl_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
