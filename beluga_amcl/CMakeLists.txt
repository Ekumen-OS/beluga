cmake_minimum_required(VERSION 3.16)

project(beluga_amcl)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PUBLIC
  src/amcl_node.cpp
  src/amcl_node_utils.cpp)
target_include_directories(amcl_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(amcl_node_component PUBLIC cxx_std_17)
ament_target_dependencies(amcl_node_component PUBLIC
  beluga
  bondcpp
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  tf2_eigen
  tf2_geometry_msgs)
rclcpp_components_register_nodes(amcl_node_component "beluga_amcl::AmclNode")

add_executable(amcl_node)
target_sources(amcl_node PUBLIC src/main.cpp)
target_link_libraries(amcl_node PUBLIC amcl_node_component)
target_compile_features(amcl_node PUBLIC cxx_std_17)
ament_target_dependencies(amcl_node PUBLIC rclcpp)

install(TARGETS
  amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
