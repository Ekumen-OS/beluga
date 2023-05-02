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
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PRIVATE src/amcl_node.cpp)
target_include_directories(amcl_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(amcl_node_component PUBLIC cxx_std_17)
target_link_libraries(amcl_node_component PUBLIC beluga::beluga)
ament_target_dependencies(amcl_node_component PUBLIC
  bondcpp
  message_filters
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
rclcpp_components_register_nodes(amcl_node_component "beluga_amcl::AmclNode")

add_executable(amcl_node)
target_sources(amcl_node PRIVATE src/main.cpp)
target_link_libraries(amcl_node PUBLIC amcl_node_component)
target_compile_features(amcl_node PUBLIC cxx_std_17)
ament_target_dependencies(amcl_node PUBLIC rclcpp)

ament_export_dependencies(
  beluga
  bondcpp
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
)
ament_export_include_directories("include/${PROJECT_NAME}")

install(TARGETS
  amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "beluga_amcl/private" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
