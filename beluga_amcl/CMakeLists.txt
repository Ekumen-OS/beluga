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
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(amcl_node_utils SHARED)
target_sources(amcl_node_utils PRIVATE
  src/amcl_node_utils.cpp)
target_include_directories(amcl_node_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(amcl_node_utils
  PUBLIC beluga::beluga ${sensor_msgs_TARGETS}
  INTERFACE ${geometry_msgs_TARGETS} ${nav2_msgs_TARGETS} tf2::tf2 tf2_eigen::tf2_eigen tf2_geometry_msgs::tf2_geometry_msgs)
target_compile_features(amcl_node_utils PUBLIC cxx_std_17)

add_library(amcl_node_component SHARED)
target_sources(amcl_node_component PRIVATE
  src/amcl_node.cpp
  src/amcl_node_utils.cpp)
target_compile_features(amcl_node_component PUBLIC cxx_std_17)
target_link_libraries(amcl_node_component PUBLIC beluga::beluga amcl_node_utils)
ament_target_dependencies(amcl_node_component PUBLIC
  bondcpp
  nav2_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs)
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
ament_export_targets(amcl_node_utils HAS_LIBRARY_TARGET)

install(TARGETS
  amcl_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  amcl_node_utils EXPORT amcl_node_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS amcl_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  PATTERN "beluga_amcl/amcl_node.hpp" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
