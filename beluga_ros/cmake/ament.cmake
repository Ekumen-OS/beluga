# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR
                             ${PROJECT_SOURCE_DIR}/tools)

install(DIRECTORY tools USE_SOURCE_PERMISSIONS DESTINATION lib/${PROJECT_NAME})

add_library(beluga_ros INTERFACE)
target_include_directories(
  beluga_ros INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(beluga_ros INTERFACE beluga::beluga)
ament_target_dependencies(
  beluga_ros
  INTERFACE geometry_msgs
            nav_msgs
            sensor_msgs
            tf2
            tf2_eigen
            tf2_geometry_msgs)
target_compile_definitions(beluga_ros INTERFACE BELUGA_ROS_VERSION=2)
target_compile_features(beluga_ros INTERFACE cxx_std_17)

ament_export_dependencies(
  beluga
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_definitions(BELUGA_ROS_VERSION=2)
ament_export_targets(beluga_ros HAS_LIBRARY_TARGET)

install(
  TARGETS beluga_ros
  EXPORT beluga_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

ament_package()
