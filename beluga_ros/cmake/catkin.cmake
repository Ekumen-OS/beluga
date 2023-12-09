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

find_package(beluga REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS nav_msgs
             sensor_msgs
             tf2
             tf2_eigen
             tf2_geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
    sensor_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
  DEPENDS beluga
  INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(${catkin_DEFINITIONS})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(
  ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} INTERFACE beluga::beluga
                                                ${catkin_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} INTERFACE BELUGA_ROS_VERSION=1)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING OR BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
