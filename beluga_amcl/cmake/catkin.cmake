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

include(cmake/lib/nodelets.cmake)

find_package(beluga REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS bondcpp
             beluga_ros
             diagnostic_updater
             dynamic_reconfigure
             message_filters
             nodelet
             roscpp
             std_srvs)

generate_dynamic_reconfigure_options(config/Amcl.cfg)

catkin_package(
  CATKIN_DEPENDS
    bondcpp
    beluga_ros
    roscpp
    std_srvs
  DEPENDS beluga
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})
add_definitions(${catkin_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME} PRIVATE src/particle_filtering.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} beluga::beluga ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_nodelet SHARED)
target_sources(${PROJECT_NAME}_nodelet PRIVATE src/amcl_nodelet.cpp)
target_compile_features(${PROJECT_NAME}_nodelet PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_nodelet PUBLIC ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)

add_nodelet_executable(amcl_node "beluga_amcl/AmclNodelet")
target_compile_features(amcl_node PUBLIC cxx_std_17)

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS amcl_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "beluga_amcl/private" EXCLUDE)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)

if(CATKIN_ENABLE_TESTING OR BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()
