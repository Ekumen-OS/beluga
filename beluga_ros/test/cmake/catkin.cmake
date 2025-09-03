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

find_package(rostest REQUIRED)

include_directories(include)

catkin_add_gmock(test_amcl test_amcl.cpp)
target_link_libraries(
  test_amcl
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_messages test_messages.cpp)
target_link_libraries(
  test_messages
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_occupancy_grid test_occupancy_grid.cpp)
target_link_libraries(
  test_occupancy_grid
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_tf2_sophus test_tf2_sophus.cpp)
target_link_libraries(
  test_tf2_sophus
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_laser_scan test_laser_scan.cpp)
target_link_libraries(
  test_laser_scan
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_particle_cloud test_particle_cloud.cpp)
target_link_libraries(
  test_particle_cloud
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_point_cloud test_point_cloud.cpp)
target_link_libraries(
  test_point_cloud
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)

catkin_add_gmock(test_sparse_point_cloud test_sparse_point_cloud.cpp)
target_link_libraries(
  test_sparse_point_cloud
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gmock_main)
