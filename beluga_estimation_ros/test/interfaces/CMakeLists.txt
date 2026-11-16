# Copyright 2026 Ekumen, Inc.
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

ament_add_ros_isolated_gmock(middleware_test middleware_test.cpp)
target_link_libraries(middleware_test ${PROJECT_NAME})

ament_add_ros_isolated_gmock(clock_rate_test clock_rate_test.cpp)
target_link_libraries(clock_rate_test ${PROJECT_NAME})

ament_add_ros_isolated_gmock(parameters_test parameters_test.cpp)
target_link_libraries(parameters_test ${PROJECT_NAME})

ament_add_ros_isolated_gmock(topics_test topics_test.cpp)
target_link_libraries(topics_test ${PROJECT_NAME} ${std_msgs_TARGETS})

ament_add_ros_isolated_gmock(transform_test transform_test.cpp)
target_link_libraries(transform_test ${PROJECT_NAME})

ament_add_ros_isolated_gmock(logger_test logger_test.cpp)
target_link_libraries(logger_test ${PROJECT_NAME})

ament_add_ros_isolated_gmock(odometry_converter_test
                             odometry_converter_test.cpp)
target_link_libraries(odometry_converter_test ${PROJECT_NAME}
                      ${nav_msgs_TARGETS})
