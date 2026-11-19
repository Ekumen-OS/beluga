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

add_executable(recursive_filter_test recursive_filter_test.cpp)
target_link_libraries(recursive_filter_test PRIVATE ${PROJECT_NAME}
                                                    GTest::gmock_main mocks)
target_compile_definitions(recursive_filter_test
                           PRIVATE BELUGA_ESTIMATION_LINK_STATIC_REGISTRY)
gtest_discover_tests(recursive_filter_test)

add_executable(odometry_motion_model_test odometry_motion_model_test.cpp)
target_link_libraries(odometry_motion_model_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(odometry_motion_model_test)

add_subdirectory(operations)
