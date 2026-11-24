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

add_executable(middleware_interface_test middleware_interface_test.cpp)
target_link_libraries(middleware_interface_test PRIVATE ${PROJECT_NAME}
                                                        GTest::gmock_main mocks)
gtest_discover_tests(middleware_interface_test)

add_executable(parameters_interface_test parameters_interface_test.cpp)
target_link_libraries(parameters_interface_test PRIVATE ${PROJECT_NAME}
                                                        GTest::gmock_main mocks)
gtest_discover_tests(parameters_interface_test)

add_executable(parameter_value_test parameter_value_test.cpp)
target_link_libraries(parameter_value_test PRIVATE ${PROJECT_NAME}
                                                   GTest::gmock_main)
gtest_discover_tests(parameter_value_test)

add_executable(topics_interface_test topics_interface_test.cpp)
target_link_libraries(topics_interface_test PRIVATE ${PROJECT_NAME}
                                                    GTest::gmock_main mocks)
gtest_discover_tests(topics_interface_test)

add_executable(transform_interface_test transform_interface_test.cpp)
target_link_libraries(transform_interface_test PRIVATE ${PROJECT_NAME}
                                                       GTest::gmock_main mocks)
gtest_discover_tests(transform_interface_test)

add_executable(message_pipe_test message_pipe_test.cpp)
target_link_libraries(message_pipe_test PRIVATE ${PROJECT_NAME}
                                                GTest::gmock_main mocks)
gtest_discover_tests(message_pipe_test)
