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

add_executable(euclidean_variable_test euclidean_variable_test.cpp)
target_link_libraries(euclidean_variable_test PRIVATE ${PROJECT_NAME}
                                                      GTest::gmock_main mocks)
gtest_discover_tests(euclidean_variable_test)

add_executable(se2_variable_test se2_variable_test.cpp)
target_link_libraries(se2_variable_test PRIVATE ${PROJECT_NAME}
                                                GTest::gmock_main mocks)
gtest_discover_tests(se2_variable_test)
