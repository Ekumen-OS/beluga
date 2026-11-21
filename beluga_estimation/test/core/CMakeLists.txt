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

add_executable(identifier_test identifier_test.cpp)
target_link_libraries(identifier_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(identifier_test)

add_executable(network_test network_test.cpp)
target_link_libraries(network_test PRIVATE ${PROJECT_NAME} GTest::gmock_main
                                           mocks)
gtest_discover_tests(network_test)

add_executable(graphviz_dumper_test graphviz_dumper_test.cpp)
target_link_libraries(graphviz_dumper_test PRIVATE ${PROJECT_NAME}
                                                   GTest::gmock_main mocks)
gtest_discover_tests(graphviz_dumper_test)

add_executable(on_manifold_estimate_test on_manifold_estimate_test.cpp)
target_link_libraries(on_manifold_estimate_test PRIVATE ${PROJECT_NAME}
                                                        GTest::gmock_main mocks)
gtest_discover_tests(on_manifold_estimate_test)

add_executable(estimate_interface_test estimate_interface_test.cpp)
target_link_libraries(estimate_interface_test PRIVATE ${PROJECT_NAME}
                                                      GTest::gmock_main mocks)
gtest_discover_tests(estimate_interface_test)

add_executable(estimate_store_test estimate_store_test.cpp)
target_link_libraries(estimate_store_test PRIVATE ${PROJECT_NAME}
                                                  GTest::gmock_main mocks)
gtest_discover_tests(estimate_store_test)
