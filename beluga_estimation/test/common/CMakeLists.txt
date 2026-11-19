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

add_executable(type_casts_test type_casts_test.cpp)
target_link_libraries(type_casts_test PRIVATE ${PROJECT_NAME} GTest::gmock_main
                                              mocks)
gtest_discover_tests(type_casts_test)

add_executable(type_traits_test type_traits_test.cpp)
target_link_libraries(type_traits_test PRIVATE ${PROJECT_NAME}
                                               GTest::gmock_main mocks)
gtest_discover_tests(type_traits_test)

add_executable(std_future_test std_future_test.cpp)
target_link_libraries(std_future_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(std_future_test)

add_executable(registry_test registry_test.cpp)
target_link_libraries(registry_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
target_compile_definitions(registry_test
                           PRIVATE BELUGA_ESTIMATION_LINK_STATIC_REGISTRY)
gtest_discover_tests(registry_test)

add_executable(builtin_registry_test builtin_registry_test.cpp)
target_link_libraries(builtin_registry_test PRIVATE ${PROJECT_NAME}
                                                    GTest::gmock_main)
target_compile_definitions(builtin_registry_test
                           PRIVATE BELUGA_ESTIMATION_LINK_STATIC_REGISTRY)
gtest_discover_tests(builtin_registry_test)

add_executable(random_test random_test.cpp)
target_link_libraries(random_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(random_test)

add_executable(views_test views_test.cpp)
target_link_libraries(views_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(views_test)

add_executable(algebra_test algebra_test.cpp)
target_link_libraries(algebra_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(algebra_test)

add_executable(format_test format_test.cpp)
target_link_libraries(format_test PRIVATE ${PROJECT_NAME} GTest::gmock_main
                                          mocks)
gtest_discover_tests(format_test)

add_executable(error_test error_test.cpp)
target_link_libraries(error_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(error_test)

add_executable(assert_test assert_test.cpp)
target_link_libraries(assert_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(assert_test)

add_executable(interval_test interval_test.cpp)
target_link_libraries(interval_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(interval_test)

add_executable(slice_test slice_test.cpp)
target_link_libraries(slice_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(slice_test)

add_executable(bitset_test bitset_test.cpp)
target_link_libraries(bitset_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(bitset_test)

add_executable(probability_test probability_test.cpp)
target_link_libraries(probability_test PRIVATE ${PROJECT_NAME}
                                               GTest::gmock_main)
gtest_discover_tests(probability_test)

add_executable(time_aware_buffer_test time_aware_buffer_test.cpp)
target_link_libraries(time_aware_buffer_test PRIVATE ${PROJECT_NAME}
                                                     GTest::gmock_main)
gtest_discover_tests(time_aware_buffer_test)

add_executable(ranges_test ranges_test.cpp)
target_link_libraries(ranges_test PRIVATE ${PROJECT_NAME} GTest::gmock_main)
gtest_discover_tests(ranges_test)
