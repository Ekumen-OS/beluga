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

add_executable(generic_tangent_distribution_test
               generic_tangent_distribution_test.cpp)
target_link_libraries(generic_tangent_distribution_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(generic_tangent_distribution_test)

add_executable(se2_normal_distribution_test se2_normal_distribution_test.cpp)
target_link_libraries(se2_normal_distribution_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_normal_distribution_test)

add_executable(dirac_mixture_distribution_test
               dirac_mixture_distribution_test.cpp)
target_link_libraries(dirac_mixture_distribution_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(dirac_mixture_distribution_test)

add_executable(normal_distribution_test normal_distribution_test.cpp)
target_link_libraries(normal_distribution_test PRIVATE ${PROJECT_NAME}
                                                       GTest::gmock_main mocks)
gtest_discover_tests(normal_distribution_test)

add_executable(se2_tangent_distribution_test se2_tangent_distribution_test.cpp)
target_link_libraries(se2_tangent_distribution_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_tangent_distribution_test)
