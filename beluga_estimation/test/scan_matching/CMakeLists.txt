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

add_executable(synthetic_scan_pair_test synthetic_scan_pair_test.cpp)
target_link_libraries(synthetic_scan_pair_test PRIVATE ${PROJECT_NAME}
                                                       GTest::gtest_main)
target_include_directories(synthetic_scan_pair_test
                           PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
gtest_discover_tests(synthetic_scan_pair_test)

add_executable(planar_scan_matcher_contract_test
               planar_scan_matcher_contract_test.cpp)
target_link_libraries(planar_scan_matcher_contract_test
                      PRIVATE ${PROJECT_NAME} GTest::gtest_main)
target_include_directories(planar_scan_matcher_contract_test
                           PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
gtest_discover_tests(planar_scan_matcher_contract_test)
