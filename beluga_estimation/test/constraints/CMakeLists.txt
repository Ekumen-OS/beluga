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

add_executable(normal_constraint_test normal_constraint_test.cpp)
target_link_libraries(normal_constraint_test PRIVATE ${PROJECT_NAME}
                                                     GTest::gmock_main mocks)
gtest_discover_tests(normal_constraint_test)

add_executable(normal_prior_constraint_test normal_prior_constraint_test.cpp)
target_link_libraries(normal_prior_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(normal_prior_constraint_test)

add_executable(normal_likelihood_constraint_test
               normal_likelihood_constraint_test.cpp)
target_link_libraries(normal_likelihood_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(normal_likelihood_constraint_test)

add_executable(se2_normal_pose_constraint_test
               se2_normal_pose_constraint_test.cpp)
target_link_libraries(se2_normal_pose_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_normal_pose_constraint_test)
add_executable(imu_step_constraint_test imu_step_constraint_test.cpp)
target_link_libraries(imu_step_constraint_test PRIVATE ${PROJECT_NAME}
                                                       GTest::gmock_main)
gtest_discover_tests(imu_step_constraint_test)

add_executable(se2_normal_pose_prior_constraint_test
               se2_normal_pose_prior_constraint_test.cpp)
target_link_libraries(se2_normal_pose_prior_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_normal_pose_prior_constraint_test)

add_executable(se2_normal_pose_likelihood_constraint_test
               se2_normal_pose_likelihood_constraint_test.cpp)
target_link_libraries(se2_normal_pose_likelihood_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)

gtest_discover_tests(se2_normal_pose_likelihood_constraint_test)
add_executable(se2_pose_kinematic_constraint_test
               se2_pose_kinematic_constraint_test.cpp)
target_link_libraries(se2_pose_kinematic_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_pose_kinematic_constraint_test)

add_executable(se2_velocity_kinematic_constraint_test
               se2_velocity_kinematic_constraint_test.cpp)
target_link_libraries(se2_velocity_kinematic_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_velocity_kinematic_constraint_test)

add_executable(se2_constant_velocity_kinematic_constraint_test
               se2_constant_velocity_kinematic_constraint_test.cpp)
target_link_libraries(se2_constant_velocity_kinematic_constraint_test
                      PRIVATE ${PROJECT_NAME} GTest::gmock_main mocks)
gtest_discover_tests(se2_constant_velocity_kinematic_constraint_test)
