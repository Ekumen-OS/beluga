# Copyright 2024 Ekumen, Inc.
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

add_executable(test_beluga_vdb sensor/test_vdb_likelihood_field_model.cpp)

target_link_libraries(test_beluga_vdb PRIVATE ${PROJECT_NAME} beluga_vdb
                                              GTest::gmock_main)
target_include_directories(test_beluga_vdb PRIVATE include)
target_compile_options(test_beluga_vdb PRIVATE -Wno-sign-compare)

include(GoogleTest)
gtest_discover_tests(test_beluga_vdb)
