# Copyright 2023 Ekumen, Inc.
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

find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(test_messages test_messages.cpp)
target_compile_options(test_messages PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_messages beluga_ros)

ament_add_gmock(test_occupancy_grid test_occupancy_grid.cpp)
target_compile_options(test_occupancy_grid PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_occupancy_grid beluga_ros)

ament_add_gmock(test_tf2_sophus test_tf2_sophus.cpp)
target_compile_options(test_tf2_sophus PRIVATE -Wno-deprecated-copy)
target_link_libraries(test_tf2_sophus beluga_ros)
