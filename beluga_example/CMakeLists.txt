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

cmake_minimum_required(VERSION 3.16)
project(beluga_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(beluga_amcl REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR
                             ${PROJECT_SOURCE_DIR}/${PROJECT_NAME})

install(
  DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.bag" EXCLUDE)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.rviz")

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.ros2.yaml")

install(DIRECTORY maps models worlds DESTINATION share/${PROJECT_NAME})

ament_package()
