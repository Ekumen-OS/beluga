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

find_package(Doxygen)

if(DOXYGEN_FOUND)
  include(FetchContent)

  FetchContent_Declare(
    doxygen-awesome-css
    GIT_REPOSITORY https://github.com/jothepro/doxygen-awesome-css
    GIT_TAG v2.2.0
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome)

  FetchContent_MakeAvailable(doxygen-awesome-css)

  # cmake-lint: disable=C0301
  # Line too long
  add_custom_target(
    beluga_docs ALL
    COMMAND
      DOXYGEN_OUTPUT_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}
      DOXYGEN_HTML_STYLESHEET=${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome/doxygen-awesome.css
      ${DOXYGEN_EXECUTABLE} ./Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
else()
  message(WARNING "Documentation skipped, doxygen not found.")
endif()
