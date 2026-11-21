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

project(beluga_estimation VERSION 3.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

set(DEFAULT_PARANOID OFF)
if(DEFINED ENV{BELUGA_PARANOID})
  set(DEFAULT_PARANOID $ENV{BELUGA_PARANOID})
endif()
option(PARANOID "Enable paranoid mode" ${DEFAULT_PARANOID})
unset(DEFAULT_PARANOID)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Sophus REQUIRED)
find_package(range-v3 REQUIRED)
find_package(concurrentqueue REQUIRED)
find_package(tl-expected REQUIRED)
find_package(fmt REQUIRED)

include(FetchContent)
FetchContent_Declare(
  kokkos_mdspan
  GIT_REPOSITORY https://github.com/kokkos/mdspan.git
  GIT_TAG 80fc772eb812b45097c28fc0a46d8ff006138d69 # stable
  SYSTEM)
FetchContent_MakeAvailable(kokkos_mdspan)

add_library(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_sources(
  ${PROJECT_NAME}
  PRIVATE src/common/random.cpp
          src/core/network.cpp
          src/core/estimate_store.cpp
          src/interfaces/parameter_value.cpp
          src/middleware/message_pipe.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         Sophus::Sophus
         mdspan::mdspan
         range-v3::range-v3
         tl::expected
         fmt::fmt
  PRIVATE $<BUILD_INTERFACE:concurrentqueue::concurrentqueue>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(
    ${PROJECT_NAME}
    PRIVATE -Wall
            -Wconversion
            -Wextra
            -Wpedantic
            $<$<BOOL:${PARANOID}>:-Werror>)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Debug")
  target_compile_options(${PROJECT_NAME} PRIVATE -fno-inline)
endif()
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_NO_DEBUG
                                                  SOPHUS_USE_BASIC_LOGGING)

add_executable(clang_tidy_findable)
target_sources(clang_tidy_findable PRIVATE src/clang_tidy_findable.cpp)
target_link_libraries(clang_tidy_findable PRIVATE ${PROJECT_NAME})

option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")
  enable_testing()
  add_subdirectory(test)
endif()

set(INSTALL_CMAKEDIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} mdspan # forward dependency
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKEDIR})

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${INSTALL_CMAKEDIR})
