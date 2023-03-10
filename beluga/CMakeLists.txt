cmake_minimum_required(VERSION 3.16)

project(beluga VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wconversion -Werror -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(range-v3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen Sophus::Sophus TBB::tbb range-v3::range-v3)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
target_compile_definitions(${PROJECT_NAME} INTERFACE EIGEN_NO_DEBUG SOPHUS_USE_BASIC_LOGGING)

add_executable(clang_tidy_findable)
target_sources(clang_tidy_findable PRIVATE src/clang_tidy_findable.cpp)
target_link_libraries(clang_tidy_findable PRIVATE ${PROJECT_NAME})
# Setting this so clang-tidy can detect the standard version from the compile commands.
# CMake may omit the -std= flag if the compiler's default standard level is at least
# that of the requested feature.
# https://cmake.org/cmake/help/v3.23/manual/cmake-compile-features.7.html#requiring-language-standards
target_compile_options(clang_tidy_findable PRIVATE -std=c++17)

if(BUILD_TESTING)
  message(STATUS "Build testing enabled.")

  enable_testing()
  add_subdirectory(test)
endif()

set(INSTALL_CMAKEDIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${INSTALL_CMAKEDIR}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${INSTALL_CMAKEDIR}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion )

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${INSTALL_CMAKEDIR}
)
