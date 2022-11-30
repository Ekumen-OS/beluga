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

find_package(ament_cmake REQUIRED)
find_package(range-v3 REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(${PROJECT_NAME} range-v3)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

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

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE .clang-format)

  find_package(ament_cmake_clang_tidy REQUIRED)
  ament_clang_tidy(${CMAKE_BINARY_DIR} CONFIG_FILE .clang-tidy)

  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  enable_testing()
  add_subdirectory(test)
endif()

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(range-v3)

ament_package()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
