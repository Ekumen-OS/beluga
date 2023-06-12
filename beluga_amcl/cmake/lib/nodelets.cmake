function(add_nodelet_executable target)
  set(EXECUTABLE ${target})
  if (ARGC LESS 2)
    message(FATAL_ERROR "No nodelets specified for ${target} executable.")
  endif()
  string(REPLACE ";" "\",\"" NODELETS "${ARGN}")
  set(NODELETS "\"${NODELETS}\"")

  configure_file(
    ${PROJECT_SOURCE_DIR}/cmake/lib/templates/nodelet_executable_main.cpp.in
    ${PROJECT_BINARY_DIR}/${target}_main.cpp.in @ONLY)
  file(
    GENERATE OUTPUT ${PROJECT_BINARY_DIR}/${target}_main.cpp
    INPUT ${PROJECT_BINARY_DIR}/${target}_main.cpp.in)
  add_executable(${target} ${PROJECT_BINARY_DIR}/${target}_main.cpp)
  target_link_libraries(${target} ${catkin_LIBRARIES})
endfunction()
