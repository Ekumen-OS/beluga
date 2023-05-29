find_package(Doxygen)

add_custom_target(beluga_docs
  ALL
  COMMAND
    DOXYGEN_OUTPUT_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}
    ${DOXYGEN_EXECUTABLE} ./Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating API documentation with Doxygen"
  VERBATIM)
