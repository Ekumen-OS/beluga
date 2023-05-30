find_package(Doxygen)

include(FetchContent)

FetchContent_Declare(
  doxygen-awesome-css
  GIT_REPOSITORY https://github.com/jothepro/doxygen-awesome-css
  GIT_TAG        v2.2.0
  SOURCE_DIR     ${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome
)

FetchContent_MakeAvailable(doxygen-awesome-css)

add_custom_target(beluga_docs
  ALL
  COMMAND
    DOXYGEN_OUTPUT_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}
    DOXYGEN_HTML_STYLESHEET=${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome/doxygen-awesome.css
    ${DOXYGEN_EXECUTABLE} ./Doxyfile
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating API documentation with Doxygen"
  VERBATIM)
