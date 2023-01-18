#!/bin/sh

# Generate documentation using doxygen.

cd $(dirname "$(readlink -f -- "$0")")

exec \
  env DOXYGEN_OUTPUT_DIRECTORY=${DOXYGEN_OUTPUT_DIRECTORY:-"."} \
  env DOXYGEN_HTML_OUTPUT=${DOXYGEN_HTML_OUTPUT:-"html"} \
  doxygen ./Doxyfile
