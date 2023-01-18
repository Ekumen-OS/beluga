#!/bin/sh

# Generate documentation using doxygen.

# Get Doxyfile path.
SCRIPT_PATH=$(dirname "$(readlink -f -- "$0")")

# Set default output directory and convert to absolute path.
DOXYGEN_OUTPUT_DIRECTORY=$(realpath ${DOXYGEN_OUTPUT_DIRECTORY:-"${SCRIPT_PATH}/../generated_docs"})
DOXYGEN_HTML_OUTPUT=${DOXYGEN_HTML_OUTPUT:-"html"}

cd $SCRIPT_PATH

exec \
  env DOXYGEN_OUTPUT_DIRECTORY=${DOXYGEN_OUTPUT_DIRECTORY} \
  env DOXYGEN_HTML_OUTPUT=${DOXYGEN_HTML_OUTPUT} \
  doxygen ./Doxyfile
