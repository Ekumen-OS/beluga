#!/bin/sh

# Generate documentation using doxygen.

cd $(dirname "$(readlink -f -- "$0")")
doxygen ./Doxyfile
