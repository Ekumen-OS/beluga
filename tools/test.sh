#!/bin/bash

# Build and test the project.

set -o errexit
cd $(dirname "$(readlink -f "$BASH_SOURCE")")/..

tools/build.sh
cd build
ctest --verbose $@
