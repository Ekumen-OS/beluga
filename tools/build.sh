#!/bin/bash

# Build the project and its dependencies.

set -o errexit
cd $(dirname "$(readlink -f "$BASH_SOURCE")")/..

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release $@
cmake --build .
