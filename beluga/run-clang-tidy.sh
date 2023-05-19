#!/bin/bash
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
run-clang-tidy -quiet -p $SCRIPT_DIR/../../../build/beluga
