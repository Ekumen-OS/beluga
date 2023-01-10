#!/bin/bash
SCRIPT_PATH=$(dirname "$(readlink -f -- "$0")")
cd $SCRIPT_PATH && doxygen ./Doxyfile && cd -
