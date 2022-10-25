#!/bin/bash

# Lint files. Return code will be non-zero if the code was formatted automatically or
# needs to be changed manually.

set -o errexit
cd $(dirname "$(readlink -f "$BASH_SOURCE")")/..

LINT_FILES=$(find . \( -iname *.h -or -iname *.cpp \) -not \( -path "./build/*" -prune \))

function format() {
  if ! clang-format -i --style=file --dry-run -Werror $LINT_FILES; then
    clang-format -i --style=file --verbose $LINT_FILES
    exit 1  # Format and exit for CI to detect the failure
  fi
}

function lint() {
  cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --log-level=ERROR
  clang-tidy --quiet -p ./build/compile_commands.json $LINT_FILES
}

if [ ${#LINT_FILES} -gt 0 ]; then
  format
  lint
fi

echo "Passed!"
