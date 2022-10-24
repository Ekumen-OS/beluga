#!/bin/bash

# Lint files.

set -o errexit
cd $(dirname "$(readlink -f "$BASH_SOURCE")")/..

LINT_FILES=$(find . \( -iname *.h -or -iname *.cpp \) -not \( -path "./build/*" -prune \))
[ ${#LINT_FILES} -gt 0 ] && clang-format -i --style=file --verbose $LINT_FILES $@

if ! git diff --quiet; then
  echo "You forgot to run the linter! Run ./tools/lint.sh"
  git --no-pager diff
  exit 1
else
  exit 0
fi
