#!/bin/bash

# Bring up a docker container for development.
# Use `--build` to build the image before starting the container.

set -o errexit
cd $(dirname "$(readlink -f "$0")")

[[ ! -z "${WITHIN_DEV}" ]] && echo "Already in the development environment!" && exit 1
HELP="Usage: $(basename $0) [-b|--build] [-p|--privileged]"

set +o errexit
VALID_ARGS=$(OPTERR=1 getopt -o bph --long build,privileged,help -- "$@")
RET_CODE=$?
set -o errexit

if [[ $RET_CODE -eq 1 ]]; then
    echo $HELP
    exit 1;
fi
if [[ $RET_CODE -ne 0 ]]; then
    >&2 echo "Unexpected getopt error"
    exit 1;
fi

BUILD=false
PRIVILEGED_CONTAINER=false

eval set -- "$VALID_ARGS"
while [[ "$1" != "" ]]; do
    case "$1" in
    -b | --build)
        BUILD=true
        shift
        ;;
    -p | --privileged)
        PRIVILEGED_CONTAINER=true
        shift
        ;;
    -h | --help)
        echo $HELP
        exit 0
        ;;
    --) # start of positional arguments
        shift
        ;;
    *)
        >&2 echo "Unrecognized positional argument: $1"
        echo $HELP
        exit 1
        ;;
    esac
done

# Note: The `--build` flag was added to docker compose run after
# https://github.com/docker/compose/releases/tag/v2.13.0.
# We have this for convenience and compatibility with previous versions.
# Otherwise, we could just forward the script arguments to the run verb.
[[ "$BUILD" = true ]] && docker compose build dev

PRIVILEGED_CONTAINER=$PRIVILEGED_CONTAINER USERID=$(id -u) GROUPID=$(id -g) docker compose run --rm dev
