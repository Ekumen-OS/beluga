#!/bin/bash

# Copyright 2023 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

DEV_CONTAINER_NAME=beluga_dev

# Kill the container JIC it already exists, if building, to ensure we don't attach to an old container.
[[ "$BUILD" = true ]] && docker compose build dev && docker rm -f $DEV_CONTAINER_NAME  2>/dev/null

# Check if the container exists
if docker ps -a --format '{{.Names}}' | grep -q $DEV_CONTAINER_NAME; then
    # Attach to the container if it is running
    echo "Found dev container running, attaching to it..."
else
    echo "Container '$DEV_CONTAINER_NAME' does not exist or is not running, proceeding to run it."
    PRIVILEGED_CONTAINER=$PRIVILEGED_CONTAINER USERID=$(id -u) GROUPID=$(id -g) docker compose run --rm --detach --name $DEV_CONTAINER_NAME  dev
fi

docker exec -it $DEV_CONTAINER_NAME /ros_entrypoint.sh /bin/bash
