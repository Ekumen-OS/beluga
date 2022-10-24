#!/bin/bash

# Bring up a docker container for development.
# Use `--build` the first time to build the docker image.

set -o errexit
cd $(dirname "$(readlink -f "$BASH_SOURCE")")/..

[[ ! -z "${WITHIN_DEV}" ]] && echo "Already in the development environment!" && exit 1

cd docker
[[ "${1}" == "--build" ]] && docker-compose build dev
USERID=$(id -u) GROUPID=$(id -g) docker-compose run --rm dev
