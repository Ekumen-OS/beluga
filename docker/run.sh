#!/bin/bash

# Bring up a docker container for development.
# Use `--build` to build the image before starting the container.

set -o errexit
cd $(dirname "$(readlink -f "$0")")

[[ ! -z "${WITHIN_DEV}" ]] && echo "Already in the development environment!" && exit 1

# Note: The `--build` flag was added to docker compose run after
# https://github.com/docker/compose/releases/tag/v2.13.0.
# We have this for convenience and compatibility with previous versions.
# Otherwise, we could just forward the script arguments to the run verb.
[[ "${1}" == "--build" ]] && docker compose build dev

USERID=$(id -u) GROUPID=$(id -g) docker compose run --rm dev
