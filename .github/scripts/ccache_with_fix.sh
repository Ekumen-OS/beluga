#!/bin/bash

BASEDIR=$(ls -d /__w/beluga/beluga/ros_ws/src/??????????? | head -n 1)

echo "Using CCACHE_BASEDIR=$BASEDIR"

exec \
  env VERBOSE=1 \
  env CCACHE_BASEDIR=$BASEDIR \
  ccache $@
