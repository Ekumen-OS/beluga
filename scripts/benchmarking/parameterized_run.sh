#!/bin/bash

# Bring up a docker container for development.
# Use `--build` to build the image before starting the container.

SCRIPT_DIR=$(cd $(dirname "$(readlink -f "$0")") && pwd)

read -r -d '' HELP << EOM
Usage: $(basename $0) [...] <PARTICLES_0> ... <PARTICLES_N>\n
\n
    PARTICLES_N For each positional argument the benchmark will be run using that ammount of particles.\n
    [--package] Package of the node to use, defaults to beluga_amcl.\n
    [--executable] Executable to use, defaults to amcl_node.\n
    [--initial-pose-x] Set initial x axis pose.\n
    [--initial-pose-y] Set initial y axis pose.\n
    [--initial-pose-yaw] Set initial yaw pose.\n
    [-b|--rosbag] Use a different rosbag, defaults to "beluga_example/bags/perfect_odemetry".\n
    [-f|--playback-frequency] Rosbag playback frequency, defaults to 3.\n
    [-p|--profile] Create a perf cpu profile of the benchmark.
EOM

set +o errexit
VALID_ARGS=$( \
    OPTERR=1 getopt -o b:f:ph --long \
    package:,executable:,initial-pose-x:,initial-pose-y:,initial-pose-yaw:,rosbag:,playback-frequency:,profile,help \
    -- "$@")
RET_CODE=$?
set -o errexit

if [[ $RET_CODE -eq 1 ]]; then
    echo -e $HELP
    exit 1;
fi
if [[ $RET_CODE -ne 0 ]]; then
    >&2 echo "Unexpected getopt error"
    exit 1;
fi

EXTRA_LAUNCH_ARGS=""
EXECUTABLE_NAME="amcl_node"
PROFILE=false

eval set -- "$VALID_ARGS"
while : ;do
    case "$1" in
    --package)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS package:=$2"
        shift 2
        ;;
    --executable)
        EXECUTABLE_NAME=$2
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS node:=$2"
        shift 2
        ;;
    -b | --rosbag)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS rosbag_path:=$2"
        shift 2
        ;;
    -f | --playback-frequency)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS playback_frequency:=$2"
        shift 2
        ;;
    --initial-pose-x)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS amcl.set_initial_pose:=True amcl.initial_pose.x:=$2"
        shift 2
        ;;
    --initial-pose-y)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS amcl.set_initial_pose:=True amcl.initial_pose.y:=$2"
        shift 2
        ;;
    --initial-pose-yaw)
        EXTRA_LAUNCH_ARGS="$EXTRA_LAUNCH_ARGS amcl.set_initial_pose:=True amcl.initial_pose.yaw:=$2"
        shift 2
        ;;
    -p | --profile)
        PROFILE=true
        shift
        ;;
    -h | --help)
        echo -e $HELP
        exit 0
        ;;
    --) # start of positional arguments
        shift
        break
        ;;
    esac
done

if [ -z "$@" ]; then
    >&2 echo "At least one number of particles must be specified"
    echo -e $HELP
    exit 1
fi

function cleanup() {
    kill -SIGINT $(jobs -p) > /dev/null 2>&1
    wait $(jobs -p) > /dev/null 2>&1
}
trap cleanup EXIT ERR

for N in $@; do
    mkdir "benchmark_${N}_particles_output"
    cd "benchmark_${N}_particles_output"
    touch timem-output.json  # if not, it's creating a directory by error and not a json....
    script -qefc " \
        ros2 launch beluga_example example_rosbag_launch.py amcl.max_particles:=${N} amcl.min_particles:=${N} \
            record_bag:=True bagfile_output:=rosbag prefix:=\"timem -o timem-output --\" $EXTRA_LAUNCH_ARGS" \
        /dev/null &
    while ! pgrep "$EXECUTABLE_NAME" > /dev/null; do
        sleep .1
    done
    if [ "$PROFILE" = "true" ]; then
        sudo perf record -F 99 --call-graph lbr -p $(pgrep $EXECUTABLE_NAME)
    fi;
    wait %1 > /dev/null 2>&1
    cd -
done

# for N in $@; do
#     # TODO POSTPROCESSING
# done