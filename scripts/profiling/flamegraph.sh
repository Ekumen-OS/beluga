#!/bin/sh

SCRIPT_DIR=$(cd $(dirname -- "$0") &> /dev/null && pwd)

set -e
if [ ! -d "$SCRIPT_DIR/FlameGraph" ]; then
    echo "Cloning FlameGraph, this will cached for the next time you run the script"
    cd $SCRIPT_DIR && git clone https://github.com/brendangregg/FlameGraph && cd -
fi
echo "Generating a flamegraph from recorded events, this may take long..."
perf script > out.perf
$SCRIPT_DIR/FlameGraph/stackcollapse-perf.pl out.perf > out.perf-folded
$SCRIPT_DIR/FlameGraph/flamegraph.pl out.perf-folded > flamegraph.svg
