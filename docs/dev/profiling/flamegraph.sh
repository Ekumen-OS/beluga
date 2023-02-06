#!/bin/sh

SCRIPT_DIR=$(cd $(dirname -- "$0") &> /dev/null && pwd)

set -e
if [ ! -d "$SCRIPT_DIR/FlameGraph" ]; then
    cd $SCRIPT_DIR && git clone https://github.com/brendangregg/FlameGraph && cd -
fi
perf script > out.perf
$SCRIPT_DIR/FlameGraph/stackcollapse-perf.pl out.perf > out.perf-folded
$SCRIPT_DIR/FlameGraph/flamegraph.pl out.perf-folded > flamegraph.svg
