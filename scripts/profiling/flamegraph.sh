#!/bin/sh

set -e
echo "Generating a flamegraph from recorded events, this may take long..."
perf script > out.perf
/opt/FlameGraph/stackcollapse-perf.pl out.perf > out.perf-folded
/opt/FlameGraph/flamegraph.pl out.perf-folded > flamegraph.svg
