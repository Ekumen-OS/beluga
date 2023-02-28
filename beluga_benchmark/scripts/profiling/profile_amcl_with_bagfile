#!/bin/bash
#
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

# TODO(ivanpauno): For some reason the `prefix` option in a launch file is not working correctly with
# perf.

# emulate tty for launch to kill process properly and to get colored output

function cleanup() {
    kill -SIGINT $(jobs -p) > /dev/null 2>&1
    wait $(jobs -p) > /dev/null 2>&1
}
trap cleanup EXIT ERR

PERF_EVENT_PARANOID=$(cat /proc/sys/kernel/perf_event_paranoid)
KPTR_RESTRICT=$(cat /proc/sys/kernel/kptr_restrict)

if [[ "$KPTR_RESTRICT" -ne "0" || "$PERF_EVENT_PARANOID" -ne "-1" ]]; then
    echo "Modifiying kernel config to be able to run perf without sudo..."
    echo "-1" | sudo tee -a /proc/sys/kernel/perf_event_paranoid 1>/dev/null
    echo 0 | sudo tee -a /proc/sys/kernel/kptr_restrict 1>/dev/null
fi

0<&- script -qefc "ros2 launch beluga_example common_nodes_launch.py" /dev/null &
perf record -e cycles -F 99 -g --call-graph dwarf --\
    /ws/install/beluga_amcl/lib/beluga_amcl/amcl_node --ros-args --log-level info --ros-args -r __node:=amcl \
    -p use_sim_time:=True --params-file /ws/install/beluga_example/share/beluga_example/config/params.yaml &
ros2 bag play /ws/install/beluga_example/share/beluga_example/bags/perfect_odometry --rate 3
