#!/bin/bash

# emulate tty for launch to kill process properly and to get colored output
0<&- script -qefc "ros2 launch beluga_example common_nodes_launch.py" /dev/null &
perf record -e cycles -F 99 -g --call-graph dwarf --\
    /ws/install/beluga_amcl/lib/beluga_amcl/amcl_node --ros-args --log-level info --ros-args -r __node:=amcl \
    -p use_sim_time:=True --params-file /ws/install/beluga_example/share/beluga_example/config/params.yaml &
ros2 bag play /ws/install/beluga_example/share/beluga_example/bags/perfect_odometry --rate 3

kill -SIGINT $(jobs -p)
