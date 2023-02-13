# How to profile Beluga

## Generating a flamegraph from recorded perf events

A flamegraph is a convenient tool for understanding how CPU time is being used.

1. To start, run the docker container in privileged mode:
    ```bash
    docker/run.sh -p
    ```

2. Your code needs to be built with debug info and may also need frame pointers.
    For that, use:
    ```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer"
    ```

3. Run the following script to generate the profiling data:
    ```bash
    source /ws/install/setup.bash
    /ws/src/scripts/profiling/profile_amcl_with_bagfile.sh
    ```
    `perf` will generate a `perf.data` file in the folder it was run.

4. To generate a flamegraph from the recorded data, run:
    ```bash
    source /ws/install/setup.bash
    /ws/src/scripts/profiling/flamegraph.sh  # This may take a long time
    ```
    To visualize the flamegraph and be able to zoom it in or out, open the generated `svg` file in a web-browser.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
