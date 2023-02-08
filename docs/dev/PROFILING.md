# How to profile Beluga

## Generating a flamegraph from recorded perf events

A flamegraph is a convenient tool for understanding how CPU time is being used.

1. To start, run the docker container in privileged mode:
    ```bash
    docker/run.sh -p
    ```
    Within the container run `perf`:
    ```bash
    perf
    ```
    If you see `perf` is asking for another version to be installed, go to [troubleshooting](#perf-version-not-available-in-docker-container).

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

## Troubleshooting

### Perf version not available in docker container

This happens when your host kernel is not supported by the container you are running.
Check the output of the following command inside the container:

```bash
ls /usr/lib/linux-tools
```

You will find a folder like for example, `5.15.0-58-generic`, in the next examples `$VERSION_NUMBER-generic`.
Simlink the perf binary in that folder to `/usr/bin`:

```bash
sudo mv /usr/bin/perf /usr/bin/perf.bkp
sudo ln -s /usr/lib/linux-tools/$VERSION_NUMBER-generic/perf /usr/bin
```

Now running `perf` should not ask to install another version.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
