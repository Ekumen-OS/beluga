# How to profile Beluga

## Generating a flamegraph from recorded perf events

The generated flamegraph is a good tool to understand what the cpu is used the time for.
We're going to record the perf events from within docker in this guide, though it is also possible to do that from the host.

To start, run the docker container in privilaged mode:

```bash
docker/run.sh -p
```

Within the container run `perf`:

```bash
perf
```

If you see `perf` is asking for another version to be installed, go to [troubleshooting](#perf-version-not-available).

Your code needs to be built with debug info and may also need frame pointers.
For that, use:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer"
```

We will need to run `perf` without sudo, please run the extra steps in your host specified in [troubleshooting](#avoiding-sudo).
You can use `perf` as a prefix of a command, for example:

```bash
source /ws/install/setup.bash
/ws/src/docs/dev/profiling/profile_amcl_with_bagfile.sh
```

NOTE: For some reason, it is not working when used as a prefix in launch, so modify the script avoid if desired.

It may also be useful to run perf on a process that already started, e.g.:

```bash
sudo perf record -F 99 -g --call-graph dwarf -p <PID>
```

In both cases, `perf` will generate a `perf.data` file in the folder it was run.
To generate a flamegraph from the recorded data, run:

```bash
source /ws/install/setup.bash
/ws/src/docs/dev/profiling/flamegraph.sh  # This may take a long time
```

To visualize the flamegraph and be able to zoom it in or out, you need to use a web-browser.

## Troubleshooting

### Perf version not available in docker container

This happens when your host kernel is not one also supported by the container you are running.
Check in the container the output of:

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

### Avoiding sudo

In your host:

```bash
echo "-1" | sudo tee -a /proc/sys/kernel/perf_event_paranoid
echo 0 | sudo tee -a /proc/sys/kernel/kptr_restrict
```

This changes are not permanent, you will need to apply them again after rebooting.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
