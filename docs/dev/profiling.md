# How to profile Beluga

## Generating a flamegraph from recorded perf events

A flamegraph is a convenient tool for understanding how CPU time is being used.
We're going to record the perf events from within docker in this guide, though it is also possible to do that from the host.

To start, run the docker container in privileged mode:

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

We will need to run `perf` without `sudo`, please run the extra steps in your host specified in [troubleshooting](#avoiding-sudo).
You can use `perf` as a prefix of a command, for example:

```bash
source /ws/install/setup.bash
/ws/src/docs/dev/profiling/profile_amcl_with_bagfile.sh
```

In both cases, `perf` will generate a `perf.data` file in the folder it was run.
To generate a flamegraph from the recorded data, run:

```bash
source /ws/install/setup.bash
/ws/src/docs/dev/profiling/flamegraph.sh  # This may take a long time
```

To visualize the flamegraph and be able to zoom in or out, open the generated `svg` file in a web-browser.

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

These changes are not permanent, you will need to apply them again after rebooting.

## References

- https://www.brendangregg.com/FlameGraphs/cpuflamegraphs.html
- https://www.brendangregg.com/perf.html
