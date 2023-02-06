# How to profile Beluga

Guide of how to profile beluga and get a flamegraph from the recorded data.

## Installing perf

Perf can be used from within the docker container or from the host to record events.
You will need `perf` installed in the docker container to process the recorded events, as the libraries are installed there.
Some extra steps are needed if you want to use it in the docker container.

### Common steps

```bash
sudo apt update && sudo apt install linux-tools-generic linux-cloud-tools-generic linux-tools-common
perf
```

The output of `perf` may ask you to install an specific version of `linux-tools-generic` and `linux-cloud-tools-generic` for your kernel.
Install the requested versions.

When installing them inside docker, the required versions may not be available.
See [troubleshooting](#perf-version-not-available).

### Extra steps needed within docker

You need to run a container with at least CAP_SYS_ADMIN added in order to be able to record perf events.
For that, you can use:

```
docker/run.sh --privileged
```

Which will run the container as privilaged.

## Building your code

For the flamegraph to have meaningful information, you need debug info and (may need) frame pointers.
Build your workspace using:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="-fno-omit-frame-pointer"
```

## Using perf to record cpu events

### Running perf when starting a process

For that, use:

```bash
sudo perf record -F 99 -g --call-graph dwarf -- <your command> <arg0> <arg1> ...
```

There is an example script for this:

```bash
source /ws/install/setup.bash
<base_repo_dir>/docs/dev/profiling/profile_amcl_with_bagfile.sh
```

You must not use sudo in this case, if not `LD_LIBRARY_PATH` will not be correctly loaded.
See [troubleshooting](#avoiding-sudo) for instruction on how to run `perf` without `sudo`.

NOTE: Though this should work with launch files passing `perf` as a prefix, for some reason it is not working.
Use the provided script as an example instead.

### Recording perf events of a running process

This can be done either from the host or within the container.

```bash
sudo perf record -F 99 -g --call-graph dwarf -p <PID>
```

To avoid using `sudo`, see [troubleshooting](#avoiding-sudo).
You can stop recording using `ctrl-c`, or it will automatically stop when the process finishes.

For example:

Run within the container a beluga launch file:

```bash
ros2 launch beluga_example example_rosbag_launch.py start_paused:=true
```

Get the beluga process pid, for example using `ps -elf | grep beluga`.
Use the first command in either the host or the container to start recording perf events.
In this case, start playing the bag:

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"
```

When the bag stops, the launch file will stop and perf will finish recording.

## Generating a flamegraph

This needs to be done in the docker container with your workspace sourced, so symbol names can be correctly extracted.

```bash
source /ws/install/setup.bash
cd <directory_with_perf.data_file>
git clone https://github.com/brendangregg/FlameGraph
perf script > out.perf # if asks for sudo access, use chown to modify owner of perf.data. May take really long for dwarf data.
./FlameGraph/stackcollapse-perf.pl out.perf > out.perf-folded
./FlameGraph/flamegraph.pl out.perf-folded > perf.svg
```

To visualize the flamegraph and be able to zoom-in/out, use a web-browser to open it.

If the resulting flamegraph does not seem to provide accurate information, check the alternative [call-graph](#alternative-call-graph-recording-options) recording options.

Alternatively, use the provided script:

```bash
source /ws/install/setup.bash
<base_repo_dir>/docs/dev/profiling/flamegraph.sh
```

which will clone the `FlameGraph` repo if needed.

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

### Alternative call-graph recording options

DWARF has resulted in good results in previous testing with beluga, as the distributed ROS 2 libraries have debug simbols.
If you are building ROS 2 from source, remember to use a build configuration that generates debug symbols as well.
The downside is that the postprocessing steps takes way longer, and it is not as leightweight as the other alternatives.

By default, perf uses frame pointers instead of DWARF to then generate the call graph.
That can be achived by adding `--call-graph fp`, or removing the `--call-graph` option from the command line.
The previous experiments with Beluga and this method were not satisfactory.
It seems that when using `std::execution::par` and this recording method the call stack is not completely recovered.
Using a sequential execution policy, a good call graph can be obtained with this method.

LBR is supported by many modern Intel and AMD processors, it is more lightweight and fast to post-process.
To use this method, pass the `--call-graph lbr` option.
The callgraph depth of this method is limited (tipically 8/16/32, depending on the processor), so the flamegraph output may not show the complete call-stack.
Previous experience using this in Beluga shows that the call stack depth is a bit short, and the resulting flamegraph has many parts that are not merged but actually have a common parent.
