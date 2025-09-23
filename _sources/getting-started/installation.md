# Installation

## Installing binaries

Beluga binaries are released to several [ROS distributions](../index.md#support), and thus are made available through package managers.

:::{important}
Only `beluga`, `beluga_ros`, and `beluga_amcl` packages are available as binaries. The rest must be [built from source](#building-from-source-with-ros).
:::

::::{tab-set}

:::{tab-item} Ubuntu
Follow the [official ROS documentation](https://docs.ros.org) on how to setup `apt` sources for the distribution of choice. You may also want to consider using `ros2-testing` repositories to get the absolute latest release for the target distribution.

Then:

```bash
sudo apt install ros-${ROS_DISTRO}-beluga*
```
:::

::::

## Building from source (with ROS)

:::{tip}
If you intend to contribute to Beluga, consider adopting Beluga's [development workflows](https://github.com/Ekumen-OS/beluga/blob/main/DEVELOPING.md).
:::

### Install ROS distribution

Follow [official ROS documentation](https://docs.ros.org) on how to install the distribution of choice. ROS development tools will be necessary -- [`colcon`](https://colcon.readthedocs.io) and [`rosdep`](https://docs.ros.org/en/independent/api/rosdep/html) at the very least.

Remember to source the corresponding installation from now on, as documented upstream:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```
:::

::::

If in doubt as to which ROS distribution to use, check the list of [supported distributions](../index.md#support).

### Clone Beluga repository

All packages in the Beluga project are ROS packages. Cloning them into a local workspace is thus best for subsequent build, install, and use:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
mkdir -p ~/ws/src
git clone https://github.com/Ekumen-OS/beluga.git ~/ws/src/beluga
```
:::

::::


### Install dependencies

First of all, ensure your system and package manager are up to date before proceeding.

::::{tab-set}

:::{tab-item} Ubuntu
```bash
sudo apt update && sudo apt upgrade
```
:::

::::

Then use `rosdep` to install Beluga packages' dependencies:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
rosdep update
rosdep install -r --from-paths ~/ws/src/beluga -y -i -t build -t exec
```
:::

::::

### Build and source workspace

For ROS 2 distributions, `colcon` can be used to build:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
colcon build --packages-up-to beluga_example --cmake-args -DBUILD_TESTING=OFF
```
:::

::::

After building, don't forget to source the environment:

::::{tab-set}

:::{tab-item} ROS 2
```bash
source install/setup.bash
```
:::

::::

## Building from source (with Bazel)

In addition to [`colcon`](https://colcon.readthedocs.io/en/released/), Beluga also supports [`bazel`](https://bazel.build/).

:::{important}
Only the `beluga` core library, which is ROS-agnostic, can be built and tested with Bazel.
:::

### Install Bazel

[Bazelisk](https://github.com/bazelbuild/bazelisk) is the recommended way to install Bazel on Ubuntu, Windows, and macOS. It automatically downloads and installs the appropriate version of Bazel. Follow the [installation instructions](https://github.com/bazelbuild/bazelisk#installation) in the official Bazelisk repository.

The [official Bazel documentation](https://bazel.build/install) also provides alternative installation methods. The required Bazel version can be found in the [.bazelversion](https://github.com/Ekumen-OS/beluga/blob/main/.bazelversion) file.

You should also have a modern C++ compiler on your system, which Bazel will detect.

### Clone Beluga repository

Clone the Beluga project source code:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
git clone https://github.com/Ekumen-OS/beluga.git
```
:::

:::{tab-item} Windows
```bash
git clone https://github.com/Ekumen-OS/beluga.git
```
:::

::::

### Build and run tests

The following command will build and run all the unit tests in the project.

::::{tab-set}

:::{tab-item} Ubuntu
```bash
bazel test //...
```
:::

:::{tab-item} Windows
```bash
bazel test //...
```
:::

::::

You can inspect the available Bazel targets with:

::::{tab-set}

:::{tab-item} Ubuntu
```bash
bazel query //...
```
:::

:::{tab-item} Windows
```bash
bazel query //...
```
:::

::::

### Depending on Beluga as a bzlmod dependency

[Bzlmod](https://bazel.build/external/module) is the new package manager for Bazel modules.
You can depend on `beluga` by adding the following lines to your `MODULE.bazel` file:

```starlark
bazel_dep(name = "beluga", version = "2.0.2")

git_override(
    module_name = "beluga",
    commit = "...",
    remote = "https://github.com/Ekumen-OS/beluga.git",
)
```

:::{note}
The `git_override` is required because Beluga is not yet in the [Bazel Central Registry](https://registry.bazel.build/).
:::
