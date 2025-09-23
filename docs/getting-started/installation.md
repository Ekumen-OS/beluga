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
