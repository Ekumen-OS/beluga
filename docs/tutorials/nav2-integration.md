# Using Beluga with Nav2

## Overview

[Nav2](https://docs.nav2.org/index.html) is a ROS 2 framework that includes several components for robot navigation. One of these components is the localization system, typically [`nav2_amcl`](https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl).

This tutorial will demonstrate how to use [`beluga_amcl`](../packages/beluga_amcl/docs/index.md) instead of `nav2_amcl` for a simulated TurtleBot 3 on ROS 2 Humble. You may also try Beluga on other supported ROS 2 versions, with different types of robots.

## Bring the code

### Clone `beluga-demos` repository

The [beluga-demos](https://github.com/Ekumen-OS/beluga-demos) public repository contains the `beluga_demo_nav2_integration` package, among others.

Clone it into a workspace:

```bash
mkdir -p ~/beluga_demo_ws/src
cd ~/beluga_demo_ws/src
git clone https://github.com/Ekumen-OS/beluga-demos.git
```

For this tutorial, we will only use the `beluga_demo_nav2` package in that repository. This package contains all the necessary files to launch the simulation.

### Check  `beluga_demo_nav2_integration` package

Since `beluga_amcl` is feature-wise compatible with `nav2_amcl` (as thoroughly discussed in the [`beluga_amcl` ROS 2 reference page](../packages/beluga_amcl/docs/ros2-reference.md)), we can just swap `beluga_amcl` for `nav2_amcl`:

:::{figure} ../_images/nav2_to_beluga_code_migration.gif
:alt: Using the `beluga_amcl` component instead of `nav2_amcl`'s.
:align: center

Loading `beluga_amcl` composable node, instead of `nav2_amcl`'s.
:::

That is what the `beluga_demo_nav2_integration` package does. Even the same parameter file can be used without changes, though the `beluga_demo_nav2_integration` does apply a few minimal changes to better configure `beluga_amcl`, as explained in its reference page.

## Run the code

To keep things as simple as possible, the `beluga-demos` repository comes with a containerized environment for demo execution:

```bash
cd beluga-demos
./docker/run.sh
```

Once inside, you can build all demos with:

```bash
demo_build
```

You can also make do without containers. You will have to install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and `rosdep install` demo dependencies before you can `colcon build` it:

```bash
source /opt/ros/humble/setup.bash
cd ~/beluga_demo_ws
rosdep install -i -y --from-path src
colcon build --symlink-install --packages-up-to beluga_demo_nav2_integration
```

After building, you can launch the demo with:

```bash
source install/setup.bash
ros2 launch beluga_demo_nav2_integration demo_office_navigation.launch.py
```

This is equivalent to the `nav2_integration_demo` command that is available within the `beluga-demos` container.

You should see Gazebo open up an office scenario with a TurtleBot in it. You should also see `rviz2` open with the corresponding map loaded.

:::{image} ../_images/beluga_nav2_demo.gif
:alt: Gazebo simulation of a robot navigating using Nav2 and Beluga AMCL
:align: center
:::

:::{attention}
If the robot is not spawned in gazebo, check the `GAZEBO_MODEL_PATH` environment variable set in the `beluga_demo_nav2_launch.py` file.
:::

You can use `rviz2` tools to navigate the robot around the map.
