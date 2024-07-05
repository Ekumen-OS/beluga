# Using Beluga with Nav2

:::{figure} ../_images/beluga_nav2.gif
:alt: Short video of the integration of beluga_amcl with nav2.
:::

## Overview

[Nav2](https://docs.nav2.org/index.html) is a ROS 2 framework that includes several components for robot navigation. One of these components is the localization system, typically [nav2_amcl](https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl).

This tutorial will demonstrate how to use [beluga_amcl](../packages/beluga_amcl/docs/index.md) in place for `nav2_amcl`.

To keep it simple, we have built this tutorial on top of the [Nav2 Getting Started Guide](https://docs.nav2.org/getting_started/index.html#). We will use the same simulated [TurtleBot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) robot and environment.

## Requirements

:::{important}
This tutorial is using ROS 2 **Humble**.
:::

Install [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), [Beluga](../getting-started/installation.md), and [Nav2](https://docs.nav2.org/getting_started/index.html#installation).

## Create the workspace

Since we are using ROS 2, we need to create a workspace. Open a terminal and run the following command:

```bash
mkdir -p ~/beluga_demo_ws/src
```

## beluga_demo_nav2 package

### Clone beluga_demo

The [beluga_demo](https://github.com/Ekumen-OS/beluga-demos) is a public repository where you can find the `beluga_demo_nav2` package, among others.

Clone the [`Ekumen-OS/beluga-demos`](https://github.com/Ekumen-OS/beluga-demos) repository in your workspace:

```bash
cd ~/beluga_demo_ws/src
git clone https://github.com/Ekumen-OS/beluga-demos.git
```

For this tutorial, we will only use the `beluga_demo_nav2` package in that repository. This package contains all the necessary files to launch the simulation.

### Explanation

Since `beluga_amcl` is feature-wise compatible with `nav2_amcl` (as thoroughly discussed in the [`beluga_amcl` ROS 2 reference page](../packages/beluga_amcl/docs/ros2-reference.md)), we can adapt to use `beluga_amcl` instead of `nav2_amcl` by loading the `beluga_amcl` component into the `nav2_container` as it is done in the `beluga_nav2_launch.py` file:

:::{figure} ../_images/beluga_amcl.png
:alt: beluga_amcl component in beluga_nav2_launch.py from beluga_demo_nav2 package.
:align: center

Load of beluga_amcl composable node from beluga_demo_nav2 package.
:::

:::{figure} ../_images/nav2_amcl.png
:alt: nav2_amcl component in localization_launch.py from nav2_bringup package.
:align: center

Load of nav2_amcl composable node from nav2_bringup package.
:::

That is what the `beluga_demo_nav2` package does. Even the same parameter file can be used unchanged, though `beluga_demo_nav2` does apply a few minimal changes to better configure `beluga_amcl`, as explained in reference page.

## Run the example

Build the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/beluga_demo_ws
colcon build --symlink-install --packages-up-to beluga_demo_nav2
```

Launch the system:

```bash
source install/local_setup.bash
ros2 launch beluga_demo_nav2 bringup_launch.py
```

You should see `rviz2` open with a map loaded and the robot initialized in an incorrect position. You should also see Gazebo open with both TurtleBot 3 world and robot spawned.

:::{image} ../_images/beluga_nav2_rviz.png
:alt: beluga_amcl component in beluga_nav2_launch.py from beluga_demo_nav2 package.
:align: center
:::

:::{image} ../_images/beluga_nav2_gazebo.png
:alt: beluga_amcl component in beluga_nav2_launch.py from beluga_demo_nav2 package.
:align: center
:::

:::{attention}
If the robot is not spawned in gazebo, check the `GAZEBO_MODEL_PATH` environment variable set in the `beluga_demo_nav2_launch.py` file.
:::

You can use `rviz2` tools to set the correct initial pose of the robot and to navigate the robot around the map.

## Conclusion

This tutorial integrates Beluga and Nav2 for a simulated TurtleBot 3 on ROS 2 Humble. You may also try using Beluga on other supported ROS 2 versions, with different types of robots.
