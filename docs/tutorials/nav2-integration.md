# Using Beluga with Nav2

:::{figure} ../_images/beluga_nav2.gif
:alt: Short video of the integration of beluga_amcl with nav2.
:::

## Overview

[Nav2](https://docs.nav2.org/index.html) is a ROS2 framework that includes several components for robot navigation. Among these components is the localization system, which typically uses the [nav2_amcl](https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl) implementation.

This tutorial will demonstrate how to integrate [beluga_amcl](../packages/beluga_amcl/docs/index.md) into the Nav2 stack as an alternative to the default `nav2_amcl`.

To keep things straightforward, we’ll base this tutorial on the [Nav2 Getting Started Guide](https://docs.nav2.org/getting_started/index.html#). We will use the simulated robot and environment provided by [TurtleBot 3](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).

## Requirements

:::{important}
This tutorial is using ROS2 **Humble**.
:::

Install [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html), [Beluga](../getting-started/installation.md) and [Nav2](https://docs.nav2.org/getting_started/index.html#installation).

## Create the workspace

Since we are using ROS2, we need to create a workspace. Open a terminal and run the following command:

```bash
mkdir -p ~/beluga_demo_ws/src
```

## beluga_demo_nav2 package

### Clone beluga_demo

The [beluga_demo](https://github.com/Ekumen-OS/beluga-demos) is a public repository where you can find the `beluga_demo_nav2` package, among others.

Clone the repository into the workspace you created earlier:

```bash
cd ~/beluga_demo_ws/src
git clone https://github.com/Ekumen-OS/beluga-demos.git
```

:::{note}
For this tutorial, we will use only the `beluga_demo_nav2` package from the `beluga_demo` repository. This package contains all the necessary files to launch the simulation.
:::

### Explanation

As explained in the [beluga_amcl ROS2 reference](../packages/beluga_amcl/docs/ros2-reference.md), `beluga_amcl` is feature-wise compatible with `nav2_amcl`.

Because of this, we copied the necessary folders and files from `nav2_bringup` to the `beluga_demo_nav2` package, simplified them, and replaced the use of `nav2_amcl` with `beluga_amcl` in the `localization_launch.py` launch file, as shown in the following images.

:::{figure} ../_images/beluga_amcl.png
:alt: beluga_amcl component in localization_launch.py from beluga_demo_nav2 package.
:align: center

Load of beluga_amcl composable node from beluga_demo_nav2 package.
:::

:::{figure} ../_images/nav2_amcl.png
:alt: nav2_amcl component in localization_launch.py from nav2_bringup package.
:align: center

Load of nav2_amcl composable node from nav2_bringup package.
:::

Similarly, the parameter file we use is the same as the one from the `nav2_bringup` package, with minimal modifications. These modifications are explained in the table found on the *beluga_amcl ROS2 reference* page.

## Run the example

Build the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/beluga_demo_ws
colcon build --symlink-install --packages-up-to beluga_demo_nav2
```

Launch the program:

```bash
source install/local_setup.bash
ros2 launch beluga_demo_nav2 beluga_demo_nav2_launch.py
```

After launching the program, you should see rviz2 open with a map loaded and the robot initialized in an incorrect position. You should also see Gazebo open with the TurtleBot 3 world and robot spawned.

:::{image} ../_images/beluga_nav2_rviz.png
:alt: nav2_amcl component in localization_launch.py from nav2_bringup package.
:align: center
:::

:::{image} ../_images/beluga_nav2_gazebo.png
:alt: nav2_amcl component in localization_launch.py from nav2_bringup package.
:align: center
:::

:::{attention}
If the robot is not spawned in gazebo, check the `GAZEBO_MODEL_PATH` environment variable set in the `beluga_demo_nav2_launch.py` file.
:::

At this point, the program is ready to start. You can use the rviz2 tools to set the correct initial pose of the robot and to navigate the robot alongside the map.

## Conclusion

This tutorial demonstrates a specific integration case using ROS2 Humble and the TurtleBot 3 with Nav2. You can also try using Beluga with other ROS/ROS2 versions and different types of robots.
