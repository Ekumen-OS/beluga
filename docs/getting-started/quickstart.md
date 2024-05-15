# Quickstart

The simplest approach to Beluga is to try out one of the few examples available.

## Prerequisites

First and foremost, you have to [install Beluga](./installation).

## Run Beluga over a recording

Data recordings are key when tuning for performance and troubleshooting issues, even if synthetic like the ROS bags used by this example. On a terminal, just run:

::::{tab-set}

:::{tab-item} ROS 2
```bash
ros2 launch beluga_example perfect_odometry.launch.xml
```
:::

:::{tab-item} ROS 1
```bash
roslaunch beluga_example perfect_odometry.launch
```
:::

::::

## Run Beluga in simulation

Simulation streamlines and speeds up robotics development workflows, even if purely functional like in this example using [Flatland](https://flatland-simulator.readthedocs.io).

:::{important}
Flatland must be installed separately to run the following commands.
:::

:::{tip}
You can use [Beluga development containers](https://github.com/Ekumen-OS/beluga/blob/main/DEVELOPING.md#environment), provisioned by default with a Flatland source installation, to run the following commands.
:::

On two separate terminals run:

::::{tab-set}

:::{tab-item} ROS 2
```bash
ros2 launch beluga_example simulation.launch.xml
```
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # for teleoperation!
```
:::

:::{tab-item} ROS 1
```bash
roslaunch beluga_example simulation.launch
```
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py  # for teleoperation!
```
:::

::::

## Run Beluga on a robot

:::{tip}
Don't have an robot at home to play with? There are plenty open source and/or open hardware alternatives you can acquire or build. Our proposal is [Andino](https://github.com/Ekumen-OS/andino).
:::

Eventually, we have to hit the hardware. This example assumes a fully functional ROS (1) or ROS 2 powered robot, equipped with a 2D lidar and already publishing odometry estimates. Furthermore, a 2D map is necessary. Any suitable SLAM solution may be used to build one. [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox), [Cartographer ROS](https://google-cartographer-ros.readthedocs.io/en/latest/), and [RTABMap ROS](https://github.com/introlab/rtabmap_ros) are quite popular. Once you are all set, on a terminal you can just run:

::::{tab-set}

:::{tab-item} ROS 2
```bash
ros2 launch beluga_example localization_launch.py use_composition:=True localization_params_file:=<PARAMS_PATH> localization_map:=<MAP_YAML_PATH>
```
:::

:::{tab-item} ROS 1
```bash
roslaunch beluga_example localization.launch localization_params_file:=<PARAMS_PATH> localization_map:=<MAP_YAML_PATH>
```
:::

::::

:::{note}
In both cases, the `localization_params_file` argument can be omitted if [default AMCL parameters](https://github.com/Ekumen-OS/beluga/blob/main/beluga_example/params) are compatible with your robot.
:::

## Visualize Beluga output

A picture is worth a thousand words. Probabilistic algorithms are no exception. Fortunately, there is RViz. Just run:

::::{tab-set}

:::{tab-item} ROS 2
```bash
rviz2 -d $(ros2 pkg prefix --share beluga_example)/rviz/rviz.ros2.rviz
```
:::

:::{tab-item} ROS 1
```bash
rviz -d $(rospack find beluga_example)/rviz/rviz.ros.rviz
```
:::

::::

:::{tip}
When in ROS 2 and subscribing to localization related topics, we recommend the following [QoS](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html) settings:

| Topic            | Depth | History   | Reliability | Durability      |
|------------------|-------|-----------|-------------|-----------------|
| `map`            | 5     | Keep last | Reliable    | Transient local |
| `particle_cloud` | 5     | Keep last | Best effort | Volatile        |
| `pose`           | 5     | Keep last | Reliable    | Volatile        |
:::
