# Beluga Example

This package contains example launch files that demonstrate the use of Beluga-based nodes (e.g. [Beluga AMCL](../beluga_amcl)) with external ROS bags or simulation software.

## Examples

See the [getting started](../GETTING_STARTED.md) tutorial to setup a development container or install the package and dependencies from source.

- Launch a pre-recorded ROS bag with perfect odometry and Beluga AMCL.
  ```bash
  ros2 launch beluga_example perfect_odometry.launch.xml
  ```

- Launch a pre-recorded ROS bag and Beluga AMCL as a composable node.
  ```bash
  ros2 launch beluga_example perfect_odometry.launch.xml use_composition:=True
  ```

- Launch a simulation that can be teleoperated together with Beluga AMCL.
  ```bash
  ros2 launch beluga_example simulation.launch.xml
  ```

- Launch Beluga AMCL, a map server, and a lifecycle manager. Useful for testing on real robots.
  ```bash
  ros2 launch beluga_example localization_launch.py use_composition:=True
  ```

- Launch a pre-recorded ROS bag and Beluga AMCL with Carpincho Bot.
  ```bash
  ros2 launch beluga_example carpinchobot.launch.xml
  ```
