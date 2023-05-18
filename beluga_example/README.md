# Beluga Example

This package contains example launch files that demonstrate the use of Beluga-based nodes (e.g. [Beluga AMCL](../beluga_amcl)) with external ROS bags or simulation software.

## Examples

See the [getting started](../GETTING_STARTED.md) tutorial to setup a development container or install the package and dependencies from source.

- Launch a pre-recorded ROS bag.
  ```bash
  ros2 launch beluga_example example_rosbag_launch.py
  ```

- Launch a simulation that can be teleoperated.
  ```bash
  ros2 launch beluga_example example_launch.py
  ```

- Launch a simulation with Beluga AMCL as a composable node.
  ```bash
  ros2 launch beluga_example example_component_launch.py
  ```
