# Beluga Example

This package contains example launch files that demonstrate the use of Beluga-based nodes (e.g. [Beluga AMCL](../beluga_amcl)) with external ROS bags or simulation software.

## Examples

1. **Run an example application using a ROS bag** (inside development container).

    For ROS 2 distributions, run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example perfect_odometry.launch.xml
    ```

    For ROS 1 distributions, run:
    ```bash
    cd /ws
    source devel*/setup.bash
    roslaunch beluga_example perfect_odometry.launch
    ```

1. **Run an example application using a simulation and teleop controls** (inside development container).

    For ROS 2 distributions, in two separate terminals run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example simulation.launch.xml
    ```
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    For ROS 1 distributions, in two separate terminals run:
    ```bash
    cd /ws
    source devel*/setup.bash
    roslaunch beluga_example simulation.launch
    ```
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard
    ```

1. **Launch a localization node manually**.

   For ROS 2 distributions, run:
   ```bash
   ros2 launch beluga_example localization_launch.py use_composition:=True localization_params_file:=<PARAMS_PATH> localization_map:=<MAP_YAML_PATH>
   ```

   For ROS 1 distributions, run:
   ```bash
   roslaunch beluga_example localization.launch localization_params_file:=<PARAMS_PATH> localization_map:=<MAP_YAML_PATH>
   ```

   The `localization_params_file` argument can be ommited if the [default AMCL parameters](beluga_example/params/default.ros2.yaml) are compatible with the robot.

1. **Use RViz to visualize the localization output**.

   For ROS 2 distributions, run:
   ```bash
   rviz2 -d $(ros2 pkg prefix --share beluga_example)/rviz/rviz.ros2.rviz
   ```

   For ROS 1 distributions, run:
   ```bash
   rviz -d $(rospack find beluga_example)/rviz/rviz.ros.rviz
   ```

   **Quality of Service**

   In ROS 2, when subscribing to the output topics from localization, we recommend the following [QoS](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html) settings:

   | Topic            | Depth | History      | Reliability  | Durability      |
   |------------------|-------|--------------|--------------|-----------------|
   | `map`            | 5     | Keep last    | Reliable     | Transient local |
   | `particle_cloud` | 5     | Keep last    | Best effort  | Volatile        |
   | `pose`           | 5     | Keep last    | Reliable     | Volatile        |


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
