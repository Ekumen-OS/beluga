# Beluga Example

This package contains example launch files that demonstrate the use of Beluga-based nodes (e.g. [Beluga AMCL](../beluga_amcl)) with external ROS bags or simulation software.

## Examples

The following examples are easier to run in [Beluga development containers](../DEVELOPING.md#environment).

1. **Run an example application using a ROS bag**.

    For ROS 2 distributions, run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example perfect_odometry.launch.xml
    ```

1. **Run an example application using a simulation and teleop controls**.

    For ROS 2 distributions, in two separate terminals run:
    ```bash
    cd /ws
    source install/setup.bash
    ros2 launch beluga_example simulation.launch.xml
    ```
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    Note that this example uses [Flatland](https://flatland-simulator.readthedocs.io) for simulation.
    A Flatland source installation is provisioned in development containers.
    You will have to provision one yourself to run this elsewhere.

1. **Launch a localization node manually**.

   For ROS 2 distributions, run:
   ```bash
   ros2 launch beluga_example localization_launch.py use_composition:=True localization_params_file:=<PARAMS_PATH> localization_map:=<MAP_YAML_PATH>
   ```

   The `localization_params_file` argument can be ommited if the [default AMCL parameters](beluga_example/params/default.ros2.yaml) are compatible with the robot.

1. **Use RViz to visualize the localization output**.

   For ROS 2 distributions, run:
   ```bash
   rviz2 -d $(ros2 pkg prefix --share beluga_example)/rviz/amcl.ros2.rviz
   ```

   **Quality of Service**

   In ROS 2, when subscribing to the output topics from localization, we recommend the following [QoS](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html) settings:

   | Topic            | Depth | History      | Reliability  | Durability      |
   |------------------|-------|--------------|--------------|-----------------|
   | `map`            | 5     | Keep last    | Reliable     | Transient local |
   | `particle_cloud` | 5     | Keep last    | Best effort  | Volatile        |
   | `pose`           | 5     | Keep last    | Reliable     | Volatile        |
