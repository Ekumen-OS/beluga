# Installing Beluga AMCL on a Robot

## Installing From Source

1. **Create a workspace directory**.

   ```bash
   mkdir -p ~/ws/src
   ```

1. **Clone the repository into your workspace**.

   ```bash
   cd ~/ws
   git clone https://github.com/Ekumen-OS/beluga.git ~/ws/src/beluga
   ```

1. **Source the ROS installation**.

   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   ```

1. **Install dependencies**.

   ```bash
   rosdep install -r --from-paths ~/ws/src/beluga -y -i -t build -t exec --skip-keys 'flatland_server flatland_plugins'
   ```

1. **Build and source the workspace**.

   In ROS 2 and ROS 1 distributions (if [`colcon`](https://colcon.readthedocs.io/en/released/user/installation.html) is installed), run:
   ```bash
   colcon build --packages-up-to beluga_example --cmake-args -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF
   source install/setup.bash
   ```

   In ROS 1 distributions, you may run the following command instead:
   ```bash
   catkin_make_isolated --only-pkg-with-deps beluga_example --install --cmake-args -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF
   source devel*/setup.bash
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
