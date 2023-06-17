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
   colcon build --packages-up-to beluga_example --cmake-args -DBUILD_TESTING=OFF -DBUILD_DOCUMENTATION=OFF
   source install/setup.bash
   ```

   In ROS 1 distributions, you may run the following command instead:
   ```bash
   catkin_make_isolated --only-pkg-with-deps beluga_example --install --cmake-args -DBUILD_TESTING=OFF -DBUILD_DOCUMENTATION=OFF
   source devel*/setup.bash
   ```

1. **Launch a localization node manually**.

   For ROS 2 distributions, run:
   ```bash
   ros2 launch beluga_example localization_launch.py use_composition:=True
   ```

   For ROS 1 distributions, run:
   ```bash
   roslaunch beluga_example localization.launch
   ```
