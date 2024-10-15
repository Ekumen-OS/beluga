# Using Beluga AMCL

## Prerequisites

Before we jump in, let’s make sure you’ve got everything you need:

- **Hardware Requirements:**
  - **Processor:** a dual-core x86-64 CPU should be enough to handle real-time localization.
  - **Memory:** at least 2 GB of RAM to comfortably run ROS 2 and AMCL, plus other tasks.
  - **Storage:** ensure you have around 4 GB of free storage for ROS 2 and mapping data.
  - **Sensors:**
    - **2D LiDAR:** scans are essential for localization as Beluga AMCL uses them to match against a known map.
    - **Odometry:** some ego-motion estimation is also necessary (e.g. wheel encoder-based odometry).

- **Software Requirements:**
  - **OS:** you can use either Ubuntu 22.04 LTS (Jammy Jellyfish) or Ubuntu 20.04 LTS (Focal Fossa).
  - **ROS 2:** you can use either ROS 2 Humble Hawksbill or ROS 2 Jazzy Jalisco.

:::{important}
Most localization and SLAM packages in ROS 2 that take 2D LiDAR scans expect them published as `sensor_msgs/LaserScan` messages over the `/scan` topic. Likewise, these packages also expect odometry broadcasted as `odom` to `base_link` transforms over `tf`. That is the case for Beluga AMCL as well. Make sure your system follows this convention before using Beluga AMCL, or take the necessary steps to [reconfigure Beluga AMCL](../packages/beluga_amcl/docs/ros2-reference.md#parameters) to match your own conventions.
:::

## Setting things up

1. **Install Beluga** by following [its installation guide](../getting-started/installation).

2. **Install additional packages** like [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) and the [Nav2 `map_server`](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server):

     ```bash
     sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-map-server
     ```

   You will need these for mapping work.

:::{tip}
Remember to source your ROS 2 environment to use it:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
```
:::

## Mapping the environment

Before the robot can localize using Beluga AMCL, we need a map of the environment. Here's how you can create one:

1. **Launch SLAM Toolbox**, a widely adopted SLAM system:

   ```bash
   ros2 launch slam_toolbox online_async_launch.py
   ```

2. **Drive the robot around**, and make sure you cover all the areas you want included in the map. As the robot moves, SLAM Toolbox will incrementally build a map. You can visualize this using Rviz:

   ```bash
   ros2 run rviz2 rviz2
   ```

    Make sure to add the `Map` display and point it to the `/map` topic using a transient local durability policy.

3. **Save the map** when you are satisfied with coverage:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/map
   ```

   This will create two files: `map.yaml` and `map.pgm`, which represent the map's metadata and image, respectively.

## Configuring Beluga AMCL

Let's put everything together in one cohesive configuration.

```bash
editor beluga.launch.xml
```

Here's a sample `launch` file in XML format you can use:

```xml
<launch>
    <arg name="map_path" description="Path to you map.yaml file." />

    <!-- Load map -->
    <node pkg="nav2_map_server" exec="map_server" name="map_server" output="screen">
        <param name="yaml_filename" value="$(var map_path)" />
    </node>

    <!-- Start AMCL -->
    <node pkg="beluga" exec="amcl" name="amcl" output="screen"/>
</launch>
```

## Running Beluga AMCL

Now, it's time to run Beluga on your robot!

1. **Launch your system** and your custom launch file alongside with it:

     ```bash
     ros2 launch beluga.launch.xml
     ```

2. **Verify your robot is properly localized** using `rviz2`:

     ```bash
     ros2 run rviz2 rviz2
     ```

    Use `map` as your global fixed frame. Use the `Map` and `LaserScan` displays to assess your robot localization. If scans and map do not line up, use the `2D Pose Estimate` tool to reset your robot pose. You can also use `PoseArray` and `MarkerArray` displays to visualize the particle cloud. When moving, it should quickly converge near the robot's true pose.

3. **Tweak Beluga AMCL as necessary** if localization isn't as accurate as expected. For example, you can:

   - **Adjust the number of particles**. If localization is unstable (e.g., the robot's position jumps), consider increasing the minimum number of particles (`min_particles`) to improve the pose estimate. If localization is slow (especially in large environments), consider lowering the maximum number of particles (`max_particles`) to reduce the computational load.
   - **Tune the odometry noise model**. If your odometry source is noisy and localization drifts, consider updating `odom_alpha1` thru `odom_alpha5` to better reflect that noise.
   - **Tune the sensor model**. If you LiDAR is noisy, consider increasing `z_rand` to account for bad range measurements.

## Troubleshooting

Some common issues you may find along the way include:

1. **Beluga AMCL fails to converge**

   - **Symptom:** the particle cloud doesn't cluster around the robot's true pose.
   - **Possible causes:**
     - Incorrect LiDAR or odometry data.
     - Map doesn't match the actual environment.
   - **Crosschecks:**
     - Verify the `/scan` topic data is correct using `ros2 topic echo /scan`.
     - Verify the `odom` to `base_link` transforms are correct using `ros2 run tf2_ros tf2_echo odom base_link`.
     - Verify the `base_link` to `lidar` transforms are correct using `ros2 run tf2_ros tf2_echo base_link lidar`.
     - Check the map matches the actual environment.

2. **Robot's estimated pose is inaccurate**

   - **Symptom:** the robot's displayed pose doesn't match its actual pose.
   - **Possible causes:**
     - Odometry drift.
     - Incorrect transformations between frames.
     - Blurry or distorted map.
   - **Crosschecks:**
     - Calibrate your robot's odometry.
     - Ensure all `tf` frames are correctly defined.
     - [Map the environment](#mapping-the-environment) again.
