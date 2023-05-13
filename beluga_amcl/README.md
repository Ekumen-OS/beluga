# Beluga AMCL

Beluga AMCL is a ROS 2 node based on [Beluga](../beluga) featuring interface parity with
[nav2's AMCL](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl).
This package can be easily integrated with code that currently uses `nav2's AMCL`.

## ROS 2 Interface

### Parameters

Beluga AMCL currently supports the majority of ROS parameters used in [nav2's AMCL](https://github.com/ros-planning/navigation2/tree/main/nav2_amcl).
See the [nav2's configuration guide](https://navigation.ros.org/configuration/packages/configuring-amcl.html) and
[Beluga examples](../beluga_example/config/params.yaml) for reference.

#### Extra Parameters

Additionally, this node supports the following extra parameters:

| Parameter                                                          | Description                                                                    |
|--------------------------------------------------------------------|--------------------------------------------------------------------------------|
| `spatial_resolution_[x, y, theta]`                                 | Resolution to divide the space in buckets for adaptive KLD resampling.         |
| <nobr>`initial_pose.covariance_[x, y, yaw, xy, xyaw, yyaw]`</nobr> | Covariance to use with the initial pose when initializing the particle filter. |
| `execution_policy`                                                 | Execution policy used to process particles [seq: sequential, par: parallel].   |

### Subscribed Topics

The subscribed topic names can be changed with the parameters `map_topic`, `scan_topic` and `initial_pose_topic`.

| Topic            | Type                                      | Description                                                              |
|------------------|-------------------------------------------|--------------------------------------------------------------------------|
| `map`            | `nav_msgs/OccupancyGrid`                  | Input topic for map updates.                                             |
| `scan`           | `sensor_msgs/LaserScan`                   | Input topic for laser scan updates.                                      |
| `initial_pose`   | `geometry_msgs/PoseWithCovarianceStamped` | Input topic for pose and covariance to initialize the particle filter.   |

### Published Topics

| Topic            | Type                                      | Description                                                              |
|------------------|-------------------------------------------|--------------------------------------------------------------------------|
| `particle_cloud` | `nav2_msgs/ParticleCloud`                 | Output topic for particle cloud published at a fixed frequency.          |
| `pose`           | `geometry_msgs/PoseWithCovarianceStamped` | Output topic for pose and covariance representing the filter estimation. |

### Transforms

The frame names can be changed with the parameters `global_frame_id`, `odom_frame_id` and `base_frame_id`.
Defaults are `map`, `odom` and `base`.

| Transform         | Description                                                                                   |
|-------------------|-----------------------------------------------------------------------------------------------|
| `odom` to `base`  | Input transform used to update the motion model and detect movement to know when to resample. |
| `base` to `laser` | Input transform used to convert laser scan points to base frame.                              |
| `map` to `odom`   | Output transform computed from the particle filter estimate.                                  |

### Exposed Services

| Topic                              | Type             | Description                                                                   |
|------------------------------------|------------------|-------------------------------------------------------------------------------|
| `reinitialize_global_localization` | `std_srvs/Empty` | Request to reinitialize global localization without an initial pose estimate. |

## Next Steps

- See [example launch files](../beluga_example) showing how to run Beluga-based nodes.
- See [available benchmarks](../beluga_benchmark) for scripts and comparison with other AMCL implementations.
