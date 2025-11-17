# Beluga AMCL

Beluga AMCL is a ROS node based on the [Beluga](../beluga) library that aims to be fully compatible with the [Navigation 2 AMCL][nav2_amcl] node.<br/>
The compatibility between `beluga_amcl` and its longstanding counterparts in the ROS ecosystem provides a simple migration path for projects that want to be able to integrate the power and modularity of the Beluga library in an existing `nav2_amcl`-based project.

## Table of Contents

- [Beluga AMCL](#beluga-amcl)
  - [Table of Contents](#table-of-contents)
  - [ROS Interface](#ros-interface)
    - [Parameters](#parameters)
    - [Subscribed Topics](#subscribed-topics)
    - [Published Topics](#published-topics)
    - [Published Transforms](#published-transforms)
    - [Advertised Services](#advertised-services)
  - [Performance](#performance)
  - [Compatibility](#compatibility-notes)
  - [Next Steps](#next-steps)

## ROS Interface

### Parameters

Beluga AMCL currently supports the majority of ROS parameters used in [Navigation 2 AMCL][nav2_amcl].<br/>
See [Beluga AMCL documentation](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html) for further reference.

### Subscribed Topics

The subscribed topic names can be changed with the parameters `map_topic`, `scan_topic`, and `initial_pose_topic`.

| Topic            | Type                                      | Description                                                                 |
|------------------|-------------------------------------------|-----------------------------------------------------------------------------|
| `map`            | `nav_msgs/OccupancyGrid`                  | Input topic for map updates.                                                |
| `scan`           | `sensor_msgs/LaserScan`                   | Input topic for laser scan updates.                                         |
| `initial_pose`   | `geometry_msgs/PoseWithCovarianceStamped` | Input topic for pose mean and covariance to initialize the particle filter. |

Alternatively, and instead of the `scan_topic`, one may set the `point_cloud_topic`. Point clouds are assumed to be contained in a z = constant
plane in the base frame of reference. It is further assumed this plane is the same plane where the map and pose estimates are defined. If this
is not the case, Beluga AMCL will misbehave. It is on the user to filter point clouds and make sure these assumption hold.

### Published Topics

| Topic              | Type                                      | Description                                                              |
|--------------------|-------------------------------------------|--------------------------------------------------------------------------|
| `particle_cloud`   | `geometry_msgs/PoseArray`                 | Output topic for particle cloud poses published at a fixed frequency.    |
| `particle_markers` | `visualization_msgs/MarkerArray`          | Output topic for particle cloud markers published at a fixed frequency.  |
| `likelihood_field` | `nav_msgs/OccupancyGrid`                  | Output topic for likelihood field, published on update when applicable.  |
| `pose`             | `geometry_msgs/PoseWithCovarianceStamped` | Output topic for estimated pose mean and covariance in map frame.        |

### Published Transforms

The frame names can be changed with the parameters `global_frame_id`, `odom_frame_id` and `base_frame_id`.
Defaults are `map`, `odom` and `base`.

| Transform         | Description                                                                                        |
|-------------------|----------------------------------------------------------------------------------------------------|
| `odom` to `base`  | Input transform used by motion models and resampling policies.                                     |
| `base` to `laser` | Input transform used to convert laser scan points to base frame.                                   |
| `map` to `odom`   | Output transform calculated from the estimated pose mean and the current _odom-to-base_ transform. |

### Advertised Services

| Topic                              | Type             | Description                                                                   |
|------------------------------------|------------------|-------------------------------------------------------------------------------|
| `reinitialize_global_localization` | `std_srvs/Empty` | Request to reinitialize global localization without an initial pose estimate. |
| `request_nomotion_update`          | `std_srvs/Empty` | Trigger a forced update of the filter estimates.                              |

## Performance

Performance reports are periodically generated to track improvements and regressions. These reports are produced using the scripts in the [beluga_benchmark](../beluga_benchmark) package, which allow comparing the performance of `beluga_amcl` against `nav2_amcl` using a synthetic dataset.

<figure>
  <img src="/beluga_amcl/docs/_images/beluga_vs_nav2.png" alt="Beluga Vs Nav2">
  <figcaption><strong>Typical trajectory plot</strong>: the ground-truth and both AMCL trajectories are so close that they effectively overlap.</figcaption>
</figure>

Further details on the benchmarking methodology and datasets can be found in the blog post [Big shoes to fill: Validating the performance of Beluga AMCL](https://ekumenlabs.com/blog/posts/big-shoes-to-fill-beluga-performance-report/)

The most recent [performance report](https://github.com/user-attachments/files/19375155/report.pdf), published in March 2025, includes a broader benchmark than the one presented in the blog.

## Compatibility notes

Beluga AMCL is a feature-by-feature re-implemention of the existing Nav2 AMCL package in ROS, but using the Beluga library as the underlying engine.

Notes on parameter and feature availability between Beluga AMCL and Nav2 AMCL can be found [here](https://ekumen-os.github.io/beluga/packages/beluga_amcl/docs/ros2-reference.html#compatibility-notes).

## Next Steps

- See [example launch files](../beluga_example) showing how to run Beluga-based nodes.
- See [available benchmarks](../beluga_benchmark) for scripts and comparison with other AMCL implementations.

[nav2_amcl]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl
[nav2_configuration_guide]: https://navigation.ros.org/configuration/packages/configuring-amcl.html
[fox2001]: https://dl.acm.org/doi/10.5555/2980539.2980632
