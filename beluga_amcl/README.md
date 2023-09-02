# Beluga AMCL

Beluga AMCL is a ROS node based on the [Beluga](../beluga) library that aims to be fully compatible with both [Navigation 2 AMCL][nav2_amcl] and [Navigation AMCL][amcl] nodes.<br/>
The compatibility between `beluga_amcl` and its longstanding counterparts in the ROS ecosystem provides a simple migration path for projects that want to be able to integrate the power and modularity of the Beluga library in an existing `nav2_amcl`-based (or `amcl`-based) project.

## Table of Contents

- [Beluga AMCL](#beluga-amcl)
  - [Table of Contents](#table-of-contents)
  - [ROS 2 Interface](#ros-2-interface)
    - [Parameters](#parameters)
    - [Subscribed Topics](#subscribed-topics)
    - [Published Topics](#published-topics)
    - [Published Transforms](#published-transforms)
    - [Advertised Services](#advertised-services)
  - [ROS 1 Interface](#ros-1-interface)
    - [Parameters](#parameters-1)
    - [Subscribed Topics](#subscribed-topics-1)
    - [Published Topics](#published-topics-1)
    - [Published Transforms](#published-transforms-1)
    - [Advertised Services](#advertised-services-1)
    - [Called Services](#called-services)
  - [Performance](#performance)
  - [Next Steps](#next-steps)

## ROS 2 Interface

### Parameters

Beluga AMCL currently supports the majority of ROS parameters used in [Navigation 2 AMCL][nav2_amcl].<br/>
See [Beluga AMCL parameter reference](docs/PARAMETERS.md) for detailed information.

### Subscribed Topics

The subscribed topic names can be changed with the parameters `map_topic`, `scan_topic` and `initial_pose_topic`.

| Topic            | Type                                      | Description                                                                 |
|------------------|-------------------------------------------|-----------------------------------------------------------------------------|
| `map`            | `nav_msgs/OccupancyGrid`                  | Input topic for map updates.                                                |
| `scan`           | `sensor_msgs/LaserScan`                   | Input topic for laser scan updates.                                         |
| `initial_pose`   | `geometry_msgs/PoseWithCovarianceStamped` | Input topic for pose mean and covariance to initialize the particle filter. |

### Published Topics

| Topic            | Type                                      | Description                                                              |
|------------------|-------------------------------------------|--------------------------------------------------------------------------|
| `particle_cloud` | `nav2_msgs/ParticleCloud`                 | Output topic for particle cloud published at a fixed frequency.          |
| `pose`           | `geometry_msgs/PoseWithCovarianceStamped` | Output topic for estimated pose mean and covariance in map frame.        |

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

## ROS 1 Interface

### Parameters

Beluga AMCL currently supports the majority of ROS parameters used in [AMCL][amcl].<br/>
See [Beluga AMCL parameter reference](docs/PARAMETERS.md) for detailed information.

### Subscribed Topics

The subscribed topic names can be changed with the parameters `map_topic`, `scan_topic` and `initial_pose_topic`.

| Topic            | Type                                      | Description                                                                 |
|------------------|-------------------------------------------|-----------------------------------------------------------------------------|
| `map`            | `nav_msgs/OccupancyGrid`                  | Input topic for map updates.                                                |
| `scan`           | `sensor_msgs/LaserScan`                   | Input topic for laser scan updates.                                         |
| `initialpose`    | `geometry_msgs/PoseWithCovarianceStamped` | Input topic for pose mean and covariance to initialize the particle filter. |

### Published Topics

| Topic            | Type                                      | Description                                                              |
|------------------|-------------------------------------------|--------------------------------------------------------------------------|
| `particlecloud`  | `geometry_msgs/PoseArray`                 | Output topic for particle cloud published at a fixed frequency.          |
| `amcl_pose`      | `geometry_msgs/PoseWithCovarianceStamped` | Output topic for estimated pose mean and covariance in map frame.        |
| `diagnostics`    | `diagnostic_msgs/DiagnosticArray`         | Output topic for node diagnostics.                                       |

### Published Transforms

The frame names can be changed with the parameters `global_frame_id`, `odom_frame_id` and `base_frame_id`.
Defaults are `map`, `odom` and `base`.

| Transform         | Description                                                                                        |
|-------------------|----------------------------------------------------------------------------------------------------|
| `odom` to `base`  | Input transform used by motion models and resampling policies.                                     |
| `base` to `laser` | Input transform used to convert laser scan points to base frame.                                   |
| `map` to `odom`   | Output transform calculated from the estimated pose mean and the current _odom-to-base_ transform. |

### Advertised Services

| Topic                              | Type              | Description                                                                   |
|------------------------------------|-------------------|-------------------------------------------------------------------------------|
| `global_localization`              | `std_srvs/Empty`  | Request to reinitialize global localization without an initial pose estimate. |
| `request_nomotion_update`          | `std_srvs/Empty`  | Trigger a forced update of the filter estimates.                              |
| `set_map`                          | `nav_msgs/SetMap` | Set a new map and initial pose estimate.                                      |

### Called Services

| Topic                              | Type              | Description                                                                   |
|------------------------------------|-------------------|-------------------------------------------------------------------------------|
| `static_map`                       | `nav_msgs/GetMap` | To retrieve map on initialization, if `use_map_topic` parameter is `false`    |

## Performance

Performance reports are periodically generated and uploaded to track performance improvements and regressions. These reports are generated using a set of scripts in the [beluga_benchmark](../beluga_benchmark) package which can be used to compare the performance of `beluga_amcl` against that of `nav2_amcl` using a synthetic dataset.

The following plot displays the RSS (Resident Set Size), CPU usage, APE (Absolute Pose Error) and processing latency statistics for both  `beluga_amcl` and `nav2_amcl`, with particle sizes ranging between 250 and 200000 and sensor model `likelihood field`.

![Beluga vs Nav2 AMCL](../beluga_benchmark/docs/reports/2023-09-02/likelihood_beluga_vs_beluga_vs_amcl.png)

The following plot displays the RSS (Resident Set Size), CPU usage, APE (Absolute Pose Error) and processing latency statistics for both  `beluga_amcl` and `nav2_amcl`, with particle sizes ranging between 250 and 200000 and sensor model `beam`.

![Beluga vs Nav2 AMCL](../beluga_benchmark/docs/reports/2023-09-02/beam_beluga_vs_beluga_vs_amcl.png)

Further details can be found in [the reports folder here](../beluga_benchmark/docs/reports/).

## Next Steps

- See [example launch files](../beluga_example) showing how to run Beluga-based nodes.
- See [available benchmarks](../beluga_benchmark) for scripts and comparison with other AMCL implementations.

[amcl]: https://github.com/ros-planning/navigation/tree/noetic-devel/amcl
[nav2_amcl]: https://github.com/ros-planning/navigation2/tree/main/nav2_amcl
[nav2_configuration_guide]: https://navigation.ros.org/configuration/packages/configuring-amcl.html
[fox2001]: https://dl.acm.org/doi/10.5555/2980539.2980632
