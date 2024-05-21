# API Reference

This is the API reference for Beluga ROS. It provides utilities to aid Beluga integration with ROS and ROS 2.

### Components

Explore the library's components:

#### Generic filters

Generic MCL implementations ready for ROS integration.

| | |
|-|-|
| beluga_ros::Amcl | 2D lidar Adaptive MCL algorithm, functionally equivalent to that of [Nav2 AMCL](https://index.ros.org/p/nav2_amcl). |

#### Data structures

Thin, Beluga-compatible wrappers for typical ROS data structures (usually messages).

| | |
|-|-|
| beluga_ros::LaserScan | A beluga::BaseLaserScan subclass that wraps [sensor_msgs/LaserScan](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) messages. |
| beluga_ros::OccupancyGrid | A beluga::BaseOccupancyGrid2 subclass that wraps [nav_msgs/OccupancyGrid](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html) messages. |

#### Utilities

| | |
|-|-|
| [particle_cloud.hpp](@ref particle_cloud.hpp) | APIs for particle cloud I/O over ROS interfaces. |
| [tf2_eigen.hpp](@ref tf2_eigen.hpp) | tf2 message conversion API overloads for Eigen types. |
| [tf2_sophus.hpp](@ref tf2_sophus.hpp) | tf2 message conversion API overloads for Sophus types. |
