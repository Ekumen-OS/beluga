# beluga_inference_ros

*Note: This package is currently located in the `beluga_ros` directory and is preparing for transition to `beluga_inference_ros`.*

## Overview

`beluga_inference_ros` is the official ROS 2 integration layer for the `beluga_inference` framework. It bridges Beluga's middleware-agnostic estimation core with the ROS 2 ecosystem, providing lifecycle-managed execution nodes, standardized message converters, tf2 coordinate transform bridges, and runtime parameter loading.

By leveraging modern ROS 2 patterns, `beluga_inference_ros` enables state estimation programs formulated in `beluga_inference` to be executed as composable lifecycle nodes with robust process supervision.

## Key Features

- **Managed ROS 2 Lifecycle Node**: Implements `beluga_estimation_ros::Node` using `rclcpp_lifecycle::LifecycleNode`, providing explicit, deterministic lifecycle state transitions (`unconfigured`, `inactive`, `active`, `finalized`).
- **Bond Heartbeat Monitoring**: Built-in support for Bond state monitoring (`bondcpp`), enabling fault detection, heartbeat timeouts, and automated coordination with lifecycle managers such as Nav2.
- **Dynamic Program & Plugin Loading**: Dynamically instantiates Bayesian programs (e.g., `RecursiveFilter`) and operations from Beluga's registry using ROS 2 parameters (`method.type`).
- **Standardized Message Converters**: Provides bidirectional conversion between standard ROS 2 message types (`sensor_msgs`, `nav_msgs`, `std_msgs`, `builtin_interfaces`) and internal `beluga` message types.
- **Middleware Bridge Implementation**: Fully implements `beluga_estimation::MiddlewareInterface` using ROS 2 primitives (`rclcpp`, `rclcpp_components`, `tf2_ros`).

## Main Modules

### `beluga_estimation_ros::Node` (`node.cpp`, `node.hpp`)

The central execution unit for running Beluga Bayesian programs in a ROS 2 environment:

- **Component Architecture**: Registered as a component via `rclcpp_components` for flexible container composition or standalone execution.
- **Program Lifecycle Management**:
  - `on_configure`: Instantiates the configured `beluga_estimation::BayesianProgram` from the registry and binds middleware adapters.
  - `on_activate`: Starts the dedicated worker execution thread and establishes the Bond heartbeat connection.
  - `on_deactivate`: Signals cooperative cancellation to worker threads, stops Bond heartbeats, and joins threads cleanly.
  - `on_cleanup`: Releases instantiated programs and resets state.
  - `on_shutdown`: Safely handles abrupt terminations from any primary state.
- **Thread & Budget Management**: Executes programs with cooperative cancellation tokens (`std_future::stop_source`) and inspects outcome actions (`RunOutcomeAction`).

### Middleware Adapters (`interfaces/`)

Implements Beluga's abstract middleware contracts using ROS 2 infrastructure:

- **`beluga_estimation_ros::Middleware`**: Root adapter providing access to node services and factory methods for middleware components.
- **`beluga_estimation_ros::Parameters`**: Handles ROS 2 parameter declarations, dynamic parameter queries, and overrides.
- **`beluga_estimation_ros::Clock` & `beluga_estimation_ros::Rate`**: Integrates ROS 2 simulation and wall-clock time sources (`rclcpp::Clock`, `rclcpp::Rate`).
- **`beluga_estimation_ros::Logger`**: Bridges logging into the ROS 2 logging subsystem (`rclcpp::Logger`).
- **`beluga_estimation_ros::Topics`**: Factory creating ROS 2 publishers and subscriptions.
- **`beluga_estimation_ros::Publisher` & `beluga_estimation_ros::Subscriber`**: Thin type-safe wrappers around `rclcpp::Publisher` and `rclcpp::Subscription`.
- **`beluga_estimation_ros::Transform`**: Coordinate frame transformation interface backed by `tf2_ros::Buffer` and `tf2_ros::TransformListener`.

### Message Converters (`converters/`)

Header-only conversion utilities between ROS 2 message types and internal Beluga representations:

- **`imu.hpp`**: Converts `sensor_msgs::msg::Imu` to `beluga_estimation::msg::Imu`.
- **`odometry.hpp`**: Converts `nav_msgs::msg::Odometry` to `beluga_estimation::msg::Odometry`.
- **`header.hpp` & `clock.hpp`**: Converts `std_msgs::msg::Header` and `builtin_interfaces::msg::Time` to `beluga_estimation::msg::Header` and system timestamps.
- **`parameter_value.hpp`**: Converts ROS 2 `rclcpp::ParameterValue` into `beluga_estimation::ParameterValue`.

### Plugins & Registry Extensions (`plugins/`, `common/`)

- **Plugin Registry**: Macros and helpers (`BELUGA_ESTIMATION_ROS_IMPORT_TYPE`) for dynamically loading external operations and Bayesian programs into the Beluga registry via `class_loader`.
- **`SE2ConnectorPlugin`**: Example plugin registering planar connector operations.

## Dependencies

- **Core Framework**: `beluga` (`beluga_inference`)
- **ROS 2 Core**: `rclcpp`, `rclcpp_components`, `rclcpp_lifecycle`
- **ROS 2 Messages**: `nav_msgs`, `sensor_msgs`, `std_msgs`
- **Transforms & Fault Tolerance**: `tf2_ros`, `bondcpp`, `class_loader`
