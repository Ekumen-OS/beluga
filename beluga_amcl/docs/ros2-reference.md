# ROS 2 Reference

## Nodes

### beluga_amcl::AmclNode

2D AMCL as a composable lifecycle node, with a [bond](https://github.com/ros/bond_core/tree/ros2) with a lifecycle manager.
The node is implemented as a thin wrapper, in charge of managing ROS 2 communication, configuration, data conversion,
and ROS 2 node initialization and shutdown, built around a single ROS 2 agnostic Beluga particle filter.

Also available as a standalone `amcl_node` executable.

#### Parameters

##### Interface Parameters

`base_frame_id` _(`string`)_
: Robot base frame name rigidly attached to the mobile robot base.
: Defaults to `base_footprint`.

`odom_frame_id` _(`string`)_
: Odometry frame name. The pose of a mobile platform relative to this frame can drift over time but it must be continuous (without discrete jumps).
: Defaults to `odom`.

`global_frame_id` _(`string`)_
: Map frame name. This node can estimate and publish the transform between global and odometry frames. Pose and map messages should use this coordinate frame.
: Defaults to `map`.

`scan_topic` _(`string`)_
: The name of the topic where laser scans will be published. A transform must exist between the coordinate frame used in the scan messages and the base frame of the robot.
: Defaults to `scan`.

`map_topic` _(`string`)_
: The name of the topic to subscribe to for map updates. Typically published by the [map server](https://github.com/ros-planning/navigation2/tree/main/nav2_map_server).
: Defaults to `map`.

`initial_pose_topic` _(`string`)_
: The name of the topic where an initial pose can be published.
: Defaults to `initialpose`.

##### Initial Pose and Transform Broadcast Parameters

`set_initial_pose` _(`boolean`)_
: Whether to set initial pose from the `initial_pose*` parameters or wait for an initial pose message.
: Defaults to `false`.

`initial_pose.[x, y, yaw]` _(`float`)_
: X, Y and yaw coordinates of initial pose of robot base frame in global frame.
: Defaults to `0.0`.

`initial_pose.covariance_[x, y, yaw, xy, xyaw, yyaw]` _(`float`)_
: Covariance to use with the initial pose when initializing the particle filter.
: Defaults to `0.0`.

`always_reset_initial_pose` _(`boolean`)_
: Whether to wait for an initial pose provided either via topic or `initial_pose*` parameter when reset or use the last known pose to initialize.
: Defaults to `false`.

`first_map_only` _(`boolean`)_
: Whether to ignore any other map messages on the `map` topic after the first one.
: Defaults to `false`.

`tf_broadcast` _(`boolean`)_
: Whether to publish the transform between the global frame and the odometry frame. The transform will be published only if an initial pose was set via topic or parameters, or if global localization was requested via the provided service.
: Defaults to `true`.

`transform_tolerance` _(`float`)_
: Time lapse, in seconds, by which to post-date the global to odom transform to indicate that it is valid in the future.
: Defaults to `1.0`.

##### Particle Filter Parameters

`max_particles` _(`integer`)_
: Maximum allowed number of particles.
: Defaults to `2000`.

`min_particles` _(`integer`)_
: Minimum allowed number of particles.
: Defaults to `500`.

`pf_z` _(`float`)_
: Upper standard normal quantile for $P$, where $P$ is the probability that the error in the estimated distribution will be less than `pf_err` in KLD resampling {cite}`fox2001adaptivekldsampling`.
: Defaults to `0.99`.

`pf_err` _(`float`)_
: Maximum particle filter population error between the true distribution and the estimated distribution. It is used in KLD resampling {cite}`fox2001adaptivekldsampling` to limit the allowed number of particles to the minimum necessary.
: Defaults to `0.05`.

`spatial_resolution_[x, y, theta]` _(`float`)_
: Spatial resolution to create buckets for KLD resampling {cite}`fox2001adaptivekldsampling`.
: Defaults to `0.5` for translation and `10°` for rotation.

`recovery_alpha_fast` _(`float`)_
: Exponential decay rate for the fast average weight filter, used in deciding when to recover from a bad approximation by adding random poses {cite}`thrun2005probabilistic`.
: Defaults to `0.0`.

`recovery_alpha_slow` _(`float`)_
: Exponential decay rate for the slow average weight filter, used in deciding when to recover from a bad approximation by adding random poses {cite}`thrun2005probabilistic`.
: Defaults to `0.0`.

`resample_interval` _(`integer`)_
: Number of filter updates required before resampling.
: Defaults to `1`.

`selective_resampling` _(`boolean`, read-only)_
: Whether to enable selective resampling {cite}`grisetti2007selectiveresampling` to help avoid loss of diversity in the particle population. The resampling step will only happen if the effective number of particles $(N_{eff} = 1/ {\sum w_i^2})$ is lower than half the current number of particles, where $w_i$ refers to the normalized weight of each particle.
: Defaults to `false`.

`update_min_a` _(`float`)_
: Minimum rotation required from last resample for resampling to happen again. Must be in the $[0, 2\pi]$ interval.
: Defaults to `0.2`.

`update_min_d` _(`float`)_
: Minimum translation required from last resample for resampling to happen again. Must be nonnegative.
: Defaults to `0.25`.

`execution_policy` _(`string`)_
: Execution policy used to apply the motion update and importance weight steps to each particle. `seq` for sequential execution and `par` for parallel execution.
: Defaults to `seq`.

##### Motion Model Parameters

`robot_model_type` _(`string`)_
: Which odometry motion model to use. Supported models are `differential_drive` {cite}`thrun2005probabilistic`, `omnidirectional_drive`, `ackermann_drive` and `stationary`.
: Defaults to `differential_drive`.

`alpha1` _(`float`)_
: Expected process noise in odometry’s rotation estimate from rotation for the `differential_drive`, `ackermann_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`alpha2` _(`float`)_
: Expected process noise in odometry’s rotation estimate from translation for the `differential_drive`, `ackermann_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`alpha3` _(`float`)_
: Expected process noise in odometry’s translation estimate from translation for the `differential_drive`, `ackermann_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`alpha4` _(`float`)_
: Expected process noise in odometry’s translation estimate from rotation for the `differential_drive`, `ackermann_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`alpha5` _(`float`)_
: Expected process noise in odometry's strafe estimate from translation for the `omnidirectional_drive` model. Must be nonnegative.
: Defaults to `0.2`.

`alpha6` _(`float`)_
: Expected process noise in odometry's orientation noise from translational velocity for the `ackermann_drive` model. Must be nonnegative.
: Defaults to `0.2`.

`alpha7` _(`float`)_
: Expected process noise in odometry's orientation noise from rotational velocity for the `ackermann_drive` model. Must be nonnegative.
: Defaults to `0.2`.

`wheelbase` _(`float`)_
: Expected length of the robot for the `ackermann_drive` model. Must be nonnegative.
: Defaults to `0.5`.
##### Observation Model Parameters

`laser_model_type` _(`string`)_
: Which observation model to use. Supported models are `beam` and `likelihood_field` as described in {cite}`thrun2005probabilistic` with the same aggregation formula used in Nav2 AMCL.
: Defaults to `likelihood_field`.

`laser_max_range` _(`float`)_
: Maximum scan range to be considered. Must be nonnegative.
: Defaults to `100.0`.

`laser_min_range` _(`float`)_
: Minimum scan range to be considered. Must be nonnegative.
: Defaults to `0.0`.

`max_beams` _(`integer`)_
: How many evenly spaced beams in each scan will be used when updating the filter.
: Defaults to `60`.

`sigma_hit` _(`float`)_
: Standard deviation of the hit distribution used in `likelihood_field` and `beam` models.
: Defaults to `0.2`.

`z_hit` _(`float`)_
: Mixture weight for the probability of hitting an obstacle used in `likelihood_field` and `beam` models.
: Defaults to `0.5`.

`z_rand` _(`float`)_
: Mixture weight for the probability of getting random measurements used in `likelihood_field` and `beam` models.
: Defaults to `0.5`.

`z_max` _(`float`)_
: Mixture weight for the probability of getting max range measurements used in the `beam` model.
: Defaults to `0.05`.

`z_short` _(`float`)_
: Mixture weight for the probability of getting short measurements used in the `beam` model.
: Defaults to `0.05`.

`lambda_short _(`float`)_
: Short readings' exponential distribution parameter used in the `beam` model.
: Defaults to `0.1`.

`laser_likelihood_max_dist` _(`float`)_
: Maximum distance, in meters, to do obstacle inflation on map used in the `likelihood_field` model.
: Defaults to `2.0`.

##### Misc Parameters

`autostart` _(`boolean`)_
: Whether the node should configure and activate itself or not. Avoids the need for a lifecycle manager.
: Defaults to `false`.

`autostart_delay` _(`float`)_
: Delay, in seconds, to wait before initiating an autostart sequence. Also the retry period when the sequence fails.
: Defaults to `0.0`.

### Published topics

`particle_cloud`
: Estimated pose distribution published as `geometry_msgs/msg/PoseArray` messages, using a sensor data QoS policy. It will only be published if subscribers are found.

`particle_markers`
: Estimated pose distribution visualization published as `visualization_msgs/msg/MarkerArray` messages, using a system default QoS policy. Each particle is depicted using an arrow. Each arrow is colored and scaled according to the weight of the corresponding state in the distribution. Large, bright red arrows represent the most likely states, whereas small, dim purple arrows represent the least likely states. The rest lie in between. It will only be published if subscribers are found.

`pose`
: Mean and covariance of the estimated pose distribution published as `geometry_msgs/msg/PoseWithCovarianceStamped` messages (assumed Gaussian), using a system default QoS policy.

### Subscribed topics

`<map_topic>`
: Occupancy grid map updates subscribed as `nav_msgs/msg/OccupancyGrid` messages, using a reliable transient local QoS policy with keep last of 1 (ie. single message latching). Actual topic name is dictated by the `map_topic` parameter.
: Only subscribed if  `use_map_topic` is `true`.

: Occupancy grid map subscribed for sensor models to work with.

`<initial_pose_topic>`
: Gaussian pose distribution subscribed as `geometry_msgs/msg/PoseWithCovarianceStamped` messages, using a system default QoS policy, for filter (re)initialization. Actual topic name is dictated by the `initial_pose_topic` parameter.

`<scan_topic>`
: Lidar scan updates subscribed as `sensor_msgs/msg/LaserScan` messages, using a sensor data QoS policy. Actual topic name is dictated by the `scan_topic` parameter.
: Lidar scans subscribed for sensor models to work with.

### Subscribed transforms

`<odom_frame_id>` → `<base_frame_id>`
: Odometry estimates as transforms from the configured odometry frame to the configured base frame. Used by motion models and resampling policies. Actual frame IDs are dictated by `odom_frame_id` and `base_frame_id` parameters.

`<base_frame_id>` → `scan_frame_id`
: Lidar extrinsics as transforms from the configured base frame to the lidar scan frame. Actual frame IDs are dictated by the `base_frame_id` parameter and `header.frame_id` member in `scan_topic` messages.

### Broadcasted transforms

`<global_frame_id>` → `<odom_frame_id>`
: Transforms from the configured global frame to the configured odometry frame, calculated such that when composed with the corresponding odometry estimate, the mean of the estimated pose distribution in the global frame results. Actual frame IDs are dictated by `global_frame_id` and `odom_frame_id` parameters.
: Only broadcasted if `tf_broadcast` is set to `true`.

### Advertised services

`reinitialize_global_localization`
: An `std_srvs/srv/Empty` service, using a default service QoS policy, to force a filter (re)initialization by sampling a uniform pose distribution over the last known map.

`request_nomotion_update`
: An `std_srvs/srv/Empty` service, using a default service QoS policy, to force a filter update upon request.

## Compatibility notes

- Beluga AMCL supports Nav2 AMCL plugin names (`nav2_amcl::DifferentialMotionModel`, `nav2_amcl::OmniMotionModel`) as a value in the `robot_model_type` parameter, but will load the equivalent Beluga model.
- Notes on parameter and feature availability between Beluga AMCL and Nav2 AMCL are condensed in the table below.

| Parameter | Notes | Navigation 2 AMCL | Beluga AMCL |  |
|---|---|:---:|:---:|---|
| `base_frame_id` |  | ✅ | ✅ |  |
| `odom_frame_id` |  | ✅ | ✅ |  |
| `global_frame_id` |  | ✅ | ✅ |  |
| `scan_topic` |  | ✅ | ✅ |  |
| `map_topic` |  | ✅ | ✅ |  |
| `initial_pose_topic` | A parameter that allows changing the topic name doesn't exist in Nav2 AMCL, but the `initialpose` topic can be [remapped externally](https://design.ros2.org/articles/static_remapping.html). |  | ✅ |  |
| `set_initial_pose` |  | ✅ | ✅ |  |
| `initial_pose.[x, y, yaw]` |  | ✅ | ✅ |  |
| `initial_pose.covariance_[x, y, yaw, xy, xyaw, yyaw]` | Nav2 AMCL considers these to be zero. |  | ✅ |  |
| `always_reset_initial_pose` |  | ✅ | ✅ |  |
| `first_map_only` |  | ✅ | ✅ |  |
| `tf_broadcast` |  | ✅ | ✅ |  |
| `transform_tolerance` |  | ✅ | ✅ |  |
| `max_particles` |  | ✅ | ✅ |  |
| `min_particles` |  | ✅ | ✅ |  |
| `pf_z` |  | ✅ | ✅ |  |
| `pf_err` |  | ✅ | ✅ |  |
| `spatial_resolution_[x, y, theta]` |  |  | ✅ |  |
| `recovery_alpha_fast` |  | ✅ | ✅ |  |
| `recovery_alpha_slow` |  | ✅ | ✅ |  |
| `resample_interval` |  | ✅ | ✅ |  |
| `selective_resampling` | This feature is currently supported by Nav AMCL in ROS 1 but it hasn't been ported to ROS 2 at the time of this writing. |  | ✅ |  |
| `update_min_a` |  | ✅ | ✅ |  |
| `update_min_d` |  | ✅ | ✅ |  |
| `execution_policy` |  |  | ✅ |  |
| `robot_model_type` | Beluga AMCL supports Nav2 AMCL plugin names (`nav2_amcl::DifferentialMotionModel`, `nav2_amcl::OmniMotionModel`) as a value in the `robot_model_type` parameter, but will load the equivalent Beluga model. | ✅ | ✅ |  |
| `alpha1` |  | ✅ | ✅ |  |
| `alpha2` |  | ✅ | ✅ |  |
| `alpha3` |  | ✅ | ✅ |  |
| `alpha4` |  | ✅ | ✅ |  |
| `alpha5` |  | ✅ | ✅ |  |
| `laser_model_type` |  | ✅ | ✅ |  |
| `laser_max_range` |  | ✅ | ✅ |  |
| `laser_min_range` |  | ✅ | ✅ |  |
| `max_beams` |  | ✅ | ✅ |  |
| `sigma_hit` |  | ✅ | ✅ |  |
| `z_hit` |  | ✅ | ✅ |  |
| `z_rand` |  | ✅ | ✅ |  |
| `z_max` |  | ✅ | ✅ |  |
| `z_short` |  | ✅ | ✅ |  |
| `lambda_short` |  | ✅ | ✅ |  |
| `laser_likelihood_max_dist` |  | ✅ | ✅ |  |
| `do_beamskip` | Whether to ignore the beams for which the majority of the particles do not match the map in the likelihood field model. Beluga AMCL does not support beam skipping. | ✅ |  |  |
| `beam_skip_distance` | Maximum distance to an obstacle to consider that a beam coincides with the map. Beluga AMCL does not support beam skipping. | ✅ |  |  |
| `beam_skip_threshold` | Minimum percentage of particles for which a particular beam must match the map to not be skipped. Beluga AMCL does not support beam skipping. | ✅ |  |  |
| `beam_skip_error_threshold` | Maximum percentage of skipped beams. Too many skipped beams trigger a full update to recover in case of bad convergence. Beluga AMCL does not support beam skipping. | ✅ |  |  |
