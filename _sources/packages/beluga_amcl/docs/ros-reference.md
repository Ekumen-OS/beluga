# ROS Reference

## Nodelets

### beluga\_amcl/AmclNodelet

2D AMCL as a nodelet. The nodelet is implemented as a thin wrapper, in charge of managing ROS communication, configuration, data conversion,
and ROS node initialization and shutdown, built around a single ROS agnostic Beluga particle filter.

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

`map_service` _(`string`)_
: The name of the service to request map updates to.
: Defaults to `static_map`.

`use_map_topic` _(`boolean`)_
: Whether to use a map topic or a map service for filter (re)initialization.
: Defaults to `true`.

`initial_pose_topic` _(`string`)_
: The name of the topic where an initial pose can be published.
: Defaults to `initialpose`.

##### Initial Pose and Transform Broadcast Parameters

`set_initial_pose` _(`boolean`)_
: Whether to set initial pose from the `initial_pose*` parameters or wait for an initial pose message.
: Defaults to `false`.

`initial_pose_[x, y, a]` _(`float`)_
: X, Y and yaw coordinates of initial pose of robot base frame in global frame.
: Defaults to `0.0`.

`initial_cov_[xx, yy, aa, xy, xa, ya]` _(`float`)_
: Covariance to use with the initial pose when initializing the particle filter.
: Defaults to `0.0`.

`always_reset_initial_pose` _(`boolean`)_
: Whether to wait for an initial pose provided either via topic or `initial_pose*` parameter when reset or use the last known pose to initialize.
: Defaults to `false`.

`save_pose_rate` _(`boolean`)_
: Rate at which to store the last estimated pose and covariance to the parameter server.
: Defaults to `0.5`.

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
: Defaults to `5000`.

`min_particles` _(`integer`)_
: Minimum allowed number of particles.
: Defaults to `100`.

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
: Defaults to `0.001`.

`recovery_alpha_slow` _(`float`)_
: Exponential decay rate for the slow average weight filter, used in deciding when to recover from a bad approximation by adding random poses {cite}`thrun2005probabilistic`.
: Defaults to `0.1`.

`resample_interval` _(`integer`)_
: Number of filter updates required before resampling.
: Defaults to `2`.

`selective_resampling` _(`boolean`, read-only)_
: Whether to enable selective resampling {cite}`grisetti2007selectiveresampling` to help avoid loss of diversity in the particle population. The resampling step will only happen if the effective number of particles $(N_{eff} = 1/ {\sum w_i^2})$ is lower than half the current number of particles, where $w_i$ refers to the normalized weight of each particle.
: Defaults to `false`.

`update_min_a` _(`float`)_
: Minimum rotation required from last resample for resampling to happen again. Must be in the $[0, 2\pi]$ interval.
: Defaults to `0.2`.

`update_min_d` _(`float`)_
: Minimum translation required from last resample for resampling to happen again. Must be nonnegative.
: Defaults to `0.523598` ($\pi / 6$).

`execution_policy` _(`string`)_
: Execution policy used to apply the motion update and importance weight steps to each particle. `seq` for sequential execution and `par` for parallel execution.
: Defaults to `seq`.

##### Motion Model Parameters

`robot_model_type` _(`string`)_
: Which odometry motion model to use. Supported models are `differential_drive` {cite}`thrun2005probabilistic`, `omnidirectional_drive` and `stationary`.
: Defaults to `differential_drive`.

`odom_alpha1` _(`float`)_
: Expected process noise in odometry’s rotation estimate from rotation for the `differential_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`odom_alpha2` _(`float`)_
: Expected process noise in odometry’s rotation estimate from translation for the `differential_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`odom_alpha3` _(`float`)_
: Expected process noise in odometry’s translation estimate from translation for the `differential_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`odom_alpha4` _(`float`)_
: Expected process noise in odometry’s translation estimate from rotation for the `differential_drive` and `omnidirectional_drive` models. Must be nonnegative.
: Defaults to `0.2`.

`odom_alpha5` _(`float`)_
: Expected process noise in odometry's strafe estimate from translation for the `omnidirectional_drive` model. Must be nonnegative.
: Defaults to `0.2`.

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

`laser_sigma_hit` _(`float`)_
: Standard deviation of the hit distribution used in `likelihood_field` and `beam` models.
: Defaults to `0.2`.

`laser_z_hit` _(`float`)_
: Mixture weight for the probability of hitting an obstacle used in `likelihood_field` and `beam` models.
: Defaults to `0.5`.

`laser_z_rand` _(`float`)_
: Mixture weight for the probability of getting random measurements used in `likelihood_field` and `beam` models.
: Defaults to `0.5`.

`laser_z_max` _(`float`)_
: Mixture weight for the probability of getting max range measurements used in the `beam` model.
: Defaults to `0.05`.

`laser_z_short` _(`float`)_
: Mixture weight for the probability of getting short measurements used in the `beam` model.
: Defaults to `0.05`.

`laser_lambda_short` _(`float`)_
: Short readings' exponential distribution parameter used in the `beam` model.
: Defaults to `0.1`.

`laser_likelihood_max_dist` _(`float`)_
: Maximum distance, in meters, to do obstacle inflation on map used in the `likelihood_field` model.
: Defaults to `2.0`.

##### Misc Parameters

`std_warn_level_[x, y, yaw]`
: Threshold on the standard deviation of the x, y, and yaw coordinates of the pose estimate before emitting warnings.\
: Defaults to `0.2`.

`restore_defaults` _(`boolean`)_
: Flag to restore parameter defaults. Reset (to `false`) upon restoration.
: Defaults to `false`.

### Published topics

`particlecloud`
: Estimated pose distribution published as `geometry_msgs/PoseArray` messages. It will only be published if subscribers are found.

`amcl_pose`
: Mean and covariance of the estimated pose distribution published as `geometry_msgs/PoseWithCovarianceStamped` messages (assumed Gaussian).

`/diagnostics`
: Filter [diagnostics](https://wiki.ros.org/diagnostics) published as `diagnostics_msgs/DiagnosticArray` messages.

### Subscribed topics

`<map_topic>`
: Occupancy grid map updates subscribed as `nav_msgs/OccupancyGrid` messages. Actual topic name is dictated by the `map_topic` parameter.
: Only subscribed if  `use_map_topic` is `true`.

`<initial_pose_topic>`
: Gaussian pose distribution subscribed as `geometry_msgs/PoseWithCovarianceStamped` messages for filter (re)initialization. Actual topic name is dictated by the `initial_pose_topic` parameter.

`<scan_topic>`
: Lidar scan updates subscribed as `sensor_msgs/LaserScan` messages. Actual topic name is dictated by the `scan_topic` parameter.

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

`global_localization`
: An `std_srvs/Empty` service to force a filter (re)initialization by sampling a uniform pose distribution over the last known map.

`request_nomotion_update`
: An `std_srvs/Empty` service to force a filter update upon request.

`set_map`
: A `nav_msgs/SetMap` service to force a map update.

### Called services

`<map_service>`
: A `nav_msgs/GetMap` service to get the first map from. Actual service name is dictated by the `map_service` parameter.
: Only called if `use_map_topic` is `false`.

### Compatibility Notes

- Beluga AMCL supports `diff-corrected` and `omni-corrected` motion models. Older `diff` and `omni` models from Navigation AMCL were not implemented.
- Unlike Navigation AMCL, Beluga AMCL needs both `laser_max_range` and `laser_min_range` to be specified (ie. -1.0 is not a valid value).
