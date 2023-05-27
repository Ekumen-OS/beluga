# Beluga AMCL Parameter Reference

This page describes the parameters supported by Beluga AMCL including a comparison table with Nav2 AMCL and compatibility notes.

## ROS Interface Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `base_frame_id` | Robot base frame name rigidly attached to the mobile robot base. | ✅ | ✅ |
| `odom_frame_id` | Odometry frame name. The pose of a mobile platform relative to this frame can drift over time but it must be continuous (without discrete jumps). | ✅ | ✅ |
| `global_frame_id` | Map frame name. This node can estimate and publish the transform between global and odometry frames. Pose and map messages should use this coordinate frame. | ✅ | ✅ |
| `scan_topic` | The name of the topic where laser scans will be published. A transform must exist between the coordinate frame used in the scan messages and the base frame of the robot. | ✅ | ✅ |
| `map_topic` | The name of the topic where the map will be published by the map server. | ✅ | ✅ |
| `initial_pose_topic` | The name of the topic where an initial pose can be published.<br/> _A parameter that allows changing the topic name doesn't exist in Nav2 AMCL, but the `initialpose` topic can be [remapped externally](https://design.ros2.org/articles/static_remapping.html)._ | | ✅ |

## Initial Pose and Transform Broadcast Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `set_initial_pose` | Whether to set initial pose from the `initial_pose*` parameters or wait for an initial pose message. | ✅ | ✅ |
| `initial_pose.[x, y, yaw]` | X, Y and yaw coordinates of initial pose of robot base frame in global frame. | ✅ | ✅ |
| <nobr>`initial_pose.covariance_[x, y, yaw, xy, xyaw, yyaw]`</nobr> | Covariance to use with the initial pose when initializing the particle filter. _Nav2 AMCL considers these to be zero._ | | ✅ |
| `always_reset_initial_pose` | Whether to wait for an initial pose provided either via topic or `initial_pose*` parameter when reset or use the last known pose to initialize. | ✅ | ✅ |
| `first_map_only` | Whether to ignore any other map messages on the `map` topic after the first one. | ✅ | ✅ |
| `save_pose_rate` | Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server. _This parameter exists in Nav AMCL in ROS 1, but it wasn't ported to ROS 2 since there is no parameter server._ | | |
| `tf_broadcast` | Whether to publish the transform between the global frame and the odometry frame. The transform will be published only if an initial pose was set via topic or parameters, or if global localization was requested via the provided service. | ✅ | ✅ |
| `transform_tolerance` | Time by which to post-date the global to odom transform to indicate that it is valid in the future. | ✅ | ✅ |

## Particle Filter Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `max_particles` | Maximum allowed number of particles. | ✅ | ✅ |
| `min_particles` | Minimum allowed number of particles. | ✅ | ✅ |
| `pf_z` | Upper standard normal quantile for $P$, where $P$ is the probability that the error in the estimated distribution will be less than `pf_err` in KLD resampling [[1]](#1). | ✅ | ✅ |
| `pf_err` | Maximum particle filter population error between the true distribution and the estimated distribution. It is used in KLD resampling [[1]](#1) to limit the allowed number of particles to the minimum necessary. | ✅ | ✅ |
| <nobr>`spatial_resolution_[x, y, theta]`</nobr> | Spatial resolution to create buckets for KLD resampling [[1]](#1). | | ✅ |
| `recovery_alpha_fast` | Exponential decay rate for the fast average weight filter, used in deciding when to recover from a bad approximation by adding random poses [[3]](#3). | ✅ | ✅ |
| `recovery_alpha_slow` | Exponential decay rate for the slow average weight filter, used in deciding when to recover from a bad approximation by adding random poses [[3]](#3). | ✅ | ✅ |
| `resample_interval` | Number of filter updates required before resampling. | ✅ | ✅ |
| `selective_resampling` | Whether to enable selective resampling [[2]](#2) to help avoid loss of diversity in the particle population. The resampling step will only happen if the effective number of particles $(N_{eff} = 1/ {\sum w_i^2})$ is lower than half the current number of particles, where $w_i$ refers to the normalized weight of each particle.<br/> _This feature is currently supported by Nav AMCL in ROS 1 but it hasn't been ported to ROS 2 at the time of this writing._ | | ✅ |
| `update_min_a` | Rotational movement required from last resample for resampling to happen again. See [compatibility notes](#compatibility-notes). | ✅ | ✅ |
| `update_min_d` | Translational movement required from last resample for resampling to happen again. See [compatibility notes](#compatibility-notes). | ✅ | ✅ |
| `execution_policy` | Execution policy used to apply the motion update and importance weight steps to each particle (`seq` for sequential execution and `par` for parallel execution). | | ✅ |

## Motion Model Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `robot_model_type` | Which odometry motion model to use. Supported models are `differential_drive` [[3]](#3), `omnidirectional_drive` and `stationary`. See [compatibility notes](#compatibility-notes). | ✅ | ✅ |
| `alpha1` | Expected process noise in odometry’s rotation estimate from rotation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha2` | Expected process noise in odometry’s rotation estimate from translation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha3` | Expected process noise in odometry’s translation estimate from translation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha4` | Expected process noise in odometry’s translation estimate from rotation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha5` | Expected process noise in odometry's strafe estimate from translation for the omnidirectional drive model. | ✅ | ✅ |

## Observation Model Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `laser_model_type` | Which observation model to use. Supported models are `beam` and `likelihood_field` as described in [[3]](#3) with the same aggregation formula used in Nav2 AMCL. | ✅ | ✅ |
| `laser_max_range` | Maximum scan range to be considered. | ✅ | ✅ |
| `laser_min_range` | Minimum scan range to be considered. | ✅ | ✅ |
| `max_beams` | How many evenly spaced beams in each scan will be used when updating the filter. | ✅ | ✅ |
| `sigma_hit` | Standard deviation of the hit distribution used in likelihood field and beam models. | ✅ | ✅ |
| `z_hit` | Mixture weight for the probability of hitting an obstacle used in likelihood field and beam models. | ✅ | ✅ |
| `z_rand` | Mixture weight for the probability of getting random measurements used in likelihood field and beam models. | ✅ | ✅ |
| `z_max` | Mixture weight for the probability of getting max range measurements used in the beam model. | ✅ | ✅ |
| `z_short` | Mixture weight for the probability of getting short measurements used in the beam model. | ✅ | ✅ |
| `lambda_short` | Short readings' exponential distribution parameter used in the beam model. | ✅ | ✅ |
| `laser_likelihood_max_dist` | Maximum distance to do obstacle inflation on map used in the likelihood field model. | ✅ | ✅ |
| `do_beamskip` | Whether to ignore the beams for which the majority of the particles do not match the map in the likelihood field model. | ✅ | |
| `beam_skip_distance` | Maximum distance to an obstacle to consider that a beam coincides with the map. | ✅ | |
| `beam_skip_threshold` | Minimum percentage of particles for which a particular beam must match the map to not be skipped. | ✅ | |
| `beam_skip_error_threshold` | Maximum percentage of skipped beams. Too many skipped beams trigger a full update to recover in case of bad convergence. | ✅ | |

## Compatibility Notes

- When applying any of the resampling policies (see `resample_interval`, `selective_resampling`, `update_min_a`, `update_min_d`), Nav2 AMCL skips the filter update entirely until the conditions are met, while Beluga AMCL skips only the resampling step (sensor and motion updates are not skipped).
- Beluga AMCL supports Nav2 AMCL plugin names (`nav2_amcl::DifferentialMotionModel`, `nav2_amcl::OmniMotionModel`) as a value in the `robot_model_type` parameter, but will load the equivalent Beluga model.

## Additional Notes

- Unless otherwise specified, all units are [REP-103](https://www.ros.org/reps/rep-0103.html) compliant.
- Coordinate frames should follow conventions specified in [REP-105](https://www.ros.org/reps/rep-0105.html).

## References

<a id="1">[1]</a> Dieter Fox. Kld-sampling: Adaptive particle filters. In Proceedings of the 14th International Conference on Neural Information Processing Systems: Natural and Synthetic, NIPS'01, pages 713–720, Cambridge, MA, USA, 2001. MIT Press. https://dl.acm.org/doi/10.5555/2980539.2980632

<a id="2">[2]</a> Giorgio Grisetti, Cyrill Stachniss, and Wolfram Burgard. Improved techniques for grid mapping with rao-blackwellized particle filters. IEEE Transactions on Robotics, 23(1):34–46, 2007. https://doi.org/10.1109/TRO.2006.889486

<a id="3">[3]</a> S. Thrun, W. Burgard, and D. Fox. Probabilistic Robotics. Intelligent Robotics and Autonomous Agents series. MIT Press, 2005. https://books.google.com.ar/books?id=jtSMEAAAQBAJ
