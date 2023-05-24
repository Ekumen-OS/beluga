# Beluga AMCL Parameter Reference

## ROS Interface Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `base_frame_id` | Robot base frame name rigidly attached to the mobile robot base. | ✅ | ✅ |
| `odom_frame_id` | Odometry frame name. The pose of a mobile platform relative to this frame can drift over time but it must be continuous (without discrete jumps). | ✅ | ✅ |
| `global_frame_id` | Map frame name. The pose of a mobile platform relative to this frame does not drift significantly over time but it can change in discrete jumps at any time. This node can estimate and publish the transform between global and odometry frames. | ✅ | ✅ |
| `scan_topic` | The name of the topic where laser scans are being published. | ✅ | ✅ |
| `map_topic` | The name of the topic where the map is published by the map server. | ✅ | ✅ |
| `initial_pose_topic` | The name of the topic where an initial pose can be published. _Navigation 2 supports a fixed subscriber in the `initialpose` topic._ | | ✅ |

## Initial Pose And Transform Broadcast Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `set_initial_pose` | Causes AMCL to set initial pose from the `initial_pose*` parameters instead of waiting for the initial pose message. | ✅ | ✅ |
| `initial_pose.[x, y, yaw]` | X, Y and yaw coordinates of initial pose of robot base frame in global frame. Units in meters and radians. | ✅ | ✅ |
| <nobr>`initial_pose.covariance_[x, y, yaw, xy, xyaw, yyaw]`</nobr> | Covariance to use with the initial pose when initializing the particle filter. _Navigation 2 considers these to be zero._ | | ✅ |
| `save_pose_rate` | Maximum rate (Hz) at which to store the last estimated pose and covariance to the parameter server. _ROS 2 does not have a parameter server, this parameter is mentioned in the documentation but it can't be implemented._ | | |
| `always_reset_initial_pose` | Requires that AMCL is provided an initial pose either via topic or `initial_pose*` parameter when reset. Otherwise, by default AMCL will use the last known pose to initialize. | ✅ | ✅ |
| `first_map_only` | When set to false, allows AMCL to accept maps more than once on the `map` topic. | ✅ | ✅ |
| `tf_broadcast` | Set this to false to prevent AMCL from publishing the transform between the global frame and the odometry frame. | ✅ | ✅ |
| `transform_tolerance` | Time with which to post-date the transform that is published to indicate that this transform is valid into the future. | ✅ | ✅ |

## Particle Filter Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `max_particles` | Maximum allowed number of particles. | ✅ | ✅ |
| `min_particles` | Minimum allowed number of particles. | ✅ | ✅ |
| `pf_err` | Maximum particle filter population error between the true distribution and the estimated distribution. It is used in [KLD resampling](fox2001) to limit the allowed number of particles to the minimum necessary. | ✅ | ✅ |
| `pf_z` | Upper standard normal quantile for P, where P is the probability that the error in the estimated distribution will be less than `pf_err` in [KLD resampling](fox2001). | ✅ | ✅ |
| `spatial_resolution_[x, y, theta]` | Spatial resolution to create buckets for [adaptive KLD resampling][fox2001].                                | | ✅ |
| `recovery_alpha_fast` | Exponential decay rate for the fast average weight filter, used in deciding when to recover from a bad approximation by adding random poses. | ✅ | ✅ |
| `recovery_alpha_slow` | Exponential decay rate for the slow average weight filter, used in deciding when to recover from a bad approximation by adding random poses. | ✅ | ✅ |
| `resample_interval` | Number of filter updates required before resampling. | ✅ | ✅ |
| `selective_resampling` | When set to true, it will reduce the resampling rate when not needed and help avoid particle deprivation. The resampling will only happen if the effective number of particles $(N_{eff} = 1/ {\sum k_i^2})$ is lower than half the current number of particles. | ✅ | ✅ |
| `update_min_a` | Rotational movement required from last resample for resampling to happen again. _Navigation 2 skips the filter update completely while Beluga only skips the resampling step. Sensor and motion updates are not skipped._ | ✅ | ✅ |
| `update_min_d` | Translational movement required from last resample for resampling to happen again. _Navigation 2 skips the filter update completely while Beluga only skips the resampling step. Sensor and motion updates are not skipped._ | ✅ | ✅ |
| `execution_policy` | Execution policy used to apply the motion update and importance weight steps to each particle [`seq`: sequential, `par`: parallel]. | | ✅ |

## Motion Model Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `robot_model_type` | Which odometry motion model to use [`differential_drive`, `omnidirectional_drive`, `stationary`]. _It also supports Navigation 2 plugin names [`nav2_amcl::DifferentialMotionModel` and `nav2_amcl::OmniMotionModel`] but it will load the equivalent Beluga model._ | ✅ | ✅ |
| `alpha1` | Expected process noise in odometry’s rotation estimate from rotation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha2` | Expected process noise in odometry’s rotation estimate from translation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha3` | Expected process noise in odometry’s translation estimate from translation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha4` | Expected process noise in odometry’s translation estimate from rotation for the differential and omnidirectional drive models. | ✅ | ✅ |
| `alpha5` | Expected process noise in odometry's strafe estimate from translation for the omnidirectional drive model. | ✅ | ✅ |

## Observation Model Parameters

| Parameter | Description | Navigation 2 AMCL | Beluga AMCL |
|-----------|-------------|:-----------------:|:-----------:|
| `laser_model_type` | Which observation model to use [`beam`, `likelihood_field`]. | ✅ | ✅ |
| `laser_max_range` | Maximum scan range to be considered. | ✅ | ✅ |
| `laser_min_range` | Minimum scan range to be considered. | ✅ | ✅ |
| `max_beams` | How many evenly-spaced beams in each scan to be used when updating the filter. | ✅ | ✅ |
| `sigma_hit` | Standard deviation of the hit distribution used in likelihood field and beam models. | ✅ | ✅ |
| `z_hit` | Mixture weight for the probability of hitting an obstacle used in likelihood field and beam models. | ✅ | ✅ |
| `z_rand` | Mixture weight for the probability of getting random measurements used in likelihood field and beam models. | ✅ | ✅ |
| `z_max` | Mixture weight for the probability of getting max range measurements used in the beam model. | ✅ | ✅ |
| `z_short` | Mixture weight for the probability of getting short measurements used in the beam model. | ✅ | ✅ |
| `lambda_short` | Short readings' exponential distribution parameter used in the beam model. | ✅ | ✅ |
| `laser_likelihood_max_dist` | Maximum distance to do obstacle inflation on map used in the likelihood field model. | ✅ | ✅ |
| `do_beamskip` | Whether to ignore the beams for which the majority of the particles do not match the map in the likelihood field model. | ✅ | |
| `beam_skip_distance` | Maximum distance to an obstacle to consider that a beam coincides with the map. Unit in meters. | ✅ | |
| `beam_skip_threshold` | Minimum percentage of particles for which a particular beam must match the map to not be skipped. | ✅ | |
| `beam_skip_error_threshold` | Maximum percentage of skipped beams. Too many skipped beams trigger a full update to recover in case of bad convergence. | ✅ | |

[fox2001]: https://dl.acm.org/doi/10.5555/2980539.2980632
