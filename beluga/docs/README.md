This library provides implementations for Monte Carlo based localization.
The existing implementations are:

- beluga::MCL: Monte carlo localization using a fixed number of particles.
- beluga::AMCL: Adaptive monte carlo localization, the number of particles is defined using the KLD criteria.

The library is extensible, allowing to:
<!--TODO(ivanpauno): When docs are completed, are links here-->
- Provide a different sensor or motion model.
- Provide a way to resample particles.
- Provide a way of getting the estimated pose based on the particles.
- Provide a way of generating the initial particles.
- Provide a way of resampling particles.
