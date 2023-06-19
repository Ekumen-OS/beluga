This library provides implementations for Monte Carlo based localization algorithms.
The existing implementations are:

- MCL: Monte Carlo Localization using a fixed number of particles.
- AMCL: Adaptive Monte Carlo Localization, the number of particles is defined using the KLD criteria.

The library is extensible, allowing to:
- Provide different [sensor](@ref SensorModelPage) or [motion](@ref MotionModelPage) models.
- Provide a way of getting the [estimated pose](@ref StateEstimatorPage) based on the particles.
- Provide a way of [generating the initial particles](@ref StateGeneratorPage).
- Provide a way of [sampling](@ref SamplerPage) from the previous particle set.
- Provide a way to [limiting](@ref LimiterPage) the amount of particles.
