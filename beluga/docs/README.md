This library provides implementations for Monte Carlo based localization.
The existing implementations are:

- beluga::MCL: Monte carlo localization using a fixed number of particles.
- beluga::AMCL: Adaptive Monte Carlo localization, the number of particles is defined using the KLD criteria.

The library is extensible, allowing to:
- Provide a different [sensor](@ref SensorModelPage) or [motion](@ref MotionModelPage) model.
- Provide a way of getting the estimated pose based on the particles.
- Provide a way of [generating the initial particles](@ref ParticleBaselineGenerationPage).
- Provide a way of [generating particles samples](@ref ParticleSampledGenerationPage) from the previous particle set.
- Provide a way to [resample](@ref ParticleResamplingPage) particles.
