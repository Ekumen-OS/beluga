# API Reference

This is the API reference for Beluga. It extends concepts from the C++ standard library (STL), emphasizing support for ranges of particles.

### Components

Explore the library's components organized by concept:

#### Sensor models

Probabilistic sensor models that describe the likelihood of obtaining a certain measurement given a specific state of the system.
They all satisfy the [sensor model named requirements](@ref SensorModelPage).

| | |
|-|-|
| beluga::BeamSensorModel | Beam sensor model for range finders |
| beluga::LikelihoodFieldModel | Likelihood field sensor model for range finders |
| beluga::LandmarkSensorModel | Generic landmark model for discrete detection sensors (both 2D and 3D) |
| beluga::BearingSensorModel | Generic bearing sensor model for discrete detection sensors (both 2D and 3D) |

#### Motion models

Probabilistic motion models that describe how the state of the system evolves over time.
They satisfy the [motion model named requirements](@ref MotionModelPage).

| | |
|-|-|
| beluga::DifferentialDriveModel | Sampled odometry model for a differential drive |
| beluga::OmnidirectionalDriveModel | Sampled odometry model for an omnidirectional drive |

#### Range views

Useful views for working with ranges of particles.
They are lazily evaluated range adaptor objects compatible with the Range-v3 library.

| | |
|-|-|
| [beluga::views::elements](@ref views/elements.hpp) | Takes a view of tuple-like values and a number N and produces a view of Nth element of each tuple |
| [beluga::views::likelihoods](@ref views/likelihoods.hpp) | Produces a view of the likelihoods obtained after applying a specific model to a range of particles
| [beluga::views::random_intersperse](@ref views/random_intersperse.hpp) | Inserts values from a generator function between contiguous elements based on a given probability |
| [beluga::views::sample](@ref views/sample.hpp) | Implements multinomial resampling for a given range of particles or distribution |
| [beluga::views::states](@ref views/particles.hpp) | Produces a view of the states of a range of particles |
| [beluga::views::take_evenly](@ref views/take_evenly.hpp) | Returns a range consisting of `count` elements evenly spaced over the source range |
| [beluga::views::take_while_kld](@ref views/take_while_kld.hpp) | Take elements from a range while the KLD condition is statisfied |
| [beluga::views::weights](@ref views/particles.hpp) | Produces a view of the weights of a range of particles |
| [beluga::views::zip](@ref views/zip.hpp) | Given N ranges, return a new range where the Mth element is a tuple of the Mth elements of all N ranges |

#### Range actions

Useful actions for working with ranges of particles.
They are eagerly evaluated range adaptor objects compatible with the Range-v3 library.

| | |
|-|-|
| [beluga::actions::assign](@ref actions/assign.hpp) | Converts a view closure into an eagerly applied action closure |
| [beluga::actions::normalize](@ref actions/normalize.hpp) | Divides each value or weight (of ranges of particles) by a specified normalization factor |
| [beluga::actions::propagate](@ref actions/propagate.hpp) | Updates particle states based on their current value and a state transition (or sampling) function |
| [beluga::actions::reweight](@ref actions/reweight.hpp) | Updates particle weights based on a given measurement likelihood function |

#### Policies

Policies that can be used to decide when to update the particle filter or resample the current set of particles.
They are lazily-evaluated possibly stateful predicate that can be composed with others using overloaded boolean operators.

| | |
|-|-|
| [beluga::policies::every_n](@ref policies/every_n.hpp) | Triggers every N calls |
| [beluga::policies::on_effective_size_drop](@ref policies/on_effective_size_drop.hpp) | Triggers when the Effective Sample Size (ESS) drops below a certain threshold |
| [beluga::policies::on_motion](@ref policies/on_motion.hpp) | Triggers on the detected motion |

#### Containers

| | |
|-|-|
| beluga::CircularArray | An implementation of generic circular array modeled after `std::array` |
| beluga::TupleVector | An implementation of a tuple of containers with an interface that looks like a container of tuples |

#### Distributions

| | |
|-|-|
| beluga::MultivariateNormalDistribution | A multivariate normal distribution modeled after `std::normal_distribution` |
| beluga::MultivariateUniformDistribution | A multivariate uniform distribution modeled after `std::uniform_distribution` |
