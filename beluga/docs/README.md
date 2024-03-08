# Beluga

## Introduction

Beluga is a ROS-agnostic C++17 library that provides implementations for Monte Carlo-based localization algorithms widely used in robotics applications.
Its modularity allows users to compose solutions from reusable modules and to combine them with new ones to configure the MCL algorithm that best suits their needs.

The API extends concepts from the STL, emphasizing support for ranges of particles.

## Components

Explore the library's components organized by concept:

### Sensor models

Probabilistic sensor models that describe the likelihood of obtaining a certain measurement given a specific state of the system. \n
They satisfy \ref SensorModelPage.

| | |
|-|-|
| beluga::BeamSensorModel | Beam sensor model for range finders |
| beluga::LikelihoodFieldModel | Likelihood field sensor model for range finders |
| beluga::LandmarkSensorModel | Generic landmark model for discrete detection sensors (both 2D and 3D) |
| beluga::BearingSensorModel | Generic bearing sensor model for discrete detection sensors (both 2D and 3D) |

### Motion models

Probabilistic motion models that describe how the state of the system evolves over time. \n
They satisfy \ref MotionModelPage.

| | |
|-|-|
| beluga::DifferentialDriveModel | Sampled odometry model for a differential drive |
| beluga::OmnidirectionalDriveModel | Sampled odometry model for an omnidirectional drive |

### Range views

Useful views for working with ranges of particles.
They are lazily evaluated range adaptor objects compatible with the Range-v3 library.

| | |
|-|-|
| \link views/elements.hpp beluga::views::elements \endlink | Takes a view of tuple-like values and a number N and produces a view of Nth element of each tuple |
| \link views/random_intersperse.hpp beluga::views::random_intersperse \endlink | Inserts values from a generator function between contiguous elements based on a given probability |
| \link views/sample.hpp beluga::views::sample \endlink | Implements multinomial resampling for a given range of particles or distribution |
| \link views/particles.hpp beluga::views::states \endlink | Produces a view of the states of a range of particles |
| \link views/take_evenly.hpp beluga::views::take_evenly \endlink | Returns a range consisting of `count` elements evenly spaced over the source range |
| \link views/take_while_kld.hpp beluga::views::take_while_kld \endlink | Take elements from a range while the KLD condition is statisfied |
| \link views/particles.hpp beluga::views::weights \endlink | Produces a view of the weights of a range of particles |
| \link views/zip.hpp beluga::views::zip \endlink | Given N ranges, return a new range where the Mth element is a tuple of the Mth elements of all N ranges |

### Range actions

Useful actions for working with ranges of particles.
They are eagerly evaluated range adaptor objects compatible with the Range-v3 library.

| | |
|-|-|
| \link actions/assign.hpp beluga::actions::assign \endlink | Converts a view closure into an eagerly applied action closure |
| \link actions/normalize.hpp beluga::actions::normalize \endlink | Divides each value or weight (of ranges of particles) by a specified normalization factor |
| \link actions/propagate.hpp beluga::actions::propagate \endlink | Updates particle states based on their current value and a state transition (or sampling) function |
| \link actions/reweight.hpp beluga::actions::reweight \endlink | Updates particle weights based on a given measurement likelihood function |

### Policies

Policies that can be used to decide when to update the particle filter or resample the current set of particles.
They are lazily-evaluated possibly stateful predicate that can be composed with others using overloaded boolean operators.

| | |
|-|-|
| \link policies/every_n.hpp beluga::policies::every_n \endlink | Triggers every N calls |
| \link policies/on_effective_size_drop.hpp beluga::policies::on_effective_size_drop \endlink | Triggers when the Effective Sample Size (ESS) drops below a certain threshold |
| \link policies/on_motion.hpp beluga::policies::on_motion \endlink | Triggers on the detected motion |

### Containers

| | |
|-|-|
| beluga::CircularArray | An implementation of generic circular array modeled after `std::array` |
| beluga::TupleVector | An implementation of a tuple of containers with an interface that looks like a container of tuples |

### Distributions

| | |
|-|-|
| beluga::MultivariateNormalDistribution | A multivariate normal distribution modeled after `std::normal_distribution` |
| beluga::MultivariateUniformDistribution | A multivariate uniform distribution modeled after `std::uniform_distribution` |
