# Beluga

## Introduction

Beluga is a ROS-agnostic C++17 library that provides implementations for Monte Carlo-based localization algorithms widely used in robotics applications.
Its modularity allows users to compose solutions from reusable modules and to combine them with new ones to configure the MCL algorithm that best suits their needs.

The API was designed to be mostly an extension of the concepts found in the STL to support ranges of particles.

## Components

Below is a list of components that can be found in this library classified by concept.

### Range views

These satisfy: https://en.cppreference.com/w/cpp/named_req/RangeAdaptorObject

- \link views/elements.hpp beluga::views::elements \endlink
- \link views/random_intersperse.hpp beluga::views::random_intersperse \endlink
- \link views/sample.hpp beluga::views::sample \endlink
- \link views/particles.hpp beluga::views::states \endlink
- \link views/take_evenly.hpp beluga::views::take_evenly \endlink
- beluga::views::take_while_kld
- beluga::views::weights

### Range actions

- beluga::actions::assign
- beluga::actions::normalize
- beluga::actions::propagate
- beluga::actions::reweight

### Policies

- beluga::policies::every_n
- beluga::policies::on_effective_size_drop
- beluga::policies::on_motion

### Containers

- beluga::CircularArray
- beluga::TupleVector

### Distributions

- beluga::MultivariateNormalDistribution
- beluga::MultivariateUniformDistribution

### Sensor models

Types that satisfy the [sensor](@ref SensorModelPage).

- beluga::BeamSensorModel
- beluga::LikelihoodFieldModel
- beluga::LandmarkSensorModel
- beluga::BearingSensorModel

### Motion models

Types that satisfy the [motion](@ref MotionModelPage).

- beluga::DifferentialDriveModel
- beluga::OmnidirectionalDriveModel

### Interface types

- beluga::BaseOccupancyGrid2
- beluga::BaseLaserScan
- beluga::LandmarkMap
