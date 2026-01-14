# Beluga

Beluga is a ROS-agnostic C++17 library that provides implementations for Monte Carlo-based localization algorithms widely used in robotics applications.
Its modularity allows users to compose solutions from reusable modules and to combine them with new ones to configure the MCL algorithm that best suits their needs.

## Features

The current set of features includes:

- Particle containers:
  - Support for [Array-of-Structures and Structure-of-Arrays][aos_soa] tuple containers
- Composable range adaptor views, actions, and algorithms:
  - Multivariate normal distributions in SE(2) and SE(3) space
  - Multivariate uniform distributions in SE(2) compatible with occupancy grids
  - Multinomial resampling from a particle range
  - [Adaptive KLD resampling][fox2001]
  - [Selective resampling][grisetti2007], on-motion resampling, and interval resampling policies
  - Support for sequential and parallel execution policies
  - Weighted mean and covariance statistics for pose estimation
- Sensor models:
  - Likelihood field model
  - Beam model
  - Landmark-based models (using landmark position or bearing)
- Motion models:
  - Differential drive model
  - Omnidirectional model

## Documentation

Auto-generated Doxygen documentation can be found in https://ekumen-os.github.io/beluga/.

## Dependencies

Beluga is built on top of the following open source libraries:

- [Eigen](https://gitlab.com/libeigen/eigen): A well-known C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- [Sophus](https://github.com/strasdat/Sophus): A C++ implementation of Lie groups using Eigen.
- [Range](https://github.com/ericniebler/range-v3): The basis library for C++20's `std::ranges`.

[aos_soa]: https://en.wikipedia.org/wiki/AoS_and_SoA
[fox2001]: https://dl.acm.org/doi/10.5555/2980539.2980632
[grisetti2007]: https://doi.org/10.1109/TRO.2006.889486
