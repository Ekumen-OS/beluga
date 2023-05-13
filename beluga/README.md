# Beluga

Beluga is a ROS-agnostic C++17 library that provides implementations for Monte Carlo-based localization algorithms widely used in robotics applications.
Its modularity allows users to compose solutions from reusable modules and to combine them with new ones to configure the MCL algorithm that best suits their needs.

## Features

The current set of features includes:

- A configurable particle filter with support for:
  - [Structure of arrays and array of structures](https://en.wikipedia.org/wiki/AoS_and_SoA) storage policies.
  - Fixed resampling and adaptive KLD resampling policies to determine how many samples to take per iteration.
  - Selective resampling, on-motion resampling, interval resampling policies to determine when to resample.
  - Sequential and parallel execution policies.
  - Weighted mean and covariance statistics for pose estimation.
- Likelihood field and beam sensor models.
- Differential drive and omnidirectional motion models.

## Dependencies

Beluga is built on top of the following open source libraries:

- [Eigen](https://gitlab.com/libeigen/eigen): A well-known C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- [Sophus](https://github.com/strasdat/Sophus): A C++ implementation of Lie groups using Eigen.
- [Range](https://github.com/ericniebler/range-v3): The basis library for C++20's `std::ranges`.
- [libciabatta](https://github.com/atomgalaxy/libciabatta): A composable mixin support library.
