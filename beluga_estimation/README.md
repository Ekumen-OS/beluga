# beluga_inference

*Note: This package is currently located in the `beluga` directory and is preparing for transition to `beluga_inference`.*

## Overview

`beluga_inference` is an extensible and reusable state estimation, localization, and mapping framework for robotics. Designed as a high-performance, middleware-agnostic C++ library, it formulates estimation problems on mathematical manifolds using a generalized form of the well-known factor graph formalism.

The framework completely decouples probabilistic estimation logic from specific communication middlewares, offering modular abstractions for manifolds, variables, constraints, distributions, and staged computational pipelines.

## Key Features

- **Manifold-Aware State Representation**: Native support for Lie groups and manifolds (including the planar special Euclidean group $\\mathrm{SE}(2)$ and $\\mathbb{R}^n$ Euclidean manifolds via Eigen and Sophus), ensuring mathematically rigorous handling of non-Euclidean state spaces, singularities, and tangent space uncertainties.
- **Dynamic Bayesian Networks (DBNs) & Factor Graphs**: Graph-based probabilistic modeling decoupling state variables from constraints, process dynamics, and measurement factors.
- **Flexible Estimation Algorithms**:
  - **Recursive Filtering Pipeline**: Staged execution architecture supporting initialization, input acquisition, preprocessing, solver update, and result publication stages.
  - **Extended Kalman Filter (EKF)**: On-manifold joint state EKF solver supporting non-linear process and measurement models with tangent-space covariance updates.
  - **Sample-Based / Mixture Distributions**: Dirac mixture distributions and sample representations suitable for particle filtering techniques.
  - **Scan Matching**: Experimental planar scan-to-scan matching contracts for 2D LiDAR scans.
- **Middleware-Agnostic Architecture**: Decouples estimation algorithms from transport layers through abstract middleware interfaces (`MiddlewareInterface`, `ClockInterface`, `ParametersInterface`, `TransformInterface`, `MessagePipe`).
- **High-Performance C++ Primitives**: Modern C++ architecture leveraging `Eigen`, `Sophus`, `range-v3`, and monadic error handling via `Result<T>`.

## Main Modules

### `core`

Provides the foundational abstractions and data structures for probabilistic estimation:

- **Foundational Concepts**: Abstract definitions for `Manifold`, `Variable`, `Constraint`, `Estimate`, `Distribution`, and `Identifiable`.
- **Dynamic Bayesian Network (DBN)**: `Network` representing variables and factor constraints, `NetworkVisitor` for graph traversal, and `EstimateStore` for managing state history.
- **On-Manifold Operations**: `OnManifoldEstimate`, `TangentDistribution`, and `LocallyNormalConstraint` for local tangent-space approximations.

### `manifolds`

Implements manifold geometry and Lie group operations:

- **`SE2Manifold`**: Special Euclidean group $\\mathrm{SE}(2)$ for planar robot poses (position $x, y$ and yaw $\\theta$).
- **`EuclideanManifold`**: Standard Euclidean vector spaces $\\mathbb{R}^n$.
- **`ProductManifold`**: Cartesian products of arbitrary manifolds (e.g., compounding pose, linear velocity, and sensor bias manifolds into a joint state space).

### `variables`

Typed state variables bound to manifolds:

- **`SE2Variable`**: Robot pose variables living on $\\mathrm{SE}(2)$.
- **`EuclideanVariable`**: Vector state variables (e.g., velocity, acceleration, sensor biases) living in $\\mathbb{R}^n$.

### `distributions`

Probability distributions defined over manifolds and their tangent spaces:

- **`NormalDistribution`**: Multivariate Gaussian distribution in vector spaces.
- **`SE2NormalDistribution` & `SE2TangentDistribution`**: Concentrated Gaussian distributions on $\\mathrm{SE}(2)$ tangent spaces.
- **`DiracMixtureDistribution`**: Discrete weighted sample mixtures representing empirical probability distributions.

### `constraints`

Probabilistic constraints and factors linking variables across time or observations:

- **Kinematic Constraints**: `SE2PoseKinematicConstraint`, `SE2VelocityKinematicConstraint`, `SE2ConstantVelocityKinematicConstraint`.
- **Priors & Likelihoods**: `NormalPriorConstraint`, `NormalLikelihoodConstraint`, `SE2NormalPosePriorConstraint`, `SE2NormalPoseLikelihoodConstraint`.
- **Sensor Constraints**: `IMUStepConstraint` for integrating inertial measurements.

### `compute`

Execution engines and estimation algorithms:

- **`RecursiveFilter`**: Configurable execution loop running staged operations (`init`, `input`, `preprocess`, `solver`, `output`).
- **Solvers**:
  - `EKF`: Extended Kalman Filter solver operating on product manifolds.
- **Operations**:
  - `IMUProbe` & `OdomProbe`: Ingest raw sensor messages and produce constraint factors.
  - `Odometry`: Odometry motion models.
  - `SE2PoseBroadcaster`: Publishes estimated $\\mathrm{SE}(2)$ poses.
  - `VariablePriorInitializer`: Bootstraps initial state variables from prior constraints.
- **Budget & Cancellation**: `ComputeBudget` ensuring real-time execution boundaries with cooperative cancellation tokens.

### `middleware`

Abstract communication interfaces:

- **Interfaces**: `MiddlewareInterface`, `ClockInterface`, `LoggerInterface`, `ParametersInterface`, `TopicsInterface`, `TransformInterface`, `RateInterface`.
- **Communication Primitives**: `MessagePipe`, `Subscriber`, `Publisher`, and `SE2Connector`.

### `msg`

Middleware-agnostic internal message data structures:

- `Header`: Timestamp and sequence metadata.
- `IMU`: Inertial measurement data.
- `Odometry`: Odometric position and velocity data.

### `common`

Core utility libraries:

- **`Result<T>`**: Monadic error handling using expected types and rich error descriptors.
- **`Registry<T>`**: Dynamic type and factory registry for runtime-configurable plugins and operations.
- **`TimeAwareBuffer`**: Chronological message buffering and time-synchronized querying.
- **`Ranges` & `Views`**: Functional range operations powered by `range-v3`.
- **Algebra Utilities**: Numerical utilities for interpolation, projection, and linear algebra.

### `experimental`

- **`PlanarScanMatcher`**: Scan-to-scan matching interfaces for planar range scans.

## Dependencies

- **Linear Algebra & Lie Groups**: `Eigen3`, `Sophus`
- **Ranges & Concurrency**: `range-v3`, `libconcurrentqueue`, `libexpected`
- **Formatting**: `fmt`
