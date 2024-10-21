# Extending Beluga

## Prerequisites

To effectively extend the Beluga library, you should be comfortable with:

- C++ programming, especially template programming
- Probability and statistical estimation techniques

:::{tip}
You may also want to take a look at the [design principles](../concepts/design-principles) and [architecture concepts](../concepts/architecture) that Beluga was built upon before jumping to code.
:::

To set up a development environment, follow Beluga [installation instructions](../getting-started/installation) first. In particular, Beluga [development workflows](https://github.com/Ekumen-OS/beluga/blob/main/DEVELOPING.md) may come in handy when contributing back to the library.

## Implementing Motion Models

Motion models in Beluga define how a robot’s state evolves over time based on control inputs. These models are crucial for state prediction, allowing the particle filter to estimate the robot's future position as it moves through its environment. For a deeper understanding of these, you may revisit Beluga's [key concepts](../concepts/key-concepts).

### Key Considerations

When implementing a motion model, you need to ensure it meets a number of requirements:
- It must define a `state_type` that represents the particle's state, often as a pose using structures like `Sophus::SE2d`.
- It must specify the `control_type` it accepts, typically representing velocities or other actions influencing the state.
- It must be callable that accepts a control action and returns a function that predicts the next state based on a given control input. This returned function must satisfy the [`StateSamplingFunction` requirements](https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/MotionModelPage.html).

### Sample implementatino

Below is a sample implementation of a velocity-based motion model that predicts the robot’s next state based on linear and angular velocities, with noise to account for uncertainty:

```cpp
#include <random>
#include <beluga/motion.hpp>
#include <Eigen/Dense>
#include <sophus/se2.hpp>

namespace beluga {

// Example of a simple velocity-based motion model using Sophus and Eigen.
class VelocityMotionModel {
 public:
  using state_type = Sophus::SE2d;       // Represents the particle's state as SE(2) pose.
  using control_type = Eigen::Vector2d;  // Represents control input: [linear_velocity, angular_velocity].

  // Constructor to initialize noise parameters for linear and angular velocities.
  VelocityMotionModel(double linear_stddev, double angular_stddev)
      : linear_noise_{0.0, linear_stddev}, angular_noise_{0.0, angular_stddev} {}

  // Callable function to produce a StateSamplingFunction based on a control input.
  auto operator()(const control_type& control) const {
    // Returns a lambda satisfying the StateSamplingFunction requirements.
    return [this, control](const state_type& current_state, auto& rng) -> state_type {
      // Sample noisy velocities using Gaussian distributions.
      const double noisy_linear_velocity = control[0] + linear_noise_(rng);
      const double noisy_angular_velocity = control[1] + angular_noise_(rng);

      // Predict the next state using noisy control and SE(2) transformations.
      Sophus::SE2d next_state = current_state;
      next_state.translation() += Eigen::Vector2d(
          noisy_linear_velocity * std::cos(current_state.angle()),
          noisy_linear_velocity * std::sin(current_state.angle()));
      next_state.so2() *= Sophus::SO2d::exp(noisy_angular_velocity);

      return next_state;
    };
  }

 private:
  std::normal_distribution<double> linear_noise_;  // Noise for linear velocity.
  std::normal_distribution<double> angular_noise_;  // Noise for angular velocity.
};

}  // namespace beluga
```

Let's break down the code to highlight the implementation techniques:

```cpp
#include <random>
#include <beluga/motion.hpp>
#include <Eigen/Dense>
#include <sophus/se2.hpp>
```

We start by including the necessary headers:
- `<random>` for pseudo-random noise generation.
- `Eigen` for vector and matrix operations.
- `Sophus` for handling Lie group elements like SE(2).

```cpp
namespace beluga {

// Example of a simple velocity-based motion model using Sophus and Eigen.
class VelocityMotionModel {
 public:
  using state_type = Sophus::SE2d;       // Represents the particle's state as SE(2) pose.
  using control_type = Eigen::Vector2d;  // Represents control input: [linear_velocity, angular_velocity].
```

Here, the `VelocityMotionModel` class is defined. `state_type` is specified as `Sophus::SE2d`, a common representation of the robot’s 2D pose, and `control_type` is defined as `Eigen::Vector2d`, consisting of linear and angular velocities.

```cpp
  // Constructor to initialize noise parameters for linear and angular velocities.
  VelocityMotionModel(double linear_stddev, double angular_stddev)
      : linear_noise_{0.0, linear_stddev}, angular_noise_{0.0, angular_stddev} {}
```

The constructor initializes the model with standard deviations for the linear and angular velocity noise. `linear_noise_` and `angular_noise_` are normal distributions centered at `0.0` with standard deviations specified by parameters.

```cpp
  // Callable function to produce a StateSamplingFunction based on a control input.
  auto operator()(const control_type& control) const {
    // Returns a lambda satisfying the StateSamplingFunction requirements.
    return [this, control](const state_type& current_state, auto& rng) -> state_type {
      // Sample noisy velocities using Gaussian distributions.
      double noisy_linear_velocity = control[0] + sample_noise(linear_noise_, rng);
      double noisy_angular_velocity = control[1] + sample_noise(angular_noise_, rng);
```

The `operator()` function returns a lambda. This lambda predicts the next state given a current state and a random number generator.

```cpp
      // Predict the next state using noisy control and SE(2) transformations.
      Sophus::SE2d next_state = current_state;
      next_state.translation() += Eigen::Vector2d(
          noisy_linear_velocity * std::cos(current_state.angle()),
          noisy_linear_velocity * std::sin(current_state.angle()));
      next_state.so2() *= Sophus::SO2d::exp(noisy_angular_velocity);

      return next_state;
    };
  }
```
The translation is updated based on the noisy linear velocity and the current orientation. The orientation (`so2`) is updated using the exponential map to handle the angular update.

### Next steps

For a real-world implementation, see [`beluga::OmnidirectionalDriveModel`](https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/classbeluga_1_1OmnidirectionalDriveModel.html)'s' implementation.

## Implementing sensor models

Sensor models in Beluga are responsible for assessing how likely it is that a given particle's state matches observed sensor data. By assigning weights to particles based on sensor data, sensor models play a crucial role in filtering out unlikely states and refining the robot's estimated position. For a deeper understanding of these, you may revisit Beluga's [key concepts](../concepts/key-concepts) too.

### Key Considerations

When implementing a sensor model in Beluga, you need to ensure it meets a number of requirements:

- It must define a `state_type` that represents the particle's state, often as a pose using structures like `Sophus::SE2d`.
- It must specify a `weight_type`, typically a numerical type, representing the weight calculated for each particle.
- It must list a `measurement_type` as the format of the sensor data, and for this you may want to look at [sensor data abstractions]()https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/dir_876c246173b27422a95ea0c3c06ba40d.html).
- It must be a callable (`operator()`) that accepts a sensor measurement and returns a function compliant with the [`StateWeightingFunction` requirements](https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/SensorModelPage.html). This function calculates the weight of a particle’s state given the measurement.

### Sample implementation

Below is an example of a simple range sensor model. This model calculates the likelihood of each particle’s state based on sensor measurements of distance from the origin:

```cpp
#include <cmath>
#include <random>
#include <beluga/sensor.hpp>
#include <Eigen/Dense>
#include <sophus/se2.hpp>

namespace beluga {

// Example of a simple range-based sensor model.
class RangeSensorModel {
 public:
  using state_type = Sophus::SE2d;        // Represents the particle's state as SE(2) pose.
  using weight_type = double;             // Particle weight, based on measurement likelihood.
  using measurement_type = double;        // Represents a distance measurement.

  // Constructor to initialize sensor noise parameters.
  RangeSensorModel(double sensor_stddev)
      : sensor_noise_{0.0, sensor_stddev} {}

  // Callable function to produce a StateWeightingFunction based on a measurement.
  auto operator()(measurement_type measurement) const {
    // Returns a lambda satisfying the StateWeightingFunction requirements.
    return [this, measurement](const state_type& current_state) -> weight_type {
      // Calculate the expected sensor position based on the robot's pose.
      double expected_measurement = current_state.translation().norm();

      // Calculate weight based on a Gaussian kernel.
      return std::exp(-0.5 * std::pow((expected_measurement - measurement) / sensor_noise_.stddev(), 2));
    };
  }

 private:
  std::normal_distribution<double> sensor_noise_;  // Noise for sensor measurement.
};

}  // namespace beluga
```

Let's break down the code to highlight the implementation techniques:

```cpp
#include <cmath>
#include <random>
#include <beluga/sensor.hpp>
#include <Eigen/Dense>
#include <sophus/se2.hpp>
```

The required headers are included:
- `<cmath>` for mathematical functions.
- `<random>` for handling noise in measurements.
- `Eigen` and `Sophus` for linear algebra and pose handling.

```cpp
namespace beluga {

// Example of a simple range-based sensor model.
class RangeSensorModel {
 public:
  using state_type = Sophus::SE2d;        // Represents the particle's state as SE(2) pose.
  using weight_type = double;             // Particle weight, based on measurement likelihood.
  using measurement_type = double;        // Represents a distance measurement.
```

Here, the `RangeSensorModel` class is defined. `state_type` is specified as `Sophus::SE2d`, representing the robot’s pose; `weight_type` is a `double`, indicating the computed weight for each particle; and `measurement_type` is `double`, representing the sensor measurement.

```cpp
  // Constructor to initialize sensor noise parameters.
  RangeSensorModel(double sensor_stddev)
      : sensor_noise_{0.0, sensor_stddev} {}
```
The constructor initializes a normal distribution, `sensor_noise_`, to simulate measurement noise. This distribution is centered at `0.0` with a standard deviation defined by `sensor_stddev`.

```cpp
  // Callable function to produce a StateWeightingFunction based on a measurement.
  auto operator()(measurement_type measurement) const {
    // Returns a lambda satisfying the StateWeightingFunction requirements.
    return [this, measurement](const state_type& current_state) -> weight_type {
      // Calculate the expected sensor position based on the robot's pose.
      double expected_measurement = current_state.translation().norm();
```
The `operator()` function returns a lambda. This lambda calculates the weight of a particle based on the error between the expected and actual measurements.

```cpp
      // Compute the distance between the expected and actual measurement.
      double distance = (expected_measurement - measurement).norm();

      // Calculate weight based on Gaussian likelihood.
      double weight = std::exp(-0.5 * std::pow(distance / sensor_noise_.stddev(), 2));

      return weight;
    };
  }
```
A Gaussian kernel is used to compute the weight, assigning higher values to states closer to the measurement.

### Next steps

For a real-world implementation, see [`beluga::LikelihoodFieldModel`](https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/classbeluga_1_1LikelihoodFieldModel.html)'s' implementation.

## Implementing Estimation Algorithms

Estimation algorithms in Beluga are responsible for synthesizing the most likely estimate of a robot's state from a set of weighted particles. Statistical location and dispersion measures such as sampled means and variances are typical.

Estimation algorithms are but one application of the [_niebloid_ concept](https://en.cppreference.com/w/Template:cpp/ranges/niebloid) in Beluga. While this section focuses on estimation algorithms, it is worth noting that niebloids are used more broadly across the library as a sane approach to customization.

### Key Considerations
To build a robust estimation algorithm that integrates well with Beluga:

- The algorithm must work with a set of particle states and their corresponding weights, which encode each particle's probability during estimation.
- The algorithm should be implemented using a niebloid construct.

### Sample implementation.

The following example demonstrates how to implement a scalar median estimation algorithm using a niebloid:

```cpp
#include <algorithm>
#include <numeric>
#include <range/v3/view/zip.hpp>
#include <vector>

namespace beluga {

namespace detail {

// Example of a niebloid for calculating the median of weighted scalar values.
struct weighted_median_fn {
  // Operator to compute the median for scalar values.
  template <class Values, class Weights, class Projection = ranges::identity>
  auto operator()(Values&& values, Weights&& weights, Projection projection = {}) const {
    using WeightType = std::decay_t<ranges::range_value_t<Weights>>;
    using ValueType = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>;
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);

    // Zip values with weights and sort them based on the projected value.
    std::vector<std::pair<ValueType, WeightType>> sorted_pairs;
    for (auto&& [value, weight] : ranges::views::zip(values, weights)) {
      sorted_pairs.emplace_back(projection(value), weight);
    }
    std::sort(sorted_pairs.begin(), sorted_pairs.end(), [](const auto& a, const auto& b) { return a.first < b.first; };);

    // Find the median based on cumulative weights.
    auto cumulative_weight = WeightType{0};
    auto half_total_weight = std::accumulate(weights.begin(), weights.end(), WeightType{0}) / 2;

    for (const auto& [projected_value, weight] : sorted_pairs) {
      cumulative_weight += weight;
      if (cumulative_weight >= half_total_weight) {
        return projected_value;
      }
    }

    // Fallback: return the last value in case no median found (should not happen with correct weights).
    return sorted_pairs.back().first;
  }
};

}  // namespace detail

// A niebloid instance to compute the median.
inline constexpr detail::weighted_median_fn weighted_median;

}  // namespace beluga
```

Let's break down the code to highlight the implementation techniques:

```cpp
#include <algorithm>
#include <numeric>
#include <range/v3/view/zip.hpp>
#include <vector>
```

We begin by including the required headers:
- `<algorithm>` and `<numeric>` for sorting and accumulating operations.
- `<range/v3/view/zip.hpp>` for zipping ranges of values and weights.
- `<vector>` for storing the sorted pairs of values and weights.

```cpp
namespace beluga {
namespace detail {
```
The implementation is structured within the `beluga` and `detail` namespaces, consistent with the library’s organization.

```cpp
// Example of a niebloid for calculating the median of weighted scalar values.
struct weighted_median_fn {
  // Operator to compute the median for scalar values.
  template <class Values, class Weights, class Projection = ranges::identity>
  auto operator()(Values&& values, Weights&& weights, Projection projection = {}) const {
    using WeightType = std::decay_t<ranges::range_value_t<Weights>>;
    using ValueType = std::decay_t<std::invoke_result_t<Projection, ranges::range_value_t<Values>>>;
    static_assert(ranges::input_range<Values>);
    static_assert(ranges::input_range<Weights>);
```
The `weighted_median_fn` struct is defined, focusing on the scalar median computation. The `operator()` function handles ranges of values and weights. Note the optional `projection`, allowing value transformation before processing. The result type of the projection may differ from the original values, requiring the use of `std::decay_t` to manage type adjustments.

```cpp
    // Zip values with weights and sort them based on the projected value.
    std::vector<std::pair<ValueType, WeightType>> sorted_pairs;
    for (auto&& [value, weight] : ranges::views::zip(values, weights)) {
      sorted_pairs.emplace_back(projection(value), weight);
    }
    std::sort(sorted_pairs.begin(), sorted_pairs.end(), [](const auto& a, const auto& b) { return a.first < b.first; };);
```
Values and weights are zipped together. The projection is applied to each value, and they are stored with their weights in `sorted_pairs`. These pairs are then sorted based on the projected value using `std::sort`.

```cpp
    auto cumulative_weight = WeightType{0};
    auto half_total_weight = std::accumulate(weights.begin(), weights.end(), WeightType{0}) / 2;

    for (const auto& [projected_value, weight] : sorted_pairs) {
      cumulative_weight += weight;
      if (cumulative_weight >= half_total_weight) {
        return projected_value;
      }
    }
```
The median is calculated using cumulative weights. The algorithm iterates through the sorted pairs, summing the weights. When the cumulative weight exceeds half the total, the median value is identified.

```cpp
    // Fallback: return the last value in case no median found (should not happen with correct weights).
    return sorted_pairs.back().first;
  }
};
```
A fallback mechanism is provided for when and if no median is found due to an anomaly in the weights.

```cpp
}  // namespace detail

// A niebloid instance to compute the median.
inline constexpr detail::weighted_mean_fn weighted_median;

}  // namespace beluga
```

The `weighted_median` niebloid is instantiated.

### Next steps

For a real-world implementation, see [`beluga/algorithm/estimation.hpp`](https://ekumen-os.github.io/beluga/packages/beluga/docs/_doxygen/generated/reference/html/estimation_8hpp.html) content.
