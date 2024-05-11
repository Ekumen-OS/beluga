# Extending Beluga

## Introduction

This guide demonstrates how to extend the capabilities of the Beluga library by adding custom models and algorithms. The focus will be on understanding the existing codebase and introducing minimal code examples that provide a foundation for building more complex and useful extensions.

Here’s how a minimal particle filter algorithm could be structured using Beluga. For the purpose of this guide, we will ignore how the particle set is initialized, and will focus only on the core of the algorithm.

```cpp
constexpr auto kMaxParticles = 1'000;

// Define and initialize key components (parameters omitted for simplicity)
auto particles = ...;
auto motion_model = ...;
auto sensor_model = ...;

for (auto&& [control, measurement] : datapoints) {

  // Update
  particles |=
    beluga::actions::propagate(motion_model(control)) |
    beluga::actions::reweight(sensor_model(measurement)) |
    beluga::actions::normalize();

  // Resampling
  particles |=
    beluga::views::sample |
    ranges::views::take_exactly(kMaxParticles) |
    beluga::actions::assign;

  // Estimation
  const auto estimate = beluga::estimate(particles);
}
```

Any part of this code can be customized or extended by adhering to specific constraints which we will discuss. In the following sections, we will explore the customization of different components, starting with the motion and sensor models.

## Motion Model

In particle filter algorithms, motion models are crucial for predicting the next state of a particle based on its current state and the control inputs. By customizing the motion model, developers can tailor the library to more accurately reflect the dynamics of specific systems or environments. This section explains how motion models work and how they can be implemented and adapted to meet specific needs.

### Definition

A motion model, also known as a state transition model, describes the posterior distribution that a particle assumes when executing an action command or control. It is represented by the conditional density:

$$
    p(x_t | u_t, x_{t-1})
$$

Where $u_t$ is the control or action, $x_t$ is the current state of the particle, and $x_{t-1}$ is the previous state.

In the context of particle filter algorithms, instead of computing the posterior for arbitrary $x_t$, $u_t$, and $x_{t-1}$, it suffices to generate a random $x_t$ sample according to the model given $u_t$ and $x_{t-1}$.

### Implementation

In this library, motion models are implemented as callable objects that take a control action and return another callable, a state sampling function.
This function can be applied to each particle, generating a random sample given the particle's previous state and a random number generator.

Before diving into the implementation, we should see how the motion model is used in the particle filter algorithm shown above:

```cpp
particles |= beluga::actions::propagate(motion_model(control));
```

This functional composition can be explained as follows:

1. The motion model uses the control value as input to generate a function that dictates how each particle's state should be adjusted.
2. The `propagate` function takes the state sampling function and creates a range action that can be applied to a range of particles.
3. The `|=` applies the range action to the entire set of particles, updating each one according to the model.

This is the basic template we will use for creating a custom motion model:

```cpp
struct MotionModel {
 public:
  using control_type = ...;
  using state_type = ...;

  auto operator()(const control_type& action) const {
    ...
    return [=](state_type state, auto& engine) -> state_type { ... };
  }
};
```

Let's now implement a simple motion model of an agent moving across a 1-dimensional world. For this we will use the following definitions:

- The state will be the position of the agent along an infinite line. Represented as a `double`.
- The control will be the desired displacement of the agent in that line. Represented as a `double`.
- The uncertainty associated with the agent reaching its intended position after a control command is issued is modeled using a Gaussian distribution.
- The spread of the Gaussian distribution is controlled by a standard deviation, which is a constant parameter that can be specified when initializing the model.

The resulting implementation could look like this:

```cpp
struct MotionModel {
 public:
  using control_type = double;
  using state_type = double;

  explicit SimpleMotionModel(double stddev) : stddev_{stddev} {}

  auto operator()(control_type action) const {
    auto param = typename std::normal_distribution<double>::param_type{action, stddev_};

    return [param, action](state_type position, auto& engine) -> state_type {
      static thread_local auto distribution = std::normal_distribution<double>{};
      return position + distribution(engine, param);
    };
  }

 private:
  double stddev_;
};
```

The implementation of the `MotionModel` demonstrates a straightforward yet powerful way to integrate custom motion dynamics into the Beluga library. This library already provides implementations that can be used as reference or starting point for custom models. See for example:

- `beluga::DifferentialDriveModel`: Sampled odometry model for a differential drive robot.
- `beluga::OmnidirectionalDriveModel`: Sampled odometry model for an omnidirectional drive robot.

## Sensor Model

Sensor models are key to updating the belief based on new measurements. By customizing sensor models, developers can fine-tune how measurements influence the state estimation process, potentially improving the accuracy and reliability of the particle filter under various operating conditions. This section explains the fundamental concepts behind sensor models and demonstrates how to develop custom implementations.

### Definition

A sensor model, also known as a measurement model, is defined as a conditional probability distribution:

$$
    p(z_t|x_t, m)
$$

Where $z_t$ is the measurement at time $t$, $x_t$ is the current state of the particle, and $m$ is the map of the environment.

This model computes the likelihood of a measurement given the particle's state and the environment. It also accounts for the uncertainty in the measurements.

### Implementation

In this library, sensor models are implemented as callable objects that take a measurement and return another callable, a state weighting function.
This function can be applied to each particle to compute its weight or likelihood based on its current state.

This is how the sensor model is used in the particle filter algorithm shown above:

```cpp
particles |= beluga::actions::reweight(sensor_model(measurement));
```

This functional composition can be explained as follows:

1. The sensor model takes the measurement as input to generate a function that dictates how each particle’s weight should be adjusted. It holds an internal reference to a map of the environment to correctly determine the likelihood of each state.
2. The `reweight` function takes the state weighting function and creates a range action that can be applied to a range of particles.
3. The `|=` applies the range action to the entire set of particles, updating the weights of all particles according to the model.

This is the basic template we will use for creating a custom sensor model:

```cpp
struct SensorModel {
 public:
  using state_type = ...;
  using weight_type = ...;
  using measurement_type = ...;

  auto operator()(measurement_type measurement) const {
    ...
    return [=](state_type state) -> weight_type { ... };
  }
};
```

Let’s implement a simple sensor model of an agent moving across a 1-dimensional world. For this we will use the following definitions:

- The state will be the position of the agent along an infinite line. Represented as a `double`.
- The weight type will be `double`.
- The sensor modeled is a landmark detector, capable of measuring the distance between the agent and nearby landmarks.
- Measurements consist of an array of distances from the agent to each visible landmark.
- All landmarks are indistinguishable from one another, treated as identical in the context of measurement.
- The likelihood of each measurement is calculated using a Gaussian function centered at the distance to the nearest landmark.
- The weight of each particle is calculated by aggregating the likelihoods derived from all detections, transformed to the particle's frame of reference.
- The model's parameters include the standard deviation of the Gaussian distribution, which models the detection noise, and a map listing the positions of all landmarks.

The resulting implementation could look like this:

```cpp
struct SensorModel {
 public:
  using state_type = double;
  using weight_type = double;
  using measurement_type = std::vector<double>;

  SensorModel(double sigma, std::vector<double> map) : sigma_{sigma}, map_{std::move(map)} {}

  auto operator()(measurement_type detections) const {
    return [this, detections](state_type position) -> weight_type {
      return std::transform_reduce(
        std::begin(detections), std::end(detections), 0.0, std::plus{},
        [this, position](double detection) -> weight_type {
          // Compute the distances from this detection to all the landmarks.
          auto distances = map_ | ranges::views::transform(
            [position, detection](double landmark) {
              return std::abs(position + detection - landmark);
            }
          );

          // Compare the current detection against the closest landmark.
          double min_distance = ranges::min(distances);
          double min_distance_squared = min_distance * min_distance;
          return std::exp(-1 * min_distance_squared / (2 * sigma_));
        }
      );
    };
  }

 private:
  double sigma_;
  std::vector<double> map_;
};
```

This one might require a breakdown to fully grasp what is happening:

1. When the sensor model is called, it creates and returns a state weighting function (lambda expression) with a specific signature:
    ```cpp
    auto operator()(measurement_type detections) const {
      return [this, detections](state_type position) -> weight_type { ... };
    }
    ```
2. The state weighting function aggregates multiple detections from the measurement and returns the final particle weight:
    ```cpp
    [this, detections](state_type position) -> weight_type {
      return std::transform_reduce(
        std::begin(detections), std::end(detections), 0.0, std::plus{},
        [this, position](double detection) -> weight_type { ... }
      );
    };
    ```
3. For each detection, given the agent position, we find the closest landmark and compute the individual likelihood:
    ```cpp
    [this, position](double detection) -> weight_type {
      // Compute the distances from this detection to all the landmarks.
      auto distances = map_ | ranges::views::transform(...);

      // Compare the current detection against the closest landmark.
      double min_distance = ranges::min(distances);
      double min_distance_squared = min_distance * min_distance;
      return std::exp(-1 * min_distance_squared / (2 * sigma_));
    }
    ```

Typically, sensor model implementations require more work than motion models. Also, there are usually tradeoffs to be made regarding accuracy and performance. For completeness, here is a non-exhaustive list that includes some of the limitations of this implementation.

- Depending on the size of the map and the amount of detections, the performance could be improved by storing a sorted version of the landmark map, and performing binary search to find the closest element.
- The map is fixed during the life of the model, which may not reflect the real world accurately. There is nothing that prevents the sensor model from taking both, the measurement and the current map as parameters for generating a state weighting function.

Moreover, sensor models in typical particle filters algorithms have intrinsic limitations:

- How individual measurement probabilities are aggregated has a significant impact on filter performance. Approaches that quickly assign very low probabilities to bad particles tend to converge toward non-optimal estimates due to impoverishment of the particle set.
- When adjusting the intrinsic parameters of a measurement model (like `sigma`), it is often useful to artificially inflate the uncertainty. This is because because we are ignoring dependencies that exist in the physical world, along with a myriad of latent variables that cause these dependencies. When such dependencies are not modeled, algorithms that integrate evidence from multiple measurements quickly become overconfident, leading to wrong estimates. See Chapter 6.7 from {cite}`thrun2005probabilistic`.

This library already provides implementations that can be used as reference or starting point for custom models. See for example:

- `beluga::LikelihoodFieldModel`
- `beluga::BeamSensorModel`
- `beluga::LandmarkSensorModel`
