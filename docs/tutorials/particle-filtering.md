# Primer on Particle Filtering with Beluga

:::{figure} ../_images/beluga_tutorial.gif
:alt: Short video of a beluga_tutorial's record.
:::

## Overview

The purpose of this tutorial is to show how to implement a custom {abbr}`MCL (Monte Carlo Localiaztion)` algorithm using Beluga,
motivated by the example given by _Probabilistic Robotics [^ProbRob], page 200, figure 8.11_.

This example is about a one-dimensional world, composed by walls and doors. The robot is capable to move in a parallel free and continuous space,
where it constantly scaning the landmarks of the walls-doors world (the robot also moves only in a one-dimensional world). The last two graphs in the GIF showed above is a graphical representation of this world, where the blue bars represent the walls, the red bars represent the doors and the green bar is the robot.

To implement this example in a code we will need to create and define different components such as the map, the motion and sensor models, the generation of the initial set of partciles, the steps of the {abbr}`MCL (Monte Carlo Localiaztion)` algorithm and the estimation of the robot's position. Each component will be detailed explain in the following sections.

## Requirements

Before starting the tutorial, first [install Beluga](../getting-started/installation.md).

:::{important}
I highly recommend reading the [key concepts](../concepts/key-concepts.md) section before diving into this tutorial.
:::

## Tasks

Based on [key concepts](../concepts/key-concepts.md), this tutorial will show an implementation of the particle filter by performing the recursive update in two steps:

* [Prediction](#15-prediction)
* [Update](#16-update)

## 1 Write the Simulation code

To start, create a `main.cpp` file and put the following code inside. After that we are going to analize the code by parts.

```cpp
#include <cmath>
#include <random>
#include <vector>

#include <range/v3/action.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <beluga/beluga.hpp>

struct Parameters {
  std::size_t map_size{100};
  std::size_t number_of_particles{300};
  std::size_t number_of_cycles{100};
  double initial_position{0.0};
  double initial_position_sd{10.0};
  double dt{1.0};
  double velocity{1.0};
  double translation_sd{1.0};
  double sensor_range{3.0};
  double sensor_model_sigma{1.0};
  double min_particle_weight{0.08};
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};
};

using Particle = std::tuple<double, beluga::Weight>;

int main() {
  const Parameters parameters;

  std::normal_distribution<double> initial_distribution(parameters.initial_position, parameters.initial_position_sd);
  auto particles = beluga::views::sample(initial_distribution) |                  //
                   ranges::views::transform(beluga::make_from_state<Particle>) |  //
                   ranges::views::take_exactly(parameters.number_of_particles) |  //
                   ranges::to<std::vector>;

  double current_position{parameters.initial_position};
  for (std::size_t n = 0; n < parameters.number_of_cycles; ++n) {
    current_position += parameters.velocity * parameters.dt;

    if (current_position > static_cast<double>(parameters.map_size)) {
      break;
    }

    auto motion_model = [&](double particle_position, auto& random_engine) {
      const double distance = parameters.velocity * parameters.dt;
      std::normal_distribution<double> distribution(distance, parameters.translation_sd);
      return particle_position + distribution(random_engine);
    };

    auto detections =
        parameters.landmark_map |                                                                            //
        ranges::views::transform([&](double landmark) { return landmark - current_position; }) |             //
        ranges::views::remove_if([&](double range) { return std::abs(range) > parameters.sensor_range; }) |  //
        ranges::to<std::vector>;

    auto sensor_model = [&](double particle_position) {
      auto particle_detections =
          parameters.landmark_map |                                                                  //
          ranges::views::transform([&](double landmark) { return landmark - particle_position; }) |  //
          ranges::to<std::vector>;

      return parameters.min_particle_weight +
             std::transform_reduce(
                 detections.begin(), detections.end(), 1.0, std::multiplies<>{}, [&](double detection) {
                   auto distances =           //
                       particle_detections |  //
                       ranges::views::transform(
                           [&](double particle_detection) { return std::abs(detection - particle_detection); });
                   const auto min_distance = ranges::min(distances);
                   return std::exp((-1 * std::pow(min_distance, 2)) / (2 * parameters.sensor_model_sigma));
                 });
    };

    particles |= beluga::actions::propagate(std::execution::seq, motion_model);

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;

    particles |= beluga::views::sample |                                        //
                 ranges::views::take_exactly(parameters.number_of_particles) |  //
                 beluga::actions::assign;

    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
  }
  return 0;
}
```

:::{note}
The code is written using C++17 with the ranges-v3 [^RangesV3] library for range handling along with its respective algorithms.
:::

### 1.1 Parameters

This struct contain all the necessary parameters the code needs to conduct the simulation.

```cpp
struct Parameters {
  /// Size of the 1D map in (m)
  /**
   * In the simulation, the 1D map is continuous,
   * but the limit is defined as an integer to simplify data visualization.
   */
  std::size_t map_size{100};

  /// Fixed number of particles used by the algorithm.
  std::size_t number_of_particles{300};

  /// Number of simulation cycles.
  std::size_t number_of_cycles{100};

  /// Robot's initial position in meters (m).
  double initial_position{0.0};

  /// Standard deviation of the robot's initial position.
  /**
   * Represents the uncertainty of the robot's initial position.
   */
  double initial_position_sd{10.0};

  /// Delta time in seconds (s).
  double dt{1.0};

  /// Translation velocity in meters per second (m/s) of the robot in 1D.
  double velocity{1.0};

  /// Translation standard deviation.
  /**
   * Represents the robot's translation noise due to drift and slippage.
   */
  double translation_sd{1.0};

  /// Sensor range of view in meters (m)
  double sensor_range{3.0};

  /// Sensor model sigma
  /**
   * Represents the precision of the modeled sensor.
   */
  double sensor_model_sigma{1.0};

  /// Minimum particle weight.
  /**
   * Used to keep all particles "alive" in the reweight step.
   */
  double min_particle_weight{0.08};

  /// Landmark coordinates in the simulated world.
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};
};
```

### 1.2 Landmark Map

A map of the environment is a list of object in the environment and their locations `M = {M1, M2, ..., Mn}`. A **landmark map** is represented by a **feature-based map**, where each feature in the map contain a property and its Cartesian location. In the tutorial, the landmark map represent the position of the doors, but all doors are equal. This means there are no distinguishing properties between them, such as an ID for each door.

We define a default map in the `Parameters` struct:

```cpp
struct Parameters {
  // Rest of parameters
  // ...

  /// Landmark coordinates in the simulated world.
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};
}
```

:::{caution}
If you are going to change the position of the landmarks, take into account the `std::size_t map_size` parameter as well.
:::

### 1.3 Generate Particles

As boundary condition the algorithm requires an initial belief $bel(x_0)$ at time $t = 0$. If ones knows the value of $x_0$
with a certain certainty, $bel(x_0)$ should be initialize with a point mass distribution that center all probability mass on the correct value of $x_0$. Otherwise, a uniform distribution over the space can be used to initialize $x_0$.

In our case, to demonstrate how the filter is capable of converging to the true state, we will initialize the particles using a normal distribution, setting the mean to the initial position of the true state and with a certain standard deviation value.

To initialize the set of particles we use the following code:

```cpp
std::normal_distribution<double> initial_distribution(parameters.initial_position, parameters.initial_position_sd);
auto particles = beluga::views::sample(initial_distribution) |                  //
                  ranges::views::transform(beluga::make_from_state<Particle>) |  //
                  ranges::views::take_exactly(parameters.number_of_particles) |  //
                  ranges::to<std::vector>;
```

* We define a `std::normal_distribution<double>` to initialize the position of the robot with some uncertainty configured by the parameters `parameters.initial_position` and `parameters.initial_position_sd`.
* `beluga::views::sample(initial_distribution)` will spread the particles around the normal distribution.
* `ranges::views::transform(beluga::make_from_state<Particle>)` convert each sample in a Particle data struct defined by `using Particle = std::tuple<double, beluga::Weight>;`.
* `ranges::views::take_exactly(tutorial_params.number_of_particles)` defines the number of particles to take, using the `parameters.number_of_particles`.
* `ranges::to<std::vector>` convert the range to a `std::vector<double>`.

:::{tip}
read about [Range Adaptor Closure Object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorClosureObject) to understand the `|` operator.
:::

### 1.4 Simulation Loop

The parameters `initial_position`, `velocity`, and `dt` are used to express the movement of the robot along the one-dimensional space and in a single direction, represented by the following equation:

```{math}
:label: robot-kinematic
x(t) = x(t-1) + v * d_t
```

where $x(t)$, or in the code `current_position`, is initialize with the parameter `initial_position`, and at the beginning of each iteration cycle, it also functions as $x(t-1)$ before being updated. Therefore, the following loop will iterate over a configurable `number_of_cycles` or until the robot exceeds the map boundaries.

```cpp
double current_position{parameters.initial_position};
for (std::size_t n = 0; n < parameters.number_of_cycles; ++n) {
  current_position += parameters.velocity * parameters.dt;

  if (current_position > static_cast<double>(parameters.map_size)) {
    break;
  }

  // Rest of the code
  // ...
}
```

### 1.5 Prediction

<!-- TODO(alon): What is it a motion model explanation should be in another doc like 'extending-beluga' -->
For the prediction step, we will be using a **Sample Motion Model Odometry** (_Probabilistic Robotics [^ProbRob], page 111_). Technically, odometry are sensors measurements, not controls (they are only available retrospectively, after the robot has moved), but this poses no problem for filter algorithms, and it will be treated as a control signal.

At time $t$, the robot's position is a random variable $x(t)$. This model considers that within the time interval $(t-1, t]$, the registered movement from odometry is a reliable estimator of the true state movement, despite any potential drift and slippage that the robot may encounter during this interval.

For the **Sample Motion Model Odometry** we will use the following definitions:

* `particle_position` represent a single particle from the range $x_{t-1}$ before applying the control update step.
* `distance = parameters.velocity * parameters.dt` is the equivalent of the odometry translation $u_t$.
* The uncertainty associated with the robot reaching its intended position after a control command is issued is modeled using a Gaussian distribution
  `std::normal_distribution<double> distribution`.
* The spread of the Gaussian distribution is controlled by a standard deviation, which is a constant `parameters.translation_sd` that can be specified when initializing the model.
* The motion model returns a single particle's state corresponding to the new range $x'_t$.

<!-- TODO(alon): The explanation of beluga::actions::propagate should be in another doc like 'extending-beluga' -->

```cpp
auto motion_model = [&](double particle_position, auto& random_engine) {
  const double distance = parameters.velocity * parameters.dt;
  std::normal_distribution<double> distribution(distance, parameters.translation_sd);
  return particle_position + distribution(random_engine);
};


```

To apply this motion model to all the `particles` range we use `beluga::actions::propagate` function.

```cpp
particles |= beluga::actions::propagate(std::execution::seq, motion_model);
```

:::{note}
The range $x'_t$ represent the posterior $bel'(x'_t)$ before incorporating the measurement $z_t$. This posterior or particle set distribution
does not match the updated belief $bel(x_t)$ explain in [key concepts](../concepts/key-concepts.md).
:::

:::{hint}
The effect of the motion model is to increase the space occupied by the particles.
:::

### 1.6 Update

We will divide this stage into two steps:

* [Measurement Update](#161-measurement-update)
* [Resample](#162-resample)

Together, these steps constitute the **Update** phase.

### 1.6.1 Measurement Update

To simluate sensor data, we will create a landmark detector, which is capable of measuring the distance between the robot's `current_position` and nearby landmarks. These measurements are called `detections`, where they consist of an array of distances from the robot's `current_position` to each visible landmark. The range of view of the sensor can be configured by `parameters.sensor_range`.

```cpp
auto detections =
      parameters.landmark_map |                                                                            //
      ranges::views::transform([&](double landmark) { return landmark - current_position; }) |             //
      ranges::views::remove_if([&](double range) { return std::abs(range) > parameters.sensor_range; }) |  //
      ranges::to<std::vector>;
```

* `parameters.landmark_map` is the map generated above.
* `ranges::views::transform` return the relative position of each landmark to the robot's `current_position`.
* `ranges::views::remove_if` will remove all landmarks farther out of the sensor's viewing range.
* `ranges::to<std::vector>` convert the range to a `std::vector<double>`.

<!-- This sensor model, receives as inputs the landmark map, the sensor data and a particle, and outputs the likelihood (or weight) of the sensor data assuming that the particle represent the true state. -->

<!-- TODO(alon): What is it a sensor model explanation should be in another doc like 'extending-beluga' -->
In this tutorial we will use an approximation to the **landmark sensor model with known correspondence** (_Probabilistic Robotics [^ProbRob], page 149_), since the doors (the landmarks in this case) are indistinguishable from one another, and the are treated as identical in the context of measurement.

For the **landmark sensor model with known correspondence** we will use the following definitions:

* The sensor model is a landmark detector, capable of measuring the distance between the particle and the landmarks.
* `particle_position` represent a single particle from the range $x'_t$.
* `particle_detections` is a range of relative distances from the landmarks to the evaluated particle.
* `tutorial_params.min_particle_weight` is the minimum weight a particle can have. Is used to maintain the particle "alive" even though its
  probability is very low.
* The sensor model holds an internal reference to a map of the environment to correctly determine the likelihood of each state.
* The likelihood of each measurement is calculated using a Gaussian function centered at the distance to the nearest landmark.
* The weight of each particle is calculated by aggregating the likelihoods derived from all detections, transformed to the particle's frame of reference.
* The model's parameters include the standard deviation of the Gaussian distribution, which models the detection noise, and a map listing the positions of all landmarks.

To calculate the weight for each particle, we previously need to build the `particle_detections` range:

```cpp
auto sensor_model = [&](double particle_position) {
  auto particle_detections =
        parameters.landmark_map |                                                                  //
        ranges::views::transform([&](double landmark) { return landmark - particle_position; }) |  //
        ranges::to<std::vector>;
  // Rest of the code
  // ...
};
```

Many probabilistic robotic algorithms assume conditional independence between features, it means, the noise in each individual measurement is independent of the noise in other measurements, when conditioned on the current system state and prior world knowledge. Under the conditional independence assumption, we can process one feature at-a-time. That is:

```{math}
:label: importance-factor
p(z_t | x_t, m) = \prod_i p(z_{t}^{i} | x_t, m)
```

`std::transform_reduce()` conduct the equation [](#importance-factor) and calculate the $p(z_t | x_t, m)$:

```cpp
auto sensor_model = [&](double particle_position) {
  auto particle_detections =
        parameters.landmark_map |                                                                  //
        ranges::views::transform([&](double landmark) { return landmark - particle_position; }) |  //
        ranges::to<std::vector>;

  return parameters.min_particle_weight +
          std::transform_reduce(
              detections.begin(), detections.end(), 1.0, std::multiplies<>{}, [&](double detection) {
                auto distances =           //
                    particle_detections |  //
                    ranges::views::transform(
                        [&](double particle_detection) { return std::abs(detection - particle_detection); });
                const auto min_distance = ranges::min(distances);
                return std::exp((-1 * std::pow(min_distance, 2)) / (2 * parameters.sensor_model_sigma));
              });
};
```

Beluga uses the function `beluga::actions::reweight` to transform a range of particle $x'_t$, to a range of **weighted** particles, by applying the sensor model to each particles from the range $x'_t$.

```cpp
particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
```

The `beluga::actions::normalize` is a range adaptor that allows users to normalize the weights of a range (or a range of particles) by dividing each weight by a specified normalization factor. If none is specified, the default normalization factor corresponds to the total sum of weights in the given range.

:::{hint}
Measurement update step doesn't affect the spatial distrition of the particle set $x'_t$.
:::

### 1.6.2 Resample

As explained in the [key concepts](../concepts/key-concepts.md), the update step is completed by performing a **resampling process**, which consist of drawing a new set of particles from the current set, with replacement, using importance weights as unnormalized probabilities. The result of this is the range of particles $x_t$:

```cpp
particles |= beluga::views::sample |                                        //
             ranges::views::take_exactly(parameters.number_of_particles) |  //
             beluga::actions::assign;
```

* `beluga::views::sample` implements a multinomial resampling on `particles`.
* `ranges::views::take_exactly` defines the number of particles to take, using the `number_of_particles` parameter.
* `beluga::actions::assign` effectively converts any view into an action and assigns the result to `particles`.

:::{note}
Adaptive Monte Carlo Localization (AMCL) uses different sampling techniques to dynamically adjust the number of particles to match the complexity of the posterior distribution. For the simplicity of this tutorial, we'll keep a fixed number of particles, as the performance is adequate.
:::

### 1.7 Estiamte

As explained in the [key concepts](../concepts/key-concepts.md), the output of the {abbr}`MCL (Monte Carlo Localiaztion)` algorithm is an estimate of the posterior probability distribution represented by the set of particles $x_t$. To determine the **mean** and **standard deviation** of this distribution, we use `beluga::estimate`:

```cpp
const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
```

* Here, `estimation` is a `std::tuple<double, double>` data structure.
* `beluga::views::states` will obtain a reference to the state of each particle in the input range lazily.
* `beluga::views::weights` will obtain a reference to the weight of each particle in the input range lazily.

:::{attention}
`estimation` is not used in this code. You can use `<iostream>` to print the value of `estimation` on the screen or complete the TODO: [Visualization tutorial]().
:::

## 2 Compile and run the code

TODO: Add the command to compile using gcc.

## Conclusion

You can see that we have put most of our effort into explaining and implementing the components that make up or are used by the particle filter, such as the landmark map, the motion and sensor models, and how to initialize the set of particles. Meanwhile, Beluga provided us with all the necessary resources to implement a custom {abbr}`MCL (Monte Carlo Localiaztion)` algorithm.

## References

[^ProbRob]: S. Thrun, W. Burgard, and D. Fox. _Probabilistic Robotics._ Intelligent Robotics and Autonomous Agents
series. MIT Press, 2005. ISBN 9780262201629. URL <https://books.google.com.ar/books?id=jtSMEAAAQBAJ>
[^RangesV3]: Ranges-v3 user manual <https://ericniebler.github.io/range-v3/>
