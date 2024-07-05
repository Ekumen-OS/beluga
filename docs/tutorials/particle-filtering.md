# Primer on Particle Filtering with Beluga

:::{figure} ../_images/beluga_tutorial.gif
:alt: Short video of a beluga_tutorial's record.
:::

## Overview

In this tutorial, we will show how to implement a custom {abbr}`MCL (Monte Carlo Localization)` algorithm using Beluga.

This tutorial takes after the one-dimensional particle filter example given in {cite}`thrun2005probabilistic{page 200, figure 8.11}`. There are walls and there are doors in our one-dimensional world, and our one-dimensional robot is free to move along this world. The last two plots in the animation shown above depict this world, where the blue bars are the walls, the red bars are the doors, and the green bar is the robot. While moving, our robot scans for doors and measures its distance to them. There is uncertainty in those measurements, however. There is also uncertainty in our robot's proprioception (i.e. its sense of self-movement). We wish to estimate the robot's position over time while acknowledging that uncertainty. So we will use a particle filter. The first two plots in the animation show how that position estimate or distribution evolves as the robot moves.

To implement this particle filter in Beluga, we will need a map, a motion model, a sensor model, and a prior or initial position estimate. We can then write an {abbr}`MCL (Monte Carlo Localization)` algorithm to estimate the robot position over time.

## Requirements

Make sure to [install Beluga](../getting-started/installation.md) first.

:::{important}
It is highly recommended to first read through the [key concepts](../concepts/key-concepts.md) section before diving into this tutorial.
:::

## Tasks

This tutorial will go over the implementation of a (bootstrap) particle filter to solve our one-dimensional state estimation problem.

### 1 Develop

To begin with, put the following code in a `main.cpp` file:

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
  double initial_position_sigma{10.0};
  double dt{1.0};
  double velocity{1.0};
  double motion_model_sigma{1.0};
  double sensor_range{3.0};
  double sensor_model_sigma{1.0};
  double min_particle_weight{0.08};
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};
};

using Particle = std::tuple<double, beluga::Weight>;

int main() {
  const Parameters parameters;

  std::normal_distribution<double> initial_position_distribution(
      parameters.initial_position, parameters.initial_position_sigma);
  auto particles = beluga::views::sample(initial_position_distribution) |         //
                   ranges::views::transform(beluga::make_from_state<Particle>) |  //
                   ranges::views::take_exactly(parameters.number_of_particles) |  //
                   ranges::to<std::vector>;

  double current_position{parameters.initial_position};
  for (std::size_t n = 0; n < parameters.number_of_cycles; ++n) {
    current_position += parameters.velocity * parameters.dt;

    if (current_position > static_cast<double>(parameters.map_size)) {
      break;
    }

    auto motion_model = [&](double position, auto& random_engine) {
      std::normal_distribution<double> motion_distribution(
          parameters.velocity * parameters.dt,
          parameters.motion_model_sigma * parameters.dt);
      return position + motion_distribution(random_engine);
    };

    const auto range_measurements =
        parameters.landmark_map |                                                                                   //
        ranges::views::transform([&](double landmark_position) { return landmark_position - current_position; }) |  //
        ranges::views::remove_if([&](double range) { return std::abs(range) > parameters.sensor_range; }) |         //
        ranges::to<std::vector>;

    const auto sensor_model = [&](double position) {
      auto range_map =
          parameters.landmark_map |                                                                           //
          ranges::views::transform([&](double landmark_position) { return landmark_position - position; }) |  //
          ranges::to<std::vector>;

      return parameters.min_particle_weight +
             std::transform_reduce(
                 range_measurements.begin(), range_measurements.end(), 1.0, std::multiplies<>{},
                 [&](double range_measurement) {
                   const auto distances = range_map | ranges::views::transform([&](double range) {
                                            return std::abs(range - range_measurement);
                                          });
                   const auto min_distance = ranges::min(distances);
                   return std::exp((-1 * std::pow(min_distance, 2)) / (2 * parameters.sensor_model_sigma));
                 });
    };

    record.current = particles;

    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    record.propagate = particles;

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
    record.reweight = particles;

    particles |= beluga::views::sample |                                        //
                 ranges::views::take_exactly(parameters.number_of_particles) |  //
                 beluga::actions::assign;
    record.resample = particles;

    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
  }
  return 0;
}
```

:::{note}
The code is written using C++17 with the [`ranges-v3`](https://ericniebler.github.io/range-v3) library for range handling along with its respective algorithms.
:::

Now, let's break it down.

### 1.1 Parameters

This data structure contains all the parameters we need to simulate our one-dimensional world, initialized to suitable defaults.

```cpp
struct Parameters {
  /// Size of the 1D map in (m)
  /**
   * 1D map space is continuous, but its size is defined
   * as an integer to simplify data visualization.
   */
  std::size_t map_size{100};

  /// Fixed number of particles used by the algorithm.
  std::size_t number_of_particles{200};

  /// Number of simulation cycles.
  std::size_t number_of_cycles{100};

  /// Initial robot position, in meters (m).
  double initial_position{0.0};

  /// Standard deviation of the robot's estimate of its initial position.
  double initial_position_sigma{1.0};

  /// Delta time in seconds (s).
  double dt{1.0};

  /// Constant robot velocity, in meters per second (m/s).
  double velocity{1.0};

  /// Motion model standard deviation, in meters per second (m/s).
  /**
   * Models the uncertainty in the robot's estimate of its own velocity.
   */
  double motion_model_sigma{1.0};

  /// Sensor field of view, in meters (m).
  double sensor_range{2.0};

  /// Sensor model standard deviation, in meters (m).
  /**
   * Models the uncertainty in range measurements to landmarks.
   */
  double sensor_model_sigma{1.0};

  /// Minimum particle weight.
  /**
   * Used to keep all particles "alive" in the reweight step.
   */
  double min_particle_weight{0.08};

  /// Landmark positions in the simulated world.
  std::vector<double> landmark_map;

  /// Dataset path.
  /**
   * The dataset file is used to save the data produced by the simulation for posterior analisys.
   */
  std::filesystem::path record_path{"./record.yaml"};
};
```

### 1.2 Landmark map

Note the landmark map among parameters. In this tutorial, the landmark map stores the position of all known doors. It is a _feature-based map_, but doors have no distinguishing properties.

```cpp
struct Parameters {
  // Rest of parameters
  // ...

  /// Landmark coordinates in the simulated world.
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};
}
```

:::{caution}
If you are going to change the position of the landmarks, make sure to update the `map_size` as well.
:::

### 1.3 Initial belief

Filtering algorithms require an initial belief $bel(x_0)$ at time $t = 0$ -- a prior distribution. If one knows the value of $x_0$ with some certainty, $bel(x_0)$ should be initialized to a distribution where all probability mass is centered at and close to the $x_0$ value. Otherwise, a uniform distribution over all space can be used to initialize $x_0$.

In our case, to demonstrate how the filter is capable of converging to the true state, we will draw particles from a normal distribution centered at the true initial position:

```cpp
std::normal_distribution<double> initial_position_distribution(
    parameters.initial_position, parameters.initial_position_sigma);
auto particles = beluga::views::sample(initial_position_distribution) |         //
                 ranges::views::transform(beluga::make_from_state<Particle>) |  //
                 ranges::views::take_exactly(parameters.number_of_particles) |  //
                 ranges::to<std::vector>;
```

Note that:

* we configure a `std::normal_distribution<double>` with `parameters.initial_position` and `parameters.initial_position_sigma` as mean and standard deviation, respectively.
* `beluga::views::sample(initial_position_distribution)` samples robot states (ie. a scalar position) from the distribution.
* `ranges::views::transform(beluga::make_from_state<Particle>)` transforms each state into a weighted particle, as defined by `using Particle = std::tuple<double, beluga::Weight>;`.
* `ranges::views::take_exactly(tutorial_params.number_of_particles)` takes as many particles as `parameters.number_of_particles` specifies.
* `ranges::to<std::vector>` turns the range into an `std::vector`.

:::{tip}
Read about [range adaptor closures](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorClosureObject) to understand the `|` operator.
:::

### 1.4 System modelling

Parameters `initial_position`, `velocity`, and `dt` configure a constant velocity model for our one-dimensional robot kinematics:

```{math}
:label: robot-kinematic
x_k = x_{k-1} + v \mathrm{d}t
```

Note $t = k \mathrm{d}t$ i.e. time is discretized.

This equation will be stepped a configurable  `number_of_cycles` or until the robot reaches its map boundary:

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

We will also simulate sensor data, measuring the distance between the robot's `current_position` and nearby landmarks. The field of view of the sensor can be configured by `parameters.sensor_range`.

```cpp
const auto range_measurements =
    parameters.landmark_map |                                                                                   //
    ranges::views::transform([&](double landmark_position) { return landmark_position - current_position; }) |  //
    ranges::views::remove_if([&](double range) { return std::abs(range) > parameters.sensor_range; }) |         //
    ranges::to<std::vector>;
```

Note that:

* `parameters.landmark_map` is the map configured above.
* `ranges::views::transform` computes the distance to each landmark.
* `ranges::views::remove_if` removes all landmarks beyond the sensor field of view.
* `ranges::to<std::vector>` turns the range into an `std::vector`.

### 1.5 Filter prediction step

For the prediction step, we will take a noisy velocity measurement as our control action and integrate it. While technically a sensor measurement, this poses no problem for the particle filter. We could have equally used a fixed velocity control setpoint (i.e. a point mass conditional distribution for a motion model) and simulated actuation noise instead.

```cpp
const auto motion_model = [&](double position, auto& random_engine) {
  std::normal_distribution<double> motion_distribution(
      parameters.velocity * parameters.dt, parameters.motion_model_sigma * parameters.dt);
  return position + motion_distribution(random_engine);
};
```

Note that:
* `position` is the robot state $x_{t-1}$ that conditions $p(x_t|x_{t-1}, u_t)$.
* `distance = parameters.velocity * parameters.dt` integrates the velocity setpoint.
* `std::normal_distribution<double> distribution` models velocity uncertainty as additive Gaussian noise.

The motion model then returns a new particle state, effectively sampling the $p(x_t|x_{t-1}, u_t)$ distribution.

To apply this motion model to the whole `particles` range, we use the `beluga::actions::propagate` action:

```cpp
particles |= beluga::actions::propagate(std::execution::seq, motion_model);
```

This yields the proposal distribution $q(x_t|x_{t-1})$ for our particle filter.

### 1.6 Filter update step

For the measurement update step, we will calculate the measurements likelihood by aggregating that of each landmark:

```{math}
:label: importance-factor
p(z_t | x_t, m) = \prod_i p(z_{t}^{i} | x_t, m)
```

:::{important}
This sensor model rests on a conditional independence assumption for landmark detections, given robot state and landmark map. This simplifying assumption is adequate in this case because there is no structure to the landmark map nor are we trying (or need) to exploit it.
:::

To do this, we need to construct the likely set of range measurements given robot position and landmark map:

```cpp
const auto sensor_model = [&](double position) {
  auto range_map =
      parameters.landmark_map |                                                                           //
      ranges::views::transform([&](double landmark_position) { return landmark_position - position; }) |  //
      ranges::to<std::vector>;
  // Rest of the code
  // ...
};
```

`std::transform_reduce()` can then be used to compute equation [](#importance-factor):

```cpp
const auto sensor_model = [&](double position) {
  auto range_map =
      parameters.landmark_map |                                                                           //
      ranges::views::transform([&](double landmark_position) { return landmark_position - position; }) |  //
      ranges::to<std::vector>;

  return parameters.min_particle_weight +
         std::transform_reduce(
             range_measurements.begin(), range_measurements.end(), 1.0, std::multiplies<>{},
             [&](double range_measurement) {
               const auto distances = range_map | ranges::views::transform([&](double range) {
                                        return std::abs(range - range_measurement);
                                      });
               const auto min_distance = ranges::min(distances);
               return std::exp((-1 * std::pow(min_distance, 2)) / (2 * parameters.sensor_model_sigma));
             });
};
```

Note that:

* `position` is the robot state $x_t$ that conditions the likelihood $p(z_t|x_t)$.
* `range_map` is a range of relative distances from the landmarks to the evaluated particle.
* `tutorial_params.min_particle_weight` prevents total weight depletion, keeping particles "alive" even when its probability is low.
* The likelihood of each landmark detection is computed by applying a Gaussian kernel to the distance to the nearest landmark.

To apply this sensor model to the whole `particles` range, we use the `beluga::actions::reweight` action. We also normalize all weights using the `beluga::actions::normalize` action:

```cpp
particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
```

This yields the posterior state distribution or updated belief.

:::{hint}
Note that the measurement update step does not change the spatial distribution of the particle set.
:::

### 1.6.2 Filter resampling step

To avoid weight degeneracy, the filter must *resample*, drawing a new set of particles from the current set, with replacement, using particle weights as the unnormalized probabilities of a multinomial distribution:

```cpp
particles |= beluga::views::sample |                                        //
             ranges::views::take_exactly(parameters.number_of_particles) |  //
             beluga::actions::assign;
```

* `beluga::views::sample` samples particles from `particles` according to their weights.
* `ranges::views::take_exactly` takes as many particles as `parameters.number_of_particles` specifies.
* `beluga::actions::assign` effectively converts any view into an action and assigns the result to `particles`.

:::{note}
Other particle filtering techniques use different resampling schemes. Adaptive Monte Carlo Localization (AMCL) dynamically adjusts the number of particles to match the complexity of the posterior distribution. For simplicity's sake, we use a fixed number of particles. This yields adequate performance.
:::

### 1.7 Estimate

The output of our {abbr}`MCL (Monte Carlo Localization)` algorithm is an estimate of the posterior state distribution or belief, represented by a set of weighted particles. To determine the **mean** and **variance** parameters of this distribution, we can use `beluga::estimate`:

```cpp
const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
```

Note that:

* `beluga::views::states` obtains a view to the state of each particle in the given range.
* `beluga::views::weights` obtains a view to the weight of each particle in the given range.

## 2 Compile and run

We are almost there! Add a `CMakeLists.txt` file next to your `main.cpp` file, and copy the following:

```cmake
cmake_minimum_required(VERSION 3.16)

project(beluga_tutorial)

find_package(beluga REQUIRED)
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} beluga::beluga)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
```

Now you can configure and build the tutorial:

```bash
mkdir build
cd build
cmake ..
make
```

:::{important}
If the `cmake ..` command fails, you may need to source the path where Beluga is installed.
:::

A `beluga_tutorial` executable will show up. Run it:

```bash
./beluga_tutorial
```
