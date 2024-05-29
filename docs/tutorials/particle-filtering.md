# Primer on Particle Filtering with Beluga

:::{figure} ../_images/beluga_tutorial.gif
:alt: Short video of a beluga_tutorial's record.
TODO: Modify the GIF using the updated code
:::

## Overview

Modern robotic systems operate in unstructured, unpredictable environments, making robust software crucial to handle diverse situations.
A key challenge is uncertainty, which occurs when robots lack critical task information. Probabilistic robotics addresses this by
representing uncertainty explicitly through probability theory, rather than relying on a single best guess.

The robot localization problem is one of the most important ones in mobile robotics, as it is a fundamental requirement for autonomous navigation. The Monte Carlo Localization (MCL) algorithm is one of the most widely used algorithms for map-based localization and has become a key component in many robotic systems [^BelugaPaper]. **TODO: It is necessary to put here the reference, or just include it at the end?**


The purpose of this tutorial is to show how to implement a custom {abbr}`MCL (Monte Carlo Localiaztion)` algorithm using **Beluga**,
motivated by the example given by _Probabilistic Robotics [^ProbRob], page 200, figure 8.11_. **TODO: Can I used the image of the book?**

The code is written using C++17 with the ranges-v3 [^RangesV3] library for range handling along with its respective algorithms.

## Requirements

Before starting the tutorial, first [install Beluga](../getting-started/installation.md).

:::{important}
I highly recommend reading the [key concepts](../concepts/key-concepts.md) section before diving into this tutorial.
:::

## Tasks

Based on [key concepts](../concepts/key-concepts.md), this tutorial will show an implementation of the particle filter by performing the recursive update in two steps:
* [Prediction](#35-prediction)
* [Update](#36-update)

:::{note}
We are using the [ROS2](https://docs.ros.org/en/humble/index.html) conventions for package management. **TODO: link to some specific tutorial in ROS2 or just provide the link of ROS2 is fine?**
:::

## 1 Create the Workspace

Open a new terminal and create the workspace directory:

```bash
mkdir -p beluga_ws/src
```

## 2 Create the Package

We are going to create a package inside the `beluga_ws/src` path. Therefore, let's start by creating the necessary directories. In the same terminal, run the following command:

```bash
mkdir -p beluga_ws/src/beluga_tutorial/src
```

`beluga_tutorial` should have a `package.xml` and a `CmakeLists.txt` in order to be able to use `colcon` for the compilation process. We will create these files later ([for more details](#4-compile-and-run-the-code)).

## 3 Write the Simulation code

To start, create a `beluga_tutorial/src/main.cpp` file and put the following code inside:

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

Let's examine the code by parts.

### 3.1 Parameters

`Parameters` is a struct that contain all the necessary parameters the code needs to conduct the simulation.

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

### 3.2 Landmark Map

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


### 3.3 Generate Particles

As boundary condition the algorithm requires an initial belief $bel(x_0)$ at time $t = 0$. If ones knows the value of $x_0$
with a certain certainty, $bel(x_0)$ should be initialize with a point mass distribution that center all probability mass on the correct value of $x_0$. Otherwise, a uniform distribution over the space can be used to initialize $x_0$.

In our case, to demonstrate how the filter is capable of converging to the true state, we will initialize the particles using a normal distribution, setting the mean to the initial position of the true state and with a certain standard deviation value.

```cpp
std::normal_distribution<double> initial_distribution(parameters.initial_position, parameters.initial_position_sd);
auto particles = beluga::views::sample(initial_distribution) |                  //
                  ranges::views::transform(beluga::make_from_state<Particle>) |  //
                  ranges::views::take_exactly(parameters.number_of_particles) |  //
                  ranges::to<std::vector>;
```

* We define a `std::normal_distribution<double>` to initialize the position of the robot with some uncertainty.
* `beluga::views::sample(initial_distribution)` will spread the particles around the normal distribution.
* `ranges::views::transform(beluga::make_from_state<Particle>)` convert each sample in a Particle data struct defined by `using Particle = std::tuple<double, beluga::Weight>;`.
* `ranges::views::take_exactly(tutorial_params.number_of_particles)` defines the number of particles to take, using the `number_of_particles` parameter.
* `ranges::to<std::vector>` convert the range to a `std::vector<double>`.

:::{tip}
read about [Range Adaptor Closure Object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorClosureObject) to understand the `|` operator.
:::

### 3.4 Simulation Loop

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

### 3.5 Prediction

The motion model comprise the state transition probability $p(x_t | x_{t-1}, u_t)$ which plays an essential role in the prediction step (or control update step) of the Bayes filter. Probabilistic robotics generalizes kinematic equations to the fact that the outcome of a control is uncertain, due to control noise or unmodeled exogenous effects. Deterministic robot actuator models are converted to probabilistic models by adding noise variables that characterize the types of uncertainty that exist in robotic actuation. Here $x_t$ and $x_{t−1}$ are both robot positions (in our case a one-dimensional position), and $u_t$ is a motion command. This model describes the posterior distribution over kinematics states that a robots assumes when executing the motion command $u_t$ when its position is $x_{t−1}$.

In this tutorial, we will be using an **Sample Motion Model Odometry** (*Probabilistic Robotics [^ProbRob], page 111*). Technically, odometry are sensors measurements, not controls (they are only available retrospectively, after the robot has moved), but this poses no problem for filter algorithms, and it will be treated as a control signal.

At time $t$, the robot's position is a random variable $x(t)$. This model considers that within the time interval $(t-1, t]$, the registered movement from odometry is a reliable estimator of the true state movement, despite any potential drift and slippage that the robot may encounter during this interval.

The **Sample Motion Model Odometry** receives as inputs the set of particles $x_{t-1}$ and the control signal $u_t$, and outputs a new set of particles $x'_t$. Beluga implements that using  the `beluga::actions::propagate` function applied to a range of particles. It accepts an execution policy and a function that applies the motion model (or transformation) to each particle in the range ([Beluga API](../packages/beluga/docs/index.md)).
<!-- TODO: doesn't work([beluga::actions API](../packages/beluga/docs/_doxygen/generated/reference/html/propagate_8hpp.html)). -->

```cpp
auto motion_model = [&](double particle_position, auto& random_engine) {
  const double distance = parameters.velocity * parameters.dt;
  std::normal_distribution<double> distribution(distance, parameters.translation_sd);
  return particle_position + distribution(random_engine);
};

particles |= beluga::actions::propagate(std::execution::seq, motion_model);
```

* The argument `particle_position` represent a single particle from the range $x_{t-1}$ before applying the control update step.
* Here, the equivalent of the odometry translation is the calculation of the `distance`.
* A, `std::normal_distribution<double> distribution` is created with a mean equal to `distance` and the standard deviation `parameters.translation_sd`. This normal distribution represent the noise obtained from an odometry measurement due to drift and slippage during the translation of the robot.
* The motion model returns a single particle's state corresponding to the new range $x'_t$.

:::{note}
The range $x'_t$ represent the posterior $bel'(x'_t)$ before incorporating the measurement $z_t$. This posterior or particle set distribution
does not match the updated belief $bel(x_t)$ explain in [key concepts](../concepts/key-concepts.md).
:::

:::{hint}
The effect of the motion model is to increase the space occupied by the particles.
:::

### 3.6 Update

We will divide this stage into two steps:
*  [Measurement Update](#361-measurement-update)
*  [Resample](#362-resample)

Together, these steps constitute the **Update** phase.

### 3.6.1 Measurement Update

Measurement models describe the formation process by which sensor measurements are generated in the physical world. Probabilistic robotics explicitly models the noise in sensor measurements. Such models account for the inherent uncertainty in the robot’s sensors. Formally, the measurement model is defined as a conditional probability distribution $p(z_t | x_t , m)$, where $x_t$ is the robot position, $z_t$ is the measurement at time $t$, and $m$ is the map of the environment.

The most common model for processing landmarks assumes that the sensor can measure the range and the bearing of the landmark relative to the robot’s
local coordinate frame. In this tutorial we will use an approximation to the **landmark sensor model with known correspondence** (*Probabilistic Robotics [^ProbRob], page 149*),
since the doors do not have a signature that differentiates them from each other, but we can calculate the relative position of each door to the robot.

This sensor model, receives as inputs the landmark map, the sensor data and a particle, and outputs the likelihood (or weight) of the sensor data assuming that the particle represent the true state.

In our case, we need to generate simulated sensor data called `detections`. This data corresponds to the relative positions of landmarks around the robot's `current_position` within a configurable range specified by `parameters.sensor_range`.

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

To calculate the weight for each particle, we previously need to build the `particle_detections` range. This range represent the relative distances of each landmark to the proposed particle.

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

The number of features identified at each time step is variable. However, many probabilistic robotic algorithms assume conditional independence between features, it means, the noise in each individual measurement is independent of the noise in other measurements, when conditioned on the current system state and prior world knowledge. Under the conditional independence assumption, we can process one feature at-a-time. That is:

```{math}
:label: importance-factor
p(z_t | x_t, m) = \prod_i p(z_{t}^{i} | x_t, m)
```

To calculate the probability $p(z_{t}^{i} | x_t, m)$ we use the equation of a simplified version of the normal distribution's PDF.

**(TODO: review this paragraph)**
In this context, each landmark lacks distinguishing features that uniquely differentiate it from others. Therefore, we employ an approximation method where the `min_distance` between the landmarks' position measured from the sensor and the landmark's position relative to the particle, is considered. This approximation assumes that the smallest distance observed in the comparison provides the most accurate correspondence between the observed landmarks.

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

* `tutorial_params.min_particle_weight` is the minimum weight a particle can have. Is used to maintain the particle "alive" even though
  its probability is very low.
* `std::transform_reduce()` conduct the equation [](#importance-factor) and calculate the $p(z_t | x_t, m)$.
* The sensor model returns the likelihood (or weight) of the sensor data assuming that the particle represent the true state.

Beluga uses the function `beluga::actions::reweight` to transform a range of particle $x'_t$, to a range of **weighted** particles. It accepts an execution policy and a function that applies the sensor model (or transformation) to each particle in the range ([Beluga API](../packages/beluga/docs/index.md)).

The `beluga::actions::normalize` is a range adaptor that allows users to normalize the weights of a range (or a range of particles) by dividing each weight by a specified normalization factor. If none is specified, the default normalization factor corresponds to the total sum of weights in the given range ([Beluga API](../packages/beluga/docs/index.md)).

```cpp
particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
```

:::{hint}
Measurement update step doesn't affect the spatial distrition of the particle set.
:::

### 3.6.2 Resample

The updated belief is prefigured by the spatial distribution of the particles $x'_t$ in the set and their importance weights. A few of the particles will have migrated to state regions with low probability, however, and their importance weights will therefore be low.
To correct this, the update step is completed by performing a **resampling process**, which consist of drawing a new set of particles $x_t$ from the current set, with replacement, using importance weights as unnormalized probabilities. This process causes particles with low weights to be discarded and particles with high weights to be propagated multiple times into the new particle set.

Adaptive Monte Carlo Localization (AMCL) uses different sampling techniques to dynamically adjust the number of particles to match the complexity of the posterior distribution. For the simplicity of this tutorial, we'll keep a fixed number of particles, as the performance is adequate.

```cpp
particles |= beluga::views::sample |                                        //
             ranges::views::take_exactly(parameters.number_of_particles) |  //
             beluga::actions::assign;
```

* `beluga::views::sample` implements a multinomial resampling on `particles` ([Beluga API](../packages/beluga/docs/index.md)).
* `ranges::views::take_exactly` defines the number of particles to take, using the `number_of_particles` parameter.
* `beluga::actions::assign` effectively converts any view into an action and assigns the result to `particles` ([Beluga API](../packages/beluga/docs/index.md)).

### 3.7 Estiamte

As explained in the [key concepts](../concepts/key-concepts.md), the output of the {abbr}`MCL (Monte Carlo Localiaztion)` algorithm is an estimate of the posterior probability distribution represented by the set of particles $x_t$. To determine the **mean** and **standard deviation** of this distribution, we use `beluga::estimate` ([Beluga API](../packages/beluga/docs/index.md)):

```cpp
const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
```

Here, `estimation` is a `std::tuple<double, double>` data structure.

:::{attention}
`estimation` is not used in this code. You can use `<iostream>` to print the value of `estimation` on the screen or complete the TODO: [Visualization tutorial]().
:::

## 4 Compile and run the code

### 4.1 package.xml

Create a `beluga_tutorial/package.xml` file and put the following inside:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>beluga_tutorial</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@mail.com">developer</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>cmake</buildtool_depend>

  <depend>beluga</depend>

  <export>
    <build_type>cmake</build_type>
  </export>
</package>
```

This declares the package needs `beluga` when its code is built and executed.

### 4.2 CMakeLists.txt

The final `beluga_tutorial/CMakeLists.txt` looks like:

```cmake
cmake_minimum_required(VERSION 3.16)

project(beluga_tutorial)

find_package(beluga REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} beluga::beluga)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})
```

* `project()` define the name of the project. `${PROJECT_NAME}` macro will use this name alongside the file.
* `find_package()` is used to import external dependencies.
* `add_executable()` create the executable called `beluga_tutorial`.
* `target_link_libraries()` link `beluga` in the compilation process.
* `install(TARGETS...)` will install the executable under the path `install/beluga_tutorial/lib/beluga_tutorial/`.

### 4.2 Build and run

Open a new terminal and build your new package:

```bash
source /opt/ros/humble/setup.bash
cd beluga_ws/
colcon build
```

In the same terminal run the code:

```bash
./install/beluga_tutorial/lib/beluga_tutorial/beluga_tutorial
```

## Conclusion

## References

[^ProbRob]: S. Thrun, W. Burgard, and D. Fox. _Probabilistic Robotics._ Intelligent Robotics and Autonomous Agents
series. MIT Press, 2005. ISBN 9780262201629. URL <https://books.google.com.ar/books?id=jtSMEAAAQBAJ>
[^BelugaPaper]: Beluga paper **TODO: add the link for the paper**
[^RangesV3]: Ranges-v3 user manual <https://ericniebler.github.io/range-v3/>
