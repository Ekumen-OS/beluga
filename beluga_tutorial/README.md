# Simple MCL Implementation Using Beluga

- [Simple MCL Implementation Using Beluga](#simple-mcl-implementation-using-beluga)
  - [Overview](#overview)
  - [Monte Carlo Localization (MCL) Background](#monte-carlo-localization-mcl-background)
  - [Requirements](#requirements)
  - [Tasks](#tasks)
  - [1 Creating the Beluga Tutorial package](#1-creating-the-beluga-tutorial-package)
  - [2 Creating the simluation](#2-creating-the-simluation)
    - [2.1 Writing the code](#21-writing-the-code)
    - [2.1.1 Tutorial Parameters](#211-tutorial-parameters)
    - [2.1.2 Landmark Map](#212-landmark-map)
    - [2.1.3 Generate Particles](#213-generate-particles)
    - [2.1.3 Simulation Loop](#213-simulation-loop)
    - [2.1.4 Control Update (Montion Model)](#214-control-update-montion-model)
    - [2.2 Compiling the code](#22-compiling-the-code)

## Overview
![Simulation Video](./resources/simulation_video.mp4)

This tutorial demonstrates how to implement an MCL particle filter for a robot moving in a one-dimensional space and in a single direction, using Beluga. The code is written using C++17 with the ranges-v3 library for range handling along with its respective algorithms.

## Monte Carlo Localization (MCL) Background
Monte Carlo Localization is part of the broader family of Bayesian state estimation methods. They are all based on the Bayes filter, which is a recursive algorithm that estimates the posterior probability distribution of the state of a system given a sequence of sensor measurements and control inputs.
The estimation at any given time is represented using a probability distribution function, called **belief**, definded as:
$$bel(x_t) = p(x_t|z_{1:t},u_{1:t})$$
where $x_t$ is the state of the system at time $t$, and $z_{1:t}$ and $u_{1:t}$ are the sequence of sensor measurements and the sequence of control inputs up to time $t$ respectively.\
A belief refelcts the robot's internal knowledge about the state of the environment. We therfore distinguish the **true state** from the robot's interal belief.

It can be shown that the posterior belief can be recursively computed using the prior belief, the current sensor readings, and the current control inputs using the following update rule:
$$bel(x_t)=ηp(z_t|x_t)\int p(x_t|x_{t-1}, u_t)bel(x_{t-1}) \, dx_{t-1}$$
where η is a normalization factor.

A particle filter approximates this update rule by representing the belief as a set of discrete **samples**, each representing a possible state of the system. Collectively, these samples (or **particles**) represent the probability distribution over the state of the system at time $t$, condition on the prior state distribution, sensor readings, and control inputs.

This tutorial proposes is to show a common implementation of the particle filter by performing the recursive update in three steps: control update, measurement update and resample.

## Requirements
This tutorial requires having the following dependencies installed:\
(Add list of dependencies)

As another option, you can use the following Docker containers as a development environment:\
(Add the options for Docker files)

## Tasks

## 1 Creating the Beluga Tutorial package


## 2 Creating the simluation
To build the simulation step by step, we'll first explain the main functionality of the code, which is assembling the particle filter, and how to compile it. In the second step, we'll add the necessary functionalities for visualizing the simulation.

### 2.1 Writing the code
Create a `beluga_tutorial/src/main.cpp` file and put the following code in:

>Note: The following code only implement the filter and calculate the estimated state. For the complete code with the 
visualization step go to (link).

```cpp
// Copyright 2022-2024 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <range/v3/action.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <beluga/beluga.hpp>

struct TutorialParams {
  int map_size{100};
  int number_of_doors{33};
  int number_of_particles{200};
  int number_of_cycles{100};
  double initial_pose{0.0};
  double initial_pose_sd{1.0};
  double dt{1.0};
  double velocity{1.0};
  double translation_sigma{1.0};
  double sensor_range{2.0};
  double sensor_model_sigam{1.0};
  double min_particle_weight{0.08};
};

using Particle = std::tuple<double, beluga::Weight>;
using LandmarkMapVector = std::vector<double>;
using SensorData = std::vector<double>;
using GroundTruth = double;
using Estimate = std::pair<double, double>;

double generateRandom(double mean, double sd) {
  static auto generator = std::mt19937{std::random_device()()};
  std::normal_distribution<double> distribution(mean, sd);

  return distribution(generator);
}

int main() {
  // Initialize tutorial's parameters
  TutorialParams tutorial_params;

  // Generate a random map
  std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
  LandmarkMapVector landmark_map = beluga::views::sample(landmark_distribution) |
                ranges::views::take_exactly(tutorial_params.number_of_doors) | ranges::to<LandmarkMapVector>;
  landmark_map |= ranges::actions::sort | ranges::actions::unique;

  // Generate particles
  std::normal_distribution<double> initial_distribution(tutorial_params.initial_pose, tutorial_params.initial_pose_sd);
  auto particles = beluga::views::sample(initial_distribution) |
                   ranges::views::transform(beluga::make_from_state<Particle>) |
                   ranges::views::take_exactly(tutorial_params.number_of_particles) | ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  double current_pose{tutorial_params.initial_pose};
  for (auto n = 0; n < tutorial_params.number_of_cycles; n++) {
    // Keep track of the the simulated ground truth
    current_pose += (tutorial_params.velocity * tutorial_params.dt);

    // Check if the simulation is out of bounds
    if (current_pose > tutorial_params.map_size) {
      break;
    }

    // Motion model
    auto motion_model = [&](double state) {
      double distance = (tutorial_params.velocity * tutorial_params.dt);
      double translation_param = generateRandom(0.0, tutorial_params.translation_sigma);
      return state + distance - translation_param;
    };

    // Generate simulated sensor data
    auto sensor_data = landmark_map | ranges::views::remove_if([&](const double lm) {
                         return std::abs(lm - current_pose) > tutorial_params.sensor_range;
                       }) |
                       ranges::views::transform([&](const double lm) { return lm - current_pose; }) |
                       ranges::to<SensorData>;
    // Sensor model
    auto sensor_model = [&](const double& state) {
      auto particle_sensor_data =
          landmark_map | ranges::views::transform([&](const double lm) { return lm - state; }) | ranges::to<SensorData>;

      LandmarkMapVector landmark_probability;
      for (const auto& sensor_datum : sensor_data) {
        auto distances = particle_sensor_data | ranges::views::transform([&](auto& particle_sensor_datum) {
                           return std::abs(sensor_datum - particle_sensor_datum);
                         });

        auto min_distance = ranges::min(distances);
        landmark_probability.push_back(exp((-1 * pow(min_distance, 2)) / (2 * tutorial_params.sensor_model_sigam)));
      }

      return ranges::accumulate(landmark_probability, 1.0, std::multiplies<>{}) + tutorial_params.min_particle_weight;
    };

    // For the propose of the tutorial, we split the process in the following stages:
    // Propagate stage
    particles |= beluga::actions::propagate(std::execution::seq, motion_model);

    // Reweight stage
    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;

    // Resample
    particles |= beluga::views::sample | ranges::views::take_exactly(tutorial_params.number_of_particles) |
                 beluga::actions::assign;

    // Calculate mean and standard deviation
    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
  }

  return 0;
}
```
The first few lines include all of the headers we need to compile.

The following `using` declarations help us to maintain the code cleaner
```cpp
using Particle = std::tuple<double, beluga::Weight>;
using LandmarkMapVector = std::vector<double>;
using SensorData = std::vector<double>;
using GroundTruth = double;
using Estimate = std::pair<double, double>;
```

A `generateRandom` function is used to simulate some normal distribution noise
```cpp
double generateRandom(double mean, double sd) {
  static auto generator = std::mt19937{std::random_device()()};
  std::normal_distribution<double> distribution(mean, sd);

  return distribution(generator);
}
```

Lets look what is happening in the `main` function.

### 2.1.1 Tutorial Parameters
A `TutorialParams tutorial_params` object is initialize with default values. `TutorialParams` is a struct that contain all the necessary parameter the code needs to conduct the simulation.
```cpp
struct TutorialParams {
int map_size{100};
int number_of_doors{33};
int number_of_particles{200};
int number_of_cycles{100};
double initial_pose{0.0};
double initial_pose_sd{1.0};
double dt{1.0};
double velocity{1.0};
double translation_sigma{1.0};
double sensor_range{2.0};
double sensor_model_sigam{1.0};
double min_particle_weight{0.08};
};
```
### 2.1.2 Landmark Map
A map of the environment is a list of object in the environment and their locations `M = {M1, M2, ..., Mn}`.\
A *landmark map* represent a *feature-based map*, where each feature in the map contain a property and its Cartesian location. In the tutorial, the landmark map represent the position of the doors, but all doors are equal. This means there are no distinguishing properties between them, such as an ID for each door. To generate a random map, we use the following code:
```cpp
// Generate a random map
std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
LandmarkMapVector landmark_map = beluga::views::sample(landmark_distribution) |
              ranges::views::take_exactly(tutorial_params.number_of_doors) | ranges::to<LandmarkMapVector>;
landmark_map |= ranges::actions::sort | ranges::actions::unique;
```
* We use a `uniform_int_distribution` to randomly assign landmark to any position of the map defined by `[0, map_size]`.
* `beluga::views::sample(landmark_distribution)` is in charge of, given a type of distribution, generate samples.
* `ranges::views::take_exactly(tutorial_params.number_of_doors)` specify to **landmark_vector** how many samples (or doors in this case) to take.
* `ranges::to<LandmarkMapVector>` convert the range to a LandmarkMapVector type defined previously.
* `landmark_map |= ranges::actions::sort | ranges::actions::unique` will remove repeated landmarks in the same position, and arrange them from lowest to highest to facilitate debugging.
>Note: read about [Range Adaptor Closure Object](https://en.cppreference.com/w/cpp/named_req/RangeAdaptorClosureObject) to understand the `|` operator.

### 2.1.3 Generate Particles
As boundry condition the algorithm requires an initial belief $bel(x_0)$ at time $t = 0$. If ones knows the value of $x_0$
with a certain certainty, $bel(x_0)$ should be initialize with a point mass distribution that center all probability mass on the correct value of $x_0$. Otherwise, a uniform distribution over the space can be used to initialize $x_0$.

In our case, to demonstrate how the filter is capable of converging to the true state, we will initialize the particles using a normal distribution, setting the mean to the initial position of the true state and with a certain standard deviation value.
```cpp
// Generate particles
std::normal_distribution<double> initial_distribution(tutorial_params.initial_pose, tutorial_params.initial_pose_sd);
auto particles = beluga::views::sample(initial_distribution) |
                ranges::views::transform(beluga::make_from_state<Particle>) |
                ranges::views::take_exactly(tutorial_params.number_of_particles) | ranges::to<beluga::TupleVector>;
``` 
* We define a `std::normal_distribution<double>` to initialize the position of the robot with some uncertainty.
* `beluga::views::sample(initial_distribution)` will spread the particles around the normal distribution.
* `ranges::views::transform(beluga::make_from_state<Particle>)` convert each sample in a Particle data struct.
* `ranges::views::take_exactly(tutorial_params.number_of_particles)` define the number of particles to take, using the **number_of_particles** parameter.
* `ranges::to<beluga::TupleVector>` convert the range to a `beluga::TupleVector`, which is a shorthand for a tuple of vectors with the default allocator, see beluga code for more detail (link).

### 2.1.3 Simulation Loop
The parameters **initial_pose**, **velocity**, and **dt** are used to express the movement of the robot along the one-dimensional space and in a single direction, represented by the following equation: 
$$x(t) = x(t-1) + v * d_t$$
wehre $x(t)$, or in the code **current_pose**, is initialize with the parameter **initial_pose**, and at the beginning of each iteration cycle, it also functions as $x(t-1)$ before being updated. Therefore, the following loop will iterate over a configurable **number_of_cycles** or until the robot exceeds the map boundaries.
```cpp
  double current_pose{tutorial_params.initial_pose};
  for (auto n = 0; n < tutorial_params.number_of_cycles; n++) {
    // Keep track of the the simulated ground truth
    current_pose += (tutorial_params.velocity * tutorial_params.dt);

    // Check if the simulation is out of bounds
    if (current_pose > tutorial_params.map_size) {
      break;
    }

    // Rest of the code
    // ...
  }
```
### 2.1.4 Control Update (Montion Model)
The motion model comprise the state transitio probability $p(x_t | x_{t-1}, u_t)$ which plays an essential role in the prediction step of the Bayes filter. Probabilistic robotics generalizes kinematic equations to the fact that the outcome of a control is uncertain, due to control noise or unmodeled exogenous effects.Deterministic robot actuator models are “probilified” by adding noise variables that characterize the types of uncertainty that exist in robotic actuation.
Kinematics is the calculus describing the effect of control actions on the configuration of a robot.
The material presented in this tutorial is largely restricted to mobile robots operating in one-dimensional environments, whose
kinematic state, or pose, is summarized by one variable (for the simplicity of the tutorial, the pose of the robot is in the x axis and with positive direction always).

The probabilistic kinematic model, or motion model plays the role of the state transition model in mobile robotics.
This model is the familiar conditional density $p(x_t | x_{t-1}, u_t)$.
Here xt and xt−1 are both robot poses (and not just its x-coordinates), and ut is a motion command. This model describes the posterior distribution over kinematics states that a robots assumes when executing the motion command ut when its pose is
xt−1 . In implementations, ut is sometimes provided by a robot’s odometry. However, for conceptual reasons we will refer
to ut as control.

In practice, odometry models tend to be more accurate than velocity models, for the
simple reasons that most commercial robots do not execute velocity commands with
the level of accuracy that can be obtained by measuring the revolution of the robot’s
wheels. However odometry is only available post-the-fact. Hence it cannot be used for
motion planning. Planning algorithms such as collision avoidance have to predict the
effects of motion. Thus, odometry models are usually applied for estimation, whereas
velocity models are used for probabilistic motion planning.





1. The **motion model** is designed for a robot with a single degree of freedom, and it moves in the positive direction. The equation representing the motion of the robot is `x(t) = x(t-1) + v * dt`. 
    ```cpp
    // Motion model
    auto motion_model = [&](double state) {
      double distance = (tutorial_params.velocity * tutorial_params.dt);
      double translation_param = generateRandom(0.0, tutorial_params.translation_sigma);
      return state + distance - translation_param;
    };
    ```
    The code represent a **Sample Motion Model Algorithm** (pag. 111), which implement the sampling approach that a particles filter requires. It generates samples of *p(x(t) | u(t), x(t-1))*. The algorithm takes an initial pose *x(t-1)* (in our case a particle named `state`), and an odometry reading *u(t)* (or `velocity * dt`)  as inputs, and outputs a random pose *x(t)* distributed according *p(x(t) | u(t), x(t-1))*. The model adds  configurable noise, represented by `translation_param` to represent a possible drift or slippage of the robot.\
    In summary, the motion model returns a **pose estimated** for a given **particle** and a **noisy traveling distance**.

    * *x(t):* Random pose in time *t*.
    * *u(t):* The control signal that can be represented by `velocity * dt` or `odom(t) - odom(t-1)`.
    *  *x(t-1):* Random pose in time *t-1* (a random particle in that case).

    >**Note:** The effect of the motion model is to increase the space occupied by the particles.
  
2. 

### 2.2 Compiling the code
