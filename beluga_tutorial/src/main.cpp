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

// https://github.com/Ekumen-OS/beluga/issues/279#issuecomment-1903914387

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <meta/meta.hpp>
#include <range/v3/action.hpp>
#include <range/v3/all.hpp>  // TODO(alon): get only the necessary headers
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <beluga/beluga.hpp>

// Tutorial parameters
static constexpr int kMapSize = 100;
static constexpr int kNumDoors = kMapSize / 3;
static constexpr int kNumParticles = 200;
static constexpr int kSimNumCycles = 100;
static constexpr int kInitialPose = 0;
static constexpr int kSimDt = 1;
static constexpr int kVelocity = 1;
static constexpr int kMeasurementDist = 2;
static constexpr int kSensorModelSigma = 3;

static constexpr double kTranslationSigma = 1.0;

bool generateRandomBool(double p) {
  static auto generator = std::mt19937{std::random_device()()};
  std::bernoulli_distribution distribution(p);
  return distribution(generator);
}

int generateRandomInt(double mean, double sd) {
  static auto generator = std::mt19937{std::random_device()()};
  std::normal_distribution<double> distribution(mean, sd);

  return static_cast<int>(distribution(generator));
}

int main() {
  std::cout << "beluga tutorial - main.cpp start!!" << std::endl;

  // Generate a random map
  std::uniform_int_distribution landmark_distribution{0, kMapSize};
  auto landmark_map = beluga::views::sample(landmark_distribution) | ranges::views::take_exactly(kNumDoors) |
                      ranges::to<std::vector<int>>;
  // TODO(alon): the map has to be sort and unique or not necessarily? for now it helps for testing purpose.
  landmark_map |= ranges::actions::sort | ranges::actions::unique;
  std::cout << "landamrk_map" << std::endl;
  for (const auto& lm : landmark_map) {
    std::cout << lm << ",";
  }
  std::cout << std::endl;
  std::cout << std::endl;

  // Save the map in a csv file
  {
    std::string filename{"map.csv"};
    std::fstream fout{filename, fout.trunc | fout.out};
    fout << "mark_pose" << std::endl;
    for (const auto& l : landmark_map) {
      fout << l << std::endl;
    }
  }

  // Generate particles
  std::uniform_int_distribution initial_distribution{0, kMapSize};
  using Particle = std::tuple<int, beluga::Weight>;
  auto particles = beluga::views::sample(initial_distribution) |
                   ranges::views::transform(beluga::make_from_state<Particle>) |
                   ranges::views::take_exactly(kNumParticles) | ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  int current_pose{kInitialPose};

  for (auto n = 0; n < kSimNumCycles; n++) {
    // Check if the simulation is out of bounds
    if (current_pose > kMapSize) {
      break;
    }

    // Motion model
    current_pose += (kVelocity * kSimDt);
    auto motion_model = [&](double state) {
      int distance = (kVelocity * kSimDt);
      int translation_param = generateRandomInt(0, kTranslationSigma);
      return state + distance - translation_param;
    };

    auto sensor_model = [&](const double& state) {
      bool measurement = ranges::any_of(
          landmark_map, [&](int lm) { return std::abs(static_cast<int>(state) - lm) <= kMeasurementDist; });

      auto landmark_probability = landmark_map | ranges::view::transform([&](int lm) {
                                    return exp(-1 * pow(static_cast<int>(state) - lm, 2) / (2 * kSensorModelSigma));
                                  });

      auto no_landmark_probability = ranges::accumulate(
          landmark_probability, 1.0, [](double init, double probability) { return init * (1 - probability); });

      auto landmark_probability_sum = ranges::accumulate(landmark_probability, 0.0);
      auto factor = landmark_probability_sum + no_landmark_probability;

      if (measurement) {
        return landmark_probability_sum / factor;
      }
      return no_landmark_probability / factor;
    };

    // TODO: To showcase the example, it may be useful to apply each range adaptor separately and save the particles
    // in a file for display them using some plot lib.
    particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
                 beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;

    // Resample
    particles |= beluga::views::sample | ranges::views::take_exactly(kNumParticles) | beluga::actions::assign;

    // Calculate mean and standard deviation
    auto states = particles | beluga::views::states | ranges::views::common;
    auto accum_state = ranges::accumulate(states, 0.0);
    int mean = static_cast<int>(accum_state / (static_cast<int>(particles.size())));
    auto variance =
        ranges::accumulate(states, 0.0, [&mean](double init, double state) { return init + pow(state - mean, 2); });
    double sd = sqrt(variance / (static_cast<double>(particles.size() - 1)));

    std::cout << "mean: " << mean << ", sd: " << sd << std::endl;
  }

  return 0;
}