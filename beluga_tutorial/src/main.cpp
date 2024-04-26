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
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <range/v3/action.hpp>
#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <yaml-cpp/yaml.h>
#include <beluga/beluga.hpp>

// TODO(alon): Explain each parameter.
struct TutorialParams {
  int map_size{100};
  int number_of_doors{33};
  int number_of_particles{200};
  int sim_number_of_cycles{100};
  int sim_initial_pose{0};
  int sim_dt{1};
  int sim_velocity{1};
  int measurement_distance{2};
  double sensor_model_sigam{1.0};
  double initial_pose_sd{1.0};
  double translation_sigma{1.0};
  std::string dataset_file_name{"dataset.yaml"};
};

using Particle = std::tuple<int, beluga::Weight>;
using LandmarkMapVector = std::vector<int>;
using GroundTruth = int;
using Estimate = std::pair<int, int>;

struct TutorialSimData {
  GroundTruth ground_truth;
  beluga::TupleVector<Particle> current;
  beluga::TupleVector<Particle> propagate;
  beluga::TupleVector<Particle> reweight;
  beluga::TupleVector<Particle> resample;
  Estimate estimation;
};

struct TutorialDataset {
  LandmarkMapVector landmark_map;
  std::vector<TutorialSimData> sim_data;
};

int generateRandomInt(double mean, double sd) {
  static auto generator = std::mt19937{std::random_device()()};
  std::normal_distribution<double> distribution(mean, sd);

  return static_cast<int>(distribution(generator));
}

void load_params_from_yaml(TutorialParams& tutorial_params) {
  try {
    YAML::Node params = YAML::LoadFile("./src/beluga/beluga_tutorial/params/tutorial.yaml");
    tutorial_params.dataset_file_name = params["dataset_file_name"].as<std::string>();
    tutorial_params.map_size = params["map_size"].as<int>();
    tutorial_params.number_of_doors = params["number_of_doors"].as<int>();
    tutorial_params.number_of_particles = params["number_of_particles"].as<int>();
    tutorial_params.sim_number_of_cycles = params["sim_number_of_cycles"].as<int>();
    tutorial_params.sim_initial_pose = params["sim_initial_pose"].as<int>();
    tutorial_params.sim_dt = params["sim_dt"].as<int>();
    tutorial_params.sim_velocity = params["sim_velocity"].as<int>();
    tutorial_params.measurement_distance = params["measurement_distance"].as<int>();
    tutorial_params.sensor_model_sigam = params["sensor_model_sigam"].as<double>();
    tutorial_params.initial_pose_sd = params["initial_pose_sd"].as<double>();
    tutorial_params.translation_sigma = params["translation_sigma"].as<double>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void save_tutorial_dataset_to_yaml(const std::string& file_name, const TutorialDataset& tutorial_dataset) {
  YAML::Node node;

  for (const auto& lmp : tutorial_dataset.landmark_map) {
    node["landamrk_map"].push_back(lmp);
  }

  for (auto&& [cycle, data] : tutorial_dataset.sim_data | ranges::views::enumerate) {
    node["simulation_dataset"][cycle]["ground_truth"] = data.ground_truth;

    for (const auto& current : data.current) {
      node["simulation_dataset"][cycle]["particles"]["current"]["states"].push_back(std::get<0>(current));
      node["simulation_dataset"][cycle]["particles"]["current"]["weights"].push_back(
          static_cast<double>(std::get<1>(current)));
    }

    for (const auto& propagate : data.propagate) {
      node["simulation_dataset"][cycle]["particles"]["propagate"]["states"].push_back(std::get<0>(propagate));
      node["simulation_dataset"][cycle]["particles"]["propagate"]["weights"].push_back(
          static_cast<double>(std::get<1>(propagate)));
    }

    for (const auto& reweight : data.reweight) {
      node["simulation_dataset"][cycle]["particles"]["reweight"]["states"].push_back(std::get<0>(reweight));
      node["simulation_dataset"][cycle]["particles"]["reweight"]["weights"].push_back(
          static_cast<double>(std::get<1>(reweight)));
    }

    for (const auto& resample : data.resample) {
      node["simulation_dataset"][cycle]["particles"]["resample"]["states"].push_back(std::get<0>(resample));
      node["simulation_dataset"][cycle]["particles"]["resample"]["weights"].push_back(
          static_cast<double>(std::get<1>(resample)));
    }

    node["simulation_dataset"][cycle]["estimation"]["mean"] = std::get<0>(data.estimation);
    node["simulation_dataset"][cycle]["estimation"]["sd"] = std::get<1>(data.estimation);
  }

  const std::string bags_path{"./src/beluga/beluga_tutorial/bags/"};
  std::filesystem::create_directory(bags_path);
  std::ofstream fout(bags_path + file_name, std::ios::trunc);
  fout << node;
}

int main() {
  // Load params from yaml file
  TutorialParams tutorial_params;
  load_params_from_yaml(tutorial_params);

  // Create the TutorialDataset object for record the simulation data
  TutorialDataset tutorial_dataset;

  // Generate a random map
  std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
  auto landmark_map = beluga::views::sample(landmark_distribution) |
                      ranges::views::take_exactly(tutorial_params.number_of_doors) | ranges::to<LandmarkMapVector>;
  landmark_map |= ranges::actions::sort | ranges::actions::unique;

  tutorial_dataset.landmark_map = landmark_map;

  // Generate particles
  std::normal_distribution<double> initial_distribution(
      tutorial_params.sim_initial_pose, tutorial_params.initial_pose_sd);
  auto particles = beluga::views::sample(initial_distribution) |
                   ranges::views::transform(beluga::make_from_state<Particle>) |
                   ranges::views::take_exactly(tutorial_params.number_of_particles) | ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  int current_pose{tutorial_params.sim_initial_pose};
  for (auto n = 0; n < tutorial_params.sim_number_of_cycles; n++) {
    // Check if the simulation is out of bounds
    if (current_pose > tutorial_params.map_size) {
      break;
    }

    // Prepare the data to be saved
    TutorialSimData tutorial_sim_data;

    // Save ground truth
    current_pose += (tutorial_params.sim_velocity * tutorial_params.sim_dt);
    tutorial_sim_data.ground_truth = current_pose;

    // Motion model
    auto motion_model = [&](double state) {
      int distance = (tutorial_params.sim_velocity * tutorial_params.sim_dt);
      int translation_param = generateRandomInt(0, tutorial_params.translation_sigma);
      return state + distance - translation_param;
    };

    auto sensor_model = [&](const double& state) {
      bool measurement = ranges::any_of(landmark_map, [&](int lm) {
        return std::abs(static_cast<int>(state) - lm) <= tutorial_params.measurement_distance;
      });

      auto landmark_probability =
          landmark_map | ranges::views::transform([&](int lm) {
            return exp(-1 * pow(static_cast<int>(state) - lm, 2) / (2 * tutorial_params.sensor_model_sigam));
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

    // In general the particles can be modified using pipe operators.
    // particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
    //              beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;

    // For the propose of the tutorial, we split the process in the following stages:
    // Current stage
    tutorial_sim_data.current = particles;

    // Propagate stage
    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    tutorial_sim_data.propagate = particles;

    // Reweight stage
    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
    tutorial_sim_data.reweight = particles;

    // Resample
    particles |= beluga::views::sample | ranges::views::take_exactly(tutorial_params.number_of_particles) |
                 beluga::actions::assign;
    tutorial_sim_data.resample = particles;

    // Calculate mean and standard deviation
    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    tutorial_sim_data.estimation = estimation;

    // Save the sim data
    tutorial_dataset.sim_data.push_back(tutorial_sim_data);
  }

  save_tutorial_dataset_to_yaml(tutorial_params.dataset_file_name, tutorial_dataset);

  return 0;
}