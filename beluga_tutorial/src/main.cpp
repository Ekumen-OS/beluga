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
#include <beluga_tutorial/tutorial_dataset.hpp>

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
// using ParticlesDataset = std::vector<std::vector<Particle>>;

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

void save_landmark_map_to_yaml(const std::string& file_name, const LandmarkMapVector& landmark_map) {
  YAML::Node node;
  if (std::filesystem::exists("./src/beluga/beluga_tutorial/bags/" + file_name)) {
    try {
      node = YAML::LoadFile("./src/beluga/beluga_tutorial/bags/" + file_name);
    } catch (YAML::BadFile& e) {
      std::cout << e.what() << "\n";
    }
  }

  for (const auto& lmp : landmark_map) {
    node["landamrk_map"].push_back(lmp);
  }
  std::ofstream fout("./src/beluga/beluga_tutorial/bags/" + file_name);
  fout << node;
}

void save_particles_to_yaml(
    const std::string& file_name,
    int cycle,
    const std::string& stage,
    const beluga::TupleVector<Particle>& particles) {
  YAML::Node node;
  if (std::filesystem::exists("./src/beluga/beluga_tutorial/bags/" + file_name)) {
    try {
      node = YAML::LoadFile("./src/beluga/beluga_tutorial/bags/" + file_name);
    } catch (YAML::BadFile& e) {
      std::cout << e.what() << "\n";
    }
  }

  for (const auto& particle : particles) {
    node["simulation_dataset"][cycle]["particles"][stage]["states"].push_back(std::get<0>(particle));
    node["simulation_dataset"][cycle]["particles"][stage]["weights"].push_back(
        static_cast<double>(std::get<1>(particle)));
  }

  std::ofstream fout("./src/beluga/beluga_tutorial/bags/" + file_name);
  fout << node;
}

void save_estimation_to_yaml(const std::string& file_name, int cycle, const std::pair<int, int>& estimation) {
  YAML::Node node;
  if (std::filesystem::exists("./src/beluga/beluga_tutorial/bags/" + file_name)) {
    try {
      node = YAML::LoadFile("./src/beluga/beluga_tutorial/bags/" + file_name);
    } catch (YAML::BadFile& e) {
      std::cout << e.what() << "\n";
    }
  }

  node["simulation_dataset"][cycle]["estimation"]["mean"] = estimation.first;
  node["simulation_dataset"][cycle]["estimation"]["sd"] = estimation.second;

  std::ofstream fout("./src/beluga/beluga_tutorial/bags/" + file_name);
  fout << node;
}

int main() {
  // Load params from yaml file
  TutorialParams tutorial_params;
  load_params_from_yaml(tutorial_params);

  // Generate a random map
  std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
  auto landmark_map = beluga::views::sample(landmark_distribution) |
                      ranges::views::take_exactly(tutorial_params.number_of_doors) | ranges::to<LandmarkMapVector>;
  landmark_map |= ranges::actions::sort | ranges::actions::unique;

  // Save the landmark map in a yaml file
  save_landmark_map_to_yaml(tutorial_params.dataset_file_name, landmark_map);

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

    // Motion model
    current_pose += (tutorial_params.sim_velocity * tutorial_params.sim_dt);
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

    save_particles_to_yaml(tutorial_params.dataset_file_name, n, "current", particles);
    // particles |= beluga::actions::propagate(std::execution::seq, motion_model) |
    //              beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;

    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    save_particles_to_yaml(tutorial_params.dataset_file_name, n, "propagate", particles);

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model);
    save_particles_to_yaml(tutorial_params.dataset_file_name, n, "reweight", particles);

    particles |= beluga::actions::normalize;

    // Resample
    particles |= beluga::views::sample | ranges::views::take_exactly(tutorial_params.number_of_particles) |
                 beluga::actions::assign;
    save_particles_to_yaml(tutorial_params.dataset_file_name, n, "resample", particles);

    // Calculate mean and standard deviation
    // TODO(alon): save the mean and standard deviation in the hdf5 file
    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    save_estimation_to_yaml(tutorial_params.dataset_file_name, n, estimation);
    // std::cout << "current_pose: " << current_pose << ", mean: " << mean << ", sd: " << sd << std::endl;
    // TODO(alon): current_pose = ground_truth, mean = estimated_pose
    // TODO(alon): play with the initial position in the tutorial
  }
  return 0;
}