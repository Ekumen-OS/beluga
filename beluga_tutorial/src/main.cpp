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
#include <range/v3/algorithm/min_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/take_exactly.hpp>

#include <yaml-cpp/yaml.h>
#include <beluga/beluga.hpp>

// TODO(alon): Explain each parameter.
struct TutorialParams {
  bool load_landmark_map_from_file{false};
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
  std::string dataset_file_name{"dataset.yaml"};
};

using Particle = std::tuple<double, beluga::Weight>;
using LandmarkMapVector = std::vector<double>;
using SensorData = std::vector<double>;
using GroundTruth = double;
using Estimate = std::pair<double, double>;

struct TutorialData {
  GroundTruth ground_truth;
  beluga::TupleVector<Particle> current;
  beluga::TupleVector<Particle> propagate;
  beluga::TupleVector<Particle> reweight;
  beluga::TupleVector<Particle> resample;
  Estimate estimation;
};

struct TutorialDataset {
  LandmarkMapVector landmark_map;
  std::vector<TutorialData> sim_data;
};

double generateRandom(double mean, double sd) {
  static auto generator = std::mt19937{std::random_device()()};
  std::normal_distribution<double> distribution(mean, sd);

  return distribution(generator);
}

void load_params_from_yaml(TutorialParams& tutorial_params) {
  try {
    YAML::Node params = YAML::LoadFile("./src/beluga/beluga_tutorial/params/tutorial.yaml");
    tutorial_params.load_landmark_map_from_file = params["load_landmark_map_from_file"].as<bool>();
    tutorial_params.dataset_file_name = params["dataset_file_name"].as<std::string>();
    tutorial_params.map_size = params["map_size"].as<int>();
    tutorial_params.number_of_doors = params["number_of_doors"].as<int>();
    tutorial_params.number_of_particles = params["number_of_particles"].as<int>();
    tutorial_params.number_of_cycles = params["number_of_cycles"].as<int>();
    tutorial_params.initial_pose = params["initial_pose"].as<double>();
    tutorial_params.initial_pose_sd = params["initial_pose_sd"].as<double>();
    tutorial_params.dt = params["dt"].as<double>();
    tutorial_params.velocity = params["velocity"].as<double>();
    tutorial_params.translation_sigma = params["translation_sigma"].as<double>();
    tutorial_params.sensor_range = params["sensor_range"].as<double>();
    tutorial_params.sensor_model_sigam = params["sensor_model_sigam"].as<double>();
    tutorial_params.min_particle_weight = params["min_particle_weight"].as<double>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void load_landmark_map_from_yaml(LandmarkMapVector& landamrk_map) {
  try {
    YAML::Node node = YAML::LoadFile("./src/beluga/beluga_tutorial/params/landmark_map.yaml");
    landamrk_map = node["landamrk_map"].as<LandmarkMapVector>();
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
  fout.close();
}

int main() {
  // Load params from yaml file
  TutorialParams tutorial_params;
  load_params_from_yaml(tutorial_params);

  // Create the TutorialDataset object for record the simulation data
  TutorialDataset tutorial_dataset;

  // Load map from file or generate a random map
  LandmarkMapVector landmark_map;
  if (tutorial_params.load_landmark_map_from_file) {
    load_landmark_map_from_yaml(landmark_map);
  } else {
    std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
    landmark_map = beluga::views::sample(landmark_distribution) |
                   ranges::views::take_exactly(tutorial_params.number_of_doors) | ranges::to<LandmarkMapVector>;
    landmark_map |= ranges::actions::sort | ranges::actions::unique;
  }

  tutorial_dataset.landmark_map = landmark_map;

  // Generate particles
  std::normal_distribution<double> initial_distribution(tutorial_params.initial_pose, tutorial_params.initial_pose_sd);
  auto particles = beluga::views::sample(initial_distribution) |
                   ranges::views::transform(beluga::make_from_state<Particle>) |
                   ranges::views::take_exactly(tutorial_params.number_of_particles) | ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  double current_pose{tutorial_params.initial_pose};
  for (auto n = 0; n < tutorial_params.number_of_cycles; n++) {
    // Prepare the data to be saved
    TutorialData tutorial_data;

    // Save ground truth
    current_pose += (tutorial_params.velocity * tutorial_params.dt);
    tutorial_data.ground_truth = current_pose;

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
    // Current stage
    tutorial_data.current = particles;

    // Propagate stage
    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    tutorial_data.propagate = particles;

    // Reweight stage
    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
    tutorial_data.reweight = particles;

    // Resample
    particles |= beluga::views::sample | ranges::views::take_exactly(tutorial_params.number_of_particles) |
                 beluga::actions::assign;
    tutorial_data.resample = particles;

    // Calculate mean and standard deviation
    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    tutorial_data.estimation = estimation;

    // Save the sim data
    tutorial_dataset.sim_data.push_back(tutorial_data);
  }

  save_tutorial_dataset_to_yaml(tutorial_params.dataset_file_name, tutorial_dataset);

  return 0;
}