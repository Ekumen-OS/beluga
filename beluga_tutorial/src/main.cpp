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

struct TutorialParams {
  /// Load the landmark map from a file.
  /**
   * If true, the landmark map is loaded from a file;
   * if false, a random map is generated.
   */
  bool load_landmark_map{false};

  /// Save the landmark map to a file.
  bool save_landmark_map{false};

  /// Size of the 1D map in (m)
  /**
   * In the simulation, the 1D map is continuous,
   * but the limit is defined as an integer to simplify data visualization.
   */
  int map_size{100};

  /// Maximum number of doors (or landmarks) in the map.
  int number_of_doors{33};

  /// Fixed number of particles used by the algorithm.
  int number_of_particles{200};

  /// Number of simulation cycles.
  int number_of_cycles{100};

  /// Robot's initial pose in meters (m).
  double initial_pose{0.0};

  /// Standard deviation of the robot's initial pose.
  /**
   * Represents the uncertainty of the robot's initial pose.
   */
  double initial_pose_sd{1.0};

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
  double sensor_range{2.0};

  /// Sensor model sigma
  /**
   * Represents the precision of the modeled sensor.
   */
  double sensor_model_sigam{1.0};

  /// Minimum particle weight.
  /**
   * Used to keep all particles "alive" in the reweight step.
   */
  double min_particle_weight{0.08};

  /// Dataset path.
  /**
   * The dataset file is used to save the data produced by the simulation for posterior analisys.
   */
  std::filesystem::path dataset_path{"./src/beluga/beluga_tutorial/bags/dataset.yaml"};

  // Name of the landmark_map to save or load from.
  std::string landmark_map_file_name{"landmark_map.yaml"};
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

void load_params_from_yaml(const std::filesystem::path& path, TutorialParams& tutorial_params) {
  try {
    YAML::Node params = YAML::LoadFile(path);
    tutorial_params.load_landmark_map = params["load_landmark_map"].as<bool>();
    tutorial_params.save_landmark_map = params["save_landmark_map"].as<bool>();
    tutorial_params.map_size = params["map_size"].as<int>();
    tutorial_params.number_of_doors = params["number_of_doors"].as<int>();
    tutorial_params.number_of_particles = params["number_of_particles"].as<int>();
    tutorial_params.number_of_cycles = params["number_of_cycles"].as<int>();
    tutorial_params.initial_pose = params["initial_pose"].as<double>();
    tutorial_params.initial_pose_sd = params["initial_pose_sd"].as<double>();
    tutorial_params.dt = params["dt"].as<double>();
    tutorial_params.velocity = params["velocity"].as<double>();
    tutorial_params.translation_sd = params["translation_sd"].as<double>();
    tutorial_params.sensor_range = params["sensor_range"].as<double>();
    tutorial_params.sensor_model_sigam = params["sensor_model_sigam"].as<double>();
    tutorial_params.min_particle_weight = params["min_particle_weight"].as<double>();
    tutorial_params.dataset_path = params["dataset_path"].as<std::string>();
    tutorial_params.landmark_map_file_name = params["landmark_map_file_name"].as<std::string>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void load_landmark_map_from_yaml(const std::filesystem::path& path, LandmarkMapVector& landmark_map) {
  try {
    YAML::Node node = YAML::LoadFile(path);
    landmark_map = node["landmark_map"].as<LandmarkMapVector>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void save_landmark_map_to_yaml(const std::filesystem::path& path, LandmarkMapVector& landmark_map) {
  YAML::Node node;
  for (const auto& lmp : landmark_map) {
    node["landmark_map"].push_back(lmp);
  }
  std::ofstream fout(path, std::ios::trunc);
  fout << node;
  fout.close();
}

void save_tutorial_dataset_to_yaml(const std::filesystem::path& path, const TutorialDataset& tutorial_dataset) {
  YAML::Node node;

  for (const auto& lmp : tutorial_dataset.landmark_map) {
    node["landmark_map"].push_back(lmp);
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

  std::filesystem::create_directory(path.parent_path());
  std::ofstream fout(path, std::ios::trunc);
  fout << node;
  fout.close();
}

int main(int argc, char* argv[]) {
  // The user must provide the path to the parameter file
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " std::cerr << Usage:  << argv[0] <<  <directory_path> <filename>\n";
    return 1;
  }

  const std::string directory = argv[1];
  const std::string filename = argv[2];
  const auto parameters_file_path = std::filesystem::path{directory} / filename;

  // Load params from yaml file
  TutorialParams tutorial_params;
  load_params_from_yaml(parameters_file_path, tutorial_params);

  // Create the TutorialDataset object for record the simulation data
  TutorialDataset tutorial_dataset;

  // Load map from file or generate a random map
  LandmarkMapVector landmark_map;
  if (tutorial_params.load_landmark_map) {
    load_landmark_map_from_yaml(
        std::filesystem::path{directory} / tutorial_params.landmark_map_file_name, landmark_map);
  } else {
    std::uniform_int_distribution landmark_distribution{0, tutorial_params.map_size};
    landmark_map = beluga::views::sample(landmark_distribution) |                  //
                   ranges::views::take_exactly(tutorial_params.number_of_doors) |  //
                   ranges::to<LandmarkMapVector>;
    landmark_map |= ranges::actions::sort | ranges::actions::unique;
  }

  // Save landmark_map if required
  if (tutorial_params.save_landmark_map) {
    save_landmark_map_to_yaml(std::filesystem::path{directory} / tutorial_params.landmark_map_file_name, landmark_map);
  }

  tutorial_dataset.landmark_map = landmark_map;

  // Generate particles
  std::normal_distribution<double> initial_distribution(tutorial_params.initial_pose, tutorial_params.initial_pose_sd);
  auto particles = beluga::views::sample(initial_distribution) |                       //
                   ranges::views::transform(beluga::make_from_state<Particle>) |       //
                   ranges::views::take_exactly(tutorial_params.number_of_particles) |  //
                   ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  double current_pose{tutorial_params.initial_pose};
  for (auto n = 0; n < tutorial_params.number_of_cycles; n++) {
    // Prepare the data to be saved
    TutorialData tutorial_data;

    // Save ground truth
    current_pose += tutorial_params.velocity * tutorial_params.dt;
    tutorial_data.ground_truth = current_pose;

    // Check if the simulation is out of bounds
    if (current_pose > tutorial_params.map_size) {
      break;
    }

    auto motion_model = [&](double state, auto& gen) {
      const double distance = (tutorial_params.velocity * tutorial_params.dt);
      std::normal_distribution<double> distribution(distance, tutorial_params.translation_sd);
      return state + distribution(gen);
    };

    // Generate simulated sensor data
    auto sensor_data =
        landmark_map |                                                                                            //
        ranges::views::transform([&](double landmark) { return landmark - current_pose; }) |                      //
        ranges::views::remove_if([&](double range) { return std::abs(range) > tutorial_params.sensor_range; }) |  //
        ranges::to<SensorData>;

    // Sensor model
    auto sensor_model = [&](const double& state) {
      auto particle_sensor_data = landmark_map |                                                           //
                                  ranges::views::transform([&](const double lm) { return lm - state; }) |  //
                                  ranges::to<SensorData>;

      return std::transform_reduce(
                 sensor_data.begin(), sensor_data.end(), 1.0, std::multiplies<>{},
                 [&](const auto& sensor_datum) {
                   auto distances = particle_sensor_data | ranges::views::transform([&](auto& particle_sensor_datum) {
                                      return std::abs(sensor_datum - particle_sensor_datum);
                                    });
                   auto min_distance = ranges::min(distances);
                   return exp((-1 * pow(min_distance, 2)) / (2 * tutorial_params.sensor_model_sigam));
                 }) +
             tutorial_params.min_particle_weight;
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

  save_tutorial_dataset_to_yaml(tutorial_params.dataset_path, tutorial_dataset);

  return 0;
}