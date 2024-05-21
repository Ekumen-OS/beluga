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

  /// Robot's initial position in meters (m).
  double initial_position{0.0};

  /// Standard deviation of the robot's initial position.
  /**
   * Represents the uncertainty of the robot's initial position.
   */
  double initial_position_sd{1.0};

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
  double sensor_model_sigma{1.0};

  /// Minimum particle weight.
  /**
   * Used to keep all particles "alive" in the reweight step.
   */
  double min_particle_weight{0.08};

  /// Dataset path.
  /**
   * The dataset file is used to save the data produced by the simulation for posterior analisys.
   */
  std::filesystem::path dataset_path{"./"};
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
    tutorial_params.map_size = params["map_size"].as<int>();
    tutorial_params.number_of_doors = params["number_of_doors"].as<int>();
    tutorial_params.number_of_particles = params["number_of_particles"].as<int>();
    tutorial_params.number_of_cycles = params["number_of_cycles"].as<int>();
    tutorial_params.initial_position = params["initial_position"].as<double>();
    tutorial_params.initial_position_sd = params["initial_position_sd"].as<double>();
    tutorial_params.dt = params["dt"].as<double>();
    tutorial_params.velocity = params["velocity"].as<double>();
    tutorial_params.translation_sd = params["translation_sd"].as<double>();
    tutorial_params.sensor_range = params["sensor_range"].as<double>();
    tutorial_params.sensor_model_sigma = params["sensor_model_sigma"].as<double>();
    tutorial_params.min_particle_weight = params["min_particle_weight"].as<double>();
    tutorial_params.dataset_path = params["dataset_path"].as<std::string>();
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

void save_tutorial_dataset_to_yaml(const std::filesystem::path& path, const TutorialDataset& tutorial_dataset) {
  YAML::Node node;

  for (const auto& landmark : tutorial_dataset.landmark_map) {
    node["landmark_map"].push_back(landmark);
  }

  for (auto&& [cycle, data] : tutorial_dataset.sim_data | ranges::views::enumerate) {
    node["simulation_dataset"][cycle]["ground_truth"] = data.ground_truth;

    auto current = node["simulation_dataset"][cycle]["particles"]["current"];
    current["states"] = beluga::views::states(data.current) | ranges::to<std::vector<double>>;
    current["weights"] = beluga::views::weights(data.current) | ranges::to<std::vector<double>>;

    auto propagate = node["simulation_dataset"][cycle]["particles"]["propagate"];
    propagate["states"] = beluga::views::states(data.propagate) | ranges::to<std::vector<double>>;
    propagate["weights"] = beluga::views::weights(data.propagate) | ranges::to<std::vector<double>>;

    auto reweight = node["simulation_dataset"][cycle]["particles"]["reweight"];
    reweight["states"] = beluga::views::states(data.reweight) | ranges::to<std::vector<double>>;
    reweight["weights"] = beluga::views::weights(data.reweight) | ranges::to<std::vector<double>>;

    auto resample = node["simulation_dataset"][cycle]["particles"]["resample"];
    resample["states"] = beluga::views::states(data.resample) | ranges::to<std::vector<double>>;
    resample["weights"] = beluga::views::weights(data.resample) | ranges::to<std::vector<double>>;

    auto estimation = node["simulation_dataset"][cycle]["estimation"];
    estimation["mean"] = std::get<0>(data.estimation);
    estimation["sd"] = std::get<1>(data.estimation);
  }

  std::ofstream fout(path, std::ios::trunc);
  fout << node;
  fout.close();
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << "<params_path>\n";
    return 1;
  }
  const std::filesystem::path params_path{argv[1]};

  TutorialParams tutorial_params;
  load_params_from_yaml(params_path, tutorial_params);

  // Create the TutorialDataset object for record the simulation data
  TutorialDataset tutorial_dataset;

  LandmarkMapVector landmark_map;
  load_landmark_map_from_yaml(params_path, landmark_map);
  tutorial_dataset.landmark_map = landmark_map;

  // Generate particles
  std::normal_distribution<double> initial_distribution(
      tutorial_params.initial_position, tutorial_params.initial_position_sd);
  auto particles = beluga::views::sample(initial_distribution) |                       //
                   ranges::views::transform(beluga::make_from_state<Particle>) |       //
                   ranges::views::take_exactly(tutorial_params.number_of_particles) |  //
                   ranges::to<beluga::TupleVector>;

  // Execute the particle filter using beluga
  double current_position{tutorial_params.initial_position};
  for (auto n = 0; n < tutorial_params.number_of_cycles; ++n) {
    TutorialData tutorial_data;

    // Save ground truth
    current_position += tutorial_params.velocity * tutorial_params.dt;
    tutorial_data.ground_truth = current_position;

    // Check if the simulation is out of bounds
    if (current_position > tutorial_params.map_size) {
      break;
    }

    auto motion_model = [&](double particle_position, auto& gen) {
      const double distance = (tutorial_params.velocity * tutorial_params.dt);
      std::normal_distribution<double> distribution(distance, tutorial_params.translation_sd);
      return particle_position + distribution(gen);
    };

    // Generate simulated sensor data
    auto detections =
        landmark_map |                                                                                            //
        ranges::views::transform([&](const double& landmark) { return landmark - current_position; }) |           //
        ranges::views::remove_if([&](double range) { return std::abs(range) > tutorial_params.sensor_range; }) |  //
        ranges::to<SensorData>;

    auto sensor_model = [&](const double& particle_position) {
      auto particle_detections =
          landmark_map |                                                                                    //
          ranges::views::transform([&](const double& landmark) { return landmark - particle_position; }) |  //
          ranges::to<SensorData>;

      return tutorial_params.min_particle_weight +
             std::transform_reduce(
                 detections.begin(), detections.end(), 1.0, std::multiplies<>{}, [&](const double& detection) {
                   auto distances = particle_detections |  //
                                    ranges::views::transform([&](const double& particle_detection) {
                                      return std::abs(detection - particle_detection);
                                    });
                   const auto min_distance = ranges::min(distances);
                   return exp((-1 * pow(min_distance, 2)) / (2 * tutorial_params.sensor_model_sigma));
                 });
    };

    tutorial_data.current = particles;

    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    tutorial_data.propagate = particles;

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
    tutorial_data.reweight = particles;

    particles |= beluga::views::sample |                                             //
                 ranges::views::take_exactly(tutorial_params.number_of_particles) |  //
                 beluga::actions::assign;
    tutorial_data.resample = particles;

    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    tutorial_data.estimation = estimation;

    tutorial_dataset.sim_data.push_back(std::move(tutorial_data));
  }

  save_tutorial_dataset_to_yaml(tutorial_params.dataset_path, tutorial_dataset);

  return 0;
}
