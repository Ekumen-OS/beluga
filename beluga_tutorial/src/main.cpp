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

struct Parameters {
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

struct RobotRecord {
  GroundTruth ground_truth;
  beluga::TupleVector<Particle> current;
  beluga::TupleVector<Particle> propagate;
  beluga::TupleVector<Particle> reweight;
  beluga::TupleVector<Particle> resample;
  Estimate estimation;
};

struct SimulationRecord {
  LandmarkMapVector landmark_map;
  std::vector<RobotRecord> robot_record;
};

void load_parameters(const std::filesystem::path& path, Parameters& parameters) {
  try {
    YAML::Node params = YAML::LoadFile(path);
    parameters.map_size = params["map_size"].as<int>();
    parameters.number_of_doors = params["number_of_doors"].as<int>();
    parameters.number_of_particles = params["number_of_particles"].as<int>();
    parameters.number_of_cycles = params["number_of_cycles"].as<int>();
    parameters.initial_position = params["initial_position"].as<double>();
    parameters.initial_position_sd = params["initial_position_sd"].as<double>();
    parameters.dt = params["dt"].as<double>();
    parameters.velocity = params["velocity"].as<double>();
    parameters.translation_sd = params["translation_sd"].as<double>();
    parameters.sensor_range = params["sensor_range"].as<double>();
    parameters.sensor_model_sigma = params["sensor_model_sigma"].as<double>();
    parameters.min_particle_weight = params["min_particle_weight"].as<double>();
    parameters.dataset_path = params["dataset_path"].as<std::string>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void load_landmark_map(const std::filesystem::path& path, LandmarkMapVector& landmark_map) {
  try {
    YAML::Node node = YAML::LoadFile(path);
    landmark_map = node["landmark_map"].as<LandmarkMapVector>();
  } catch (YAML::BadFile& e) {
    std::cout << e.what() << "\n";
  }
}

void save_simulation_records(const std::filesystem::path& path, const SimulationRecord& simulation_record) {
  YAML::Node node;

  for (const auto& landmark : simulation_record.landmark_map) {
    node["landmark_map"].push_back(landmark);
  }

  for (auto&& [cycle, record] : simulation_record.robot_record | ranges::views::enumerate) {
    node["simulation_records"][cycle]["ground_truth"] = record.ground_truth;

    auto current = node["simulation_records"][cycle]["particles"]["current"];
    current["states"] = beluga::views::states(record.current) | ranges::to<std::vector<double>>;
    current["weights"] = beluga::views::weights(record.current) | ranges::to<std::vector<double>>;

    auto propagate = node["simulation_records"][cycle]["particles"]["propagate"];
    propagate["states"] = beluga::views::states(record.propagate) | ranges::to<std::vector<double>>;
    propagate["weights"] = beluga::views::weights(record.propagate) | ranges::to<std::vector<double>>;

    auto reweight = node["simulation_records"][cycle]["particles"]["reweight"];
    reweight["states"] = beluga::views::states(record.reweight) | ranges::to<std::vector<double>>;
    reweight["weights"] = beluga::views::weights(record.reweight) | ranges::to<std::vector<double>>;

    auto resample = node["simulation_records"][cycle]["particles"]["resample"];
    resample["states"] = beluga::views::states(record.resample) | ranges::to<std::vector<double>>;
    resample["weights"] = beluga::views::weights(record.resample) | ranges::to<std::vector<double>>;

    auto estimation = node["simulation_records"][cycle]["estimation"];
    estimation["mean"] = std::get<0>(record.estimation);
    estimation["sd"] = std::get<1>(record.estimation);
  }

  std::ofstream fout(path, std::ios::trunc);
  fout << node;
  fout.close();
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << "<parameters_path>\n";
    return 1;
  }

  Parameters parameters;
  load_parameters(argv[1], parameters);

  SimulationRecord simulation_record;

  LandmarkMapVector landmark_map;
  load_landmark_map(argv[1], landmark_map);
  simulation_record.landmark_map = landmark_map;

  std::normal_distribution<double> initial_distribution(parameters.initial_position, parameters.initial_position_sd);
  auto particles = beluga::views::sample(initial_distribution) |                  //
                   ranges::views::transform(beluga::make_from_state<Particle>) |  //
                   ranges::views::take_exactly(parameters.number_of_particles) |  //
                   ranges::to<beluga::TupleVector>;

  double current_position{parameters.initial_position};
  for (auto n = 0; n < parameters.number_of_cycles; ++n) {
    RobotRecord robot_record;

    current_position += parameters.velocity * parameters.dt;
    robot_record.ground_truth = current_position;

    if (current_position > parameters.map_size) {
      break;
    }

    auto motion_model = [&](double particle_position, auto& gen) {
      const double distance = (parameters.velocity * parameters.dt);
      std::normal_distribution<double> distribution(distance, parameters.translation_sd);
      return particle_position + distribution(gen);
    };

    auto detections =
        landmark_map |                                                                                       //
        ranges::views::transform([&](const double& landmark) { return landmark - current_position; }) |      //
        ranges::views::remove_if([&](double range) { return std::abs(range) > parameters.sensor_range; }) |  //
        ranges::to<SensorData>;

    auto sensor_model = [&](const double& particle_position) {
      auto particle_detections =
          landmark_map |                                                                                    //
          ranges::views::transform([&](const double& landmark) { return landmark - particle_position; }) |  //
          ranges::to<SensorData>;

      return parameters.min_particle_weight +
             std::transform_reduce(
                 detections.begin(), detections.end(), 1.0, std::multiplies<>{}, [&](const double& detection) {
                   auto distances = particle_detections |  //
                                    ranges::views::transform([&](const double& particle_detection) {
                                      return std::abs(detection - particle_detection);
                                    });
                   const auto min_distance = ranges::min(distances);
                   return exp((-1 * pow(min_distance, 2)) / (2 * parameters.sensor_model_sigma));
                 });
    };

    robot_record.current = particles;

    particles |= beluga::actions::propagate(std::execution::seq, motion_model);
    robot_record.propagate = particles;

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) | beluga::actions::normalize;
    robot_record.reweight = particles;

    particles |= beluga::views::sample |                                        //
                 ranges::views::take_exactly(parameters.number_of_particles) |  //
                 beluga::actions::assign;
    robot_record.resample = particles;

    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    robot_record.estimation = estimation;

    simulation_record.robot_record.push_back(std::move(robot_record));
  }

  save_simulation_records(parameters.dataset_path, simulation_record);

  return 0;
}
