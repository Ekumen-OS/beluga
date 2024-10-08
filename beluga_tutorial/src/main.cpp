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

namespace beluga::tutorial {

struct Parameters {
  /// Size of the 1D map in (m)
  /**
   * 1D map space is continuous, but its size is defined
   * as an integer to simplify data visualization.
   */
  std::size_t map_size{100};

  /// Fixed number of particles used by the algorithm.
  std::size_t number_of_particles{300};

  /// Number of simulation cycles.
  std::size_t number_of_cycles{100};

  /// Initial robot position, in meters (m).
  double initial_position{0.0};

  /// Standard deviation of the robot's estimate of its initial position.
  double initial_position_sigma{10.0};

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
  double sensor_range{3.0};

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
  std::vector<double> landmark_map{5, 12, 25, 37, 52, 55, 65, 74, 75, 87, 97};

  /// Dataset path.
  /**
   * The dataset file is used to save the data produced by the simulation for posterior analisys.
   */
  std::filesystem::path record_path{"./record.yaml"};
};

using Particle = std::tuple<double, beluga::Weight>;

struct RobotRecord {
  double ground_truth;
  std::vector<Particle> current;
  std::vector<Particle> prediction;
  std::vector<Particle> update;
  std::pair<double, double> estimation;
};

}  // namespace beluga::tutorial

namespace YAML {

template <>
struct convert<beluga::tutorial::Parameters> {
  static bool decode(const Node& node, beluga::tutorial::Parameters& parameters) {
    parameters.map_size = node["map_size"].as<std::size_t>();
    parameters.number_of_particles = node["number_of_particles"].as<std::size_t>();
    parameters.number_of_cycles = node["number_of_cycles"].as<std::size_t>();
    parameters.initial_position = node["initial_position"].as<double>();
    parameters.initial_position_sigma = node["initial_position_sigma"].as<double>();
    parameters.dt = node["dt"].as<double>();
    parameters.velocity = node["velocity"].as<double>();
    parameters.motion_model_sigma = node["motion_model_sigma"].as<double>();
    parameters.sensor_range = node["sensor_range"].as<double>();
    parameters.sensor_model_sigma = node["sensor_model_sigma"].as<double>();
    parameters.min_particle_weight = node["min_particle_weight"].as<double>();
    parameters.record_path = node["record_path"].as<std::string>();
    parameters.landmark_map = node["landmark_map"].as<std::vector<double>>();
    return true;
  }
};

template <>
struct convert<std::vector<beluga::tutorial::Particle>> {
  static Node encode(const std::vector<beluga::tutorial::Particle>& particles) {
    Node node;
    node["states"] = beluga::views::states(particles) | ranges::to<std::vector<double>>;
    node["weights"] = beluga::views::weights(particles) | ranges::to<std::vector<double>>;
    return node;
  }
};

template <>
struct convert<std::vector<beluga::tutorial::RobotRecord>> {
  static Node encode(const std::vector<beluga::tutorial::RobotRecord>& records) {
    Node node;
    for (auto&& [cycle, record] : ranges::views::enumerate(records)) {
      node[cycle]["ground_truth"] = record.ground_truth;

      auto particles = node[cycle]["particles"];
      particles["current"] = record.current;
      particles["prediction"] = record.prediction;
      particles["update"] = record.update;

      auto estimation = node[cycle]["estimation"];
      estimation["mean"] = std::get<0>(record.estimation);
      estimation["sd"] = std::sqrt(std::get<1>(record.estimation));
    }
    return node;
  }
};

void DumpFile(const std::filesystem::path& path, const Node& node) {
  std::ofstream fout(path, std::ios::trunc);
  fout << node;
}

}  // namespace YAML

namespace beluga::tutorial {

int run(const std::filesystem::path& path) {
  const auto parameters = YAML::LoadFile(path).as<beluga::tutorial::Parameters>();

  std::normal_distribution<double> initial_position_distribution(
      parameters.initial_position, parameters.initial_position_sigma);
  auto particles = beluga::views::sample(initial_position_distribution) |         //
                   ranges::views::transform(beluga::make_from_state<Particle>) |  //
                   ranges::views::take_exactly(parameters.number_of_particles) |  //
                   ranges::to<std::vector>;

  std::vector<RobotRecord> records;
  records.reserve(parameters.number_of_cycles);

  double current_position{parameters.initial_position};
  for (std::size_t n = 0; n < parameters.number_of_cycles; ++n) {
    RobotRecord record;

    current_position += parameters.velocity * parameters.dt;
    record.ground_truth = current_position;

    if (current_position > static_cast<double>(parameters.map_size)) {
      break;
    }

    const auto motion_model = [&](double position, auto& random_engine) {
      std::normal_distribution<double> motion_distribution(
          parameters.velocity * parameters.dt, parameters.motion_model_sigma * parameters.dt);
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
    record.prediction = particles;

    particles |= beluga::actions::reweight(std::execution::seq, sensor_model) |  //
                 beluga::actions::normalize |                                    //
                 beluga::views::sample |                                         //
                 ranges::views::take_exactly(parameters.number_of_particles) |   //
                 beluga::actions::assign;
    record.update = particles;

    const auto estimation = beluga::estimate(beluga::views::states(particles), beluga::views::weights(particles));
    record.estimation = estimation;

    records.push_back(std::move(record));
  }

  YAML::Node output;
  output["landmark_map"] = parameters.landmark_map;
  output["simulation_records"] = std::move(records);
  YAML::DumpFile(parameters.record_path, output);

  return 0;
}

}  // namespace beluga::tutorial

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <parameters_path>\n";
    return 1;
  }

  return beluga::tutorial::run(argv[1]);
}
