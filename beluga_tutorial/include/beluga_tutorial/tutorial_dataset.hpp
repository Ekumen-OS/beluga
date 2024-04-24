// Copyright 2023 Ekumen, Inc.
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

#ifndef BELUGA_TUTORIAL_TUTORIAL_DATASET
#define BELUGA_TUTORIAL_TUTORIAL_DATASET

#include <H5Cpp.h>
#include <Eigen/Core>
#include <beluga/primitives.hpp>
#include <iostream>
#include <tuple>
#include <vector>

namespace beluga_tutorial {

using Particle = std::tuple<int, beluga::Weight>;
using LandmarkMapVector = std::vector<int>;
using ParticlesDataset = std::vector<std::vector<Particle>>;

// TODO(alon): Explain each parameter.
struct TutorialParams {
  int mapSize;
  int numOfDoors;
  int numOfParticles;
  int numOfCycles;
  int initialPose;
  int simDt;
  int velocity;
  int measurementDist;
  double sensorModelSigma;
  double initialPoseSd;
  double translationSigma;
};

// TODO(alon): Make this class template for an int or a double landmark map position.
class TutorialDataset {
 public:
  explicit TutorialDataset(const std::string& file_name) : file_name_(file_name) {
    H5::H5File file_handler(file_name, H5F_ACC_TRUNC);
  }

  void save_params(const TutorialParams& params) {
    // Open the file in read/write mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDWR);

    // Define the data space
    std::array<hsize_t, 1> dim{sizeof(TutorialParams)};
    H5::DataSpace dataspace(1, dim.data());

    // Create a compouse data type
    H5::CompType params_type = get_params_data_type();

    // Create a data set with a name, data type and data space
    H5::DataSet dataset_tutorial_params(file_handler.createDataSet("params", params_type, dataspace));

    // Write to the file
    dataset_tutorial_params.write(&params, params_type);

    // Close all the resources
    dataset_tutorial_params.close();
    dataspace.close();
    file_handler.close();
  }

  void save_landmark_map(const LandmarkMapVector& landamrk_map) {
    // Open the file in read/write mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDWR);

    // Define the data space
    std::array<hsize_t, 1> dim{landamrk_map.size()};
    H5::DataSpace landamrk_map_dataspace(1, dim.data());

    // Define the data type
    H5::DataType landamrk_map_type(H5::PredType::NATIVE_INT);

    // Create a data set with a name, data type and data space
    H5::DataSet landmark_map_dataset(
        file_handler.createDataSet("landmark_map", landamrk_map_type, landamrk_map_dataspace));

    // Write to the file
    landmark_map_dataset.write(landamrk_map.data(), landamrk_map_type);

    // Close all the resources
    landmark_map_dataset.close();
    landamrk_map_dataspace.close();
    file_handler.close();
  }

  void save_particles_dataset(const ParticlesDataset& particles_dataset) {
    // Open the file in read/write mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDWR);

    // Particles dimensions
    std::array<hsize_t, 2> particles_dim{particles_dataset.size(), particles_dataset[0].size()};

    // Define the data space for states and weights
    H5::DataSpace states_dataspace(2, particles_dim.data());
    H5::DataSpace weights_dataspace(2, particles_dim.data());

    // Define the data type for states and weigths
    H5::IntType state_type(H5::PredType::NATIVE_INT);
    H5::FloatType weight_type(H5::PredType::NATIVE_FLOAT);

    // Create a data set with a name, data type and data space for states and weights
    H5::DataSet states_dataset(file_handler.createDataSet("states", state_type, states_dataspace));
    H5::DataSet weight_dataset(file_handler.createDataSet("weights", weight_type, weights_dataspace));

    // save the states and weights in an eigen matrix
    // TODO(alon): make the eigen matrix of dynamics sizes
    static constexpr int kNumParticles = 200;
    static constexpr int kSimNumCycles = 100;
    Eigen::Matrix<int, kSimNumCycles, kNumParticles> states_matrix;
    Eigen::Matrix<float, kSimNumCycles, kNumParticles> weights_matrix;
    for (size_t i = 0; i < particles_dataset.size(); i++) {
      for (size_t j = 0; j < particles_dataset[0].size(); j++) {
        states_matrix(static_cast<int>(i), static_cast<int>(j)) = std::get<0>(particles_dataset.at(i).at(j));
        weights_matrix(static_cast<int>(i), static_cast<int>(j)) =
            static_cast<float>(std::get<1>(particles_dataset.at(i).at(j)));
      }
    }

    // Write states
    states_dataset.write(states_matrix.data(), state_type);
    states_dataset.close();
    states_dataspace.close();

    // Write weights
    weight_dataset.write(weights_matrix.data(), weight_type);
    weight_dataset.close();
    weights_dataspace.close();

    // Close file
    file_handler.close();
  }

  // TODO(alon): The function is printing well the data but at the end it throwing the message: *** stack smashing
  // detected ***: terminated
  void print_params() {
    // Open the file in read only mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDONLY);

    // Create a compouse data type
    H5::CompType params_type = get_params_data_type();

    // Open params dataset
    H5::DataSet params_dataset = file_handler.openDataSet("params");

    // Get and print the dimension
    std::array<hsize_t, 1> dim;
    params_dataset.getSpace().getSimpleExtentDims(dim.data(), nullptr);
    std::cout << "params dim: " << dim[0] << "\n";

    // std::array<TutorialParams, 1> params;
    TutorialParams params;
    params_dataset.read(&params, params_type);
    std::cout << "params.mapSize: " << params.mapSize << "\n";
    std::cout << "params.numOfDoors: " << params.numOfDoors << "\n";
    std::cout << "params.numOfParticles: " << params.numOfParticles << "\n";
    std::cout << "params.numOfCycles: " << params.numOfCycles << "\n";
    std::cout << "params.initialPose: " << params.initialPose << "\n";
    std::cout << "params.simDt: " << params.simDt << "\n";
    std::cout << "params.velocity: " << params.velocity << "\n";
    std::cout << "params.measurementDist: " << params.measurementDist << "\n";
    std::cout << "params.sensorModelSigma: " << params.sensorModelSigma << "\n";
    std::cout << "params.initialPoseSd: " << params.initialPoseSd << "\n";
    std::cout << "params.translationSigma: " << params.translationSigma << "\n";
    std::cout << "\n";

    params_dataset.close();
    file_handler.close();
  }

  void print_landmark_map() {
    // Open the file in read only mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDONLY);

    // Open the landmark_map dataset
    // TODO(alon): save the fields name in a constexpr
    H5::DataSet landmark_map_dataset = file_handler.openDataSet("landmark_map");

    // Get and print the dimension
    std::array<hsize_t, 1> dim;
    landmark_map_dataset.getSpace().getSimpleExtentDims(dim.data(), nullptr);
    std::cout << "Landmark map dim: " << dim[0] << "\n";

    LandmarkMapVector landamrk_map_out(dim[0]);
    landmark_map_dataset.read(landamrk_map_out.data(), H5::PredType::NATIVE_INT);
    std::cout << "Landmark map: "
              << "\n";
    for (const auto& lmp : landamrk_map_out) {
      std::cout << lmp << ",";
    }
    std::cout << "\n";

    landmark_map_dataset.close();
    file_handler.close();
  }

  void print_particles_row(int row) {
    // Open the file in read only mode
    H5::H5File file_handler(file_name_, H5F_ACC_RDONLY);

    // Open the states and weights datasets
    H5::DataSet states_dataset = file_handler.openDataSet("states");
    H5::DataSet weights_dataset = file_handler.openDataSet("weights");

    // Get and print states and weights dimensions
    std::array<hsize_t, 2> states_dims;
    states_dataset.getSpace().getSimpleExtentDims(states_dims.data(), nullptr);
    std::cout << "states dims: "
              << "\n";
    std::cout << "rows: states_dims[0]: " << states_dims[0] << "\n";
    std::cout << "columns: states_dims[1]: " << states_dims[1] << "\n";

    std::array<hsize_t, 2> weights_dims;
    weights_dataset.getSpace().getSimpleExtentDims(weights_dims.data(), nullptr);
    std::cout << "weights dims: "
              << "\n";
    std::cout << "rows: weights_dims[0]: " << weights_dims[0] << "\n";
    std::cout << "columns: weights_dims[1]: " << weights_dims[1] << "\n";

    // Read and print states and weights
    // TODO(alon): make the eigen matrix a dynamic matrix
    Eigen::Matrix<int, 100, 200> states_out;
    states_dataset.read(states_out.data(), H5::PredType::NATIVE_INT);
    for (size_t i = 0; i < states_dims[1]; i++) {
      std::cout << "index: " << i << ", state: " << states_out(row, static_cast<int>(i)) << "\n";
    }
    states_dataset.close();

    Eigen::Matrix<float, 100, 200> weights_out;
    weights_dataset.read(weights_out.data(), H5::PredType::NATIVE_FLOAT);
    for (size_t i = 0; i < weights_dims[1]; i++) {
      std::cout << "index: " << i << ", weight: " << weights_out(row, static_cast<int>(i)) << "\n";
    }
    weights_dataset.close();

    file_handler.close();
  }

 private:
  static H5::CompType get_params_data_type() {
    // Create a compouse data type
    H5::CompType tutorial_params_type(sizeof(TutorialParams));
    tutorial_params_type.insertMember("mapSize", HOFFSET(TutorialParams, mapSize), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember("numOfDoors", HOFFSET(TutorialParams, numOfDoors), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember(
        "numOfParticles", HOFFSET(TutorialParams, numOfParticles), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember("numOfCycles", HOFFSET(TutorialParams, numOfCycles), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember("initialPose", HOFFSET(TutorialParams, initialPose), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember("simDt", HOFFSET(TutorialParams, simDt), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember("velocity", HOFFSET(TutorialParams, velocity), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember(
        "measurementDist", HOFFSET(TutorialParams, measurementDist), H5::PredType::NATIVE_INT);
    tutorial_params_type.insertMember(
        "sensorModelSigma", HOFFSET(TutorialParams, sensorModelSigma), H5::PredType::NATIVE_DOUBLE);
    tutorial_params_type.insertMember(
        "initialPoseSd", HOFFSET(TutorialParams, initialPoseSd), H5::PredType::NATIVE_DOUBLE);
    tutorial_params_type.insertMember(
        "translationSigma", HOFFSET(TutorialParams, translationSigma), H5::PredType::NATIVE_DOUBLE);

    return tutorial_params_type;
  }

  std::string file_name_;
};
}  // namespace beluga_tutorial

#endif  // BELUGA_TUTORIAL_TUTORIAL_DATASET