// Copyright 2026 Ekumen, Inc.
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

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <beluga_visual_odometry/types.hpp>

namespace beluga_visual_odometry {
namespace {

template <typename T>
void ReadIfPresent(const cv::FileNode& node, const char* name, T& value) {
  if (!node[name].empty()) {
    node[name] >> value;
  }
}

cv::Mat ReadMatrix(cv::FileStorage& storage, const char* name) {
  cv::Mat value;
  storage[name] >> value;
  if (value.empty()) {
    throw std::invalid_argument(std::string("Missing calibration field: ") + name);
  }
  value.convertTo(value, CV_64F);
  return value;
}

Sophus::SE3d ReadBaseFromCamera(const cv::FileNode& projection) {
  const auto transform = projection["t_base_camera"];
  std::vector<double> translation{0.0, 0.0, 0.0};
  std::vector<double> quaternion{0.0, 0.0, 0.0, 1.0};
  ReadIfPresent(transform, "translation", translation);
  ReadIfPresent(transform, "quaternion_xyzw", quaternion);
  if (translation.size() != 3 || quaternion.size() != 4) {
    throw std::invalid_argument("Invalid t_base_camera transform");
  }
  Eigen::Quaterniond rotation(
      quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
  if (rotation.norm() <= 1.0e-12) {
    throw std::invalid_argument("Invalid t_base_camera quaternion");
  }
  rotation.normalize();
  return Sophus::SE3d(
      Sophus::SO3d(rotation.toRotationMatrix()),
      Eigen::Vector3d(translation[0], translation[1], translation[2]));
}

}  // namespace

StereoCalibration StereoCalibration::LoadFromFile(const std::string& path) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::invalid_argument("Could not open stereo calibration file: " + path);
  }
  StereoCalibration calibration;
  calibration.image_size = cv::Size(
      static_cast<int>(storage["image_width"]),
      static_cast<int>(storage["image_height"]));
  calibration.left_camera_matrix = ReadMatrix(storage, "K_left");
  calibration.left_distortion = ReadMatrix(storage, "D_left");
  calibration.right_camera_matrix = ReadMatrix(storage, "K_right");
  calibration.right_distortion = ReadMatrix(storage, "D_right");
  calibration.rotation_right_left = ReadMatrix(storage, "R_right_left");
  calibration.translation_right_left =
      ReadMatrix(storage, "t_right_left").reshape(1, 3).clone();
  if (calibration.image_size.width <= 0 || calibration.image_size.height <= 0 ||
      calibration.left_camera_matrix.rows != 3 ||
      calibration.left_camera_matrix.cols != 3 ||
      calibration.right_camera_matrix.rows != 3 ||
      calibration.right_camera_matrix.cols != 3 ||
      calibration.rotation_right_left.rows != 3 ||
      calibration.rotation_right_left.cols != 3 ||
      cv::norm(calibration.translation_right_left) <= 0.0) {
    throw std::invalid_argument("Invalid stereo calibration");
  }
  return calibration;
}

VisualOdometryConfig LoadConfiguration(const std::string& path) {
  cv::FileStorage storage(path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::invalid_argument("Could not open VO configuration file: " + path);
  }
  const cv::FileNode root = storage.root();
  std::string calibration_path;
  ReadIfPresent(root, "calibration_file", calibration_path);
  if (calibration_path.empty()) {
    throw std::invalid_argument("Configuration must define calibration_file");
  }
  std::filesystem::path calibration_file(calibration_path);
  if (calibration_file.is_relative()) {
    calibration_file = std::filesystem::path(path).parent_path() / calibration_file;
  }

  VisualOdometryConfig config;
  config.calibration = StereoCalibration::LoadFromFile(calibration_file.string());
  ReadIfPresent(root, "sensor_frame", config.sensor_frame);
  const auto features = root["features"];
  ReadIfPresent(features, "nfeatures", config.orb_features);
  ReadIfPresent(features, "scale_factor", config.orb_scale_factor);
  ReadIfPresent(features, "nlevels", config.orb_levels);
  ReadIfPresent(features, "fast_threshold", config.orb_fast_threshold);
  const auto matching = root["matching"];
  ReadIfPresent(matching, "ratio_threshold", config.descriptor_ratio);
  const auto geometry = root["geometry"];
  ReadIfPresent(geometry, "max_vertical_error", config.max_vertical_error_pixels);
  ReadIfPresent(geometry, "min_disparity", config.min_disparity_pixels);
  ReadIfPresent(geometry, "min_inliers", config.min_inliers);
  const auto triangulation = root["triangulation"];
  ReadIfPresent(triangulation, "min_depth", config.min_depth_meters);
  ReadIfPresent(triangulation, "max_depth", config.max_depth_meters);
  const auto motion = root["motion"];
  ReadIfPresent(motion, "pnp_reprojection_error", config.pnp_reprojection_error_pixels);
  ReadIfPresent(motion, "pnp_iterations", config.pnp_iterations);
  ReadIfPresent(motion, "pnp_confidence", config.pnp_confidence);
  ReadIfPresent(root["uncertainty"], "pixel_noise_sigma", config.pixel_noise_sigma);
  ReadIfPresent(root["evaluation"], "minimum_tracking_ratio",
                config.minimum_tracking_ratio);
  config.T_base_camera = ReadBaseFromCamera(root["projection"]);
  return config;
}

}  // namespace beluga_visual_odometry
