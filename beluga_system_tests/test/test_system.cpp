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

#include <filesystem>
#include <memory>
#include <sstream>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/particle_filter.hpp>
#include <beluga/localization.hpp>
#include <beluga/motion/differential_drive_model.hpp>
#include <beluga/random/multivariate_normal_distribution.hpp>
#include <beluga/sensor/likelihood_field_model.hpp>
#include <beluga_amcl/amcl_node_utils.hpp>
#include <beluga_amcl/occupancy_grid.hpp>
#include <beluga_amcl/tf2_sophus.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <range/v3/view/any_view.hpp>
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/generate.hpp>
#include <range/v3/view/generate_n.hpp>

namespace {

using namespace beluga;

struct InitialPose {
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;
};

struct TestDataPoint {
  Sophus::SE2d odom;
  std::vector<std::pair<double, double>> scan;
  Sophus::SE2d ground_truth;
};

struct TestData {
  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  InitialPose initial_pose;
  // to avoid problem with rosbag2_cpp::Reader destructor (captured objects).
  // see comment below
  ranges::any_view<TestDataPoint> data_points = ranges::empty_view<TestDataPoint>();
  double tol_distance;
  double tol_angle;
};

// Workaround to avoid opening/closing bagfiles at static initialization time.
using TestDataBuilder = std::function<TestData()>;

class BelugaSystemTest : public testing::TestWithParam<TestDataBuilder> {};

std::string se2d_to_string(const Sophus::SE2d& input) {
  std::ostringstream oss;
  oss << "x=" << input.translation().x() << " y=" << input.translation().y() << " theta=" << input.so2().log();
  return oss.str();
}

TEST_P(BelugaSystemTest, test_estimated_path) {
  auto test_data = GetParam()();
  // TODO(ivanpauno): Parameterize particle filter when Nuhue's PR is ready

  auto sampler_params = AdaptiveSamplerParam{};
  sampler_params.alpha_slow = 0.001;
  sampler_params.alpha_fast = 0.1;

  auto limiter_params = KldLimiterParam{};
  limiter_params.min_samples = 500ul;
  limiter_params.max_samples = 2000ul;
  limiter_params.spatial_resolution = 0.1;
  limiter_params.kld_epsilon = 0.05;
  limiter_params.kld_z = 3.;

  auto resample_on_motion_params = ResampleOnMotionPolicyParam{};
  resample_on_motion_params.update_min_d = 0.25;
  resample_on_motion_params.update_min_a = 0.2;

  auto resample_interval_params = ResampleIntervalPolicyParam{};
  resample_interval_params.resample_interval_count = 1;

  auto sensor_params = LikelihoodFieldModelParam{};
  sensor_params.max_obstacle_distance = 2.0;
  sensor_params.max_laser_distance = 100.0;
  sensor_params.z_hit = 0.5;
  sensor_params.z_random = 0.5;
  sensor_params.sigma_hit = 0.2;

  auto motion_params = DifferentialDriveModelParam{};
  motion_params.rotation_noise_from_rotation = 0.2;
  motion_params.rotation_noise_from_translation = 0.2;
  motion_params.translation_noise_from_translation = 0.2;
  motion_params.translation_noise_from_rotation = 0.2;

  auto selective_resampling_params = SelectiveResamplingPolicyParam{};
  selective_resampling_params.enabled = false;

  using DifferentialDrive =
      beluga::mixin::descriptor<beluga::DifferentialDriveModel, beluga::DifferentialDriveModelParam>;
  using LikelihoodField = beluga::mixin::descriptor<
      ciabatta::curry<beluga::LikelihoodFieldModel, beluga_amcl::OccupancyGrid>::mixin,
      beluga::LikelihoodFieldModelParam>;

  auto pf = mixin::make_unique<LaserLocalizationInterface2d, AdaptiveMonteCarloLocalization2d>(
      sampler_params, limiter_params, resample_on_motion_params, resample_interval_params, selective_resampling_params,
      DifferentialDrive{motion_params}, LikelihoodField{sensor_params}, beluga_amcl::OccupancyGrid{test_data.map});

  pf->initialize_states(
      ranges::views::generate([distribution = MultivariateNormalDistribution{
                                   test_data.initial_pose.mean, test_data.initial_pose.covariance}]() mutable {
        static auto generator = std::mt19937{std::random_device()()};
        const auto sample = distribution(generator);
        return Sophus::SE2d{Sophus::SO2d{sample.z()}, Eigen::Vector2d{sample.x(), sample.y()}};
      }));
  for (const auto [iteration, data_point] : ranges::views::enumerate(test_data.data_points)) {
    pf->update_motion(data_point.odom);
    pf->sample();
    pf->update_sensor(data_point.scan);
    pf->importance_sample();
    pf->resample();
    auto estimation = pf->estimate();
    auto error = estimation.first * data_point.ground_truth.inverse();
    auto distance_error = error.translation().norm();
    EXPECT_LE(distance_error, test_data.tol_distance) << "iteration: " << iteration << "\nestimation\n"
                                                      << se2d_to_string(estimation.first) << "\nactual\n"
                                                      << se2d_to_string(data_point.ground_truth) << std::endl;
    auto angle_error = error.so2().log();
    EXPECT_LE(angle_error, test_data.tol_angle) << "iteration: " << iteration << "\nestimation\n"
                                                << se2d_to_string(estimation.first) << "\nactual\n"
                                                << se2d_to_string(data_point.ground_truth) << std::endl;
  }
}

template <typename MessageT>
struct OneTopicReader {
  OneTopicReader(const std::filesystem::path& bagfile_path, std::string_view topic_name)
      : reader_{std::make_unique<rosbag2_cpp::Reader>()} {
    reader_->open(bagfile_path.native());
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(std::string{topic_name});
    reader_->set_filter(filter);
    for (const auto& topic_info_with_count : reader_->get_metadata().topics_with_message_count) {
      if (topic_info_with_count.topic_metadata.name == topic_name) {
        size_ = topic_info_with_count.message_count;
        return;
      }
    }
  }

  MessageT next() { return reader_->read_next<MessageT>(); }

  std::size_t size() { return size_; }

 private:
  // wrapped in a unique pointer to make it movable :)
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::size_t size_{0ul};
};

struct SE2dFromOdometryReader {
  SE2dFromOdometryReader(const std::filesystem::path& bagfile_path, std::string_view topic_name)
      : reader_{bagfile_path, std::move(topic_name)} {}

  Sophus::SE2d next() {
    Sophus::SE2d ret;
    tf2::convert(reader_.next().pose.pose, ret);
    return ret;
  }

  std::size_t size() { return reader_.size(); }

 private:
  OneTopicReader<nav_msgs::msg::Odometry> reader_;
};

struct LaserScanInfo {
  Sophus::SE3d laser_transform;
  std::size_t max_beam_count;
  float range_min;
  float range_max;
};

struct PointsFromScanReader {
  PointsFromScanReader(const std::filesystem::path& bagfile_path, std::string_view topic_name, LaserScanInfo info)
      : reader_{bagfile_path, std::move(topic_name)}, info_{std::move(info)} {}

  auto next() {
    auto scan_msg = reader_.next();
    return beluga_amcl::utils::make_points_from_laser_scan(
        scan_msg, info_.laser_transform, info_.max_beam_count, info_.range_min, info_.range_max);
  }

  std::size_t size() { return reader_.size(); }

 private:
  OneTopicReader<sensor_msgs::msg::LaserScan> reader_;
  LaserScanInfo info_;
};

template <typename GroundTruthReader>
TestData test_data_from_ros2bag(
    const std::filesystem::path& bagfile_path,
    InitialPose initial_pose,
    LaserScanInfo laser_scan_info,
    std::string_view map_topic,
    std::string_view odom_topic,
    std::string_view scan_topic,
    std::string_view ground_truth_topic,
    // Tolerance value shouldn't be testing accuracy,
    // but only that it's working grossly well
    double tol_distance = 0.5,
    double tol_angle = 30. * Sophus::Constants<double>::pi() / 180.) {
  TestData test_data;
  test_data.initial_pose = std::move(initial_pose);
  test_data.tol_distance = tol_distance;
  test_data.tol_angle = tol_angle;

  OneTopicReader<nav_msgs::msg::OccupancyGrid> map_reader{bagfile_path, map_topic};
  if (map_reader.size() != 1) {
    std::ostringstream oss;
    oss << "Expected map topic [" << map_topic << "] to have 1 message, got: " << map_reader.size();
    throw std::runtime_error{oss.str()};
  }
  test_data.map = std::make_shared<nav_msgs::msg::OccupancyGrid>(map_reader.next());

  PointsFromScanReader scan_reader{bagfile_path, scan_topic, laser_scan_info};
  SE2dFromOdometryReader odometry_reader{bagfile_path, odom_topic};
  GroundTruthReader ground_truth_reader{bagfile_path, ground_truth_topic};
  auto range_size = scan_reader.size();
  if (range_size != odometry_reader.size() || range_size != ground_truth_reader.size()) {
    std::ostringstream oss;
    oss << "Expected scan, odometry and ground truth topics to have the same number of messages, got: " << range_size
        << ", " << odometry_reader.size() << ", " << ground_truth_reader.size();
    throw std::runtime_error{oss.str()};
  }
  auto generate_fn = [scan_reader = std::move(scan_reader), odometry_reader = std::move(odometry_reader),
                      ground_truth_reader = std::move(ground_truth_reader)]() mutable {
    TestDataPoint next_test_data;
    next_test_data.scan = scan_reader.next();
    next_test_data.odom = odometry_reader.next();
    next_test_data.ground_truth = ground_truth_reader.next();
    return next_test_data;
  };
  auto generate_view = ranges::generate_n_view<std::decay_t<decltype(generate_fn)>>(std::move(generate_fn), range_size);
  test_data.data_points = std::move(generate_view);
  return test_data;
}

std::vector<TestDataBuilder> get_test_parameters() {
  std::vector<TestDataBuilder> ret;
  ret.emplace_back([]() {
    InitialPose initial_pose{
        Eigen::Vector3d{0.0, 2.0, 0.0}, Eigen::Matrix3d{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
    // initial_pose.mean(1) = 2.0;
    LaserScanInfo laser_info;
    laser_info.laser_transform = Sophus::SE3d{Eigen::Quaterniond{1., 0., 0., 0.}, Eigen::Vector3d{0.28, 0., 0.}};
    auto format = beluga_amcl::utils::make_eigen_comma_format();
    std::cout << laser_info.laser_transform.matrix().format(format) << std::endl;
    laser_info.max_beam_count = 60;
    laser_info.range_max = 100.;
    laser_info.range_min = 0.;
    return test_data_from_ros2bag<SE2dFromOdometryReader>(
        "./bags/perfect_odometry", initial_pose, laser_info, "/map", "/odometry/ground_truth", "/scan",
        "/odometry/ground_truth");
  });
  return ret;
}

INSTANTIATE_TEST_SUITE_P(BagFileTests, BelugaSystemTest, testing::ValuesIn(get_test_parameters()));
}  // namespace