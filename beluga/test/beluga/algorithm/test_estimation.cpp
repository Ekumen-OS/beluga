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

#include <gmock/gmock.h>

#include <beluga/algorithm/estimation.hpp>
#include <ciabatta/ciabatta.hpp>

#include "beluga/test/utils/sophus_matchers.hpp"

namespace {

using testing::ReturnRef;
using testing::SE2Eq;
using testing::SO2Eq;
using testing::Vector2Eq;

using Constants = Sophus::Constants<double>;
using Eigen::Vector2d;
using Sophus::SE2d;
using Sophus::SO2d;

struct CovarianceCalculation : public testing::Test {};

TEST_F(CovarianceCalculation, UniformWeightOverload) {
  // Covariance matrix calculated for items with uniform weights.
  // The following Octave code was used to validate the results:
  /*
      translations = [ 0 2 0 2 0 2 0 2 0 2; 0 2 0 2 0 0 2 2 2 0 ]';
      cov_matrix = cov(translations)
  */
  const auto translation_vector = std::vector{
      Eigen::Vector2<double>{0, 0}, Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 0},
      Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 0}, Eigen::Vector2<double>{2, 0},
      Eigen::Vector2<double>{0, 2}, Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 2},
      Eigen::Vector2<double>{2, 0},
  };
  const auto translation_mean = Eigen::Vector2<double>{1, 1};
  const auto covariance = beluga::calculate_covariance(translation_vector, translation_mean);
  ASSERT_NEAR(covariance(0, 0), 1.1111, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.2222, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.2222, 0.001);
  ASSERT_NEAR(covariance(1, 1), 1.1111, 0.001);
}

TEST_F(CovarianceCalculation, NonUniformWeightOverload) {
  // Covariance matrix calculated with non-uniform weights.
  // The following Octave code was used to validate the results:
  /*
      translations = [ 0 2 0 2 0 2 0 2 0 2; 0 2 0 2 0 0 2 2 2 0 ]';
      weights = [0 1 2 1 0 1 2 1 0 1]';
      normalized_weight = weights ./ sum(weights);
      weighted_mean = sum(normalized_weight .* translations)
      deviations = translations - weighted_mean;
      weighted_cov_matrix =  (normalized_weight .* deviations)' * deviations ./ (1 - sum(normalized_weight.^2))
  */
  const auto translation_vector = std::vector{
      Eigen::Vector2<double>{0, 0}, Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 0},
      Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 0}, Eigen::Vector2<double>{2, 0},
      Eigen::Vector2<double>{0, 2}, Eigen::Vector2<double>{2, 2}, Eigen::Vector2<double>{0, 2},
      Eigen::Vector2<double>{2, 0},
  };
  auto weights = std::vector{0.0, 1.0, 2.0, 1.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0};
  const auto total_weight = std::accumulate(weights.begin(), weights.end(), 0.0);
  std::for_each(weights.begin(), weights.end(), [total_weight](auto& weight) { weight /= total_weight; });
  const auto translation_mean = Eigen::Vector2<double>{1.1111, 1.1111};
  const auto covariance = beluga::calculate_covariance(translation_vector, weights, translation_mean);
  ASSERT_NEAR(covariance(0, 0), 1.1765, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.1176, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.1176, 0.001);
  ASSERT_NEAR(covariance(1, 1), 1.1765, 0.001);
}

struct PoseCovarianceEstimation : public testing::Test {
  // The following Octave code can be used to validate the results in the tests that derive from this fixture:
  /*
     # inputs
     translations = [ x1 y1 rot1; x2 y2 rot2; x3 y3 yaw3; ... xn yn rotn ] ;
     weights = [w1 w2 w3 ... wn]; ]';

     # auxiliar variables
     xy_translation = translations(:, 1:2);
     complex_rotation = exp(i* translations(:, 3));
     normalized_weight = weights ./ sum(weights);
     complex_rotation_mean = sum(normalized_weight .* complex_rotation);
     R = abs(complex_rotation_mean);
     # mean estimations
     xy_mean = sum(normalized_weight .* xy_translation);
     rot_mean = imag(log(complex_rotation_mean / abs(complex_rotation_mean)));
     # covariance estimations
     xy_deviations = xy_translation - xy_mean;
     xy_cov_matrix =  (normalized_weight .* xy_deviations)' * xy_deviations ./ (1 - sum(normalized_weight.^2));
     rot_cov = -2 * log(R);
     # results
     means = [ xy_mean, rot_mean ]
     covariance_matrix = [xy_cov_matrix, [0; 0]; [0 0], rot_cov]
  */
};

TEST_F(PoseCovarianceEstimation, PureTranslation) {
  // test the mean and covariance estimations for states that have different translations but the same rotation
  const auto states = std::vector{SE2d{SO2d{0.0}, Vector2d{1.0, 2.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}};
  const auto weights = std::vector(states.size(), 1.0);
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{0.5, 1.0}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{0.0}));
  ASSERT_NEAR(covariance(0, 0), 0.5, 0.001);
  ASSERT_NEAR(covariance(0, 1), 1.0, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 0), 1.0, 0.001);
  ASSERT_NEAR(covariance(1, 1), 2.0, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.0, 0.001);
}

TEST_F(PoseCovarianceEstimation, PureRotation) {
  // test the mean and covariance estimations for states that have different rotations but the same translation
  const auto states =
      std::vector{SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}, SE2d{SO2d{0.0}, Vector2d{0.0, 0.0}}};
  const auto weights = std::vector(states.size(), 1.0);
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{0.0, 0.0}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{-Constants::pi() / 4}));
  ASSERT_NEAR(covariance(0, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.693, 0.001);
}

TEST_F(PoseCovarianceEstimation, JointTranslationAndRotation) {
  // test the mean and covariance estimations for states that have different translations and rotations
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 6}, Vector2d{0.0, -3.0}}, SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},
      SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, -1.0}}, SE2d{SO2d{0.0}, Vector2d{3.0, 0.0}}};
  const auto weights = std::vector(states.size(), 1.0);
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{1.5, -1.5}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{Constants::pi() / 4}));
  ASSERT_NEAR(covariance(0, 0), 1.666, 0.001);
  ASSERT_NEAR(covariance(0, 1), 1.666, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 1.666, 0.001);
  ASSERT_NEAR(covariance(1, 1), 1.666, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.357, 0.001);
}

TEST_F(PoseCovarianceEstimation, CancellingOrientations) {
  // test mean and covariance for two states with opposite angles that cause a singularity in angular covariance
  // estimation
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 2}, Vector2d{0.0, 0.0}}, SE2d{SO2d{-Constants::pi() / 2}, Vector2d{0.0, 0.0}}};
  const auto weights = std::vector(states.size(), 1.0);
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{0.0, 0.0}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{0.0}));
  ASSERT_NEAR(covariance(0, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 1), 0.0, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0, 0.001);
  ASSERT_EQ(covariance(2, 2), std::numeric_limits<double>::infinity());
}

TEST_F(PoseCovarianceEstimation, RandomWalkWithSmoothRotationWithUniformWeights) {
  // test the mean and covariance estimations for states with random variations of translation and rotation
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() * 0.1}, Vector2d{0.0, -2.0}},  //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{1.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.3}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{3.0, 2.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{1.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.3}, Vector2d{2.0, -2.0}},  //
      SE2d{SO2d{Constants::pi() * 0.4}, Vector2d{3.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.5}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.4}, Vector2d{1.0, 2.0}},   //
  };
  const auto weights = std::vector(states.size(), 1.0);
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{1.700, 0.0}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{0.8762}));
  ASSERT_NEAR(covariance(0, 0), 0.900, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.5556, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.5556, 0.001);
  ASSERT_NEAR(covariance(1, 1), 2.4444, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.1355, 0.001);
}

TEST_F(PoseCovarianceEstimation, WeightsCanSingleOutOneSample) {
  // test the weights have effect by selecting a few states and ignoring others
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 6}, Vector2d{0.0, -3.0}},  //
      SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},  //
      SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},  //
  };
  const auto weights = std::vector{0.0, 1.0, 0.0, 1.0};
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{1.0, -2.0}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{Constants::pi() / 2}));
  ASSERT_NEAR(covariance(0, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.000, 0.001);
}

TEST_F(PoseCovarianceEstimation, RandomWalkWithSmoothRotationAndNonUniformWeights) {
  // test the mean and covariance estimations for states with random variations of translation, rotation and weights
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() * 0.1}, Vector2d{0.0, -2.0}},  //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{1.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.3}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{3.0, 2.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.2}, Vector2d{1.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.3}, Vector2d{2.0, -2.0}},  //
      SE2d{SO2d{Constants::pi() * 0.4}, Vector2d{3.0, -1.0}},  //
      SE2d{SO2d{Constants::pi() * 0.5}, Vector2d{2.0, 1.0}},   //
      SE2d{SO2d{Constants::pi() * 0.4}, Vector2d{1.0, 2.0}},   //
  };
  const auto weights = std::vector{0.1, 0.4, 0.7, 0.1, 0.9, 0.2, 0.2, 0.4, 0.1, 0.4};
  const auto [pose, covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose.translation(), Vector2Eq(Vector2d{1.800, 0.3143}));
  ASSERT_THAT(pose.so2(), SO2Eq(SO2d{0.8687}));
  ASSERT_NEAR(covariance(0, 0), 0.5946, 0.001);
  ASSERT_NEAR(covariance(0, 1), 0.0743, 0.001);
  ASSERT_NEAR(covariance(0, 2), 0.0000, 0.001);
  ASSERT_NEAR(covariance(1, 0), 0.0743, 0.001);
  ASSERT_NEAR(covariance(1, 1), 1.8764, 0.001);
  ASSERT_NEAR(covariance(1, 2), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 0), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 1), 0.0000, 0.001);
  ASSERT_NEAR(covariance(2, 2), 0.0855, 0.001);
}

template <class Mixin>
class MockMixin : public Mixin {
 public:
  MOCK_METHOD(const std::vector<Sophus::SE2d>&, states, (), (const));
  MOCK_METHOD(const std::vector<double>&, weights, (), (const));
};

TEST(SimpleStateEstimator, SimpleStateEstimatorUsesUniformWeights) {
  // test that the simple state estimator uses uniform weights for mean and covariance estimation
  using UUT = ciabatta::mixin<
      beluga::SimpleStateEstimator2d, MockMixin, ciabatta::provides<beluga::EstimationInterface2d>::mixin>;
  const auto mixin = UUT{};
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 6}, Vector2d{0.0, -3.0}}, SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},
      SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, -1.0}}, SE2d{SO2d{0.0}, Vector2d{3.0, 0.0}}};
  const auto weights = std::vector(states.size(), 1.0);
  EXPECT_CALL(mixin, states).WillRepeatedly(ReturnRef(states));
  EXPECT_CALL(mixin, weights).Times(0);
  const auto [pose, covariance] = mixin.estimate();
  const auto [expected_pose, expected_covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose, SE2Eq(expected_pose));
  ASSERT_NEAR(covariance(0, 0), expected_covariance(0, 0), 0.001);
  ASSERT_NEAR(covariance(0, 1), expected_covariance(0, 1), 0.001);
  ASSERT_NEAR(covariance(0, 2), expected_covariance(0, 2), 0.001);
  ASSERT_NEAR(covariance(1, 0), expected_covariance(1, 0), 0.001);
  ASSERT_NEAR(covariance(1, 1), expected_covariance(1, 1), 0.001);
  ASSERT_NEAR(covariance(1, 2), expected_covariance(1, 2), 0.001);
  ASSERT_NEAR(covariance(2, 0), expected_covariance(2, 0), 0.001);
  ASSERT_NEAR(covariance(2, 1), expected_covariance(2, 1), 0.001);
  ASSERT_NEAR(covariance(2, 2), expected_covariance(2, 2), 0.001);
}

TEST(WeightedStateEstimator, WeightedStateEstimatorDoesWeightedEstimation) {
  // test that the weighted state estimator uses the provided weights for mean and covariance estimation
  using UUT = ciabatta::mixin<
      beluga::WeightedStateEstimator2d, MockMixin, ciabatta::provides<beluga::EstimationInterface2d>::mixin>;
  const auto mixin = UUT{};
  const auto states = std::vector{
      SE2d{SO2d{Constants::pi() / 6}, Vector2d{0.0, -3.0}}, SE2d{SO2d{Constants::pi() / 2}, Vector2d{1.0, -2.0}},
      SE2d{SO2d{Constants::pi() / 3}, Vector2d{2.0, -1.0}}, SE2d{SO2d{0.0}, Vector2d{3.0, 0.0}}};
  const auto weights = std::vector{0.1, 0.5, 0.5, 0.1};
  EXPECT_CALL(mixin, states).WillRepeatedly(ReturnRef(states));
  EXPECT_CALL(mixin, weights).WillRepeatedly(ReturnRef(weights));
  const auto [pose, covariance] = mixin.estimate();
  const auto [expected_pose, expected_covariance] = beluga::estimate(states, weights);
  ASSERT_THAT(pose, SE2Eq(expected_pose));
  ASSERT_NEAR(covariance(0, 0), expected_covariance(0, 0), 0.001);
  ASSERT_NEAR(covariance(0, 1), expected_covariance(0, 1), 0.001);
  ASSERT_NEAR(covariance(0, 2), expected_covariance(0, 2), 0.001);
  ASSERT_NEAR(covariance(1, 0), expected_covariance(1, 0), 0.001);
  ASSERT_NEAR(covariance(1, 1), expected_covariance(1, 1), 0.001);
  ASSERT_NEAR(covariance(1, 2), expected_covariance(1, 2), 0.001);
  ASSERT_NEAR(covariance(2, 0), expected_covariance(2, 0), 0.001);
  ASSERT_NEAR(covariance(2, 1), expected_covariance(2, 1), 0.001);
  ASSERT_NEAR(covariance(2, 2), expected_covariance(2, 2), 0.001);
}

}  // namespace
