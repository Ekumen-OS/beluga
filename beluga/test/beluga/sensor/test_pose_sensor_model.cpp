#include <gtest/gtest.h>

// standard library
#include <cmath>

// external
#include <Eigen/Core>
#include <sophus/common.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>

// project
#include "beluga/sensor/pose_sensor_model.hpp"

namespace beluga {

namespace {

using Sensor2D = beluga::PoseSensorModel2d;
using Sensor3D = beluga::PoseSensorModel3d;

PoseSensorModelParam get_default_params() {
  PoseSensorModelParam params;
  params.min_weight = 1e-5;
  return params;
}

// Default diagonal covariances: sigma_trans = 1.0 m, sigma_rot = pi/4 rad.

Eigen::Matrix3d make_diagonal_covariance_2d(
    double sigma_x = 1.0,
    double sigma_y = 1.0,
    double sigma_theta = Sophus::Constants<double>::pi() / 4.0) {
  return Eigen::DiagonalMatrix<double, 3>{
      sigma_x * sigma_x, sigma_y * sigma_y, sigma_theta * sigma_theta};
}

Eigen::Matrix<double, 6, 6> make_diagonal_covariance_3d(
    double sigma_trans = 1.0,
    double sigma_rot = Sophus::Constants<double>::pi() / 4.0) {
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  cov.diagonal() << sigma_trans * sigma_trans,
      sigma_trans * sigma_trans,
      sigma_trans * sigma_trans,
      sigma_rot * sigma_rot,
      sigma_rot * sigma_rot,
      sigma_rot * sigma_rot;
  return cov;
}

// Typed helpers for constructing identity measurements and displaced particles.

template <typename SensorType>
typename SensorType::measurement_type make_identity_measurement();

template <>
PoseMeasurement2d make_identity_measurement<Sensor2D>() {
  return {Sophus::SE2d{}, make_diagonal_covariance_2d()};
}

template <>
PoseMeasurement3d make_identity_measurement<Sensor3D>() {
  return {Sophus::SE3d{}, make_diagonal_covariance_3d()};
}

template <typename SensorType>
typename SensorType::state_type make_x_displaced_particle(double dx);

template <>
Sophus::SE2d make_x_displaced_particle<Sensor2D>(double dx) {
  return Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{dx, 0.0}};
}

template <>
Sophus::SE3d make_x_displaced_particle<Sensor3D>(double dx) {
  return Sophus::SE3d{Sophus::SO3d{}, Eigen::Vector3d{dx, 0.0, 0.0}};
}

template <typename SensorType>
typename SensorType::state_type make_y_displaced_particle(double dy);

template <>
Sophus::SE2d make_y_displaced_particle<Sensor2D>(double dy) {
  return Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, dy}};
}

template <>
Sophus::SE3d make_y_displaced_particle<Sensor3D>(double dy) {
  return Sophus::SE3d{Sophus::SO3d{}, Eigen::Vector3d{0.0, dy, 0.0}};
}

template <typename SensorType>
typename SensorType::state_type make_yaw_rotated_particle(double dtheta);

template <>
Sophus::SE2d make_yaw_rotated_particle<Sensor2D>(double dtheta) {
  return Sophus::SE2d{Sophus::SO2d{dtheta}, Eigen::Vector2d::Zero()};
}

template <>
Sophus::SE3d make_yaw_rotated_particle<Sensor3D>(double dtheta) {
  return Sophus::SE3d{Sophus::SO3d::rotZ(dtheta), Eigen::Vector3d::Zero()};
}

// ─── Typed test suite ────────────────────────────────────────────────────────

template <typename T>
struct PoseSensorModelTests : public ::testing::Test {};

using PoseSensorModelTestsTypes = ::testing::Types<Sensor2D, Sensor3D>;

TYPED_TEST_SUITE(PoseSensorModelTests, PoseSensorModelTestsTypes, );

// Particle at the exact measured pose → weight = 1.0.
TYPED_TEST(PoseSensorModelTests, BullsEyeDetection) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  EXPECT_NEAR(1.0, fn(typename TypeParam::state_type{}), 1e-9);
}

// Particle displaced by exactly 1σ along x → mahal² = 1 → weight = exp(-0.5).
TYPED_TEST(PoseSensorModelTests, OneStdInTranslationX) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const auto particle = make_x_displaced_particle<TypeParam>(1.0);  // sigma_x = 1.0
  EXPECT_NEAR(std::exp(-0.5), fn(particle), 1e-4);
}

// Particle displaced by exactly 1σ along y → mahal² = 1 → weight = exp(-0.5).
TYPED_TEST(PoseSensorModelTests, OneStdInTranslationY) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const auto particle = make_y_displaced_particle<TypeParam>(1.0);  // sigma_y = 1.0
  EXPECT_NEAR(std::exp(-0.5), fn(particle), 1e-4);
}

// Particle rotated by exactly 1σ (pi/4 rad) around z → mahal² = 1 → weight = exp(-0.5).
// Uses zero translation to avoid Lie algebra Jacobian coupling.
TYPED_TEST(PoseSensorModelTests, OneStdInYaw) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const auto sigma_theta = Sophus::Constants<double>::pi() / 4.0;
  const auto particle = make_yaw_rotated_particle<TypeParam>(sigma_theta);
  EXPECT_NEAR(std::exp(-0.5), fn(particle), 1e-4);
}

// Particle with combined 0.5σ displacements in x, y and yaw.
// Building the particle from exp(tangent) makes the log map trivially recover
// the tangent vector, so expected mahalanobis² = 3 * 0.25 = 0.75 regardless of
// Jacobian coupling: weight = exp(-0.375).
TYPED_TEST(PoseSensorModelTests, HalfStdInXYAndYaw) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const double dx = 0.5;  // 0.5 * sigma_x
  const double dy = 0.5;  // 0.5 * sigma_y
  const double dtheta = Sophus::Constants<double>::pi() / 8.0;  // 0.5 * sigma_theta
  typename TypeParam::state_type particle;
  if constexpr (std::is_same_v<TypeParam, Sensor2D>) {
    particle = Sophus::SE2d::exp(Eigen::Vector3d{dx, dy, dtheta});
  } else {
    Eigen::Matrix<double, 6, 1> tangent;
    tangent << dx, dy, 0.0, 0.0, 0.0, dtheta;
    particle = Sophus::SE3d::exp(tangent);
  }
  EXPECT_NEAR(std::exp(-0.375), fn(particle), 1e-4);
}

// Particle displaced by 2σ along x → mahal² = 4 → weight = exp(-2).
TYPED_TEST(PoseSensorModelTests, TwoStdInTranslationX) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const auto particle = make_x_displaced_particle<TypeParam>(2.0);  // 2 * sigma_x
  EXPECT_NEAR(std::exp(-2.0), fn(particle), 1e-4);
}

// Particle very far from measurement → raw weight ≈ 0 → clamped to min_weight.
TYPED_TEST(PoseSensorModelTests, MinWeightFloor) {
  const auto model = TypeParam{get_default_params()};
  auto fn = model(make_identity_measurement<TypeParam>());
  const auto far_particle = make_x_displaced_particle<TypeParam>(1000.0);
  EXPECT_NEAR(get_default_params().min_weight, fn(far_particle), 1e-9);
}

// Measurement at a non-identity pose; particle at the same pose → weight = 1.0.
// Validates that the model uses pose.inverse() * state rather than assuming identity.
TYPED_TEST(PoseSensorModelTests, NonIdentityMeasurementBullsEye) {
  const auto model = TypeParam{get_default_params()};
  // Use a non-trivial measured pose with both translation and rotation.
  typename TypeParam::measurement_type meas;
  if constexpr (std::is_same_v<TypeParam, Sensor2D>) {
    meas = {Sophus::SE2d{Sophus::SO2d{Sophus::Constants<double>::pi() / 3.0}, Eigen::Vector2d{2.0, 5.0}},
            make_diagonal_covariance_2d()};
  } else {
    meas = {Sophus::SE3d{Sophus::SO3d::rotZ(Sophus::Constants<double>::pi() / 3.0), Eigen::Vector3d{2.0, 5.0, 0.0}},
            make_diagonal_covariance_3d()};
  }

  const auto measured_pose = meas.pose;
  auto fn = model(std::move(meas));
  // Particle exactly at the measured pose → error = 0 → weight = 1.0.
  EXPECT_NEAR(1.0, fn(measured_pose), 1e-9);
}

// ─── 2D-only tests ───────────────────────────────────────────────────────────

// Correlated XY covariance: verifies that off-diagonal terms are used.
// Cov = [[1, 0.5, 0], [0.5, 1, 0], [0, 0, (pi/4)²]]
// For error e = [1, 0, 0]:
//   2x2 block: [[1,0.5],[0.5,1]], det = 0.75, inv[0,0] = 4/3
//   mahal² = 1 * (4/3) * 1 = 4/3
//   weight = exp(-2/3) ≈ 0.5134
TEST(PoseSensorModel2dTests, OffDiagonalCovariance) {
  const auto model = PoseSensorModel2d{get_default_params()};
  Eigen::Matrix3d cov;
  // clang-format off
  cov << 1.0, 0.5, 0.0,
         0.5, 1.0, 0.0,
         0.0, 0.0, (Sophus::Constants<double>::pi() / 4.0) * (Sophus::Constants<double>::pi() / 4.0);
  // clang-format on
  PoseMeasurement2d meas{Sophus::SE2d{}, cov};
  auto fn = model(std::move(meas));
  const auto particle = Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1.0, 0.0}};
  EXPECT_NEAR(std::exp(-2.0 / 3.0), fn(particle), 1e-4);
}

// Custom min_weight parameter is respected.
TEST(PoseSensorModel2dTests, CustomMinWeight) {
  PoseSensorModelParam params;
  params.min_weight = 0.1;
  const auto model = PoseSensorModel2d{params};
  PoseMeasurement2d meas{Sophus::SE2d{}, make_diagonal_covariance_2d()};
  auto fn = model(std::move(meas));
  const auto far_particle = Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{1000.0, 0.0}};
  EXPECT_NEAR(0.1, fn(far_particle), 1e-9);
}

}  // namespace

}  // namespace beluga
