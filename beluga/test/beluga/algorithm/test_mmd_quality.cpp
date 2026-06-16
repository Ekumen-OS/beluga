// Copyright 2024 Ekumen, Inc.
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

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <sophus/se2.hpp>

#include <beluga/algorithm/mmd_quality.hpp>
#include <beluga/views/particles.hpp>

namespace {

// Fixed seed for deterministic tests
auto make_generator() {
  return std::mt19937{42};
}

// Default reference covariance (matching expected stddev params)
auto make_ref_cov() {
  Sophus::Matrix3d cov = Sophus::Matrix3d::Zero();
  cov(0, 0) = 0.1 * 0.1;    // expected_pose_x_stddev^2
  cov(1, 1) = 0.1 * 0.1;    // expected_pose_y_stddev^2
  cov(2, 2) = 0.05 * 0.05;  // expected_pose_yaw_stddev^2
  return cov;
}

// Default kernel config (matching defaults in MmdKernelConfig)
auto make_kernel_config() {
  return beluga::MmdKernelConfig{};
}

// Helper: create a vector of particles with uniform weight.
auto make_uniform_particles(std::size_t count, const Sophus::SE2d& pose, double noise_stddev) {
  struct Particle {
    Sophus::SE2d state;
    double weight;
  };
  auto rng = std::mt19937{1};
  auto dist = std::normal_distribution<double>{0.0, noise_stddev};
  std::vector<Particle> particles;
  particles.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double dx = dist(rng);
    const double dy = dist(rng);
    const double dt = dist(rng);
    const auto state = Sophus::SE2d{pose.so2() * Sophus::SO2d{dt}, pose.translation() + Eigen::Vector2d{dx, dy}};
    particles.push_back(Particle{state, 1.0});
  }
  return particles;
}

// ===========================
// Kernel unit tests
// ===========================

TEST(MmdQualityTest, KernelSelfIsOne) {
  const auto config = make_kernel_config();
  const beluga::detail::mmd_product_kernel_fn kernel;

  const Eigen::Vector3d a{1.0, 2.0, 0.5};
  ASSERT_DOUBLE_EQ(kernel(a, a, config), 1.0);
}

TEST(MmdQualityTest, KernelIsBoundedByOne) {
  const auto config = make_kernel_config();
  const beluga::detail::mmd_product_kernel_fn kernel;

  const Eigen::Vector3d a{0.0, 0.0, 0.0};
  const Eigen::Vector3d b{10.0, 10.0, 1.0};

  const double val = kernel(a, b, config);
  ASSERT_LE(val, 1.0);
  ASSERT_GE(val, 0.0);
}

TEST(MmdQualityTest, KernelSymmetric) {
  const auto config = make_kernel_config();
  const beluga::detail::mmd_product_kernel_fn kernel;

  const Eigen::Vector3d a{0.0, 0.0, 0.0};
  const Eigen::Vector3d b{1.0, 2.0, 0.3};

  ASSERT_DOUBLE_EQ(kernel(a, b, config), kernel(b, a, config));
}

// ===========================
// MMD² computation tests
// ===========================

TEST(MmdQualityTest, MmdZeroForIdenticalSamples) {
  const auto config = make_kernel_config();
  const Eigen::Vector3d a{0.0, 0.0, 0.0};
  const Eigen::Vector3d b{1.0, 0.0, 0.0};
  const Eigen::Vector3d c{0.0, 1.0, 0.0};
  const Eigen::Vector3d d{0.0, 0.0, 0.5};

  const std::vector<Eigen::Vector3d> X = {a, b, c, d};
  const std::vector<Eigen::Vector3d> Y = {a, b, c, d};

  const double mmd2 = beluga::detail::compute_paired_mmd2(X, Y, config);
  ASSERT_NEAR(mmd2, 0.0, 1e-10);
}

TEST(MmdQualityTest, MmdPositiveForDifferentSamples) {
  const auto config = make_kernel_config();
  const Eigen::Vector3d a1{0.0, 0.0, 0.0};
  const Eigen::Vector3d a2{0.0, 0.0, 0.0};
  const Eigen::Vector3d b1{10.0, 10.0, 1.0};
  const Eigen::Vector3d b2{10.0, 10.0, 1.0};

  const std::vector<Eigen::Vector3d> X = {a1, a2};
  const std::vector<Eigen::Vector3d> Y = {b1, b2};

  const double mmd2 = beluga::detail::compute_paired_mmd2(X, Y, config);
  ASSERT_GT(mmd2, 0.0);
}

TEST(MmdQualityTest, MmdZeroForFewerThanTwoSamples) {
  const auto config = make_kernel_config();
  const Eigen::Vector3d a{0.0, 0.0, 0.0};

  const std::vector<Eigen::Vector3d> X = {a};
  const std::vector<Eigen::Vector3d> Y = {a};

  const double mmd2 = beluga::detail::compute_paired_mmd2(X, Y, config);
  ASSERT_DOUBLE_EQ(mmd2, 0.0);
}

// ===========================
// Parametric mode tests
// ===========================

TEST(MmdQualityTest, ParametricIdenticalDistributions) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto mean = Sophus::SE2d{};

  // sample_count before config in the new API
  const double quality = beluga::mmd_quality(mean, ref_cov, mean, ref_cov, 8, config, gen);

  EXPECT_GT(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

TEST(MmdQualityTest, ParametricCloseVsFar) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};
  const auto far_pose = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{100.0, 100.0}};

  const double close_q = beluga::mmd_quality(origin, ref_cov, origin, ref_cov, 20, config, gen);
  const double far_q = beluga::mmd_quality(far_pose, ref_cov, origin, ref_cov, 20, config, gen);

  EXPECT_GT(close_q, far_q);
  EXPECT_GE(close_q, 0.0);
  EXPECT_LE(close_q, 1.0);
  EXPECT_GE(far_q, 0.0);
  EXPECT_LE(far_q, 1.0);
}

TEST(MmdQualityTest, ParametricPValueInZeroOne) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto mean = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}};
  const auto far_mean = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{100.0, 100.0}};

  // sample_count before config
  const double q1 = beluga::mmd_quality(mean, ref_cov, mean, ref_cov, 20, config, gen);
  EXPECT_GE(q1, 0.0);
  EXPECT_LE(q1, 1.0);

  const double q2 = beluga::mmd_quality(far_mean, ref_cov, mean, ref_cov, 20, config, gen);
  EXPECT_GE(q2, 0.0);
  EXPECT_LE(q2, 1.0);
}

// ===========================
// Particle mode tests
// ===========================

TEST(MmdQualityTest, ParticleModeEmptyReturnsOne) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  struct EmptyParticle {
    Sophus::SE2d state;
    double weight;
  };
  const std::vector<EmptyParticle> empty_particles;

  const auto ref_mean = Sophus::SE2d{};
  const double quality = beluga::mmd_quality(empty_particles, ref_mean, ref_cov, 100, config, gen);

  ASSERT_DOUBLE_EQ(quality, 1.0);
}

TEST(MmdQualityTest, ParticleModeSimilarToPosterior) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto mean = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}};
  const auto particles = make_uniform_particles(200, mean, 0.01);

  const double quality = beluga::mmd_quality(particles, mean, ref_cov, 100, config, gen);

  EXPECT_GT(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

TEST(MmdQualityTest, ParticleModeCloseVsFar) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};
  const auto far_pose = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{10.0, 10.0}};

  const auto close_particles = make_uniform_particles(100, origin, 0.01);
  const auto far_particles = make_uniform_particles(100, far_pose, 0.01);

  const double close_q = beluga::mmd_quality(close_particles, origin, ref_cov, 20, config, gen);
  const double far_q = beluga::mmd_quality(far_particles, origin, ref_cov, 20, config, gen);

  EXPECT_GT(close_q, far_q);
  EXPECT_GE(close_q, 0.0);
  EXPECT_LE(close_q, 1.0);
  EXPECT_GE(far_q, 0.0);
  EXPECT_LE(far_q, 1.0);
}

TEST(MmdQualityTest, ParticleModeZeroWeights) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  struct ZeroWeightParticle {
    Sophus::SE2d state;
    double weight;
  };
  std::vector<ZeroWeightParticle> particles = {
      {Sophus::SE2d{}, 0.0},
      {Sophus::SE2d{}, 0.0},
  };

  const double quality = beluga::mmd_quality(particles, Sophus::SE2d{}, ref_cov, 10, config, gen);
  EXPECT_GE(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

TEST(MmdQualityTest, ParametricMinSampleCount) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto mean = Sophus::SE2d{};
  // sample_count = 1 should be clamped to 2
  const double quality = beluga::mmd_quality(mean, ref_cov, mean, ref_cov, 1, config, gen);
  EXPECT_GE(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

TEST(MmdQualityTest, ParticleModeFewerParticlesThanSampleCount) {
  auto gen = make_generator();
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto mean = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{0.0, 0.0}};
  // Only 10 particles but request 100 samples (resamples with replacement)
  const auto particles = make_uniform_particles(10, mean, 0.01);

  const double quality = beluga::mmd_quality(particles, mean, ref_cov, 100, config, gen);

  EXPECT_GE(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

// ===========================
// Default PRNG tests (no explicit generator)
// ===========================

TEST(MmdQualityTest, ParametricDefaultPrng) {
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};
  const auto far_pose = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{100.0, 100.0}};

  // No explicit generator — uses ranges default PRNG
  const double close_q = beluga::mmd_quality(origin, ref_cov, origin, ref_cov, 20, config);
  const double far_q = beluga::mmd_quality(far_pose, ref_cov, origin, ref_cov, 20, config);

  EXPECT_GT(close_q, far_q);
  EXPECT_GE(close_q, 0.0);
  EXPECT_LE(close_q, 1.0);
  EXPECT_GE(far_q, 0.0);
  EXPECT_LE(far_q, 1.0);
}

TEST(MmdQualityTest, ParticleModeDefaultPrng) {
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};
  const auto far_pose = Sophus::SE2d{Sophus::SO2d{0.0}, Eigen::Vector2d{10.0, 10.0}};

  const auto close_particles = make_uniform_particles(100, origin, 0.01);
  const auto far_particles = make_uniform_particles(100, far_pose, 0.01);

  // No explicit generator — uses ranges default PRNG
  const double close_q = beluga::mmd_quality(close_particles, origin, ref_cov, 20, config);
  const double far_q = beluga::mmd_quality(far_particles, origin, ref_cov, 20, config);

  EXPECT_GT(close_q, far_q);
  EXPECT_GE(close_q, 0.0);
  EXPECT_LE(close_q, 1.0);
  EXPECT_GE(far_q, 0.0);
  EXPECT_LE(far_q, 1.0);
}

TEST(MmdQualityTest, ParticleModeDefaultSampleCount) {
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};
  const auto particles = make_uniform_particles(50, origin, 0.01);

  // No sample_count provided — uses particle count (50)
  const double quality = beluga::mmd_quality(particles, origin, ref_cov, config);

  EXPECT_GE(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

TEST(MmdQualityTest, ParametricDefaultSampleCount) {
  const auto config = make_kernel_config();
  const auto ref_cov = make_ref_cov();

  const auto origin = Sophus::SE2d{};

  // No sample_count — uses default 500
  const double quality = beluga::mmd_quality(origin, ref_cov, origin, ref_cov, config);

  EXPECT_GE(quality, 0.0);
  EXPECT_LE(quality, 1.0);
}

}  // namespace
