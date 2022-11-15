#include <gtest/gtest.h>

#include <beluga/algorithm/particle_filter.h>

namespace beluga {

template <>
struct spatial_hash<double, void> {
 public:
  constexpr std::size_t operator()(double value, double resolution = 1.) const {
    return spatial_hash<std::tuple<double>>{}(std::make_tuple(value), resolution);
  }
};

}  // namespace beluga

namespace {

template <class Mixin>
class MockMotionModel : public Mixin {
 public:
  template <class... Args>
  explicit MockMotionModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] double apply_motion(double state) { return state; }
};

template <class Mixin>
class MockSensorModel : public Mixin {
 public:
  template <class... Args>
  explicit MockSensorModel(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  [[nodiscard]] double importance_weight(double) { return 1.; }

  template <class Generator>
  [[nodiscard]] double generate_random_state(Generator&) {
    return 0.;
  }
};

TEST(MCL, InitializeFilter) {
  auto filter =
      beluga::MCL<MockMotionModel, MockSensorModel, double>{beluga::FixedResamplingParam{.max_samples = 1'000}};
  ASSERT_EQ(filter.particles().size(), 1'000);
}

TEST(AMCL, InitializeFilter) {
  auto filter = beluga::AMCL<MockMotionModel, MockSensorModel, double>{
      beluga::AdaptiveGenerationParam{.alpha_slow = 0.001, .alpha_fast = 0.1},
      beluga::KldResamplingParam{
          .min_samples = 1'000, .max_samples = 2'000, .spatial_resolution = 1., .kld_epsilon = 0.05, .kld_z = 3.}};
  ASSERT_GE(filter.particles().size(), 1'000);
}

}  // namespace
