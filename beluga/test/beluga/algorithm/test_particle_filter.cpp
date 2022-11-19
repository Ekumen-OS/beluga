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
  constexpr std::size_t kMaxSamples = 1'000;
  auto filter = beluga::MCL<MockMotionModel, MockSensorModel, double>{beluga::FixedResamplingParam{kMaxSamples}};
  ASSERT_EQ(filter.particles().size(), kMaxSamples);
}

TEST(AMCL, InitializeFilter) {
  constexpr double kAlphaSlow = 0.001;
  constexpr double kAlphaFast = 0.1;
  constexpr std::size_t kMinSamples = 2'000;
  constexpr std::size_t kMaxSamples = 2'000;
  constexpr double kSpatialResolution = 1.;
  constexpr double kKldEpsilon = 0.05;
  constexpr double kKldZ = 3.;
  auto filter = beluga::AMCL<MockMotionModel, MockSensorModel, double>{
      beluga::AdaptiveGenerationParam{kAlphaSlow, kAlphaFast},
      beluga::KldResamplingParam{kMinSamples, kMaxSamples, kSpatialResolution, kKldEpsilon, kKldZ}};
  ASSERT_GE(filter.particles().size(), kMaxSamples);
}

}  // namespace
