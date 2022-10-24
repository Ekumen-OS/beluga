#include <gtest/gtest.h>

#include <beluga/particle_filter.h>

namespace {

TEST(ParticleFilter, CanBeInstantiated) {
  [[maybe_unused]] auto filter = beluga::core::ParticleFilter{};
}

}  // namespace
