// Copyright 2022 Ekumen, Inc.
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

#ifndef BELUGA_ALGORITHM_PARTICLE_FILTER_HPP
#define BELUGA_ALGORITHM_PARTICLE_FILTER_HPP

#include <execution>

#include <beluga/algorithm/estimation.hpp>
#include <beluga/algorithm/sampling.hpp>
#include <beluga/tuple_vector.hpp>
#include <beluga/type_traits.hpp>
#include <ciabatta/ciabatta.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/transform.hpp>
#include <range/v3/view/const.hpp>

namespace beluga {

template <class Mixin, class Container>
struct BootstrapParticleFilter : public Mixin {
 public:
  template <class... Args>
  explicit BootstrapParticleFilter(Args&&... args)
      : Mixin(std::forward<Args>(args)...), particles_{this->self().max_samples()} {
    using particle_type = typename Container::value_type;
    auto first = std::begin(views::all(particles_));
    auto last =
        ranges::copy(this->self().template generate_samples<particle_type>() | this->self().take_samples(), first).out;
    particles_.resize(std::distance(first, last));
  }

  auto particles() { return views::all(particles_); }
  auto particles() const { return views::all(particles_) | ranges::views::const_; }

  auto states() { return views::states(particles_); }
  auto states() const { return views::states(particles_) | ranges::views::const_; }

  auto weights() { return views::weights(particles_); }
  auto weights() const { return views::weights(particles_) | ranges::views::const_; }

  void update() {
    sample();
    importance_sample();
    resample();
  }

  void sample() {
    auto states = views::states(particles_);
    ranges::transform(
        states, std::begin(states), [this](const auto& state) { return this->self().apply_motion(state); });
  }

  void importance_sample() {
    const auto states = views::states(particles_);
    std::transform(
        std::execution::par, states.begin(), states.end(), views::weights(particles_).begin(),
        [this](const auto& state) { return this->self().importance_weight(state); });
  }

  void resample() {
    auto new_particles = Container{this->self().max_samples()};
    auto first = std::begin(views::all(new_particles));
    auto last = ranges::copy(this->self().generate_samples_from(particles_) | this->self().take_samples(), first).out;
    new_particles.resize(std::distance(first, last));
    particles_ = std::move(new_particles);
  }

 private:
  Container particles_;
};

template <
    template <class>
    class MotionModel,
    template <class>
    class SensorModel,
    class State,
    class Container = TupleVector<std::pair<State, double>>>
struct MCL : public ciabatta::mixin<
                 MCL<MotionModel, SensorModel, State, Container>,
                 ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
                 ciabatta::curry<BaselineGeneration>::template mixin,
                 ciabatta::curry<NaiveGeneration>::template mixin,
                 ciabatta::curry<FixedResampling>::template mixin,
                 ciabatta::curry<SimpleEstimation>::template mixin,
                 MotionModel,
                 SensorModel> {
  using ciabatta::mixin<
      MCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<NaiveGeneration>::template mixin,
      ciabatta::curry<FixedResampling>::template mixin,
      ciabatta::curry<SimpleEstimation>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

template <
    template <class>
    class MotionModel,
    template <class>
    class SensorModel,
    class State,
    class Container = TupleVector<std::tuple<State, double, std::size_t>>>
struct AMCL : public ciabatta::mixin<
                  AMCL<MotionModel, SensorModel, State, Container>,
                  ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
                  ciabatta::curry<BaselineGeneration>::template mixin,
                  ciabatta::curry<AdaptiveGeneration>::template mixin,
                  ciabatta::curry<KldResampling>::template mixin,
                  ciabatta::curry<SimpleEstimation>::template mixin,
                  MotionModel,
                  SensorModel> {
  using ciabatta::mixin<
      AMCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<BootstrapParticleFilter, Container>::template mixin,
      ciabatta::curry<BaselineGeneration>::template mixin,
      ciabatta::curry<AdaptiveGeneration>::template mixin,
      ciabatta::curry<KldResampling>::template mixin,
      ciabatta::curry<SimpleEstimation>::template mixin,
      MotionModel,
      SensorModel>::mixin;
};

}  // namespace beluga

#endif
