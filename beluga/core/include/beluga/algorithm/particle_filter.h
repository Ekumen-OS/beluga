#pragma once

#include <range/v3/algorithm/copy.hpp>
#include <range/v3/algorithm/transform.hpp>

#include <beluga/algorithm/sampling.h>
#include <beluga/tuple_vector.h>
#include <beluga/type_traits.h>
#include <ciabatta/ciabatta.h>

namespace beluga {

template <class Mixin, class Container>
struct ParticleFilter : public Mixin {
 public:
  template <class... Args>
  explicit ParticleFilter(Args&&... args) : Mixin(std::forward<Args>(args)...), particles_{this->self().max_samples()} {
    using particle_type = typename Container::value_type;
    auto first = std::begin(views::all(particles_));
    auto last =
        ranges::copy(this->self().template generate_samples<particle_type>() | this->self().take_samples(), first).out;
    particles_.resize(std::distance(first, last));
  }

  auto particles() { return views::all(particles_); }

  void update() {
    sample();
    resample();
  }

 private:
  Container particles_;

  void sample() {
    auto&& states = views::states(particles_);
    ranges::transform(
        states, std::begin(states), [this](const auto& state) { return this->self().apply_motion(state); });
    ranges::transform(states, std::begin(views::weights(particles_)), [this](const auto& state) {
      return this->self().importance_weight(state);
    });
  }

  void resample() {
    auto new_particles = Container{this->self().max_samples()};
    auto first = std::begin(views::all(new_particles));
    auto last = ranges::copy(this->self().generate_samples(particles_) | this->self().take_samples(), first).out;
    new_particles.resize(std::distance(first, last));
    particles_ = std::move(new_particles);
  }
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
                 ciabatta::curry<ParticleFilter, Container>::template mixin,
                 ciabatta::curry<NaiveGeneration>::template mixin,
                 ciabatta::curry<FixedResampling>::template mixin,
                 MotionModel,
                 SensorModel> {
  using ciabatta::mixin<
      MCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<ParticleFilter, Container>::template mixin,
      ciabatta::curry<NaiveGeneration>::template mixin,
      ciabatta::curry<FixedResampling>::template mixin,
      // TODO(nahuel): Add estimate mixin, which given a set of particles
      // provides the interface to obtain the best estimate of the state
      // together with a covariance.
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
                  ciabatta::curry<ParticleFilter, Container>::template mixin,
                  ciabatta::curry<AdaptiveGeneration>::template mixin,
                  ciabatta::curry<KldResampling>::template mixin,
                  MotionModel,
                  SensorModel> {
  using ciabatta::mixin<
      AMCL<MotionModel, SensorModel, State, Container>,
      ciabatta::curry<ParticleFilter, Container>::template mixin,
      ciabatta::curry<AdaptiveGeneration>::template mixin,
      ciabatta::curry<KldResampling>::template mixin,
      // TODO(nahuel): Add estimate mixin, which given a set of particles
      // provides the interface to obtain the best estimate of the state
      // together with a covariance.
      MotionModel,
      SensorModel>::mixin;
};

}  // namespace beluga
