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

#ifndef BELUGA_STORAGE_HPP
#define BELUGA_STORAGE_HPP

#include <execution>
#include <utility>

#include <beluga/tuple_vector.hpp>
#include <beluga/type_traits.hpp>
#include <range/v3/algorithm/copy.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/const.hpp>
#include <range/v3/view/take.hpp>

namespace beluga {

template <class State>
struct StorageInterface {
  using state_type = State;
  using input_view = ranges::any_view<state_type>;
  using output_view = ranges::any_view<state_type, ranges::category::random_access | ranges::category::sized>;
  virtual ~StorageInterface() = default;
  virtual void initialize_states(input_view) = 0;
  [[nodiscard]] virtual output_view states_view() const = 0;
};

template <
    class Mixin,
    class Container,
    class Particle = typename Container::value_type,
    class State = typename particle_traits<Particle>::state_type>
class StoragePolicy : public Mixin {
 public:
  using particle_type = Particle;
  using state_type = State;

  using input_view = ranges::any_view<state_type>;
  using output_view = ranges::any_view<state_type, ranges::category::random_access | ranges::category::sized>;

  template <class... Args>
  explicit StoragePolicy(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Initializes a new particle container from an input view.
  /**
   * The size of the container will be at most `max_samples`, determined by the ParticleResampling
   * named requirements.
   *
   * \param input A particle range view to initialize the container.
   */
  template <class Range>
  void initialize_particles(Range&& input) {
    static_assert(std::is_same_v<particle_type, ranges::range_value_t<Range>>, "Invalid value type");
    const std::size_t size = this->self().max_samples();
    particles_.resize(size);
    const auto first = std::begin(views::all(particles_));
    const auto last = ranges::copy(input | ranges::views::take(size), first).out;
    particles_.resize(std::distance(first, last));
  }

  void initialize_states(input_view input) final {
    initialize_particles(input | ranges::views::transform(beluga::make_from_state<particle_type>));
  }

  [[nodiscard]] output_view states_view() const final { return this->states(); }

  /// Returns a view of the particles container.
  [[nodiscard]] auto particles() { return views::all(particles_); }
  /// Returns a const view of the particles container.
  [[nodiscard]] auto particles() const { return views::all(particles_) | ranges::views::const_; }

  /// Returns a view of the particles states.
  [[nodiscard]] auto states() { return views::states(particles_); }
  /// Returns a const view of the particles states.
  [[nodiscard]] auto states() const { return views::states(particles_) | ranges::views::const_; }

  /// Returns a view of the particles weight.
  [[nodiscard]] auto weights() { return views::weights(particles_); }
  /// Returns a const view of the particles weight.
  [[nodiscard]] auto weights() const { return views::weights(particles_) | ranges::views::const_; }

 private:
  Container particles_;
};

template <class Mixin, class... Types>
using StructureOfArrays = StoragePolicy<Mixin, TupleVector<std::tuple<Types...>>>;

template <class Mixin, class... Types>
using ArrayOfStructures = StoragePolicy<Mixin, std::vector<std::tuple<Types...>>>;

}  // namespace beluga

#endif
