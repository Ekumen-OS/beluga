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

/**
 * \file
 * \brief Implementation of storage policies.
 */

namespace beluga {

/**
 * \page ParticlePage Beluga named requirements: Particle
 * What an implementation of a particle in Beluga should provide.
 *
 * \section ParticleRequirements Requirements
 * `T` is a `Particle` if given:
 * - An instance `p` of `T`.
 * - A possibly const instance `cp` of `T`.
 *
 * The following is satisfied:
 * - \c particle_traits<T>::state_type is a valid type.
 * - \c particle_traits<T>::state(cp) returns an instance of \c particle_traits<T>::state_type.
 * - Given `s` an instance of \c particle_traits<T>::state_type, \c particle_traits<T>::state(p) = `s`
 *   is a valid expression and assigns the state `s` to the particle `p`. \n
 *   i.e. after the assignment `s` == \c particle_traits<T>::state(p) is `true`.
 * - \c particle_traits<T>::weight_type is a valid arithmetic type (that is, an integral
 *   type or a floating-point type).
 * - \c particle_traits<T>::weight(cp) is a valid expression and the return type is
 *   \c particle_traits<T>::weight_type.
 * - Given `w` an instance of \c particle_traits<T>::weight_type, \c particle_traits<T>::weight(p) = `w`
 *   is a valid expression and assigns the weight `w` to the particle `p`. \n
 *   i.e. after the assignment `w` == \c particle_traits<T>::weight(p) is `true`.
 */

/**
 * \page ParticleContainerPage Beluga named requirements: ParticleContainer
 * What an implementation of a particle container in Beluga should provide.
 *
 * \section ParticleContainerRequirements Requirements
 * `T` is a `ParticleContainer` if:
 * - `T` satisfies [C++ named requirements: Container](https://en.cppreference.com/w/cpp/named_req/Container).
 * - `T::value_type` satisfies \ref ParticlePage.
 */

/**
 * \page StoragePolicyPage Beluga named requirements: StoragePolicy
 * Base requirements of a storage policy in beluga.
 *
 * \section StoragePolicyRequirements Requirements
 * `P` is a `StoragePolicy` if given
 * - `T`, the type named by `P::particle_type`.
 * - `S`, the type named by `T::state_type`.
 * - `W`, the type named by `T::weight_type`.
 * - `V`, a range view type whose elements are of the same type as `T`.
 * - `p`, a value of type `P`.
 * - `cp`, a possibly const value of type `P`.
 * - `v`, possibly const value of type `V`.
 *
 * The following is satisfied:
 * - `cp.particles()` is valid and returns a range view whose value type is `T`.
 * - `cp.states()` is valid and returns a range view whose value type is `S`.
 * - `cp.weights()` is valid and returns a range view whose value type is `W`.
 * - `p.initialize_particles(v)` is valid and initializes the particle set with the given range.
 *
 * \section StoragePolicyLinks See also
 * - beluga::StoragePolicy
 */

/// Pure abstract class representing the storage interface.
/**
 * \tparam State State type of a particle.
 * \tparam Weight Weight type of a particle.
 */
template <class State, class Weight>
struct StorageInterface {
  /// State type of a particle.
  using state_type = State;
  /// Weight type of a particle.
  using weight_type = Weight;
  /// Input view type of particle states.
  using input_view_type = ranges::any_view<state_type>;
  /// Output view type of particle states.
  using output_view_type = ranges::any_view<state_type, ranges::category::random_access | ranges::category::sized>;
  /// Output view type of particle weights.
  using weights_view_type = ranges::any_view<weight_type, ranges::category::random_access | ranges::category::sized>;

  /// Virtual destructor.
  virtual ~StorageInterface() = default;

  /// Initializes the particle set from an input view of states.
  /**
   * \param input Type-erased range view whose value type is `state_type`.
   */
  virtual void initialize_states(input_view_type input) = 0;

  /// Returns the number of particles in the filter.
  [[nodiscard]] virtual std::size_t particle_count() const = 0;

  /// Returns a type-erased view of the particle states.
  [[nodiscard]] virtual output_view_type states_view() const = 0;

  /// Returns a type-erased view of the particle weights.
  [[nodiscard]] virtual weights_view_type weights_view() const = 0;
};

/// Configurable storage policy.
/**
 * This class implements StorageInterface and satisfies \ref StoragePolicyPage.
 *
 * \tparam Mixin The mixed-in type. An instance `m` of `Mixin` must provide:
 * - A `max_samples()` method that satisfies the requirements specified in \ref LimiterPage.
 * \tparam Container The particle container type. It must satisfy \ref ParticleContainerPage.
 * \tparam Particle The particle type, deduced from the container value type.
 * \tparam State The state type of a particle.
 * \tparam Weight The weigth type of a particle.
 */
template <
    class Mixin,
    class Container,
    class Particle = typename Container::value_type,
    class State = typename particle_traits<Particle>::state_type,
    class Weight = typename particle_traits<Particle>::weight_type>
class StoragePolicy : public Mixin {
 public:
  /// Type of a particle.
  using particle_type = Particle;
  /// State type of a particle.
  using state_type = State;
  /// Weight type of a particle.
  using weight_type = Weight;

  /// Input view type of particle states.
  using input_view_type = ranges::any_view<state_type>;
  /// Output view type of particle states.
  using output_view_type = ranges::any_view<state_type, ranges::category::random_access | ranges::category::sized>;
  /// Output view type of particle weights.
  using weights_view_type = ranges::any_view<weight_type, ranges::category::random_access | ranges::category::sized>;

  /// Constructs a StoragePolicy instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit StoragePolicy(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// Initializes the particle set from an input view.
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
    particles_[1].resize(size);
    const auto first = std::begin(views::all(particles_[1]));
    const auto last = ranges::copy(input | ranges::views::take(size), first).out;
    particles_[1].resize(static_cast<std::size_t>(std::distance(first, last)));
    std::swap(particles_[0], particles_[1]);
  }

  /// \copydoc StorageInterface::initialize_states()
  void initialize_states(input_view_type input) final {
    initialize_particles(input | ranges::views::transform(beluga::make_from_state<particle_type>));
  }

  /// \copydoc StorageInterface::particle_count()
  [[nodiscard]] std::size_t particle_count() const final { return particles_[0].size(); }

  /// \copydoc StorageInterface::states_view()
  [[nodiscard]] output_view_type states_view() const final { return this->states(); }

  /// \copydoc StorageInterface::weights_view()
  [[nodiscard]] weights_view_type weights_view() const final { return this->weights(); }

  /// Returns a view of the particles container.
  [[nodiscard]] auto particles() { return views::all(particles_[0]); }
  /// Returns a const view of the particles container.
  [[nodiscard]] auto particles() const { return views::all(particles_[0]) | ranges::views::const_; }

  /// Returns a view of the particles states.
  [[nodiscard]] auto states() { return views::states(particles_[0]); }
  /// Returns a const view of the particles states.
  [[nodiscard]] auto states() const { return views::states(particles_[0]) | ranges::views::const_; }

  /// Returns a view of the particles weight.
  [[nodiscard]] auto weights() { return views::weights(particles_[0]); }
  /// Returns a const view of the particles weight.
  [[nodiscard]] auto weights() const { return views::weights(particles_[0]) | ranges::views::const_; }

 private:
  Container particles_[2];
};

/// A storage policy that implements a structure of arrays layout.
template <class Mixin, class... Types>
using StructureOfArrays = StoragePolicy<Mixin, TupleOfVectors<Types...>>;

/// A storage policy that implements an array of structures layout.
template <class Mixin, class... Types>
using ArrayOfStructures = StoragePolicy<Mixin, VectorOfTuples<Types...>>;

}  // namespace beluga

#endif
