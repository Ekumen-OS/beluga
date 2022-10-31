#pragma once

#include <tuple>
#include <vector>

#include <range/v3/view.hpp>

namespace beluga::core::storage {

template <class T>
using Vector = std::vector<T, std::allocator<T>>;

template <class StateType, template <class> class InternalContainer = Vector>
struct ArrayOfStructures {
  struct Particle {
    StateType state;
    double weight;
    std::size_t cluster;
  };

  using state_type = StateType;
  using particle_type = Particle;
  using container_type = InternalContainer<Particle>;

  static auto state_view(container_type& container) {
    return container | ranges::views::transform(&Particle::state) | ranges::views::common;
  }

  static auto weight_view(container_type& container) {
    return container | ranges::views::transform(&Particle::weight) | ranges::views::common;
  }

  static auto cluster_view(container_type& container) {
    return container | ranges::views::transform(&Particle::cluster) | ranges::views::common;
  }
};

template <class StateType, template <class> class InternalContainer = Vector>
struct StructureOfArrays {
  struct Particle {
    StateType state;
    double weight;
    std::size_t cluster;
  };

  using state_type = StateType;
  using particle_type = Particle;

  class Container {
   public:
    friend struct StructureOfArrays;

    using value_type = Particle;
    using size_type = std::size_t;

    [[nodiscard]] bool empty() const noexcept { return std::get<0>(particles_).empty(); }

    [[nodiscard]] size_type size() const noexcept { return std::get<0>(particles_).size(); }

    void clear() noexcept {
      std::apply([](auto&&... containers) { (containers.clear(), ...); }, particles_);
    }

    void reserve(size_type new_cap) {
      std::apply([new_cap](auto&&... containers) { (containers.reserve(new_cap), ...); }, particles_);
    }

    void resize(size_type count) {
      std::apply([count](auto&&... containers) { (containers.resize(count), ...); }, particles_);
    }

    void push_back(const value_type& value) {
      std::get<0>(particles_).push_back(value.state);
      std::get<1>(particles_).push_back(value.weight);
      std::get<2>(particles_).push_back(value.cluster);
    }

    void push_back(value_type&& value) {
      std::get<0>(particles_).push_back(std::move(value.state));
      std::get<1>(particles_).push_back(std::move(value.weight));
      std::get<2>(particles_).push_back(std::move(value.cluster));
    }

   private:
    std::tuple<InternalContainer<state_type>, InternalContainer<double>, InternalContainer<std::size_t>> particles_;
  };

  using container_type = Container;

  static auto state_view(container_type& container) {
    return std::get<0>(container.particles_) | ranges::views::all | ranges::views::common;
  }

  static auto weight_view(container_type& container) {
    return std::get<1>(container.particles_) | ranges::views::all | ranges::views::common;
  }

  static auto cluster_view(container_type& container) {
    return std::get<2>(container.particles_) | ranges::views::all | ranges::views::common;
  }
};

}  // namespace beluga::core::storage
