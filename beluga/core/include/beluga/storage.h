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

  static auto particle_view(container_type& container) {
    return container | ranges::views::all | ranges::views::common;
  }

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
  template <class... Types>
  class Container {
   public:
    friend struct StructureOfArrays;

    using value_type = std::tuple<Types...>;
    using size_type = std::size_t;

    [[nodiscard]] constexpr bool empty() const noexcept { return std::get<0>(particles_).empty(); }

    [[nodiscard]] constexpr size_type size() const noexcept { return std::get<0>(particles_).size(); }

    constexpr void clear() noexcept {
      std::apply([](auto&&... containers) { (containers.clear(), ...); }, particles_);
    }

    constexpr void reserve(size_type new_cap) {
      std::apply([new_cap](auto&&... containers) { (containers.reserve(new_cap), ...); }, particles_);
    }

    constexpr void resize(size_type count) {
      std::apply([count](auto&&... containers) { (containers.resize(count), ...); }, particles_);
    }

    constexpr void push_back(value_type&& value) {
      push_back_impl(std::move(value), std::make_integer_sequence<std::size_t, std::tuple_size_v<value_type>>());
    }

    constexpr void push_back(const value_type& value) {
      push_back_impl(value, std::make_integer_sequence<std::size_t, std::tuple_size_v<value_type>>());
    }

   private:
    std::tuple<InternalContainer<Types>...> particles_;

    template <typename T, std::size_t... Ids>
    constexpr void push_back_impl(T&& value, std::integer_sequence<std::size_t, Ids...>) {
      (std::get<Ids>(particles_).push_back(std::get<Ids>(std::forward<T>(value))), ...);
    }
  };

  using state_type = StateType;
  using container_type = Container<StateType, double, std::size_t>;

  struct Particle {
    StateType state;
    double weight;
    std::size_t cluster;

    using tuple_type = typename container_type::value_type;

    /* implicit */ operator tuple_type() & { return tuple_type{state, weight, cluster}; }              // NOLINT
    /* implicit */ operator tuple_type() && { return tuple_type{std::move(state), weight, cluster}; }  // NOLINT
  };

  using particle_type = typename container_type::value_type;

  static auto particle_view(container_type& container) {
    return ranges::views::zip(
               std::get<0>(container.particles_), std::get<1>(container.particles_),
               std::get<2>(container.particles_)) |
           ranges::views::common;
  }

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
