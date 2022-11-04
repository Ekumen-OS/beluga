#pragma once

#include <tuple>
#include <vector>

#include <range/v3/view/zip.hpp>

#include <beluga/type_traits.h>

namespace beluga {

template <template <class> class InternalContainer, class Tuple>
class TupleContainer;

template <template <class> class InternalContainer, class... Types>
class TupleContainer<InternalContainer, std::tuple<Types...>> {
 public:
  using value_type = std::tuple<Types...>;
  using size_type = std::size_t;

  constexpr TupleContainer() = default;

  explicit constexpr TupleContainer(size_type count) : sequences_{((void)sizeof(Types), count)...} {}

  [[nodiscard]] constexpr bool empty() const noexcept { return std::get<0>(sequences_).empty(); }

  [[nodiscard]] constexpr size_type size() const noexcept { return std::get<0>(sequences_).size(); }

  [[nodiscard]] constexpr size_type capacity() const noexcept { return std::get<0>(sequences_).capacity(); }

  constexpr void clear() noexcept {
    std::apply([](auto&&... containers) { (containers.clear(), ...); }, sequences_);
  }

  constexpr void reserve(size_type new_cap) {
    std::apply([new_cap](auto&&... containers) { (containers.reserve(new_cap), ...); }, sequences_);
  }

  constexpr void resize(size_type count) {
    std::apply([count](auto&&... containers) { (containers.resize(count), ...); }, sequences_);
  }

  constexpr void push_back(value_type&& value) {
    push_back_impl(std::move(value), std::make_index_sequence<sizeof...(Types)>());
  }

  constexpr void push_back(const value_type& value) {
    push_back_impl(value, std::make_index_sequence<sizeof...(Types)>());
  }

  constexpr auto view_all() {
    return std::apply([](auto&&... containers) { return ranges::views::zip(containers...); }, sequences_);
  }

 private:
  std::tuple<InternalContainer<Types>...> sequences_;

  template <typename T, std::size_t... Ids>
  constexpr void push_back_impl(T&& value, std::index_sequence<Ids...>) {
    (std::get<Ids>(sequences_).push_back(std::get<Ids>(std::forward<T>(value))), ...);
  }
};

template <template <class> class InternalContainer, class T>
struct container_traits<TupleContainer<InternalContainer, T>> {
  using type = TupleContainer<InternalContainer, T>;
  using value_type = typename type::value_type;
  using size_type = typename type::size_type;

  template <class U>
  static constexpr auto view_all(U&& container) {
    return container.view_all();
  }
};

template <class T>
using Vector = std::vector<T, std::allocator<T>>;

template <class Tuple>
using TupleVector = TupleContainer<Vector, Tuple>;

}  // namespace beluga
