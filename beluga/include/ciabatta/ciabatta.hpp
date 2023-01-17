// Copyright 2019 Gašper Ažman
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

/*
 * CHANGELOG:
 * - Add explicit keyword to mixin constructor and remove
 *   default constructor and assignment operators.
 * - Remove macro usage.
 * - Invert the argument order on the curry struct template to
 *   support default template parameters.
 */

#ifndef CIABATTA_CIABATTA_HPP
#define CIABATTA_CIABATTA_HPP

/// \cond ciabatta

namespace ciabatta {

template <typename MostDerived>
struct ciabatta_top { /* not a mixin */
  using self_type = MostDerived;
  decltype(auto) self() & { return static_cast<self_type&>(*this); }
  decltype(auto) self() && { return static_cast<self_type&&>(*this); }
  decltype(auto) self() const& { return static_cast<self_type const&>(*this); }
  decltype(auto) self() const&& { return static_cast<self_type const&&>(*this); }
};

struct deferred {
  deferred() = delete;
};

namespace detail {

template <typename Concrete, template <class> class H, template <class> class... Tail>
struct chain_inherit {
  using result = typename chain_inherit<Concrete, Tail...>::type;
  using type = H<result>;
};

template <typename Concrete, template <class> class H>
struct chain_inherit<Concrete, H> {
  using type = H<Concrete>;
};

template <typename Concrete, template <class> class... Mixins>
using mixin_impl = typename chain_inherit<ciabatta_top<Concrete>, Mixins...>::type;

}  // namespace detail

template <typename Concrete, template <class> class... Mixins>
struct mixin : ::ciabatta::detail::mixin_impl<Concrete, Mixins...> {
  template <typename... Rest>
  constexpr explicit mixin(Rest&&... rest)
      : ::ciabatta::detail::mixin_impl<Concrete, Mixins...>(static_cast<decltype(rest)>(rest)...) {}
};

template <template <class...> class Mixin, typename... Args>
struct curry {
  template <typename Base>
  using mixin = Mixin<Base, Args...>;
};

namespace mixins {

template <typename Interface, typename Base = ::ciabatta::deferred>
struct provides : Base, Interface {
  template <typename B>
  using mixin = typename curry<provides, Interface>::template mixin<B>;

  template <typename... Args>
  constexpr explicit provides(Args&&... args) : Base(static_cast<decltype(args)>(args)...) {}
};

}  // namespace mixins

}  // namespace ciabatta

/// \endcond

#endif
