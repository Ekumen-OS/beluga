// Copyright 2026 Ekumen, Inc.
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

#pragma once

/**
 * @file
 * @brief Includes a static-linkage singleton type registry and registration macros.
 */

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include <beluga_estimation/common/macros.hpp>
#include <beluga_estimation/common/result.hpp>

namespace beluga_estimation {

/// @brief Detection idiom: evaluates to `true` if `T::Make(Args...)` is a valid expression.
template <typename T, typename... Args>
using beluga_make_return_type = decltype(T::Make(std::declval<Args>()...));

namespace detail {
template <typename, template <typename...> class, typename...>
struct is_detected : std::false_type {};

template <template <typename...> class Op, typename... Args>
struct is_detected<std::void_t<Op<Args...>>, Op, Args...> : std::true_type {};
}  // namespace detail

/// @brief Evaluates to `true` if `Op<Args...>` is a valid expression.
template <template <typename...> class Op, typename... Args>
using is_detected = detail::is_detected<void, Op, Args...>;

/// @brief Evaluates to `true` if `T` provides a static `Make(Args...)`.
template <typename T, typename... Args>
inline constexpr bool has_make_v = is_detected<beluga_make_return_type, T, Args...>::value;

/**
 * @brief Customization point for a registry base type.
 *
 * @note Intentionally left undefined — use BELUGA_ESTIMATION_EXPORT_TYPE().
 *
 * @tparam Base The base type for which to define the factory signature.
 */
template <typename Base>
struct RegistryTraits;

/**
 * @brief Abstract source of type factories for a given base type.
 *
 * @tparam Base The base type whose derived-type factories are managed.
 * @tparam Args The argument types accepted by every registered factory.
 */
template <typename Base, typename... Args>
class RegistrySource {
 public:
  static constexpr int kDefaultPriority = 0;

  virtual ~RegistrySource() = default;

  /// @brief Higher values are queried first.
  virtual int GetPriority() const = 0;

  virtual bool HasType(std::string_view type_name) const = 0;

  /// @return An error with `Error::kFailure` if the type is not recognized.
  virtual Result_<std::unique_ptr<Base>> MakeInstance(std::string_view type_name, Args... args) const = 0;
};

namespace detail {

/**
 * @brief One registration entity per `BELUGA_ESTIMATION_REGISTER_TYPE()` call site.
 *
 * Specialised once per call site by the `BELUGA_ESTIMATION_REGISTER_TYPE()` macro. Because
 * the entity is `inline`, every translation unit that includes the same call
 * site refers to the same entity, so the registration side effect runs once per
 * linked image instead of once per translation unit.
 *
 * @note The specialization is emitted only when `BELUGA_ESTIMATION_LINK_STATIC_REGISTRY` is
 * defined; see @ref BELUGA_ESTIMATION_REGISTER_TYPE.
 *
 * @tparam Derived The concrete type being registered.
 * @tparam Base The base type whose registry receives the type.
 * @tparam SiteId A value unique per call site within a program — the line of
 * the macro invocation.
 */
template <class Derived, class Base, std::size_t SiteId>
inline const bool kTypeRegistration = false;

}  // namespace detail

/**
 * @brief Registry source backed by static, compile-time type registration.
 *
 * @tparam Base The base type whose derived-type factories are managed.
 * @tparam Args The argument types accepted by every registered factory.
 */
template <typename Base, typename... Args>
class StaticRegistrySource : public RegistrySource<Base, Args...> {
 public:
  static constexpr int kPriority = RegistrySource<Base, Args...>::kDefaultPriority + 10;

  ~StaticRegistrySource() = default;
  StaticRegistrySource(const StaticRegistrySource&) = delete;
  StaticRegistrySource& operator=(const StaticRegistrySource&) = delete;
  StaticRegistrySource(StaticRegistrySource&&) = delete;
  StaticRegistrySource& operator=(StaticRegistrySource&&) = delete;

  /// @brief Returns the singleton instance of this static registry source.
  static const std::shared_ptr<StaticRegistrySource>& Instance() {
    static std::shared_ptr<StaticRegistrySource> instance{new StaticRegistrySource()};
    return instance;
  }

  int GetPriority() const override { return kPriority; }

  bool HasType(std::string_view type_name) const override {
    std::lock_guard lock(mutex_);
    return factory_map_.count(std::string(type_name)) > 0;
  }

  Result_<std::unique_ptr<Base>> MakeInstance(std::string_view type_name, Args... args) const override {
    std::lock_guard lock(mutex_);
    const auto it = factory_map_.find(std::string(type_name));
    if (it == factory_map_.end()) {
      return Err(Error::kFailure, fmt::format("type '{}' is not registered", type_name));
    }
    return it->second(std::move(args)...);
  }

  /**
   * @brief Registers `Derived` under `type_name`.
   *
   * A key may be bound to a single factory. Registering a key that is already
   * bound is a programming error that cannot be resolved at runtime: it means
   * the registration side effect ran more than once, so either the ODR is
   * broken across linked images or two distinct types claim one key.
   * Registration therefore fails with a `BELUGA_ESTIMATION_DEMAND` naming the key.
   *
   * @note This runs during static initialization, so a duplicate registration
   * terminates the process; that is the intended diagnostic. To keep a single
   * process from having several linked images register into one registry, the
   * @ref BELUGA_ESTIMATION_REGISTER_TYPE side effect is emitted only when
   * `BELUGA_ESTIMATION_LINK_STATIC_REGISTRY` is defined.
   */
  template <typename Derived>
  void RegisterType(std::string_view type_name) {
    static_assert(std::is_base_of_v<Base, Derived>, "Registered type must derive from registry base");
    static_assert(
        has_make_v<Derived, Args...>,
        "Registered types must provide a static Make(...) returning Result_<std::unique_ptr<Derived>>.");
    std::lock_guard lock(mutex_);
    auto key = std::string(type_name);
    BELUGA_ESTIMATION_DEMAND(factory_map_.count(key) == 0, "Type '{}' is already bound.", type_name);
    auto factory = [](Args&&... args) -> Result_<std::unique_ptr<Base>> {
      auto result = Derived::Make(std::forward<Args>(args)...);
      if (!result) {
        return Err(CollectError(result));
      }
      return Result_<std::unique_ptr<Base>>{Unwrap(result)};
    };
    factory_map_.emplace(std::move(key), std::move(factory));
  }

 private:
  StaticRegistrySource() = default;

  mutable std::mutex mutex_;
  using FactoryFunction = std::function<Result_<std::unique_ptr<Base>>(Args...)>;
  std::unordered_map<std::string, FactoryFunction> factory_map_;
};

/**
 * @brief Singleton type registry for a given base type.
 *
 * @note Thread-safe for concurrent reads and registrations.
 *
 * @tparam Base The base type whose derived-type factories are managed.
 */
template <typename Base>
class Registry {
 private:
  using Source = typename RegistryTraits<Base>::template Instantiate<RegistrySource>;

 public:
  ~Registry() = default;
  Registry(const Registry&) = delete;
  Registry& operator=(const Registry&) = delete;
  Registry(Registry&&) = delete;
  Registry& operator=(Registry&&) = delete;

  /// @brief Returns the singleton instance of this registry.
  static const std::shared_ptr<Registry>& Instance() {
    static std::shared_ptr<Registry> instance{new Registry()};
    return instance;
  }

  /// @brief Idempotent. Sources are kept sorted by decreasing priority.
  void RegisterSource(std::shared_ptr<Source> new_source) {
    std::lock_guard lock(mutex_);
    auto insert_pos = sources_.begin();
    for (const auto& source_ref : sources_) {
      if (auto source = source_ref.lock()) {
        if (source == new_source) {
          return;
        }
        if (source->GetPriority() > new_source->GetPriority()) {
          ++insert_pos;
        }
      }
    }
    sources_.insert(insert_pos, std::move(new_source));
  }

  bool HasType(std::string_view type_name) const {
    std::lock_guard lock(mutex_);
    for (const auto& source_ref : sources_) {
      if (auto source = source_ref.lock()) {
        if (source->HasType(type_name)) {
          return true;
        }
      }
    }
    return false;
  }

  /// @return The created instance, or an error if the type is not registered or the factory fails.
  template <typename... Args>
  Result_<std::unique_ptr<Base>> MakeInstance(std::string_view type_name, Args&&... args) const {
    std::lock_guard lock(mutex_);
    for (const auto& source_ref : sources_) {
      if (auto source = source_ref.lock()) {
        // Pre-check if type is available to avoid unnecessary parameter copies if
        // the factory ultimately fails to create an instance.
        if (source->HasType(type_name)) {
          return source->MakeInstance(type_name, std::forward<Args>(args)...);
        }
      }
    }
    return Err(Error::kFailure, fmt::format("type '{}' is not registered", type_name));
  }

 private:
  Registry() = default;

  mutable std::recursive_mutex mutex_;
  std::vector<std::weak_ptr<Source>> sources_;
};

template <typename Derived, typename Base>
inline void RegisterType(std::string_view type_name) {
  using Source = typename RegistryTraits<Base>::template Instantiate<StaticRegistrySource>;
  auto static_source = Source::Instance();
  auto registry = Registry<Base>::Instance();
  registry->RegisterSource(static_source);
  static_source->template RegisterType<Derived>(type_name);
}

}  // namespace beluga_estimation

/**
 * @brief Specialises `beluga_estimation::RegistryTraits<Base>` for the given factory argument types.
 *
 * Must appear once per `Base` type at namespace scope.
 *
 * @param Base The base type.
 * @param Args Parenthesised, comma-separated factory argument types, e.g. `(std::string, int)`.
 */
#define BELUGA_ESTIMATION_EXPORT_TYPE(Base, Args)                                \
  template <>                                                                    \
  struct beluga_estimation::RegistryTraits<Base> {                               \
    template <template <typename...> class T, typename... TArgs>                 \
    using Instantiate = T<TArgs..., Base, BELUGA_ESTIMATION_STRIP_PARENS(Args)>; \
  }

/**
 * @brief Registers `Derived` in `beluga_estimation::Registry<Base>` via a static initialiser.
 *
 * Each invocation defines an `inline` variable-template specialization whose
 * dynamic initializer performs the registration. Every translation unit that
 * includes the same call site refers to the same specialization, so the
 * registration runs once per linked image rather than once per translation unit.
 *
 * @note This macro is a no-op unless `BELUGA_ESTIMATION_LINK_STATIC_REGISTRY` is defined for
 * the translation unit. `beluga_estimation` builds as a static library, so every image that
 * links it carries its own copy of the static registry, and the registry
 * singleton may be shared across those images. Emitting registrations from more
 * than one image would then bind the same keys repeatedly into one registry.
 * Define `BELUGA_ESTIMATION_LINK_STATIC_REGISTRY` only on the single target that should own
 * the static registrations (typically the executable); plugin libraries must not
 * opt in and should register through the dynamic plugin mechanism instead.
 * @note Must be invoked at namespace scope, at global scope or inside an
 * enclosing namespace of `beluga_estimation`. It cannot appear inside a function body.
 * @note Each invocation must occupy its own line. The line identifies the
 * registration site, so two invocations on the same line for the same
 * `(Derived, Base)` pair would collide.
 * @note Registration happens only if the translation unit containing the
 * invocation is part of the link. In particular, `Registry<Base>::Instance()`
 * does not discover registrations whose headers were never included.
 *
 * @param Derived The concrete type to register.
 * @param Base    The base type whose registry to register `Derived` with.
 * @param Name    *(optional)* Registry key string. Defaults to `#Derived`.
 */
#ifdef BELUGA_ESTIMATION_LINK_STATIC_REGISTRY
/// @cond
#define BELUGA_ESTIMATION_REGISTER_TYPE_3(Name, Derived, Base)                              \
  template <>                                                                               \
  inline const bool beluga_estimation::detail::kTypeRegistration<Derived, Base, __LINE__> = \
      (beluga_estimation::RegisterType<Derived, Base>(Name), true)

#define BELUGA_ESTIMATION_REGISTER_TYPE_2(Derived, Base) BELUGA_ESTIMATION_REGISTER_TYPE_3(#Derived, Derived, Base)

#define BELUGA_ESTIMATION_REGISTER_TYPE_OVERLOAD(_1, _2, _3, NAME, ...) NAME
/// @endcond

#define BELUGA_ESTIMATION_REGISTER_TYPE(...)                                                \
  BELUGA_ESTIMATION_REGISTER_TYPE_OVERLOAD(                                                 \
      __VA_ARGS__, BELUGA_ESTIMATION_REGISTER_TYPE_3, BELUGA_ESTIMATION_REGISTER_TYPE_2, ~) \
  (__VA_ARGS__)
#else
/// @cond
#define BELUGA_ESTIMATION_REGISTER_TYPE(...)
/// @endcond
#endif  // BELUGA_ESTIMATION_LINK_STATIC_REGISTRY
