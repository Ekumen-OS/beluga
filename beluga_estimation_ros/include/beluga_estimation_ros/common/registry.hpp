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
 * @brief Class_loader-backed registry source and associated macros.
 */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>

#include <ament_index_cpp/get_resources.hpp>
#include <class_loader/class_loader.hpp>

#include <beluga_estimation/common/macros.hpp>
#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>

namespace beluga_estimation_ros {

namespace detail {

/// @cond

/**
 * @brief Abstract factory callable loaded by class_loader for `Base`.
 *
 * @note `Base` need not be default-constructible; only the factory itself is loaded.
 *
 * @tparam Base The base type whose instances are constructed.
 * @tparam Args The argument types accepted by the factory.
 */
template <typename Base, typename... Args>
class FactoryPlugin {
 public:
  virtual ~FactoryPlugin() = default;

  /// @brief Returns the registry key under which `MakeInstance` is reachable.
  virtual std::string GetTypeName() const = 0;

  virtual beluga_estimation::Result_<std::unique_ptr<Base>> MakeInstance(Args&&... args) const = 0;
};

/**
 * @brief Concrete factory for `Derived`, registered with class_loader.
 *
 * @tparam Derived The concrete type to construct.
 * @tparam Base    The base type produced.
 * @tparam Args    Factory argument types.
 */
template <typename Name, typename Derived, typename Base, typename... Args>
class FactoryPluginImplementation : public FactoryPlugin<Base, Args...> {
 public:
  std::string GetTypeName() const override { return Name::value; }

  beluga_estimation::Result_<std::unique_ptr<Base>> MakeInstance(Args&&... args) const override {
    auto result = Derived::Make(std::forward<Args>(args)...);
    if (!result) {
      return beluga_estimation::Err(beluga_estimation::CollectError(result));
    }
    return beluga_estimation::Result_<std::unique_ptr<Base>>{beluga_estimation::Unwrap(result)};
  }
};

/// @endcond

}  // namespace detail

/**
 * @brief Registry source backed by class_loader for runtime plugin discovery.
 *
 * On first use, reads the `plugins` ament index resource for the `beluga_estimation` package,
 * loads each listed shared library, and instantiates every FactoryPlugin found.
 *
 * @tparam Base The base type whose plugin factories are managed.
 * @tparam Args The argument types accepted by every registered factory.
 */
template <typename Base, typename... Args>
class ClassLoaderRegistrySource : public beluga_estimation::RegistrySource<Base, Args...> {
 public:
  ~ClassLoaderRegistrySource() = default;
  ClassLoaderRegistrySource(const ClassLoaderRegistrySource&) = delete;
  ClassLoaderRegistrySource& operator=(const ClassLoaderRegistrySource&) = delete;
  ClassLoaderRegistrySource(ClassLoaderRegistrySource&&) = delete;
  ClassLoaderRegistrySource& operator=(ClassLoaderRegistrySource&&) = delete;

  /// @brief Returns the singleton instance of this class_loader registry source.
  static const std::shared_ptr<ClassLoaderRegistrySource>& Instance() {
    static std::shared_ptr<ClassLoaderRegistrySource> instance{new ClassLoaderRegistrySource()};
    return instance;
  }

  int GetPriority() const override { return beluga_estimation::RegistrySource<Base, Args...>::kDefaultPriority; }

  bool HasType(std::string_view type_name) const override {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return plugin_factories_.count(std::string(type_name)) > 0;
  }

  /// @return An error with `Error::kFailure` if the type is not available or construction fails.
  beluga_estimation::Result_<std::unique_ptr<Base>> MakeInstance(std::string_view type_name, Args... args)
      const override {
    std::lock_guard<std::recursive_mutex> lock(mutex_);

    const auto it = plugin_factories_.find(std::string(type_name));
    if (it == plugin_factories_.end()) {
      return beluga_estimation::Err(
          beluga_estimation::Error::kFailure, fmt::format("type '{}' is not registered", type_name));
    }

    try {
      return it->second->MakeInstance(std::move(args)...);
    } catch (const std::exception& error) {
      // A plugin factory that throws instead of returning an error is a boundary
      // violation; convert it into an error rather than losing it.
      return beluga_estimation::Err(beluga_estimation::Error::kFailure, error.what());
    } catch (...) {
      return beluga_estimation::Err(beluga_estimation::Error::kFailure, "plugin factory threw an unknown exception");
    }
  }

 private:
  ClassLoaderRegistrySource() {
    // NOTE: plugin libraries are eagerly loaded to trigger initialization of class_loader singletons
    // and establish a construction and destruction order: class_loader singletons *must* be constructed
    // before this registry source to ensure they are destructed after it.
    const std::map<std::string, std::string> plugin_index = ament_index_cpp::get_resources("beluga_plugins");
    if (plugin_index.empty()) {
      return;
    }
    for (const auto& [resource_name, resource_prefix] : plugin_index) {
      const std::filesystem::path resource_prefix_path(resource_prefix);
      const std::filesystem::path resource_path =
          resource_prefix_path / "share" / "ament_index" / "resource_index" / "beluga_plugins" / resource_name;
      std::ifstream resource_stream(resource_path);
      if (!resource_stream.is_open()) {
        continue;
      }
      std::string library_path;
      while (std::getline(resource_stream, library_path)) {
        try {
          constexpr bool kOnDemand = true;
          auto loader = std::make_unique<class_loader::ClassLoader>(library_path, !kOnDemand);
          for (const auto& class_name : loader->getAvailableClasses<FactoryPlugin>()) {
            auto factory = loader->createUniqueInstance<FactoryPlugin>(class_name);
            const std::string type_name = factory->GetTypeName();
            if (plugin_factories_.count(type_name) > 0) {
              std::cerr << "Duplicate plugin for type '" << type_name << "' found in library '" << library_path
                        << "'; ignoring this plugin." << std::endl;
              continue;
            }
            plugin_factories_.insert_or_assign(type_name, std::move(factory));
          }
          loaders_.push_back(std::move(loader));
        } catch (const std::exception& e) {
          std::cerr << "Error loading plugins from library '" << library_path << "': " << e.what() << std::endl;
        }
      }
    }
  }

  mutable std::recursive_mutex mutex_;
  mutable std::vector<std::unique_ptr<class_loader::ClassLoader>> loaders_;
  using FactoryPlugin = detail::FactoryPlugin<Base, Args...>;
  using FactoryPluginUniquePtr = class_loader::ClassLoader::UniquePtr<FactoryPlugin>;
  mutable std::unordered_map<std::string, FactoryPluginUniquePtr> plugin_factories_;
};

template <typename Base>
inline void RegisterClassLoaderSource() {
  using Source = typename beluga_estimation::RegistryTraits<Base>::template Instantiate<ClassLoaderRegistrySource>;
  auto registry = beluga_estimation::Registry<Base>::Instance();
  registry->RegisterSource(Source::Instance());
}

}  // namespace beluga_estimation_ros

/// @cond

#define BELUGA_ESTIMATION_ROS_IMPORT_TYPE(Base)                           \
  [[maybe_unused, gnu::used]] static const bool BELUGA_ESTIMATION_CONCAT( \
      kBelugaEstimationRosClassLoaderSourceRegistered_, __COUNTER__) =    \
      (beluga_estimation_ros::RegisterClassLoaderSource<Base>(), true)

#define BELUGA_ESTIMATION_ROS_REGISTER_TYPE_2(Derived, Base) \
  BELUGA_ESTIMATION_ROS_REGISTER_TYPE_3(#Derived, Derived, Base)

#define BELUGA_ESTIMATION_ROS_REGISTER_TYPE_3(Name, Derived, Base)                                    \
  namespace {                                                                                         \
  struct BELUGA_ESTIMATION_CONCAT(TypeName, __LINE__) {                                               \
    static constexpr char value[] = Name;                                                             \
  };                                                                                                  \
  using BELUGA_ESTIMATION_CONCAT(DerivedFactory, __LINE__) =                                          \
      typename beluga_estimation::RegistryTraits<Base>::template Instantiate<                         \
          beluga_estimation_ros::detail::FactoryPluginImplementation,                                 \
          BELUGA_ESTIMATION_CONCAT(TypeName, __LINE__),                                               \
          Derived>;                                                                                   \
  using BELUGA_ESTIMATION_CONCAT(BaseFactory, __LINE__) = typename beluga_estimation::RegistryTraits< \
      Base>::template Instantiate<beluga_estimation_ros::detail::FactoryPlugin>;                      \
  }                                                                                                   \
  CLASS_LOADER_REGISTER_CLASS(                                                                        \
      BELUGA_ESTIMATION_CONCAT(DerivedFactory, __LINE__), BELUGA_ESTIMATION_CONCAT(BaseFactory, __LINE__))

#define BELUGA_ESTIMATION_ROS_REGISTER_TYPE_OVERLOAD(_1, _2, _3, NAME, ...) NAME

/// @endcond

/**
 * @brief Exports a factory plugin for `Derived` via class_loader and registers the source.
 *
 * @note Must only be used in a `.cpp` translation unit, never in a header.
 * @note The registry key is determined by `FactoryPluginImplementation::GetTypeName()`.
 *
 * @param Derived The concrete type whose factory plugin is being exported.
 * @param Base    The base type whose registry `Derived` is being registered with.
 */
#define BELUGA_ESTIMATION_ROS_REGISTER_TYPE(...)                                                    \
  BELUGA_ESTIMATION_ROS_REGISTER_TYPE_OVERLOAD(                                                     \
      __VA_ARGS__, BELUGA_ESTIMATION_ROS_REGISTER_TYPE_3, BELUGA_ESTIMATION_ROS_REGISTER_TYPE_2, ~) \
  (__VA_ARGS__)
