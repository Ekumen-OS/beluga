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
 * @brief Type-erased uniform random bit generator.
 */

#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <type_traits>
#include <utility>

namespace beluga_estimation {

/// @brief Type-erased wrapper for any UniformRandomBitGenerator.
///
/// Satisfies the UniformRandomBitGenerator requirements and can wrap any
/// compatible random engine (e.g., std::mt19937, std::mt19937_64).
///
/// @tparam ResultType The result type (must be an unsigned integral type).
///
/// Example usage:
/// @code
/// // Construct with seed
/// beluga_estimation::RandomNumberGenerator64 rng1(42);
///
/// // Construct from engine
/// std::mt19937_64 engine(123);
/// beluga_estimation::RandomNumberGenerator64 rng2(engine);
///
/// // Use with distributions
/// std::uniform_int_distribution<int> dist(1, 6);
/// int value = dist(rng1);
/// @endcode
template <typename ResultType>
class RandomNumberGenerator_ {
 public:
  /// @brief The result type of the random number generator.
  using result_type = ResultType;

  static_assert(std::is_unsigned_v<result_type>, "ResultType must be an unsigned integral type");

  /// @brief Default constructor using std::random_device for seeding.
  RandomNumberGenerator_();

  /// @brief Constructs with a specific seed value.
  ///
  /// @param seed The seed value for the internal random engine.
  explicit RandomNumberGenerator_(result_type seed);

  /// @brief Constructs from any compatible UniformRandomBitGenerator.
  ///
  /// @tparam Generator A type satisfying UniformRandomBitGenerator requirements.
  /// @param generator The random number generator to wrap.
  template <
      typename Generator,
      typename = std::enable_if_t<
          !std::is_same_v<std::decay_t<Generator>, RandomNumberGenerator_> &&
          !std::is_arithmetic_v<std::decay_t<Generator>>>>
  explicit RandomNumberGenerator_(Generator&& generator)
      : impl_(std::make_unique<Engine<std::decay_t<Generator>>>(std::forward<Generator>(generator))) {
    using Gen = std::decay_t<Generator>;
    using GenResultType = typename Gen::result_type;

    static_assert(std::is_unsigned_v<GenResultType>, "Generator result_type must be an unsigned integral type");

    static_assert(
        Gen::min() == static_cast<GenResultType>(min()),
        "Generator min() must match RandomNumberGenerator min() (the minimum value of result_type)");

    static_assert(
        Gen::max() == static_cast<GenResultType>(max()),
        "Generator max() must match RandomNumberGenerator max() (the maximum value of result_type)");
  }

  RandomNumberGenerator_(const RandomNumberGenerator_& other);
  RandomNumberGenerator_(RandomNumberGenerator_&& other) noexcept = default;
  RandomNumberGenerator_& operator=(const RandomNumberGenerator_& other);
  RandomNumberGenerator_& operator=(RandomNumberGenerator_&& other) noexcept = default;
  ~RandomNumberGenerator_() = default;

  /// @brief Generates a random number.
  result_type operator()();

  /// @brief Returns the minimum value that can be generated.
  static constexpr result_type min() { return std::numeric_limits<result_type>::min(); }

  /// @brief Returns the maximum value that can be generated.
  static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }

 private:
  /// @brief Abstract base class for type-erased engine storage.
  class EngineBase {
   public:
    virtual ~EngineBase() = default;
    virtual result_type operator()() = 0;
    virtual std::unique_ptr<EngineBase> clone() const = 0;
  };

  /// @brief Templated implementation that wraps a specific engine type.
  template <typename Generator>
  class Engine : public EngineBase {
   public:
    explicit Engine(Generator generator) : generator_(std::move(generator)) {}

    result_type operator()() override {
      // Cast is safe because of static assertions on range and result_type compatibility
      return static_cast<result_type>(generator_());
    }

    std::unique_ptr<EngineBase> clone() const override { return std::make_unique<Engine>(generator_); }

   private:
    Generator generator_;
  };

  std::unique_ptr<EngineBase> impl_;
};

/// @brief 32-bit random number generator.
///
/// Compatible with generators like std::mt19937.
using RandomNumberGenerator32 = RandomNumberGenerator_<std::uint32_t>;

/// @brief 64-bit random number generator.
///
/// Compatible with generators like std::mt19937_64.
using RandomNumberGenerator64 = RandomNumberGenerator_<std::uint64_t>;

/// @brief Default random number generator (64-bit).
using RandomNumberGenerator = RandomNumberGenerator64;

}  // namespace beluga_estimation
