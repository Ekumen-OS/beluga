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
 * @file views.hpp
 * @brief Zero-copy view conversion utilities for vectors, matrices, Lie groups, and range types.
 *
 * This header provides the `to_view()` function and `view_traits` customization point
 * for converting values to lightweight, non-owning views. All conversions are zero-copy
 * and preserve const-correctness.
 *
 * ## Supported Conversions
 *
 * ### Direct Conversions (via view_traits specializations)
 * - **Algebraic types**: `Vector<T, N>`, `VectorX<T>`, `Matrix<T, N, M>`, `MatrixX<T>` → span/mdspan
 * - **Lie groups**: `SE2<T>`, `SO2<T>`, `SE3<T>`, `SO3<T>` → span over underlying memory
 * - **Arithmetic containers**: `std::vector<T>`, `std::array<T, N>` (where `T` is arithmetic) → span
 *
 * ### Range Conversions (automatic analysis)
 * - **Visible ranges**: Ranges containing viewable elements are recursively transformed
 *   - Example: `std::vector<Vector3d>` → `view<span<double, 3>>`
 *   - Example: `std::vector<std::vector<double>>` → `view<span<double>>` (nested arithmetic)
 * - **Opaque ranges**: Ranges containing non-viewable elements are wrapped without transformation
 *   - Example: `std::vector<CustomStruct>` → `view<CustomStruct>`
 *
 * ## Conversion Strategy
 *
 * The `to_view()` function uses SFINAE-based overload resolution:
 *
 * 1. **Direct conversion**: For types with `view_traits` specializations
 * 2. **Recursive transform**: For visible ranges (lazy, element-by-element)
 * 3. **Pass-through wrap**: For opaque ranges (no element transformation)
 * 4. **Compile error**: For non-viewable, non-range types
 *
 * ## Usage Example
 *
 * ```cpp
 * // Direct conversions
 * beluga_estimation::Vector3d vec{1, 2, 3};
 * auto vec_view = to_view(vec);  // span<double, 3>
 *
 * std::vector<double> data{1, 2, 3, 4};
 * auto data_view = to_view(data);  // span<double>
 *
 * // Visible range (recursive)
 * std::vector<beluga_estimation::Vector3d> vecs{vec, vec};
 * auto vecs_view = to_view(vecs);  // view<span<double, 3>>
 *
 * // Opaque range (pass-through)
 * struct Custom { int id; };
 * std::vector<Custom> customs{{1}, {2}};
 * auto customs_view = to_view(customs);  // view<Custom>
 * ```
 *
 * ## Customization
 *
 * To add view support for custom types, specialize `view_traits`:
 *
 * ```cpp
 * template <>
 * struct beluga_estimation::view_traits<MyType> {
 *   using type = span<double>;  // Or any view type
 *   static auto make(MyType& value) -> type {
 *     return {value.data(), value.size()};
 *   }
 * };
 * ```
 *
 * All conversions are lazy - no data is copied or materialized until iteration.
 *
 * @see view_traits for the customization point
 * @see to_view() for the main conversion function
 */

#include <type_traits>
#include <utility>

#include <mdspan/mdspan.hpp>
#include <range/v3/range/concepts.hpp>
#include <range/v3/range/traits.hpp>
#include <range/v3/view/any_view.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/std_future.hpp>

namespace beluga_estimation {

/**
 * @brief Type alias for type-erased, random access, sized views.
 *
 * This is the primary view type used throughout Beluga for generic collections.
 * It supports random access and size queries, making it suitable for algorithms
 * that need indexed access or parallelization.
 */
template <typename T>
using view = ranges::any_view<T, ranges::category::random_access | ranges::category::sized>;

/**
 * @brief Type alias for type-erased, forward-only views.
 *
 * Use this for single-pass iteration where random access is not needed.
 * More efficient than `view` for sequential-only algorithms.
 */
template <typename T>
using single_pass_view = ranges::any_view<T, ranges::category::input>;

/**
 * @brief Type trait to determine the appropriate view type for a given type.
 *
 * This is a customization point that allows users to define view conversions
 * for their own types. Each specialization must provide:
 * - `type` - the resulting view type (typically span, mdspan, or any_view)
 * - `make(value)` - static method performing the zero-copy conversion
 *
 * The generic fallback handles any sized range by recursively applying `to_view`
 * to each element, creating a view of views. Explicit specializations are provided
 * for vectors, matrices, and Lie group types for optimal performance.
 *
 * @tparam T The type to convert to a view (without reference qualifiers).
 *
 * @note When specializing, ensure T is the non-reference type. Use separate
 *       specializations for const and mutable versions to preserve const-correctness.
 */
template <typename T, typename = void>
struct view_traits;

/// Specialization for constant numerical vector.
template <typename T, typename Allocator>
struct view_traits<const std::vector<T, Allocator>, std::enable_if_t<std::is_arithmetic_v<T>>> {
  using type = std_future::span<const T>;

  static auto make(const std::vector<T, Allocator>& vec) -> type { return type(vec.data(), vec.size()); }
};

/// Specialization for mutable numerical vector.
template <typename T, typename Allocator>
struct view_traits<std::vector<T, Allocator>, std::enable_if_t<std::is_arithmetic_v<T>>> {
  using type = std_future::span<T>;

  static auto make(std::vector<T, Allocator>& vec) -> type { return type(vec.data(), vec.size()); }
};

/// Specialization for constant numerical array.
template <typename T, std::size_t N>
struct view_traits<const std::array<T, N>, std::enable_if_t<std::is_arithmetic_v<T>>> {
  using type = std_future::span<const T, N>;

  static auto make(const std::array<T, N>& arr) -> type { return type(arr.data(), arr.size()); }
};

/// Specialization for mutable numerical array.
template <typename T, std::size_t N>
struct view_traits<std::array<T, N>, std::enable_if_t<std::is_arithmetic_v<T>>> {
  using type = std_future::span<T, N>;

  static auto make(std::array<T, N>& arr) -> type { return type(arr.data(), arr.size()); }
};

/// Specialization for references to constant algebraic vectors (fixed size).
template <typename T, int N>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::Vector<T, N>>> {
  using type = std_future::span<const T, N>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::Vector<T, N>>& vec) -> type {
    return type(vec.data(), vec.size());
  }
};

/// Specialization for constant algebraic vectors (fixed size).
template <typename T, int N>
struct view_traits<const beluga_estimation::Vector<T, N>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::Vector<T, N>>> {};

/// Specialization for references to mutable algebraic vectors (fixed size).
template <typename T, int N>
struct view_traits<beluga_estimation::Ref<beluga_estimation::Vector<T, N>>> {
  using type = std_future::span<T, N>;

  static auto make(beluga_estimation::Ref<beluga_estimation::Vector<T, N>> vec) -> type {
    return type(vec.data(), vec.size());
  }
};

/// Specialization for mutable algebraic vectors (fixed size).
template <typename T, int N>
struct view_traits<beluga_estimation::Vector<T, N>>
    : view_traits<beluga_estimation::Ref<beluga_estimation::Vector<T, N>>> {};

/// Specialization for references to constant algebraic vectors (dynamic size).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::VectorX<T>>> {
  using type = std_future::span<const T>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::VectorX<T>>& vec) -> type {
    return type(vec.data(), vec.size());
  }
};

/// Specialization for constant algebraic vectors (dynamic size).
template <typename T>
struct view_traits<const beluga_estimation::VectorX<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::VectorX<T>>> {};

/// Specialization for references to mutable algebraic vectors (dynamic size).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::VectorX<T>>> {
  using type = std_future::span<T>;

  static auto make(beluga_estimation::Ref<beluga_estimation::VectorX<T>> vec) -> type {
    return type(vec.data(), vec.size());
  }
};

/// Specialization for mutable algebraic vectors (dynamic size).
template <typename T>
struct view_traits<beluga_estimation::VectorX<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::VectorX<T>>> {
};

/// Specialization for references to constant matrices (fixed size, row-major).
template <typename T, int N, int M>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::Matrix<T, N, M>>> {
  using type = std_future::mdspan<const T, 2>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::Matrix<T, N, M>>& mat) -> type {
    const auto strides = std::array<std::size_t, 2>{
        static_cast<std::size_t>(mat.outerStride()), static_cast<std::size_t>(mat.innerStride())};
    using extents_type = typename type::extents_type;
    const auto shape = extents_type(static_cast<std::size_t>(mat.rows()), static_cast<std::size_t>(mat.cols()));
    using mapping_type = typename type::mapping_type;
    return type(mat.data(), mapping_type(shape, strides));
  }
};

/// Specialization for constant matrices (fixed size, row-major).
template <typename T, int N, int M>
struct view_traits<const beluga_estimation::Matrix<T, N, M>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::Matrix<T, N, M>>> {};

/// Specialization for references to mutable matrices (fixed size, row-major).
template <typename T, int N, int M>
struct view_traits<beluga_estimation::Ref<beluga_estimation::Matrix<T, N, M>>> {
  using type = std_future::mdspan<T, 2>;

  static auto make(beluga_estimation::Ref<beluga_estimation::Matrix<T, N, M>> mat) -> type {
    const auto strides = std::array<std::size_t, 2>{
        static_cast<std::size_t>(mat.outerStride()), static_cast<std::size_t>(mat.innerStride())};
    using extents_type = typename type::extents_type;
    const auto shape = extents_type(static_cast<std::size_t>(mat.rows()), static_cast<std::size_t>(mat.cols()));
    using mapping_type = typename type::mapping_type;
    return type(mat.data(), mapping_type(shape, strides));
  }
};

/// Specialization for mutable matrices (fixed size, row-major).
template <typename T, int N, int M>
struct view_traits<beluga_estimation::Matrix<T, N, M>>
    : view_traits<beluga_estimation::Ref<beluga_estimation::Matrix<T, N, M>>> {};

/// Specialization for references to constant algebraic matrices (dynamic size, row-major).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::MatrixX<T>>> {
  using type = std_future::mdspan<const T, 2>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::MatrixX<T>>& mat) -> type {
    const auto strides = std::array<std::size_t, 2>{
        static_cast<std::size_t>(mat.outerStride()), static_cast<std::size_t>(mat.innerStride())};
    using extents_type = typename type::extents_type;
    const auto shape = extents_type(static_cast<std::size_t>(mat.rows()), static_cast<std::size_t>(mat.cols()));
    using mapping_type = typename type::mapping_type;
    return type(mat.data(), mapping_type(shape, strides));
  }
};

/// Specialization for constant algebraic matrices (dynamic size, row-major).
template <typename T>
struct view_traits<const beluga_estimation::MatrixX<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::MatrixX<T>>> {};

/// Specialization for references to mutable algebraic matrices (dynamic size, row-major).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::MatrixX<T>>> {
  using type = std_future::mdspan<T, 2>;

  static auto make(beluga_estimation::Ref<beluga_estimation::MatrixX<T>> mat) -> type {
    const auto strides = std::array<std::size_t, 2>{
        static_cast<std::size_t>(mat.outerStride()), static_cast<std::size_t>(mat.innerStride())};
    using extents_type = typename type::extents_type;
    const auto shape = extents_type(static_cast<std::size_t>(mat.rows()), static_cast<std::size_t>(mat.cols()));
    using mapping_type = typename type::mapping_type;
    return type(mat.data(), mapping_type(shape, strides));
  }
};

/// Specialization for mutable algebraic matrices (dynamic size, row-major).
template <typename T>
struct view_traits<beluga_estimation::MatrixX<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::MatrixX<T>>> {
};

/// Specialization for constant algebraic matrix blocks (row-major).
template <typename Derived>
struct view_traits<const Eigen::Block<Derived>> : view_traits<beluga_estimation::Ref<const Derived>> {};

/// Specialization for mutable algebraic matrix blocks (row-major).
template <typename Derived>
struct view_traits<Eigen::Block<Derived>> : view_traits<beluga_estimation::Ref<Derived>> {};

/// Specialization for references to constant SE2 (4 elements: cos, sin, x, y).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::SE2<T>>> {
  using type = std_future::span<const T>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::SE2<T>>& se2) -> type {
    return type(se2.data(), 4);
  }
};

/// Specialization for constant SE2 (4 elements: cos, sin, x, y).
template <typename T>
struct view_traits<const beluga_estimation::SE2<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::SE2<T>>> {};

/// Specialization for references to mutable SE2 (4 elements: cos, sin, x, y).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::SE2<T>>> {
  using type = std_future::span<T>;

  static auto make(beluga_estimation::Ref<beluga_estimation::SE2<T>> se2) -> type { return type(se2.data(), 4); }
};

/// Specialization for mutable SE2 (4 elements: cos, sin, x, y).
template <typename T>
struct view_traits<beluga_estimation::SE2<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::SE2<T>>> {};

/// Specialization for references to constant SO2 (2 elements: cos, sin).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::SO2<T>>> {
  using type = std_future::span<const T>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::SO2<T>>& so2) -> type {
    return type(so2.data(), 2);
  }
};

/// Specialization for constant SO2 (2 elements: cos, sin).
template <typename T>
struct view_traits<const beluga_estimation::SO2<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::SO2<T>>> {};

/// Specialization for references to mutable SO2 (2 elements: cos, sin).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::SO2<T>>> {
  using type = std_future::span<T>;

  static auto make(beluga_estimation::Ref<beluga_estimation::SO2<T>> so2) -> type { return type(so2.data(), 2); }
};

/// Specialization for mutable SO2 (2 elements: cos, sin).
template <typename T>
struct view_traits<beluga_estimation::SO2<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::SO2<T>>> {};

/// Specialization for references to constant SE3 (7 elements: qx, qy, qz, qw, x, y, z).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::SE3<T>>> {
  using type = std_future::span<const T>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::SE3<T>>& se3) -> type {
    return type(se3.data(), 7);
  }
};

/// Specialization for constant SE3 (7 elements: qx, qy, qz, qw, x, y, z).
template <typename T>
struct view_traits<const beluga_estimation::SE3<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::SE3<T>>> {};

/// Specialization for references to mutable SE3 (7 elements: qx, qy, qz, qw, x, y, z).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::SE3<T>>> {
  using type = std_future::span<T>;

  static auto make(beluga_estimation::Ref<beluga_estimation::SE3<T>> se3) -> type { return type(se3.data(), 7); }
};

/// Specialization for mutable SE3 (7 elements: qx, qy, qz, qw, x, y, z).
template <typename T>
struct view_traits<beluga_estimation::SE3<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::SE3<T>>> {};

/// Specialization for references to constant SO3 (4 elements: qx, qy, qz, qw).
template <typename T>
struct view_traits<beluga_estimation::Ref<const beluga_estimation::SO3<T>>> {
  using type = std_future::span<const T>;

  static auto make(const beluga_estimation::Ref<const beluga_estimation::SO3<T>>& so3) -> type {
    return type(so3.data(), 4);
  }
};

/// Specialization for constant SO3 (4 elements: qx, qy, qz, qw).
template <typename T>
struct view_traits<const beluga_estimation::SO3<T>>
    : view_traits<beluga_estimation::Ref<const beluga_estimation::SO3<T>>> {};

/// Specialization for references to mutable SO3 (4 elements: qx, qy, qz, qw).
template <typename T>
struct view_traits<beluga_estimation::Ref<beluga_estimation::SO3<T>>> {
  using type = std_future::span<T>;

  static auto make(beluga_estimation::Ref<beluga_estimation::SO3<T>> so3) -> type { return type(so3.data(), 4); }
};

/// Specialization for mutable SO3 (4 elements: qx, qy, qz, qw).
template <typename T>
struct view_traits<beluga_estimation::SO3<T>> : view_traits<beluga_estimation::Ref<beluga_estimation::SO3<T>>> {};

/**
 * @brief Helper alias to get the view type for a given type.
 *
 * @tparam T The type to convert (may include reference qualifiers).
 */
template <typename T>
using view_type = typename view_traits<std::remove_reference_t<T>>::type;

/**
 * @brief Type trait to detect whether `view_traits` is specialized for a given type.
 *
 * This trait checks if `view_traits<T>` provides a valid `type` member, indicating
 * that the type has an explicit view conversion defined. Used internally by `to_view()`
 * to determine the appropriate conversion strategy.
 *
 * @tparam T The type to check for view_traits specialization.
 *
 * @example
 * ```cpp
 * static_assert(has_view_type_v<beluga_estimation::Vector3d>);      // true - explicit specialization
 * static_assert(has_view_type_v<beluga_estimation::SE2d>);          // true - explicit specialization
 * static_assert(has_view_type_v<std::vector<double>>);   // true - arithmetic container specialization
 * static_assert(!has_view_type_v<double>);               // false - no specialization
 * static_assert(!has_view_type_v<std::vector<CustomType>>); // false (unless CustomType has view_traits)
 * ```
 */
template <typename T, typename = void>
struct has_view_type : std::false_type {};

/// @cond INTERNAL
template <typename T>
struct has_view_type<T, std::void_t<view_type<T>>> : std::true_type {};
/// @endcond

/**
 * @brief Helper variable template for `has_view_type`.
 *
 * Provides a convenient boolean constant to check if `view_traits<T>` is specialized.
 *
 * @tparam T The type to check for view_traits specialization.
 */
template <typename T>
inline constexpr bool has_view_type_v = has_view_type<T>::value;

/**
 * @brief Type trait to detect contiguous ranges with random access and known size.
 *
 * A contiguous range satisfies both `ranges::random_access_range` and `ranges::sized_range`,
 * making it suitable for efficient view conversions. Examples include `std::vector`,
 * `std::array`, and `std::deque`.
 *
 * @tparam T The type to check.
 *
 * @note This trait does NOT verify actual memory contiguity (which is not reliably detectable
 *       in C++17), only the required range concepts.
 */
template <typename T>
using is_contiguous_range = std::integral_constant<bool, ranges::random_access_range<T> && ranges::sized_range<T>>;

/// @brief Helper variable template for `is_contiguous_range`.
template <typename T>
inline constexpr bool is_contiguous_range_v = is_contiguous_range<T>::value;

/// @cond INTERNAL
// Forward declaration for mutual recursion with has_visible_value
template <typename T>
struct is_visible_range;
/// @endcond

/**
 * @brief Type trait to detect if a range's elements have view conversions.
 *
 * A range has visible values if its element type either:
 * 1. Has an explicit `view_traits` specialization (`has_view_type`), OR
 * 2. Is itself a visible range (recursive check for nested containers)
 *
 * This trait is used to determine if `to_view()` should recursively transform
 * range elements (visible values) or pass them through as-is (opaque values).
 *
 * @tparam T A range type.
 *
 * @example
 * ```cpp
 * static_assert(has_visible_value<std::vector<beluga_estimation::Vector3d>>::value);  // true - Vector3d has
 * view_traits static_assert(has_visible_value<std::vector<std::vector<double>>>::value); // true - inner vector is
 * visible range static_assert(!has_visible_value<std::vector<CustomStruct>>::value);     // false - CustomStruct has no
 * view_traits
 * ```
 */
template <typename T>
struct has_visible_value
    : std::disjunction<has_view_type<ranges::range_reference_t<T>>, is_visible_range<ranges::range_reference_t<T>>> {};

/**
 * @brief Type trait to detect ranges whose elements should be recursively converted to views.
 *
 * A visible range is a contiguous range containing elements that have view conversions.
 * When `to_view()` encounters a visible range, it applies `to_view()` recursively to each
 * element, creating a view of views (e.g., `view<span<double, 3>>` from `std::vector<Vector3d>`).
 *
 * A range is visible if:
 * 1. It is a contiguous range (`is_contiguous_range`), AND
 * 2. Its elements have view conversions (`has_visible_value`)
 *
 * @tparam T The range type to check.
 *
 * @example
 * ```cpp
 * static_assert(is_visible_range_v<std::vector<beluga_estimation::Vector3d>>);        // true
 * static_assert(is_visible_range_v<std::array<beluga_estimation::SE2d, 10>>);         // true
 * static_assert(is_visible_range_v<std::vector<std::vector<double>>>);     // true - nested visible range
 * static_assert(!is_visible_range_v<std::vector<CustomStruct>>);           // false - opaque range
 * static_assert(!is_visible_range_v<std::list<beluga_estimation::Vector3d>>);         // false - not contiguous
 * ```
 *
 * @see is_opaque_range for the complementary trait
 */
template <typename T>
struct is_visible_range : std::conjunction<is_contiguous_range<T>, has_visible_value<T>> {};

/// @brief Helper variable template for `is_visible_range`.
template <typename T>
inline constexpr bool is_visible_range_v = is_visible_range<T>::value;

/**
 * @brief Type trait to detect if a range's elements do NOT have view conversions.
 *
 * A range has opaque values if its element type:
 * 1. Does NOT have a `view_traits` specialization, AND
 * 2. Is NOT itself a visible range
 *
 * Opaque values are passed through without transformation when converting a range to a view.
 *
 * @tparam T A range type.
 *
 * @example
 * ```cpp
 * static_assert(has_opaque_value<std::vector<int>>::value);           // true (despite int specialization via
 * std::vector) static_assert(has_opaque_value<std::vector<std::string>>::value);   // true
 * static_assert(has_opaque_value<std::vector<CustomStruct>>::value);  // true
 * static_assert(!has_opaque_value<std::vector<beluga_estimation::Vector3d>>::value); // false - Vector3d is visible
 * ```
 */
template <typename T>
struct has_opaque_value : std::conjunction<
                              std::negation<has_view_type<ranges::range_reference_t<T>>>,
                              std::negation<is_visible_range<ranges::range_reference_t<T>>>> {};

/**
 * @brief Type trait to detect ranges whose elements should be passed through without conversion.
 *
 * An opaque range is a contiguous range containing elements without view conversions.
 * When `to_view()` encounters an opaque range, it wraps the range in a type-erased `view`
 * without transforming elements (e.g., `view<CustomStruct>` from `std::vector<CustomStruct>`).
 *
 * A range is opaque if:
 * 1. It is a contiguous range (`is_contiguous_range`), AND
 * 2. Its elements do NOT have view conversions (`has_opaque_value`)
 *
 * @tparam T The range type to check.
 *
 * @example
 * ```cpp
 * static_assert(is_opaque_range_v<std::vector<std::string>>);        // true
 * static_assert(is_opaque_range_v<std::array<CustomStruct, 5>>);     // true
 * static_assert(!is_opaque_range_v<std::vector<beluga_estimation::Vector3d>>);  // false - visible range
 * static_assert(!is_opaque_range_v<std::list<int>>);                 // false - not contiguous
 * ```
 *
 * @see is_visible_range for the complementary trait
 */
template <typename T>
struct is_opaque_range : std::conjunction<is_contiguous_range<T>, has_opaque_value<T>> {};

/// @brief Helper variable template for `is_opaque_range`.
template <typename T>
inline constexpr bool is_opaque_range_v = is_opaque_range<T>::value;

/**
 * @brief Convert a value to its corresponding view type (zero-copy).
 *
 * This function provides a unified interface for creating lightweight, non-owning
 * views over various data structures. All conversions are zero-copy and preserve
 * const-correctness. The function has multiple overloads selected via SFINAE based
 * on the input type:
 *
 * 1. **Direct conversion** (`has_view_type_v<T>`): Types with explicit `view_traits`
 *    specializations (vectors, matrices, Lie groups, arithmetic containers) are
 *    converted directly using `view_traits<T>::make()`.
 *
 * 2. **Recursive transformation** (`is_visible_range_v<T>`): Ranges containing elements
 *    with view conversions are lazily transformed element-by-element, creating a
 *    `view<view_type<Element>>`. Enables nested conversions like
 *    `std::vector<Vector3d>` → `view<span<double, 3>>`.
 *
 * 3. **Pass-through wrapping** (`is_opaque_range_v<T>`): Ranges containing elements
 *    without view conversions are wrapped in a type-erased `view<Element>` without
 *    transformation. Examples: `std::vector<std::string>` → `view<std::string>`.
 *
 * 4. **Compile error** (all other types): Types that are neither viewable nor ranges
 *    trigger a static assertion, indicating a missing `view_traits` specialization.
 *
 * @tparam T The input type (deduced, may include cv-qualifiers and references).
 * @param value The value to convert to a view.
 * @return A view of the appropriate type as determined by `view_traits<T>` or range analysis.
 *
 * @note The returned view is non-owning and may dangle if the source data is destroyed.
 *
 * @example
 * ```cpp
 * // Direct conversions (overload 1)
 * beluga_estimation::Vector3d vec(1.0, 2.0, 3.0);
 * auto span_view = to_view(vec);  // span<double, 3>
 *
 * beluga_estimation::SE2d pose(beluga_estimation::SO2d(0.1), beluga_estimation::Vector2d(1, 2));
 * auto pose_view = to_view(pose);  // span<double, 4>
 *
 * std::vector<double> data = {1.0, 2.0, 3.0};
 * auto data_span = to_view(data);  // span<double> (arithmetic container specialization)
 *
 * // Recursive transformation (overload 2 - visible ranges)
 * std::vector<beluga_estimation::Vector3d> vecs = {vec, vec, vec};
 * auto vecs_view = to_view(vecs);  // view<span<double, 3>>
 * for (auto elem_span : vecs_view) {  // Iteration is lazy
 *   // elem_span is span<double, 3>
 * }
 *
 * // Nested conversions
 * std::vector<std::vector<beluga_estimation::Vector2d>> nested = {{vec2, vec2}, {vec2}};
 * auto nested_view = to_view(nested);  // view<view<span<double, 2>>>
 *
 * // Pass-through wrapping (overload 3 - opaque ranges)
 * struct CustomData { int id; std::string name; };
 * std::vector<CustomData> custom = {{1, "foo"}, {2, "bar"}};
 * auto custom_view = to_view(custom);  // view<CustomData> (no element transformation)
 * for (const auto& elem : custom_view) {
 *   // elem is CustomData (original type)
 * }
 * ```
 *
 * @see view_traits for customization points
 * @see has_view_type_v, is_visible_range_v, is_opaque_range_v for type classification
 */

/// @brief Overload 1: Direct conversion for types with explicit view_traits specializations.
template <typename T, std::enable_if_t<has_view_type_v<T>, int> = 0>
auto to_view(T&& value) {
  using U = std::remove_reference_t<T>;
  return view_traits<U>::make(std::forward<T>(value));
}

/// @brief Overload 2: Recursive transformation for visible ranges (ranges with viewable elements).
template <typename T, std::enable_if_t<!has_view_type_v<T> && is_visible_range_v<T>, int> = 0>
auto to_view(T&& value) {
  using element_view_type = decltype(to_view(std::declval<ranges::range_reference_t<T>>()));
  return view<element_view_type>{
      std::forward<T>(value) | ranges::views::transform([](auto&& elem) { return to_view(elem); })};
}

/// @brief Overload 3: Pass-through wrapping for opaque ranges (ranges with non-viewable elements).
template <typename T, std::enable_if_t<!has_view_type_v<T> && is_opaque_range_v<T>, int> = 0>
auto to_view(T&& value) {
  using element_type = std::remove_reference_t<ranges::range_reference_t<T>>;
  return view<element_type>{std::forward<T>(value)};
}

/// @brief Overload 4: Compile-time error for non-viewable, non-range types.
template <typename T, std::enable_if_t<!has_view_type_v<T> && !is_contiguous_range_v<T>, int> = 0>
void to_view(T&&) {
  static_assert(sizeof(T) == 0, "missing view_traits specialization?");
}

}  // namespace beluga_estimation
