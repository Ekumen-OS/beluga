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
 * @brief Defines core algebraic types, including Eigen and Sophus aliases.
 */

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>

#include <beluga_estimation/common/std_future.hpp>

namespace beluga_estimation {

// Forward declarations for beluga_estimation::Ref (defined later)
template <typename T, int Options = 0>
class Ref;

// --- Base Matrix Interface ---
template <typename Derived>
using MatrixBase = Eigen::MatrixBase<Derived>;

// --- Templated Row-Major Matrices ---
template <typename T, int Rows, int Cols>
using Matrix = Eigen::Matrix<T, Rows, Cols, Eigen::RowMajor>;

template <typename T>
using MatrixX = Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using Matrix1 = Matrix<T, 1, 1>;

template <typename T>
using Matrix2 = Matrix<T, 2, 2>;

template <typename T>
using Matrix3 = Matrix<T, 3, 3>;

template <typename T>
using Matrix4 = Matrix<T, 4, 4>;

template <typename T>
using Matrix5 = Matrix<T, 5, 5>;

template <typename T>
using Matrix6 = Matrix<T, 6, 6>;

// --- Dynamic Matrices ---
using MatrixXd = MatrixX<double>;
using MatrixXf = MatrixX<float>;

// --- Fixed Matrices ---
using Matrix1d = Matrix1<double>;
using Matrix1f = Matrix1<float>;

using Matrix2d = Matrix2<double>;
using Matrix3d = Matrix3<double>;
using Matrix4d = Matrix4<double>;
using Matrix5d = Matrix5<double>;
using Matrix6d = Matrix6<double>;

// --- Vectors ---
template <typename T, int Size>
using Vector = Eigen::Matrix<T, Size, 1>;

template <typename T>
using VectorX = Vector<T, Eigen::Dynamic>;

template <typename T>
using Vector1 = Vector<T, 1>;

template <typename T>
using Vector2 = Vector<T, 2>;

template <typename T>
using Vector3 = Vector<T, 3>;

template <typename T>
using Vector4 = Vector<T, 4>;

template <typename T>
using Vector5 = Vector<T, 5>;

template <typename T>
using Vector6 = Vector<T, 6>;

using VectorXd = VectorX<double>;
using VectorXf = VectorX<float>;

using Vector1d = Vector1<double>;
using Vector1f = Vector1<float>;

using Vector2d = Vector2<double>;
using Vector3d = Vector3<double>;
using Vector4d = Vector4<double>;
using Vector5d = Vector5<double>;
using Vector6d = Vector6<double>;

/// @brief Converts a 3D vector to its cross-product matrix representation.
///
/// Given a vector \f$v = (v_0, v_1, v_2)\f$, returns the \f$3 \times 3\f$
/// skew-symmetric matrix \f$[v]_\times\f$ such that \f$[v]_\times u = v \times u\f$
/// for any 3D vector \f$u\f$. This is commonly used to express cross products as
/// matrix multiplications (e.g. in rigid-body kinematics).
///
/// @tparam T The scalar type of the vector.
/// @param vector A 3-element vector.
/// @return The \f$3 \times 3\f$ cross-product matrix \f$[v]_\times\f$.
template <typename T>
Matrix3<T> ToCrossProductMatrix(const Ref<const Vector3<T>>& vector) {
  Matrix3<T> matrix;
  matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1), vector(0), 0;
  return matrix;
}

/// @brief Converts a 3D vector expression to its cross-product matrix representation.
///
/// Convenience overload that accepts any Eigen 3-vector expression (e.g. a
/// `Vector3d`, a block expression, or a `Sophus` translation member) and forwards
/// it to the @ref ToCrossProductMatrix(const Ref<const Vector3<T>>&) overload.
///
/// @tparam Derived The Eigen vector expression type.
/// @param vector A 3-element vector expression.
/// @return The \f$3 \times 3\f$ cross-product matrix \f$[v]_\times\f$.
template <typename Derived>
Matrix3<typename Derived::Scalar> ToCrossProductMatrix(const MatrixBase<Derived>& vector) {
  return ToCrossProductMatrix(Ref<const Vector3<typename Derived::Scalar>>(vector.derived()));
}

// --- Quaternions ---
template <typename T>
using Quaternion = Eigen::Quaternion<T>;

using Quaterniond = Quaternion<double>;
using Quaternionf = Quaternion<float>;

// --- Sophus Types ---
template <typename T>
using SO2 = Sophus::SO2<T>;
using SO2d = SO2<double>;
using SO2f = SO2<float>;

template <typename T>
using SO3 = Sophus::SO3<T>;
using SO3d = SO3<double>;
using SO3f = SO3<float>;

template <typename T>
using SE2 = Sophus::SE2<T>;
using SE2d = SE2<double>;
using SE2f = SE2<float>;

template <typename T>
using SE3 = Sophus::SE3<T>;
using SE3d = SE3<double>;
using SE3f = SE3<float>;

/**
 * @brief Projects an SE(3) pose onto the SE(2) plane.
 *
 * Keeps the x/y translation and the yaw (z component of the axis-angle
 * rotation vector). Accepts any SE(3) expression, including `SE3` objects,
 * `beluga_estimation::Ref` references, and `beluga_estimation::Map` views.
 *
 * @tparam Derived The SE(3) base expression type.
 * @param se3 The SE(3) pose to project.
 * @return The projected SE(2) pose.
 */
template <typename Derived>
inline SE2<typename Derived::Scalar> ToSE2(const Sophus::SE3Base<Derived>& se3) {
  using Scalar = typename Derived::Scalar;
  return SE2<Scalar>(SO2<Scalar>(se3.so3().log()(2)), se3.translation().template head<2>());
}

}  // namespace beluga_estimation

// Eigen::internal::traits for beluga_estimation::Ref<Sophus::XXX>
namespace Eigen {
namespace internal {

template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<Sophus::SO2<Scalar_, Options_>>>
    : traits<Eigen::Map<Sophus::SO2<Scalar_, Options_>>> {};

template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<const Sophus::SO2<Scalar_, Options_>>>
    : traits<Eigen::Map<const Sophus::SO2<Scalar_, Options_>>> {};

// SO3 traits
template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<Sophus::SO3<Scalar_, Options_>>>
    : traits<Eigen::Map<Sophus::SO3<Scalar_, Options_>>> {};

template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<const Sophus::SO3<Scalar_, Options_>>>
    : traits<Eigen::Map<const Sophus::SO3<Scalar_, Options_>>> {};

// SE2 traits
template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<Sophus::SE2<Scalar_, Options_>>>
    : traits<Eigen::Map<Sophus::SE2<Scalar_, Options_>>> {};

template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<const Sophus::SE2<Scalar_, Options_>>>
    : traits<Eigen::Map<const Sophus::SE2<Scalar_, Options_>>> {};

// SE3 traits
template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<Sophus::SE3<Scalar_, Options_>>>
    : traits<Eigen::Map<Sophus::SE3<Scalar_, Options_>>> {};

template <typename Scalar_, int Options_>
struct traits<beluga_estimation::Ref<const Sophus::SE3<Scalar_, Options_>>>
    : traits<Eigen::Map<const Sophus::SE3<Scalar_, Options_>>> {};

}  // namespace internal
}  // namespace Eigen

namespace beluga_estimation {

/**
 * @brief Unified map wrapper for Eigen and Sophus objects.
 *
 * `beluga_estimation::Map` is a thin wrapper over `Eigen::Map` with a dynamic-stride backend,
 * allowing explicit control of memory layout while preserving the mapped type API.
 * It can map:
 *
 * - **Eigen matrices/vectors** (fixed or dynamic size)
 * - **Sophus Lie groups** (`SO2`, `SO3`, `SE2`, `SE3`)
 *
 * ## Usage with fixed-size types
 *
 * ```cpp
 * std::array<double, 9> data{};
 * beluga_estimation::Map<beluga_estimation::Matrix3d> matrix_map(data.data());
 * matrix_map.setIdentity();
 * ```
 *
 * ## Usage with dynamic-size matrices
 *
 * ```cpp
 * std::vector<double> data(6);
 * beluga_estimation::Map<beluga_estimation::MatrixXd> matrix_map(data.data(), 2, 3);
 * matrix_map << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
 * ```
 *
 * ## Usage with spans
 *
 * ```cpp
 * std::vector<double> data{1.0, 2.0, 3.0};
 * beluga_estimation::Map<beluga_estimation::Vector3d> vector_map(beluga_estimation::to_view(data));
 * ```
 *
 * ## Usage with mdspans
 *
 * ```cpp
 * beluga_estimation::MatrixXd matrix(2, 2);
 * auto matrix_map = beluga_estimation::Map<beluga_estimation::MatrixXd>(beluga_estimation::to_view(matrix));
 * ```
 *
 * ## Usage with Sophus types
 *
 * ```cpp
 * std::vector<double> se2_data = {1.0, 0.0, 2.0, 3.0};
 * beluga_estimation::Map<beluga_estimation::SE2d> se2_map(se2_data.data());
 * auto translation = se2_map.translation();
 * ```
 *
 * ## Design rationale
 *
 * - **Interoperability**: Common map type for Eigen and Sophus code paths
 * - **Zero-copy**: Views external memory without allocation
 * - **Stride-aware**: Supports custom and non-contiguous layouts
 * - **Compatibility**: Preserves Eigen/Sophus mapped object behavior
 *
 * @tparam Derived Mapped Eigen or Sophus type.
 * @tparam Options Eigen alignment options (default = 0).
 *
 * @note Fixed-size constructors require compile-time fixed size and non-dynamic inner stride.
 * @note Runtime shape constructors support explicit rows/cols or vector size.
 *
 * @see Eigen::Map for Eigen map semantics
 * @see beluga_estimation::Ref for unified reference semantics
 */
template <typename Derived, int Options = 0>
class Map : public Eigen::Map<Derived, Options, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> {
 public:
  using StrideType = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
  using Base = Eigen::Map<Derived, Options, StrideType>;
  using PointerArgType = typename Base::PointerArgType;

  explicit Map(PointerArgType data)
      : Base(
            data,
            StrideType(
                Derived::IsRowMajor ? Derived::ColsAtCompileTime : Derived::RowsAtCompileTime,
                Derived::InnerStrideAtCompileTime)) {
    EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived)
    static_assert(Derived::InnerStrideAtCompileTime != Eigen::Dynamic);
  }

  Map(PointerArgType data, const StrideType& stride) : Base(data, stride) {}

  Map(PointerArgType data, Eigen::Index size) : Base(data, size, StrideType(size, Derived::InnerStrideAtCompileTime)) {
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
    static_assert(Derived::InnerStrideAtCompileTime != Eigen::Dynamic);
  }

  Map(PointerArgType data, Eigen::Index size, const StrideType& stride) : Base(data, size, stride) {}

  template <typename U, auto N, std::enable_if_t<std::is_convertible_v<U*, PointerArgType>, int> = 0>
  explicit Map(std_future::span<U, N> span) : Map(span.data(), static_cast<Eigen::Index>(span.size())) {}

  Map(PointerArgType data, Eigen::Index rows, Eigen::Index cols)
      : Base(data, rows, cols, StrideType(Derived::IsRowMajor ? cols : rows, Derived::InnerStrideAtCompileTime)) {
    static_assert(Derived::InnerStrideAtCompileTime != Eigen::Dynamic);
  }

  template <typename U, std::enable_if_t<std::is_convertible_v<U*, PointerArgType>, int> = 0>
  explicit Map(std_future::mdspan<U, 2> span)
      : Map(span.data_handle(),
            static_cast<Eigen::Index>(span.extent(0)),
            static_cast<Eigen::Index>(span.extent(1)),
            StrideType(static_cast<Eigen::Index>(span.stride(0)), static_cast<Eigen::Index>(span.stride(1)))) {}

  Map(const Eigen::Map<Derived, Options>& other) : Map(other.data(), other.rows(), other.cols()) {}

  Map(PointerArgType data, Eigen::Index rows, Eigen::Index cols, const StrideType& stride)
      : Base(data, rows, cols, stride) {}

  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<Eigen::Quaternion<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<Eigen::Quaternion<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<Eigen::Quaternion<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<const Eigen::Quaternion<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<const Eigen::Quaternion<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<const Eigen::Quaternion<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

// ----------------------------------------------------------------------------
// Sophus Map specializations
// ----------------------------------------------------------------------------

template <typename Scalar_, int Options_, int MapOptions>
class Map<Sophus::SO2<Scalar_, Options_>, MapOptions> : public Eigen::Map<Sophus::SO2<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<Sophus::SO2<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<const Sophus::SO2<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<const Sophus::SO2<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<const Sophus::SO2<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<Sophus::SO3<Scalar_, Options_>, MapOptions> : public Eigen::Map<Sophus::SO3<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<Sophus::SO3<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<const Sophus::SO3<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<const Sophus::SO3<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<const Sophus::SO3<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<Sophus::SE2<Scalar_, Options_>, MapOptions> : public Eigen::Map<Sophus::SE2<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<Sophus::SE2<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<const Sophus::SE2<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<const Sophus::SE2<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<const Sophus::SE2<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<Sophus::SE3<Scalar_, Options_>, MapOptions> : public Eigen::Map<Sophus::SE3<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<Sophus::SE3<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

template <typename Scalar_, int Options_, int MapOptions>
class Map<const Sophus::SE3<Scalar_, Options_>, MapOptions>
    : public Eigen::Map<const Sophus::SE3<Scalar_, Options_>, MapOptions> {
 public:
  using Base = Eigen::Map<const Sophus::SE3<Scalar_, Options_>, MapOptions>;

  using Base::Base;
  using Base::operator=;
};

}  // namespace beluga_estimation

/**
 * @brief Unified reference wrapper for Eigen and Sophus types.
 *
 * `beluga_estimation::Ref` provides zero-copy reference semantics for both Eigen matrices/vectors
 * and Sophus Lie group types. It automatically dispatches to the appropriate implementation:
 *
 * - **For Eigen types**: Wraps `Eigen::Ref`, providing all its functionality
 * - **For Sophus types**: Custom implementation using `Eigen::Map` internally
 *
 * This design allows writing generic code that works uniformly with both Eigen and Sophus types.
 *
 * ## Usage with Eigen Types
 *
 * For Eigen matrices and vectors, `beluga_estimation::Ref` forwards to `Eigen::Ref`:
 *
 * ```cpp
 * beluga_estimation::Matrix3d matrix = beluga_estimation::Matrix3d::Identity();
 * beluga_estimation::Ref<beluga_estimation::Matrix3d> mat_ref(matrix);
 * mat_ref(0, 0) = 2.0;  // Modifies original matrix
 * ```
 *
 * ## Usage with Sophus Types
 *
 * For Sophus Lie groups (SO2, SO3, SE2, SE3), `beluga_estimation::Ref` provides custom specializations:
 *
 * ```cpp
 * beluga_estimation::SE2d se2(M_PI / 2.0, beluga_estimation::Vector2d{1.0, 2.0});
 * beluga_estimation::Ref<beluga_estimation::SE2d> se2_ref(se2);
 *
 * // Access components
 * auto rotation = se2_ref.so2();
 * auto translation = se2_ref.translation();
 *
 * // Modify translation (modifies original se2)
 * se2_ref.translation().x() = 3.0;
 *
 * // Perform Lie group operations
 * auto log_rotation = se2_ref.so2().log();
 * auto inverse = se2_ref.inverse();
 * ```
 *
 * ## Const References
 *
 * Both mutable and const references are supported:
 *
 * ```cpp
 * const beluga_estimation::SO2d so2(M_PI / 4.0);
 * beluga_estimation::Ref<const beluga_estimation::SO2d> const_ref(so2);
 * double angle = const_ref.log();  // Read-only access
 * ```
 *
 * ## Compatibility with Eigen::Map and Sophus::Map
 *
 * `beluga_estimation::Ref` can be constructed from both owned objects and Map views:
 *
 * ```cpp
 * std::vector<double> data(4);
 * Eigen::Map<beluga_estimation::SE2d> map(data.data());
 * beluga_estimation::Ref<beluga_estimation::SE2d> ref(map);  // Works with Map
 * ```
 *
 * ## Design Rationale
 *
 * - **Unified API**: Write generic functions accepting `beluga_estimation::Ref<T>` for any type T
 * - **Zero-copy**: References existing data without allocation
 * - **Type safety**: Compile-time dispatch to correct implementation
 * - **Seamless integration**: Works with existing Eigen and Sophus code
 *
 * @tparam T The type to reference (Eigen matrix/vector or Sophus Lie group)
 * @tparam Options Alignment options (default = 0)
 *
 * @note For Sophus types, `beluga_estimation::Ref` uses `Eigen::Map` internally for component storage.
 * @note Sophus-specific operations (log, exp, inverse, etc.) are inherited from Sophus base classes.
 *
 * @see Eigen::Ref for Eigen-specific reference semantics
 * @see Sophus::Map for Sophus-specific Map implementation
 */
namespace beluga_estimation {

// ----------------------------------------------------------------------------
// Primary template: Forward to Eigen::Ref for Eigen types
// ----------------------------------------------------------------------------

/// Primary template forwards to Eigen::Ref for Eigen matrices and vectors.
template <typename Derived, int Options>
class Ref : public Eigen::Ref<Derived, Options, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> {
 public:
  using Base = Eigen::Ref<Derived, Options, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>>;

  using Base::Base;
  using Base::operator=;
};

// ----------------------------------------------------------------------------
// SO2 Ref specializations
// ----------------------------------------------------------------------------

/**
 * @brief Mutable reference to SO(2) rotation.
 *
 * Provides zero-copy reference semantics for 2D rotations. Supports all SO(2)
 * operations including composition, inverse, logarithm, and exponential maps.
 *
 * @tparam Scalar_ Scalar type (e.g., double, float)
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<Sophus::SO2<Scalar_, Options_>> : public Sophus::SO2Base<Ref<Sophus::SO2<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SO2Base<Ref<Sophus::SO2<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  /// Base is friend so unit_complex_nonconst() can be accessed from Base.
  friend class Sophus::SO2Base<Ref<Sophus::SO2<Scalar_, Options_>>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  /// Construct from mutable SO(2) object.
  SOPHUS_FUNC Ref(Sophus::SO2<Scalar_, Options_>& so2) : unit_complex_(so2.data()) {}

  /// Construct from mutable Eigen::Map of SO(2).
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SO2<Scalar_, Options_>> map) : unit_complex_(map.data()) {}

  /// Access the unit complex representation (read-only).
  SOPHUS_FUNC const Eigen::Map<Sophus::Vector2<Scalar_, Options_>, Options_>& unit_complex() const {
    return unit_complex_;
  }

  /// Access the unit complex representation (mutable).
  SOPHUS_FUNC Scalar* data() { return unit_complex_.data(); }

  /// Access the unit complex representation (read-only).
  SOPHUS_FUNC const Scalar* data() const { return unit_complex_.data(); }

 protected:
  /// Mutator of unit_complex is protected to ensure class invariant.
  SOPHUS_FUNC Eigen::Map<Sophus::Vector2<Scalar_, Options_>, Options_>& unit_complex_nonconst() {
    return unit_complex_;
  }

  Eigen::Map<Sophus::Vector2<Scalar_, Options_>, Options_> unit_complex_;
};

/**
 * @brief Const reference to SO(2) rotation.
 *
 * Provides read-only zero-copy reference semantics for 2D rotations.
 *
 * @tparam Scalar_ Scalar type (e.g., double, float)
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<const Sophus::SO2<Scalar_, Options_>> : public Sophus::SO2Base<Ref<const Sophus::SO2<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SO2Base<Ref<const Sophus::SO2<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*;

  /// Construct from const SO2
  SOPHUS_FUNC Ref(const Sophus::SO2<Scalar_, Options_>& so2) : unit_complex_(so2.data()) {}

  /// Construct from const Map<SO2>
  SOPHUS_FUNC Ref(const Eigen::Map<const Sophus::SO2<Scalar_, Options_>>& map) : unit_complex_(map.unit_complex()) {}

  /// Accessor of unit_complex
  SOPHUS_FUNC const Eigen::Map<const Sophus::Vector2<Scalar_, Options_>, Options_>& unit_complex() const {
    return unit_complex_;
  }

  /// Access the unit complex representation (read-only).
  SOPHUS_FUNC const Scalar* data() const { return unit_complex_.data(); }

 protected:
  Eigen::Map<const Sophus::Vector2<Scalar_, Options_>, Options_> unit_complex_;
};

/**
 * @brief Mutable reference to SO(3) rotation.
 *
 * Provides zero-copy reference semantics for 3D rotations using quaternions.
 *
 * @tparam Scalar_ Scalar type (e.g., double, float)
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<Sophus::SO3<Scalar_, Options_>> : public Sophus::SO3Base<Ref<Sophus::SO3<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SO3Base<Ref<Sophus::SO3<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  /// Base is friend so unit_quaternion_nonconst() can be accessed from Base.
  friend class Sophus::SO3Base<Ref<Sophus::SO3<Scalar_, Options_>>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  /// Construct from mutable SO3
  SOPHUS_FUNC Ref(Sophus::SO3<Scalar_, Options_>& so3) : unit_quaternion_(so3.data()) {}

  /// Construct from mutable Map<SO3>
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SO3<Scalar_, Options_>> map) : unit_quaternion_(map.unit_quaternion()) {}

  /// Accessor of unit_quaternion
  SOPHUS_FUNC const Eigen::Map<Eigen::Quaternion<Scalar_>, Options_>& unit_quaternion() const {
    return unit_quaternion_;
  }

  /// Access the unit quaternion representation (mutable).
  SOPHUS_FUNC Scalar* data() { return unit_quaternion_.coeffs().data(); }

  /// Access the unit quaternion representation (read-only).
  SOPHUS_FUNC const Scalar* data() const { return unit_quaternion_.coeffs().data(); }

 protected:
  /// Mutator of unit_quaternion is protected to ensure class invariant.
  SOPHUS_FUNC Eigen::Map<Eigen::Quaternion<Scalar_>, Options_>& unit_quaternion_nonconst() { return unit_quaternion_; }

  Eigen::Map<Eigen::Quaternion<Scalar_>, Options_> unit_quaternion_;
};

/**
 * @brief Const reference to SO(3) rotation.
 *
 * Provides read-only zero-copy reference semantics for 3D rotations.
 *
 * @tparam Scalar_ Scalar type
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<const Sophus::SO3<Scalar_, Options_>> : public Sophus::SO3Base<Ref<const Sophus::SO3<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SO3Base<Ref<const Sophus::SO3<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;

  using Base::operator*;

  /// Construct from const SO3
  SOPHUS_FUNC Ref(const Sophus::SO3<Scalar_, Options_>& so3) : unit_quaternion_(so3.data()) {}

  /// Construct from const Map<SO3>
  SOPHUS_FUNC Ref(const Eigen::Map<const Sophus::SO3<Scalar_, Options_>>& map)
      : unit_quaternion_(map.unit_quaternion()) {}

  /// Construct from mutable Map<SO3>
  SOPHUS_FUNC Ref(const Eigen::Map<Sophus::SO3<Scalar_, Options_>> map) : unit_quaternion_(map.unit_quaternion()) {}

  /// Accessor of unit_quaternion
  SOPHUS_FUNC const Eigen::Map<const Eigen::Quaternion<Scalar_>, Options_>& unit_quaternion() const {
    return unit_quaternion_;
  }

  /// Access the unit quaternion representation (read-only).
  SOPHUS_FUNC const Scalar* data() const { return unit_quaternion_.coeffs().data(); }

 protected:
  Eigen::Map<const Eigen::Quaternion<Scalar_>, Options_> unit_quaternion_;
};

/**
 * @brief Mutable reference to SE(2) transformation.
 *
 * Provides zero-copy reference semantics for 2D rigid body transformations (rotation + translation).
 * Supports access to rotation (SO2) and translation components.
 *
 * @tparam Scalar_ Scalar type
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<Sophus::SE2<Scalar_, Options_>> : public Sophus::SE2Base<Ref<Sophus::SE2<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SE2Base<Ref<Sophus::SE2<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using SO2Member = Eigen::Map<Sophus::SO2<Scalar_, Options_>, Options_>;
  using TranslationMember = Eigen::Map<Sophus::Vector2<Scalar_, Options_>, Options_>;

  /// Base is friend so so2() and translation() can be accessed from Base.
  friend class Sophus::SE2Base<Ref<Sophus::SE2<Scalar_, Options_>>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  /// Construct from mutable SE2
  SOPHUS_FUNC Ref(Sophus::SE2<Scalar_, Options_>& se2)
      : so2_(se2.data()), translation_(se2.data() + Sophus::SO2<Scalar_>::num_parameters) {}

  /// Construct from mutable Map<SE2>
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SE2<Scalar_, Options_>> map) : so2_(map.so2()), translation_(map.translation()) {}

  /// Mutator of SO2
  SOPHUS_FUNC SO2Member& so2() { return so2_; }

  /// Accessor of SO2
  SOPHUS_FUNC const SO2Member& so2() const { return so2_; }

  /// Mutator of translation vector
  SOPHUS_FUNC TranslationMember& translation() { return translation_; }

  /// Accessor of translation vector
  SOPHUS_FUNC const TranslationMember& translation() const { return translation_; }

  /// Access the raw data pointer (mutable).
  SOPHUS_FUNC Scalar* data() { return so2_.data(); }

  /// Access the raw data pointer (read-only).
  SOPHUS_FUNC const Scalar* data() const { return so2_.data(); }

 protected:
  SO2Member so2_;
  TranslationMember translation_;
};

/**
 * @brief Const reference to SE(2) transformation.
 *
 * Provides read-only zero-copy reference semantics for 2D rigid body transformations.
 *
 * @tparam Scalar_ Scalar type
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<const Sophus::SE2<Scalar_, Options_>> : public Sophus::SE2Base<Ref<const Sophus::SE2<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SE2Base<Ref<const Sophus::SE2<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using SO2Member = Eigen::Map<const Sophus::SO2<Scalar_, Options_>, Options_>;
  using TranslationMember = Eigen::Map<const Sophus::Vector2<Scalar_, Options_>, Options_>;

  using Base::operator*;

  /// Construct from const SE2
  SOPHUS_FUNC Ref(const Sophus::SE2<Scalar_, Options_>& se2)
      : so2_(se2.data()), translation_(se2.data() + Sophus::SO2<Scalar_>::num_parameters) {}

  /// Construct from const Map<SE2>
  SOPHUS_FUNC Ref(const Eigen::Map<const Sophus::SE2<Scalar_, Options_>>& map)
      : so2_(map.so2()), translation_(map.translation()) {}

  /// Construct from mutable Map<SE2>
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SE2<Scalar_, Options_>> map) : so2_(map.so2()), translation_(map.translation()) {}

  /// Accessor of SO2
  SOPHUS_FUNC const SO2Member& so2() const { return so2_; }

  /// Accessor of translation vector
  SOPHUS_FUNC const TranslationMember& translation() const { return translation_; }

  /// Access the raw data pointer (read-only).
  SOPHUS_FUNC const Scalar* data() const { return so2_.data(); }

 protected:
  const SO2Member so2_;
  const TranslationMember translation_;
};

/**
 * @brief Mutable reference to SE(3) transformation.
 *
 * Provides zero-copy reference semantics for 3D rigid body transformations (rotation + translation).
 * Supports access to rotation (SO3) and translation components.
 *
 * @tparam Scalar_ Scalar type
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<Sophus::SE3<Scalar_, Options_>> : public Sophus::SE3Base<Ref<Sophus::SE3<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SE3Base<Ref<Sophus::SE3<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using SO3Member = Eigen::Map<Sophus::SO3<Scalar_, Options_>, Options_>;
  using TranslationMember = Eigen::Map<Sophus::Vector3<Scalar_, Options_>, Options_>;

  /// Base is friend so so3() and translation() can be accessed from Base.
  friend class Sophus::SE3Base<Ref<Sophus::SE3<Scalar_, Options_>>>;

  using Base::operator=;
  using Base::operator*=;
  using Base::operator*;

  /// Construct from mutable SE3
  SOPHUS_FUNC Ref(Sophus::SE3<Scalar_, Options_>& se3)
      : so3_(se3.data()), translation_(se3.data() + Sophus::SO3<Scalar_>::num_parameters) {}

  /// Construct from mutable Map<SE3>
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SE3<Scalar_, Options_>> map) : so3_(map.so3()), translation_(map.translation()) {}

  /// Mutator of SO3
  SOPHUS_FUNC SO3Member& so3() { return so3_; }

  /// Accessor of SO3
  SOPHUS_FUNC const SO3Member& so3() const { return so3_; }

  /// Mutator of translation vector
  SOPHUS_FUNC TranslationMember& translation() { return translation_; }

  /// Accessor of translation vector
  SOPHUS_FUNC const TranslationMember& translation() const { return translation_; }

  /// Access the raw data pointer (mutable).
  SOPHUS_FUNC Scalar* data() { return so3_.data(); }

  /// Access the raw data pointer (read-only).
  SOPHUS_FUNC const Scalar* data() const { return so3_.data(); }

 protected:
  SO3Member so3_;
  TranslationMember translation_;
};

/**
 * @brief Const reference to SE(3) transformation.
 *
 * Provides read-only zero-copy reference semantics for 3D rigid body transformations.
 *
 * @tparam Scalar_ Scalar type
 * @tparam Options_ Eigen alignment options
 */
template <typename Scalar_, int Options_>
class Ref<const Sophus::SE3<Scalar_, Options_>> : public Sophus::SE3Base<Ref<const Sophus::SE3<Scalar_, Options_>>> {
 public:
  using Base = Sophus::SE3Base<Ref<const Sophus::SE3<Scalar_, Options_>>>;
  using Scalar = Scalar_;
  using Transformation = typename Base::Transformation;
  using Point = typename Base::Point;
  using HomogeneousPoint = typename Base::HomogeneousPoint;
  using Tangent = typename Base::Tangent;
  using Adjoint = typename Base::Adjoint;
  using SO3Member = Eigen::Map<const Sophus::SO3<Scalar_, Options_>, Options_>;
  using TranslationMember = Eigen::Map<const Sophus::Vector3<Scalar_, Options_>, Options_>;

  using Base::operator*;

  /// Construct from const SE3
  SOPHUS_FUNC Ref(const Sophus::SE3<Scalar_, Options_>& se3)
      : so3_(se3.data()), translation_(se3.data() + Sophus::SO3<Scalar_>::num_parameters) {}

  /// Construct from const Map<SE3>
  SOPHUS_FUNC Ref(const Eigen::Map<const Sophus::SE3<Scalar_, Options_>>& map)
      : so3_(map.so3()), translation_(map.translation()) {}

  /// Construct from mutable Map<SE3>
  SOPHUS_FUNC Ref(Eigen::Map<Sophus::SE3<Scalar_, Options_>> map) : so3_(map.so3()), translation_(map.translation()) {}

  /// Accessor of SO3
  SOPHUS_FUNC const SO3Member& so3() const { return so3_; }

  /// Accessor of translation vector
  SOPHUS_FUNC const TranslationMember& translation() const { return translation_; }

  /// Access the raw data pointer (read-only).
  SOPHUS_FUNC const Scalar* data() const { return so3_.data(); }

 protected:
  const SO3Member so3_;
  const TranslationMember translation_;
};

}  // namespace beluga_estimation
