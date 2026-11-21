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

#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <range/v3/range/concepts.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/views.hpp>

namespace {

// ============================================================================
// 1. TYPE CLASSIFICATION SYSTEM
// ============================================================================

// Test the type trait system that determines conversion strategy
TEST(TypeTraits, HasViewType) {
  // Types WITH explicit view_traits specializations
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::Vector3d>, "Vector3d should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::VectorXd>, "VectorXd should have view_traits");
  static_assert(
      beluga_estimation::has_view_type_v<beluga_estimation::Matrix<double, 2, 3>>,
      "Fixed matrix should have view_traits");
  static_assert(
      beluga_estimation::has_view_type_v<beluga_estimation::MatrixX<double>>, "Dynamic matrix should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::SE2d>, "SE2d should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::SO2d>, "SO2d should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::SE3d>, "SE3d should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<beluga_estimation::SO3d>, "SO3d should have view_traits");

  // Arithmetic containers have view_traits specializations
  static_assert(beluga_estimation::has_view_type_v<std::vector<double>>, "vector<double> should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<std::array<float, 5>>, "array<float> should have view_traits");
  static_assert(beluga_estimation::has_view_type_v<std::vector<int>>, "vector<int> should have view_traits");

  // Types WITHOUT view_traits specializations
  static_assert(!beluga_estimation::has_view_type_v<double>, "Scalar types should NOT have view_traits");
  static_assert(!beluga_estimation::has_view_type_v<int>, "Scalar types should NOT have view_traits");
  static_assert(!beluga_estimation::has_view_type_v<std::string>, "std::string should NOT have view_traits");

  // Custom struct without view_traits
  struct CustomData {
    int id;
    double value;
  };
  static_assert(!beluga_estimation::has_view_type_v<CustomData>, "Custom struct should NOT have view_traits");

  // Ranges of non-arithmetic types don't have direct view_traits (will be classified differently)
  static_assert(
      !beluga_estimation::has_view_type_v<std::vector<CustomData>>, "Range of custom types lacks view_traits");
  static_assert(
      !beluga_estimation::has_view_type_v<std::vector<beluga_estimation::Vector3d>>,
      "Range of viewable types lacks direct view_traits");
}

TEST(TypeTraits, IsContiguousRange) {
  // Contiguous ranges: random_access_range && sized_range
  static_assert(beluga_estimation::is_contiguous_range_v<std::vector<int>>, "vector is contiguous range");
  static_assert(beluga_estimation::is_contiguous_range_v<std::array<int, 5>>, "array is contiguous range");
  static_assert(
      beluga_estimation::is_contiguous_range_v<std::vector<beluga_estimation::Vector3d>>,
      "vector of Vector3d is contiguous range");

  // Eigen vectors satisfy range concepts
  static_assert(
      beluga_estimation::is_contiguous_range_v<beluga_estimation::Vector3d>, "Vector3d satisfies range concepts");

  // Non-ranges are not contiguous ranges
  static_assert(!beluga_estimation::is_contiguous_range_v<int>, "scalar is not a range");
  static_assert(!beluga_estimation::is_contiguous_range_v<double>, "scalar is not a range");
}

TEST(TypeTraits, VisibleAndOpaqueRanges) {
  // Visible ranges: contiguous ranges with viewable elements (recursive transform)
  static_assert(
      beluga_estimation::is_visible_range_v<std::vector<beluga_estimation::Vector3d>>,
      "vector<Vector3d> is visible range");
  static_assert(
      beluga_estimation::is_visible_range_v<std::array<beluga_estimation::SE2d, 10>>, "array<SE2d> is visible range");
  static_assert(
      beluga_estimation::is_visible_range_v<std::vector<std::vector<double>>>,
      "nested vector<vector<double>> is visible range");

  // Opaque ranges: contiguous ranges with non-viewable elements (pass-through)
  struct CustomData {
    int id;
  };
  static_assert(beluga_estimation::is_opaque_range_v<std::vector<CustomData>>, "vector<CustomData> is opaque range");
  static_assert(beluga_estimation::is_opaque_range_v<std::vector<std::string>>, "vector<string> is opaque range");

  // Arithmetic containers: classified as opaque (elements are scalars without view_traits)
  // but have direct view_traits specialization, so overload 1 takes precedence
  static_assert(beluga_estimation::is_opaque_range_v<std::vector<double>>, "vector<double> elements are opaque");
  static_assert(beluga_estimation::is_opaque_range_v<std::vector<int>>, "vector<int> elements are opaque");
  static_assert(
      !beluga_estimation::is_visible_range_v<std::vector<double>>, "vector<double> elements lack view_traits");

  // Eigen vectors: have view_traits but also satisfy range concepts
  static_assert(
      !beluga_estimation::is_visible_range_v<beluga_estimation::Vector3d>,
      "Vector3d is not a visible range (scalar elements)");
  static_assert(
      beluga_estimation::is_opaque_range_v<beluga_estimation::Vector3d>, "Vector3d is opaque range (scalar elements)");
}

TEST(TypeTraits, HasVisibleValue) {
  // Ranges with visible values (elements have view_traits or are visible ranges)
  static_assert(
      beluga_estimation::has_visible_value<std::vector<beluga_estimation::Vector3d>>::value,
      "vector<Vector3d> elements are visible");
  static_assert(
      beluga_estimation::has_visible_value<std::vector<std::vector<double>>>::value,
      "nested vector elements are visible");

  // Ranges with opaque values (elements lack view_traits and aren't visible ranges)
  struct CustomData {
    int id;
  };
  static_assert(
      !beluga_estimation::has_visible_value<std::vector<CustomData>>::value, "CustomData elements are opaque");
  static_assert(!beluga_estimation::has_visible_value<std::vector<double>>::value, "double elements are opaque");
}

TEST(TypeTraits, HasOpaqueValue) {
  // Ranges with opaque values
  struct CustomData {
    int id;
  };
  static_assert(beluga_estimation::has_opaque_value<std::vector<CustomData>>::value, "CustomData elements are opaque");
  static_assert(beluga_estimation::has_opaque_value<std::vector<std::string>>::value, "string elements are opaque");
  static_assert(beluga_estimation::has_opaque_value<std::vector<double>>::value, "double elements are opaque");
  static_assert(beluga_estimation::has_opaque_value<std::vector<int>>::value, "int elements are opaque");

  // Ranges with visible values
  static_assert(
      !beluga_estimation::has_opaque_value<std::vector<beluga_estimation::Vector3d>>::value,
      "Vector3d elements are NOT opaque");
}

// ============================================================================
// 2. DIRECT CONVERSIONS (Overload 1: has_view_type_v<T>)
// ============================================================================

// Test types with explicit view_traits specializations
// Test both read and write access, plus const-correctness

TEST(DirectConversion, VectorFixedSize) {
  // Mutable vector: read and write
  beluga_estimation::Vector3d vec(1.0, 2.0, 3.0);
  beluga_estimation::std_future::span<double, 3> view = beluga_estimation::to_view(vec);

  EXPECT_EQ(view.size(), 3);
  EXPECT_DOUBLE_EQ(view[0], 1.0);
  EXPECT_DOUBLE_EQ(view[2], 3.0);

  // Verify write access modifies underlying data
  view[1] = 99.0;
  EXPECT_DOUBLE_EQ(vec[1], 99.0);

  // Const vector: read-only
  const beluga_estimation::Vector3d const_vec(4.0, 5.0, 6.0);
  beluga_estimation::std_future::span<const double, 3> const_view = beluga_estimation::to_view(const_vec);

  EXPECT_EQ(const_view.size(), 3);
  EXPECT_DOUBLE_EQ(const_view[0], 4.0);
}

TEST(DirectConversion, VectorDynamicSize) {
  // Mutable dynamic vector
  beluga_estimation::VectorXd vec = beluga_estimation::VectorXd::LinSpaced(5, 0.0, 4.0);
  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(vec);

  EXPECT_EQ(view.size(), 5);
  EXPECT_DOUBLE_EQ(view[0], 0.0);
  EXPECT_DOUBLE_EQ(view[4], 4.0);

  view[2] = 100.0;
  EXPECT_DOUBLE_EQ(vec[2], 100.0);

  // Const dynamic vector
  const beluga_estimation::VectorXd& const_vec = vec;
  beluga_estimation::std_future::span<const double> const_view = beluga_estimation::to_view(const_vec);

  EXPECT_EQ(const_view.size(), 5);
}

TEST(DirectConversion, MatrixFixedSizeRowMajor) {
  // Fixed-size matrix with row-major layout
  beluga_estimation::Matrix<double, 2, 3> mat;
  mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  beluga_estimation::std_future::mdspan<double, 2> view = beluga_estimation::to_view(mat);

  // Verify dimensions
  EXPECT_EQ(view.extent(0), 2);
  EXPECT_EQ(view.extent(1), 3);

  // Verify row-major layout: row 0 = [1, 2, 3], row 1 = [4, 5, 6]
  EXPECT_DOUBLE_EQ(view(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(view(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(view(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(view(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(view(1, 1), 5.0);
  EXPECT_DOUBLE_EQ(view(1, 2), 6.0);

  // Verify write access
  view(1, 2) = 99.0;
  EXPECT_DOUBLE_EQ(mat(1, 2), 99.0);

  // Const matrix
  const beluga_estimation::Matrix<double, 2, 3>& const_mat = mat;
  beluga_estimation::std_future::mdspan<const double, 2> const_view = beluga_estimation::to_view(const_mat);
}

TEST(DirectConversion, MatrixDynamicSize) {
  beluga_estimation::MatrixX<double> mat(3, 2);
  mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  beluga_estimation::std_future::mdspan<double, 2> view = beluga_estimation::to_view(mat);

  EXPECT_EQ(view.extent(0), 3);
  EXPECT_EQ(view.extent(1), 2);

  // Verify row-major: [1,2], [3,4], [5,6]
  EXPECT_DOUBLE_EQ(view(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(view(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(view(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(view(2, 1), 6.0);
}

TEST(DirectConversion, MatrixBlocks) {
  beluga_estimation::MatrixXd mat(3, 3);
  mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  {
    beluga_estimation::Ref<beluga_estimation::MatrixXd> block = mat.topLeftCorner(2, 2);
    beluga_estimation::std_future::mdspan<double, 2> view = beluga_estimation::to_view(block);

    EXPECT_EQ(view.extent(0), 2);
    EXPECT_EQ(view.extent(1), 2);

    EXPECT_DOUBLE_EQ(view(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(view(0, 1), 2.0);
    EXPECT_DOUBLE_EQ(view(1, 0), 4.0);
    EXPECT_DOUBLE_EQ(view(1, 1), 5.0);
  }

  {
    beluga_estimation::Ref<beluga_estimation::MatrixXd> block = mat.bottomRightCorner(2, 2);
    beluga_estimation::std_future::mdspan<double, 2> view = beluga_estimation::to_view(block);

    EXPECT_EQ(view.extent(0), 2);
    EXPECT_EQ(view.extent(1), 2);

    EXPECT_DOUBLE_EQ(view(0, 0), 5.0);
    EXPECT_DOUBLE_EQ(view(0, 1), 6.0);
    EXPECT_DOUBLE_EQ(view(1, 0), 8.0);
    EXPECT_DOUBLE_EQ(view(1, 1), 9.0);
  }
}

TEST(DirectConversion, LieGroupSE2) {
  // SE2: [cos(theta), sin(theta), x, y]
  beluga_estimation::SE2d se2(beluga_estimation::SO2d(0.1), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(se2);

  EXPECT_EQ(view.size(), 4);
  EXPECT_NEAR(view[0], std::cos(0.1), 1e-10);
  EXPECT_NEAR(view[1], std::sin(0.1), 1e-10);
  EXPECT_DOUBLE_EQ(view[2], 1.0);
  EXPECT_DOUBLE_EQ(view[3], 2.0);

  // Verify write access
  view[2] = 10.0;
  EXPECT_DOUBLE_EQ(se2.data()[2], 10.0);

  // Const SE2
  const beluga_estimation::SE2d const_se2(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(3.0, 4.0));
  beluga_estimation::std_future::span<const double> const_view = beluga_estimation::to_view(const_se2);

  EXPECT_NEAR(const_view[0], std::cos(0.2), 1e-10);
}

TEST(DirectConversion, LieGroupSO2) {
  // SO2: [cos(theta), sin(theta)]
  beluga_estimation::SO2d so2(0.5);
  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(so2);

  EXPECT_EQ(view.size(), 2);
  EXPECT_NEAR(view[0], std::cos(0.5), 1e-10);
  EXPECT_NEAR(view[1], std::sin(0.5), 1e-10);
}

TEST(DirectConversion, LieGroupSE3) {
  // SE3: [qx, qy, qz, qw, x, y, z]
  beluga_estimation::Vector3d translation(1.0, 2.0, 3.0);
  beluga_estimation::Quaterniond rotation(1.0, 0.0, 0.0, 0.0);  // Identity: w=1, x=y=z=0
  beluga_estimation::SE3d se3(rotation, translation);

  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(se3);

  EXPECT_EQ(view.size(), 7);
  // Quaternion: qx, qy, qz, qw
  EXPECT_DOUBLE_EQ(view[0], 0.0);  // qx
  EXPECT_DOUBLE_EQ(view[1], 0.0);  // qy
  EXPECT_DOUBLE_EQ(view[2], 0.0);  // qz
  EXPECT_DOUBLE_EQ(view[3], 1.0);  // qw
  // Translation: x, y, z
  EXPECT_DOUBLE_EQ(view[4], 1.0);
  EXPECT_DOUBLE_EQ(view[5], 2.0);
  EXPECT_DOUBLE_EQ(view[6], 3.0);
}

TEST(DirectConversion, LieGroupSO3) {
  // SO3: [qx, qy, qz, qw]
  beluga_estimation::Quaterniond rotation(1.0, 0.0, 0.0, 0.0);  // Identity
  beluga_estimation::SO3d so3(rotation);

  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(so3);

  EXPECT_EQ(view.size(), 4);
  EXPECT_DOUBLE_EQ(view[0], 0.0);  // qx
  EXPECT_DOUBLE_EQ(view[1], 0.0);  // qy
  EXPECT_DOUBLE_EQ(view[2], 0.0);  // qz
  EXPECT_DOUBLE_EQ(view[3], 1.0);  // qw
}

TEST(DirectConversion, ArithmeticContainer) {
  // Test one arithmetic container type (represents all via std::is_arithmetic_v)
  std::vector<double> vec = {1.0, 2.0, 3.0, 4.0, 5.0};
  beluga_estimation::std_future::span<double> view = beluga_estimation::to_view(vec);

  EXPECT_EQ(view.size(), 5);
  EXPECT_DOUBLE_EQ(view[0], 1.0);
  EXPECT_DOUBLE_EQ(view[4], 5.0);

  // Write access
  view[2] = 99.0;
  EXPECT_DOUBLE_EQ(vec[2], 99.0);

  // Const version
  const std::vector<double>& const_vec = vec;
  beluga_estimation::std_future::span<const double> const_view = beluga_estimation::to_view(const_vec);

  EXPECT_EQ(const_view.size(), 5);

  // Fixed-size arithmetic array
  std::array<float, 3> arr = {1.0f, 2.0f, 3.0f};
  beluga_estimation::std_future::span<float, 3> arr_view = beluga_estimation::to_view(arr);

  EXPECT_EQ(arr_view.size(), 3);
  EXPECT_FLOAT_EQ(arr_view[1], 2.0f);

  arr_view[0] = 99.0f;
  EXPECT_FLOAT_EQ(arr[0], 99.0f);
}

// ============================================================================
// 3. VISIBLE RANGES (Overload 2: Recursive Transformation)
// ============================================================================

// Test ranges containing viewable elements (lazy element-by-element conversion)

TEST(VisibleRange, VectorOfVectors) {
  std::vector<beluga_estimation::Vector3d> vecs;
  vecs.push_back(beluga_estimation::Vector3d(1.0, 2.0, 3.0));
  vecs.push_back(beluga_estimation::Vector3d(4.0, 5.0, 6.0));
  vecs.push_back(beluga_estimation::Vector3d(7.0, 8.0, 9.0));

  beluga_estimation::view<beluga_estimation::std_future::span<double, 3>> view = beluga_estimation::to_view(vecs);

  // Verify view is random access and sized
  static_assert(ranges::random_access_range<decltype(view)>, "View should be random access");
  static_assert(ranges::sized_range<decltype(view)>, "View should be sized");

  EXPECT_EQ(ranges::size(view), 3);

  // Test random access
  auto span0 = view[0];
  EXPECT_EQ(span0.size(), 3);
  EXPECT_DOUBLE_EQ(span0[0], 1.0);
  EXPECT_DOUBLE_EQ(span0[1], 2.0);
  EXPECT_DOUBLE_EQ(span0[2], 3.0);

  auto span2 = view[2];
  EXPECT_DOUBLE_EQ(span2[0], 7.0);
  EXPECT_DOUBLE_EQ(span2[2], 9.0);

  // Test write access through view
  span0[1] = 100.0;
  EXPECT_DOUBLE_EQ(vecs[0][1], 100.0);

  // Test iteration (lazy evaluation)
  int count = 0;
  for (auto elem_span : view) {
    EXPECT_EQ(elem_span.size(), 3);
    ++count;
  }
  EXPECT_EQ(count, 3);
}

TEST(VisibleRange, VectorOfLieGroups) {
  std::vector<beluga_estimation::SE2d> poses;
  poses.push_back(beluga_estimation::SE2d(beluga_estimation::SO2d(0.1), beluga_estimation::Vector2d(1.0, 2.0)));
  poses.push_back(beluga_estimation::SE2d(beluga_estimation::SO2d(0.2), beluga_estimation::Vector2d(3.0, 4.0)));

  beluga_estimation::view<beluga_estimation::std_future::span<double>> view = beluga_estimation::to_view(poses);

  EXPECT_EQ(ranges::size(view), 2);

  // Verify first pose
  auto span0 = view[0];
  EXPECT_EQ(span0.size(), 4);
  EXPECT_NEAR(span0[0], std::cos(0.1), 1e-10);
  EXPECT_NEAR(span0[1], std::sin(0.1), 1e-10);

  // Verify second pose
  auto span1 = view[1];
  EXPECT_NEAR(span1[0], std::cos(0.2), 1e-10);
  EXPECT_DOUBLE_EQ(span1[2], 3.0);

  // Write access
  span0[3] = 99.0;
  EXPECT_DOUBLE_EQ(poses[0].data()[3], 99.0);
}

TEST(VisibleRange, ArrayOfVectors) {
  std::array<beluga_estimation::Vector2d, 2> vecs;
  vecs[0] = beluga_estimation::Vector2d(1.0, 2.0);
  vecs[1] = beluga_estimation::Vector2d(3.0, 4.0);

  beluga_estimation::view<beluga_estimation::std_future::span<double, 2>> view = beluga_estimation::to_view(vecs);

  EXPECT_EQ(ranges::size(view), 2);

  auto span0 = view[0];
  EXPECT_EQ(span0.size(), 2);
  EXPECT_DOUBLE_EQ(span0[0], 1.0);
  EXPECT_DOUBLE_EQ(span0[1], 2.0);

  // Write access
  span0[1] = 99.0;
  EXPECT_DOUBLE_EQ(vecs[0][1], 99.0);
}

TEST(VisibleRange, NestedVectorOfVectors) {
  // Test recursive visible range: vector<vector<Vector2d>>
  std::vector<std::vector<beluga_estimation::Vector2d>> nested;
  nested.push_back({beluga_estimation::Vector2d(1.0, 2.0), beluga_estimation::Vector2d(3.0, 4.0)});
  nested.push_back({beluga_estimation::Vector2d(5.0, 6.0)});

  auto view = beluga_estimation::to_view(nested);

  // Outer view
  EXPECT_EQ(ranges::size(view), 2);

  // First inner view
  auto inner0 = view[0];
  EXPECT_EQ(ranges::size(inner0), 2);

  auto span00 = inner0[0];
  EXPECT_DOUBLE_EQ(span00[0], 1.0);
  EXPECT_DOUBLE_EQ(span00[1], 2.0);

  auto span01 = inner0[1];
  EXPECT_DOUBLE_EQ(span01[0], 3.0);
  EXPECT_DOUBLE_EQ(span01[1], 4.0);

  // Second inner view
  auto inner1 = view[1];
  EXPECT_EQ(ranges::size(inner1), 1);

  auto span10 = inner1[0];
  EXPECT_DOUBLE_EQ(span10[0], 5.0);
  EXPECT_DOUBLE_EQ(span10[1], 6.0);

  // Test iteration at multiple levels
  int outer_count = 0;
  int total_vectors = 0;
  for (auto inner_view : view) {
    ++outer_count;
    for (auto vec_span : inner_view) {
      EXPECT_EQ(vec_span.size(), 2);
      ++total_vectors;
    }
  }
  EXPECT_EQ(outer_count, 2);
  EXPECT_EQ(total_vectors, 3);
}

TEST(VisibleRange, NestedArithmeticContainers) {
  // Nested arithmetic containers: vector<vector<double>>
  std::vector<std::vector<double>> nested = {{1.0, 2.0, 3.0}, {4.0, 5.0}};

  auto view = beluga_estimation::to_view(nested);

  // Should be view<span<double>> (inner vectors converted to spans)
  EXPECT_EQ(ranges::size(view), 2);

  auto span0 = view[0];
  EXPECT_EQ(span0.size(), 3);
  EXPECT_DOUBLE_EQ(span0[0], 1.0);
  EXPECT_DOUBLE_EQ(span0[2], 3.0);

  auto span1 = view[1];
  EXPECT_EQ(span1.size(), 2);
  EXPECT_DOUBLE_EQ(span1[0], 4.0);
  EXPECT_DOUBLE_EQ(span1[1], 5.0);

  // Write access
  span0[1] = 99.0;
  EXPECT_DOUBLE_EQ(nested[0][1], 99.0);
}

// ============================================================================
// 4. OPAQUE RANGES (Overload 3: Pass-Through Wrapping)
// ============================================================================

// Test ranges containing non-viewable elements (no transformation)

TEST(OpaqueRange, VectorOfCustomStruct) {
  struct CustomData {
    int id;
    double value;
    std::string name;
  };

  std::vector<CustomData> vec;
  vec.push_back({1, 1.5, "first"});
  vec.push_back({2, 2.5, "second"});
  vec.push_back({3, 3.5, "third"});

  beluga_estimation::view<CustomData> view = beluga_estimation::to_view(vec);

  // Verify has_view_type_v returns false for CustomData
  static_assert(!beluga_estimation::has_view_type_v<CustomData>, "CustomData should not have view_traits");

  EXPECT_EQ(ranges::size(view), 3);

  // Elements are original type
  auto elem0 = view[0];
  EXPECT_EQ(elem0.id, 1);
  EXPECT_DOUBLE_EQ(elem0.value, 1.5);
  EXPECT_EQ(elem0.name, "first");

  auto elem2 = view[2];
  EXPECT_EQ(elem2.id, 3);
  EXPECT_DOUBLE_EQ(elem2.value, 3.5);
  EXPECT_EQ(elem2.name, "third");

  // Test iteration
  int count = 0;
  for (const auto& elem : view) {
    EXPECT_GT(elem.id, 0);
    ++count;
  }
  EXPECT_EQ(count, 3);
}

TEST(OpaqueRange, VectorOfStrings) {
  std::vector<std::string> vec = {"foo", "bar", "baz"};

  beluga_estimation::view<std::string> view = beluga_estimation::to_view(vec);

  EXPECT_EQ(ranges::size(view), 3);
  EXPECT_EQ(view[0], "foo");
  EXPECT_EQ(view[2], "baz");

  // Iteration
  std::vector<std::string> collected;
  for (const auto& str : view) {
    collected.push_back(str);
  }
  EXPECT_EQ(collected.size(), 3);
  EXPECT_EQ(collected[1], "bar");
}

// ============================================================================
// 5. EDGE CASES & FAILURE MODES
// ============================================================================

TEST(EdgeCases, EmptyContainers) {
  // Empty arithmetic vector
  std::vector<double> empty_vec;
  auto view1 = beluga_estimation::to_view(empty_vec);
  EXPECT_EQ(view1.size(), 0);

  // Empty visible range
  std::vector<beluga_estimation::Vector3d> empty_vecs;
  auto view2 = beluga_estimation::to_view(empty_vecs);
  EXPECT_EQ(ranges::size(view2), 0);

  // Empty opaque range
  std::vector<std::string> empty_strings;
  auto view3 = beluga_estimation::to_view(empty_strings);
  EXPECT_EQ(ranges::size(view3), 0);

  // Zero-sized array
  std::array<float, 0> zero_arr;
  beluga_estimation::std_future::span<float, 0> view4 = beluga_estimation::to_view(zero_arr);
  EXPECT_EQ(view4.size(), 0);
}

TEST(EdgeCases, SingleElement) {
  // Single arithmetic element
  std::vector<int> single = {42};
  auto view1 = beluga_estimation::to_view(single);
  EXPECT_EQ(view1.size(), 1);
  EXPECT_EQ(view1[0], 42);

  // Single visible element
  std::vector<beluga_estimation::Vector2d> single_vec = {beluga_estimation::Vector2d(1.0, 2.0)};
  auto view2 = beluga_estimation::to_view(single_vec);
  EXPECT_EQ(ranges::size(view2), 1);
  auto span = view2[0];
  EXPECT_EQ(span.size(), 2);
  EXPECT_DOUBLE_EQ(span[0], 1.0);

  // Single opaque element
  std::vector<std::string> single_str = {"only"};
  auto view3 = beluga_estimation::to_view(single_str);
  EXPECT_EQ(ranges::size(view3), 1);
  EXPECT_EQ(view3[0], "only");
}

TEST(EdgeCases, ConstCorrectness) {
  // Mutable container
  std::vector<beluga_estimation::Vector3d> mutable_vecs;
  mutable_vecs.push_back(beluga_estimation::Vector3d(1.0, 2.0, 3.0));

  auto mutable_view = beluga_estimation::to_view(mutable_vecs);
  auto mutable_span = mutable_view[0];

  // Should be able to write
  mutable_span[0] = 99.0;
  EXPECT_DOUBLE_EQ(mutable_vecs[0][0], 99.0);

  // Const reference to same container
  const auto& const_vecs = mutable_vecs;
  auto const_view = beluga_estimation::to_view(const_vecs);
  auto const_span = const_view[0];

  // Should be able to read
  EXPECT_DOUBLE_EQ(const_span[0], 99.0);
  EXPECT_DOUBLE_EQ(const_span[1], 2.0);
}

TEST(EdgeCases, LargeNestedStructure) {
  // Stress test: large nested visible range
  std::vector<std::vector<beluga_estimation::Vector2d>> large_nested;
  for (int i = 0; i < 100; ++i) {
    std::vector<beluga_estimation::Vector2d> inner;
    for (int j = 0; j < 10; ++j) {
      inner.push_back(beluga_estimation::Vector2d(static_cast<double>(i), static_cast<double>(j)));
    }
    large_nested.push_back(inner);
  }

  auto view = beluga_estimation::to_view(large_nested);

  EXPECT_EQ(ranges::size(view), 100);

  // Spot check
  auto inner50 = view[50];
  EXPECT_EQ(ranges::size(inner50), 10);

  auto vec_span = inner50[5];
  EXPECT_DOUBLE_EQ(vec_span[0], 50.0);
  EXPECT_DOUBLE_EQ(vec_span[1], 5.0);
}

TEST(EdgeCases, MixedConstAndMutable) {
  // Test mixed const/mutable in nested structures
  std::vector<beluga_estimation::Vector2d> mutable_vecs = {beluga_estimation::Vector2d(1.0, 2.0)};
  const std::vector<beluga_estimation::Vector2d> const_vecs = {beluga_estimation::Vector2d(3.0, 4.0)};

  std::vector<decltype(beluga_estimation::to_view(mutable_vecs))> views;
  views.push_back(beluga_estimation::to_view(mutable_vecs));

  auto mutable_outer_view = views[0];
  auto span = mutable_outer_view[0];

  // Should be able to write through mutable path
  span[0] = 99.0;
  EXPECT_DOUBLE_EQ(mutable_vecs[0][0], 99.0);

  // Const path should be read-only
  auto const_view = beluga_estimation::to_view(const_vecs);
  auto const_span = const_view[0];
  EXPECT_DOUBLE_EQ(const_span[0], 3.0);
}

}  // namespace
