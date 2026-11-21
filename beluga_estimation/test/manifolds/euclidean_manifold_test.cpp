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

#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

#include <gmock/gmock.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>

namespace {

using ::testing::DoubleNear;
using ::testing::Pointwise;

using beluga_estimation::contiguous_slice;

constexpr double kTolerance = 1e-6;

MATCHER(IsApproxEqual, "") {
  return std::abs(std::get<0>(arg) - std::get<1>(arg)) < kTolerance;
}

class EuclideanManifoldTest : public ::testing::Test {
 protected:
  const beluga_estimation::Euclidean3Manifold& manifold = *beluga_estimation::Euclidean3Manifold::Instance();
};

TEST_F(EuclideanManifoldTest, Dimensions) {
  EXPECT_EQ(manifold.AmbientSize(), 3);
  EXPECT_EQ(manifold.TangentSize(), 3);
}

TEST_F(EuclideanManifoldTest, PlusOperation) {
  beluga_estimation::Vector3d lhs(1.0, 2.0, 3.0);
  beluga_estimation::Vector3d rhs(0.5, -1.0, 2.5);
  beluga_estimation::Vector3d result;

  manifold.Plus(
      beluga_estimation::to_view(lhs), beluga_estimation::to_view(rhs), beluga_estimation::to_view(result),
      std::nullopt, std::nullopt);

  beluga_estimation::Vector3d expected(1.5, 1.0, 5.5);
  EXPECT_THAT(result, Pointwise(IsApproxEqual(), expected));
}

TEST_F(EuclideanManifoldTest, MinusOperation) {
  beluga_estimation::Vector3d lhs(1.5, 1.0, 5.5);
  beluga_estimation::Vector3d rhs(1.0, 2.0, 3.0);
  beluga_estimation::Vector3d result;

  manifold.Minus(
      beluga_estimation::to_view(lhs), beluga_estimation::to_view(rhs), beluga_estimation::to_view(result),
      std::nullopt, std::nullopt);

  beluga_estimation::Vector3d expected(0.5, -1.0, 2.5);
  EXPECT_THAT(result, Pointwise(IsApproxEqual(), expected));
}

// --- EmbedAmbient tests ---

TEST_F(EuclideanManifoldTest, EmbedAmbientContiguousSlice) {
  beluga_estimation::Vector3d input(10.0, 20.0, 30.0);
  beluga_estimation::Vector2d output;

  manifold.EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(1, 2), beluga_estimation::to_view(output));

  EXPECT_DOUBLE_EQ(output[0], 20.0);
  EXPECT_DOUBLE_EQ(output[1], 30.0);
}

TEST_F(EuclideanManifoldTest, EmbedAmbientMultipleSlices) {
  beluga_estimation::Vector3d input(10.0, 20.0, 30.0);
  beluga_estimation::Vector2d output;
  contiguous_slice dims[]{contiguous_slice::Span(0, 1), contiguous_slice::Span(2, 1)};  // indices 0 and 2

  manifold.EmbedAmbient(beluga_estimation::to_view(input), dims, beluga_estimation::to_view(output));

  EXPECT_DOUBLE_EQ(output[0], 10.0);
  EXPECT_DOUBLE_EQ(output[1], 30.0);
}

TEST_F(EuclideanManifoldTest, EmbedAmbientFullRange) {
  beluga_estimation::Vector3d input(10.0, 20.0, 30.0);
  beluga_estimation::Vector3d output;

  manifold.EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(0, 3), beluga_estimation::to_view(output));

  EXPECT_DOUBLE_EQ(output[0], 10.0);
  EXPECT_DOUBLE_EQ(output[1], 20.0);
  EXPECT_DOUBLE_EQ(output[2], 30.0);
}

// --- LiftAmbient tests ---

TEST_F(EuclideanManifoldTest, LiftAmbientLeavesNonSelectedUntouched) {
  beluga_estimation::Vector2d sub(10.0, 20.0);
  beluga_estimation::Vector3d full(7.0, 8.0, 9.0);  // sentinel values

  manifold.LiftAmbient(beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], 7.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(full[1], 10.0);
  EXPECT_DOUBLE_EQ(full[2], 20.0);
}

TEST_F(EuclideanManifoldTest, LiftEmbedAmbientRoundTrip) {
  beluga_estimation::Vector3d full(10.0, 20.0, 30.0);
  beluga_estimation::Vector2d sub;

  manifold.EmbedAmbient(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 2), beluga_estimation::to_view(sub));
  beluga_estimation::Vector3d lifted(7.0, 8.0, 9.0);  // sentinel values
  manifold.LiftAmbient(
      beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), beluga_estimation::to_view(lifted));

  EXPECT_DOUBLE_EQ(lifted[0], 7.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(lifted[1], full[1]);
  EXPECT_DOUBLE_EQ(lifted[2], full[2]);
}

// --- LiftTangent tests ---

TEST_F(EuclideanManifoldTest, LiftTangentLeavesNonSelectedUntouched) {
  beluga_estimation::Vector2d sub(10.0, 20.0);
  beluga_estimation::Vector3d full(7.0, 8.0, 9.0);  // sentinel values

  manifold.LiftTangent(beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], 7.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(full[1], 10.0);
  EXPECT_DOUBLE_EQ(full[2], 20.0);
}

TEST_F(EuclideanManifoldTest, LiftTangentMultipleSlices) {
  beluga_estimation::Vector2d sub(10.0, 30.0);
  beluga_estimation::Vector3d full(7.0, 8.0, 9.0);  // sentinel values
  contiguous_slice dims[]{contiguous_slice::Span(0, 1), contiguous_slice::Span(2, 1)};

  manifold.LiftTangent(beluga_estimation::to_view(sub), dims, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], 10.0);
  EXPECT_DOUBLE_EQ(full[1], 8.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(full[2], 30.0);
}

TEST_F(EuclideanManifoldTest, LiftEmbedTangentRoundTrip) {
  beluga_estimation::Vector3d full(10.0, 20.0, 30.0);
  beluga_estimation::Vector2d sub;

  manifold.EmbedTangent(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 2), beluga_estimation::to_view(sub));
  beluga_estimation::Vector3d lifted(7.0, 8.0, 9.0);  // sentinel values
  manifold.LiftTangent(
      beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), beluga_estimation::to_view(lifted));

  EXPECT_DOUBLE_EQ(lifted[0], 7.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(lifted[1], full[1]);
  EXPECT_DOUBLE_EQ(lifted[2], full[2]);
}

// --- Matrix LiftTangent/EmbedTangent tests ---

TEST_F(EuclideanManifoldTest, LiftTangentMatrixSquareBlock) {
  // The 3-argument form applies the same selection to rows and columns,
  // placing a square sub-block over the tangent space.
  beluga_estimation::MatrixXd sub(2, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0;
  // clang-format on
  beluga_estimation::MatrixXd full = beluga_estimation::MatrixXd::Constant(3, 3, -1.0);  // sentinel values

  manifold.LiftTangent(beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), beluga_estimation::to_view(full));

  // Block at rows/cols [1,3) copied; everything else untouched.
  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(full(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(1, 1), 1.0);
  EXPECT_DOUBLE_EQ(full(1, 2), 2.0);
  EXPECT_DOUBLE_EQ(full(2, 1), 3.0);
  EXPECT_DOUBLE_EQ(full(2, 2), 4.0);
}

TEST_F(EuclideanManifoldTest, EmbedTangentMatrixSquareBlock) {
  beluga_estimation::MatrixXd full(3, 3);
  // clang-format off
  full << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
  // clang-format on
  beluga_estimation::MatrixXd sub = beluga_estimation::MatrixXd::Constant(2, 2, -1.0);

  manifold.EmbedTangent(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 2), beluga_estimation::to_view(sub));

  EXPECT_DOUBLE_EQ(sub(0, 0), 5.0);
  EXPECT_DOUBLE_EQ(sub(0, 1), 6.0);
  EXPECT_DOUBLE_EQ(sub(1, 0), 8.0);
  EXPECT_DOUBLE_EQ(sub(1, 1), 9.0);
}

// --- Matrix Lift/Embed with distinct row and column subspaces ---

TEST_F(EuclideanManifoldTest, LiftTangentMatrixDistinctRowColDims) {
  beluga_estimation::MatrixXd sub(2, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0;
  // clang-format on
  beluga_estimation::MatrixXd full = beluga_estimation::MatrixXd::Constant(3, 3, -1.0);  // sentinel values

  manifold.LiftTangent(
      beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(full));

  // Block at rows [1,3), cols [0,2) copied; everything else untouched.
  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(1, 0), 1.0);
  EXPECT_DOUBLE_EQ(full(1, 1), 2.0);
  EXPECT_DOUBLE_EQ(full(1, 2), -1.0);
  EXPECT_DOUBLE_EQ(full(2, 0), 3.0);
  EXPECT_DOUBLE_EQ(full(2, 1), 4.0);
  EXPECT_DOUBLE_EQ(full(2, 2), -1.0);
}

TEST_F(EuclideanManifoldTest, EmbedTangentMatrixDistinctRowColDims) {
  const beluga_estimation::MatrixXd full =
      (beluga_estimation::MatrixXd(3, 3) << 11.0, 12.0, 13.0, 21.0, 22.0, 23.0, 31.0, 32.0, 33.0).finished();
  beluga_estimation::MatrixXd sub = beluga_estimation::MatrixXd::Constant(2, 2, -1.0);

  manifold.EmbedTangent(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 2), contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(sub));

  EXPECT_DOUBLE_EQ(sub(0, 0), 21.0);
  EXPECT_DOUBLE_EQ(sub(0, 1), 22.0);
  EXPECT_DOUBLE_EQ(sub(1, 0), 31.0);
  EXPECT_DOUBLE_EQ(sub(1, 1), 32.0);
}

TEST_F(EuclideanManifoldTest, LiftEmbedTangentMatrixDistinctDimsRoundTrip) {
  beluga_estimation::MatrixXd sub(2, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0;
  // clang-format on
  beluga_estimation::MatrixXd full = beluga_estimation::MatrixXd::Constant(3, 3, -1.0);

  manifold.LiftTangent(
      beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(full));
  beluga_estimation::MatrixXd recovered = beluga_estimation::MatrixXd::Constant(2, 2, -1.0);
  manifold.EmbedTangent(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 2), contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(recovered));

  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST_F(EuclideanManifoldTest, LiftTangentMatrixInvalidColDimsThrows) {
  const beluga_estimation::MatrixXd sub = beluga_estimation::MatrixXd::Zero(2, 2);
  beluga_estimation::MatrixXd full = beluga_estimation::MatrixXd::Zero(3, 3);

  EXPECT_THROW(
      manifold.LiftTangent(
          beluga_estimation::to_view(sub), contiguous_slice::Span(1, 2), contiguous_slice::Span(3, 1),
          beluga_estimation::to_view(full)),
      std::runtime_error);
}

// --- Invalid dims tests ---

TEST_F(EuclideanManifoldTest, LiftTangentOverlappingDimsThrows) {
  beluga_estimation::Vector3d sub(10.0, 20.0, 30.0);
  beluga_estimation::Vector3d full;
  contiguous_slice dims[]{contiguous_slice::Span(0, 2), contiguous_slice::Span(1, 1)};  // overlap at 1
  EXPECT_THROW(
      manifold.LiftTangent(beluga_estimation::to_view(sub), dims, beluga_estimation::to_view(full)),
      std::runtime_error);
}

TEST_F(EuclideanManifoldTest, LiftTangentOutOfBoundsThrows) {
  beluga_estimation::Vector2d sub(10.0, 20.0);
  beluga_estimation::Vector3d full;
  EXPECT_THROW(
      manifold.LiftTangent(
          beluga_estimation::to_view(sub), contiguous_slice::Span(2, 2), beluga_estimation::to_view(full)),
      std::runtime_error);
}

TEST_F(EuclideanManifoldTest, LiftTangentEmptyDimsThrows) {
  beluga_estimation::Vector3d full;
  std::vector<contiguous_slice> dims = {};
  EXPECT_THROW(
      manifold.LiftTangent(beluga_estimation::to_view(full), dims, beluga_estimation::to_view(full)),
      std::runtime_error);
}

TEST_F(EuclideanManifoldTest, AmbientSizeOfSubManifold) {
  // The sub-manifold ambient size equals the selected tangent dimensions.
  EXPECT_EQ(manifold.AmbientSize(contiguous_slice::Span(1, 2)), 2u);
  EXPECT_EQ(manifold.AmbientSize(contiguous_slice::Span(0, 3)), 3u);
  contiguous_slice dims[]{contiguous_slice::Span(0, 1), contiguous_slice::Span(2, 1)};
  EXPECT_EQ(manifold.AmbientSize(dims), 2u);
}

TEST_F(EuclideanManifoldTest, AmbientSizeOfSubManifoldInvalidThrows) {
  EXPECT_THROW(manifold.AmbientSize(contiguous_slice::Span(2, 2)), std::runtime_error);
  std::vector<contiguous_slice> dims = {};
  EXPECT_THROW(manifold.AmbientSize(dims), std::runtime_error);
}

// --- GetSubManifold tests ---

TEST_F(EuclideanManifoldTest, GetSubManifoldFullSet) {
  auto sub = manifold.GetSubManifold(contiguous_slice::Span(0, 3));
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 3);
  EXPECT_EQ(sub->TangentSize(), 3);
}

TEST_F(EuclideanManifoldTest, GetSubManifoldProperSubset) {
  auto sub = manifold.GetSubManifold(contiguous_slice::Span(1, 2));
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 2);
  EXPECT_EQ(sub->TangentSize(), 2);
}

TEST_F(EuclideanManifoldTest, GetSubManifoldInvalidOutOfBounds) {
  auto sub = manifold.GetSubManifold(contiguous_slice::Span(2, 2));  // [2, 4) exceeds N=3
  EXPECT_EQ(sub, nullptr);
}

TEST_F(EuclideanManifoldTest, GetSubManifoldZeroSize) {
  // A zero-sized slice has start == end, representing an empty range.
  auto sub = manifold.GetSubManifold(contiguous_slice{2, 2});
  EXPECT_EQ(sub, nullptr);
}

TEST_F(EuclideanManifoldTest, GetSubManifoldDynamicSize) {
  // A proper subset returns a dynamically-sized manifold.
  auto sub = manifold.GetSubManifold(contiguous_slice::Span(0, 2));
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 2);

  // Verify EmbedAmbient works on the dynamic sub-manifold.
  beluga_estimation::Vector2d input(1.0, 2.0);
  beluga_estimation::Vector2d projected;
  sub->EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(0, 2), beluga_estimation::to_view(projected));
  EXPECT_DOUBLE_EQ(projected[0], 1.0);
  EXPECT_DOUBLE_EQ(projected[1], 2.0);
}

TEST_F(EuclideanManifoldTest, GetSubManifoldAllDimensionsViaMultipleSlices) {
  // Two non-overlapping slices that together cover all 3 dimensions.
  contiguous_slice dims[]{contiguous_slice::Span(0, 1), contiguous_slice::Span(1, 2)};
  auto sub = manifold.GetSubManifold(dims);
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 3);
}

// --- EmbedAmbient on dynamic manifold ---

TEST_F(EuclideanManifoldTest, EmbedAmbientOnDynamicManifold) {
  auto manifold2 = beluga_estimation::Euclidean2Manifold::Instance();
  beluga_estimation::Vector2d input(10.0, 20.0);
  beluga_estimation::Vector<double, 1> output;
  manifold2->EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(1, 1), beluga_estimation::to_view(output));
  EXPECT_DOUBLE_EQ(output[0], 20.0);
}

}  // namespace
