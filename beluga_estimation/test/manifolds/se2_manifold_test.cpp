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
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>

using beluga_estimation::SE2Manifold;

class SE2ManifoldTest : public ::testing::Test {
 protected:
  std::shared_ptr<const SE2Manifold> manifold_ = SE2Manifold::Instance();
};

TEST_F(SE2ManifoldTest, Construction) {
  ASSERT_EQ(manifold_->AmbientSize(), 4);
  ASSERT_EQ(manifold_->TangentSize(), 3);
}

TEST_F(SE2ManifoldTest, PlusOperation) {
  beluga_estimation::SE2d initial_state(beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::Vector3d tangent_vec(1.0, 0.0, 0.0);
  beluga_estimation::SE2d result_state;

  manifold_->Plus(
      beluga_estimation::to_view(initial_state), beluga_estimation::to_view(tangent_vec),
      beluga_estimation::to_view(result_state), std::nullopt, std::nullopt);

  EXPECT_NEAR(result_state.translation().x(), 1.0, 1e-6);
  EXPECT_NEAR(result_state.translation().y(), 3.0, 1e-6);
  EXPECT_NEAR(result_state.so2().log(), M_PI / 2.0, 1e-6);
}

TEST_F(SE2ManifoldTest, MinusOperation) {
  beluga_estimation::SE2d initial_state(beluga_estimation::SO2d(M_PI / 2.0), beluga_estimation::Vector2d(1.0, 2.0));
  beluga_estimation::SE2d target_state(beluga_estimation::SO2d(M_PI), beluga_estimation::Vector2d(2.0, 3.0));
  beluga_estimation::Vector3d result_tangent_vec;

  manifold_->Minus(
      beluga_estimation::to_view(target_state), beluga_estimation::to_view(initial_state),
      beluga_estimation::to_view(result_tangent_vec), std::nullopt, std::nullopt);

  EXPECT_NEAR(result_tangent_vec(2), M_PI / 2.0, 1e-6);
}

TEST_F(SE2ManifoldTest, SizeValidation) {
  std::vector<double> valid_ambient(4, 0.0);
  std::vector<double> valid_tangent(3, 0.0);
  std::vector<double> invalid(2, 0.0);

  auto a = beluga_estimation::to_view(valid_ambient);
  auto t = beluga_estimation::to_view(valid_tangent);
  auto inv = beluga_estimation::to_view(invalid);

#ifndef NDEBUG
  EXPECT_DEATH(manifold_->Plus(inv, t, a, std::nullopt, std::nullopt), "lhs.size\\(\\) == AmbientSize\\(\\)");
  EXPECT_DEATH(manifold_->Plus(a, inv, a, std::nullopt, std::nullopt), "rhs.size\\(\\) == TangentSize\\(\\)");
  EXPECT_DEATH(manifold_->Plus(a, t, inv, std::nullopt, std::nullopt), "result.size\\(\\) == AmbientSize\\(\\)");

  EXPECT_DEATH(manifold_->Minus(inv, a, t, std::nullopt, std::nullopt), "lhs.size\\(\\) == AmbientSize\\(\\)");
  EXPECT_DEATH(manifold_->Minus(a, inv, t, std::nullopt, std::nullopt), "rhs.size\\(\\) == AmbientSize\\(\\)");
  EXPECT_DEATH(manifold_->Minus(a, a, inv, std::nullopt, std::nullopt), "result.size\\(\\) == TangentSize\\(\\)");
#endif
}

TEST_F(SE2ManifoldTest, LiftTangentFullSelection) {
  beluga_estimation::Vector3d sub_tangent(1.0, 2.0, 3.0);
  beluga_estimation::Vector3d full_tangent;

  manifold_->LiftTangent(
      beluga_estimation::to_view(sub_tangent), beluga_estimation::contiguous_slice::Span(0, 3),
      beluga_estimation::to_view(full_tangent));

  EXPECT_DOUBLE_EQ(full_tangent(0), 1.0);
  EXPECT_DOUBLE_EQ(full_tangent(1), 2.0);
  EXPECT_DOUBLE_EQ(full_tangent(2), 3.0);
}

TEST_F(SE2ManifoldTest, EmbedTangentFullSelection) {
  beluga_estimation::Vector3d full_tangent(1.0, 2.0, 3.0);
  beluga_estimation::Vector3d sub_tangent;

  manifold_->EmbedTangent(
      beluga_estimation::to_view(full_tangent), beluga_estimation::contiguous_slice::Span(0, 3),
      beluga_estimation::to_view(sub_tangent));

  EXPECT_DOUBLE_EQ(sub_tangent(0), 1.0);
  EXPECT_DOUBLE_EQ(sub_tangent(1), 2.0);
  EXPECT_DOUBLE_EQ(sub_tangent(2), 3.0);
}

TEST_F(SE2ManifoldTest, LiftTangentPartialSelectionLeavesRestUntouched) {
  // Tangent spaces are vector spaces, so partial selections are valid; the
  // non-selected dimensions are left untouched, not zero-padded.
  beluga_estimation::Vector2d sub_tangent(1.0, 2.0);
  beluga_estimation::Vector3d full_tangent = beluga_estimation::Vector3d::Constant(-1.0);  // sentinel values

  manifold_->LiftTangent(
      beluga_estimation::to_view(sub_tangent), beluga_estimation::contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(full_tangent));

  EXPECT_DOUBLE_EQ(full_tangent(0), 1.0);
  EXPECT_DOUBLE_EQ(full_tangent(1), 2.0);
  EXPECT_DOUBLE_EQ(full_tangent(2), -1.0);  // non-selected dimension untouched
}

TEST_F(SE2ManifoldTest, LiftTangentMatrixFullSelection) {
  // The 3-argument form applies the same selection to rows and columns.
  beluga_estimation::MatrixXd sub(3, 3);
  // clang-format off
  sub << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
  // clang-format on
  beluga_estimation::MatrixXd full = beluga_estimation::MatrixXd::Constant(3, 3, -1.0);

  manifold_->LiftTangent(
      beluga_estimation::to_view(sub), beluga_estimation::contiguous_slice::Span(0, 3),
      beluga_estimation::to_view(full));

  EXPECT_TRUE(full.isApprox(sub));
}

TEST_F(SE2ManifoldTest, EmbedTangentMatrixPartialSelection) {
  beluga_estimation::MatrixXd full(3, 3);
  // clang-format off
  full << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
  // clang-format on
  beluga_estimation::MatrixXd sub = beluga_estimation::MatrixXd::Constant(2, 2, -1.0);

  manifold_->EmbedTangent(
      beluga_estimation::to_view(full), beluga_estimation::contiguous_slice::Span(0, 2),
      beluga_estimation::to_view(sub));

  // Top-left 2x2 block (rows and cols 0,1) extracted.
  EXPECT_DOUBLE_EQ(sub(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(sub(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(sub(1, 0), 4.0);
  EXPECT_DOUBLE_EQ(sub(1, 1), 5.0);
}

TEST_F(SE2ManifoldTest, LiftAmbientFullSelection) {
  beluga_estimation::SE2d sub_pose(beluga_estimation::SO2d(0.5), beluga_estimation::Vector2d(10.0, 20.0));
  beluga_estimation::SE2d full_pose;

  manifold_->LiftAmbient(
      beluga_estimation::to_view(sub_pose), beluga_estimation::contiguous_slice::Span(0, 3),
      beluga_estimation::to_view(full_pose));

  EXPECT_DOUBLE_EQ(full_pose.so2().log(), 0.5);
  EXPECT_DOUBLE_EQ(full_pose.translation().x(), 10.0);
  EXPECT_DOUBLE_EQ(full_pose.translation().y(), 20.0);
}

TEST_F(SE2ManifoldTest, LiftAmbientFullSelectionViaView) {
  // The view-based overload accepts a container of slices; a single slice
  // covering the full tangent space is a valid selection.
  beluga_estimation::SE2d sub_pose(beluga_estimation::SO2d(0.5), beluga_estimation::Vector2d(10.0, 20.0));
  beluga_estimation::SE2d full_pose;
  std::vector<beluga_estimation::contiguous_slice> dims = {beluga_estimation::contiguous_slice::Span(0, 3)};

  manifold_->LiftAmbient(beluga_estimation::to_view(sub_pose), dims, beluga_estimation::to_view(full_pose));

  EXPECT_DOUBLE_EQ(full_pose.so2().log(), 0.5);
  EXPECT_DOUBLE_EQ(full_pose.translation().x(), 10.0);
  EXPECT_DOUBLE_EQ(full_pose.translation().y(), 20.0);
}

TEST_F(SE2ManifoldTest, LiftAmbientPartialSelectionThrows) {
  beluga_estimation::SE2d sub_pose;
  beluga_estimation::SE2d full_pose;

  EXPECT_THROW(
      manifold_->LiftAmbient(
          beluga_estimation::to_view(sub_pose), beluga_estimation::contiguous_slice::Span(0, 2),
          beluga_estimation::to_view(full_pose)),
      std::runtime_error);
}

TEST_F(SE2ManifoldTest, LiftAmbientFullSelectionOpenEnded) {
  // An open-ended interval with no start defaults to 0, still a full selection.
  beluga_estimation::SE2d sub_pose(beluga_estimation::SO2d(0.5), beluga_estimation::Vector2d(10.0, 20.0));
  beluga_estimation::SE2d full_pose;

  manifold_->LiftAmbient(
      beluga_estimation::to_view(sub_pose), beluga_estimation::contiguous_slice::Until(3),
      beluga_estimation::to_view(full_pose));

  EXPECT_DOUBLE_EQ(full_pose.so2().log(), 0.5);
  EXPECT_DOUBLE_EQ(full_pose.translation().x(), 10.0);
  EXPECT_DOUBLE_EQ(full_pose.translation().y(), 20.0);
}

TEST_F(SE2ManifoldTest, EmbedAmbientFullSelection) {
  beluga_estimation::SE2d full_pose(beluga_estimation::SO2d(0.5), beluga_estimation::Vector2d(10.0, 20.0));
  beluga_estimation::SE2d sub_pose;

  manifold_->EmbedAmbient(
      beluga_estimation::to_view(full_pose), beluga_estimation::contiguous_slice::Span(0, 3),
      beluga_estimation::to_view(sub_pose));

  EXPECT_DOUBLE_EQ(sub_pose.so2().log(), 0.5);
  EXPECT_DOUBLE_EQ(sub_pose.translation().x(), 10.0);
  EXPECT_DOUBLE_EQ(sub_pose.translation().y(), 20.0);
}

TEST_F(SE2ManifoldTest, EmbedAmbientPartialSelectionThrows) {
  beluga_estimation::SE2d full_pose;
  std::vector<double> sub_ambient(2, 0.0);

  EXPECT_THROW(
      manifold_->EmbedAmbient(
          beluga_estimation::to_view(full_pose), beluga_estimation::contiguous_slice::Span(0, 2),
          beluga_estimation::to_view(sub_ambient)),
      std::runtime_error);
}

TEST_F(SE2ManifoldTest, AmbientSizeOfSubManifold) {
  // Only the full selection is supported; its ambient size is 4.
  EXPECT_EQ(manifold_->AmbientSize(beluga_estimation::contiguous_slice::Span(0, 3)), 4u);
  EXPECT_THROW(manifold_->AmbientSize(beluga_estimation::contiguous_slice::Span(0, 2)), std::runtime_error);
}

// --- GetSubManifold tests ---

TEST_F(SE2ManifoldTest, GetSubManifoldFullSelection) {
  // The full selection returns the manifold itself.
  auto sub = manifold_->GetSubManifold(beluga_estimation::contiguous_slice::Span(0, 3));
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub.get(), manifold_.get());
  EXPECT_EQ(sub->AmbientSize(), 4u);
  EXPECT_EQ(sub->TangentSize(), 3u);
}

TEST_F(SE2ManifoldTest, GetSubManifoldFullSelectionOpenEnded) {
  // An open-ended interval with no end defaults to the tangent size, still a full selection.
  auto sub = manifold_->GetSubManifold(beluga_estimation::contiguous_slice::From(0));
  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub.get(), manifold_.get());
}

TEST_F(SE2ManifoldTest, GetSubManifoldPartialSelectionReturnsNull) {
  auto sub = manifold_->GetSubManifold(beluga_estimation::contiguous_slice::Span(0, 2));
  EXPECT_EQ(sub, nullptr);
}

TEST_F(SE2ManifoldTest, GetSubManifoldOutOfBoundsReturnsNull) {
  auto sub =
      manifold_->GetSubManifold(beluga_estimation::contiguous_slice::Span(0, 4));  // [0, 4) exceeds the tangent size
  EXPECT_EQ(sub, nullptr);
}

TEST_F(SE2ManifoldTest, GetSubManifoldZeroSizeReturnsNull) {
  // A zero-sized interval has start == end, representing an empty range.
  auto sub = manifold_->GetSubManifold(beluga_estimation::contiguous_slice{std::size_t{2}, std::size_t{2}});
  EXPECT_EQ(sub, nullptr);
}

TEST_F(SE2ManifoldTest, GetSubManifoldEmptySelectionReturnsNull) {
  std::vector<beluga_estimation::contiguous_slice> dims;
  auto sub = manifold_->GetSubManifold(dims);
  EXPECT_EQ(sub, nullptr);
}

TEST_F(SE2ManifoldTest, GetSubManifoldMultipleSlicesReturnsNull) {
  // SE(2) only supports a single slice; even two slices covering the full
  // tangent space are rejected.
  std::vector<beluga_estimation::contiguous_slice> dims = {
      beluga_estimation::contiguous_slice::Span(0, 2), beluga_estimation::contiguous_slice::Span(2, 1)};
  auto sub = manifold_->GetSubManifold(dims);
  EXPECT_EQ(sub, nullptr);
}
