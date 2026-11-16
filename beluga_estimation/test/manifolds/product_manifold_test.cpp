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

#include <cstddef>
#include <initializer_list>
#include <utility>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/slice.hpp>
#include <beluga_estimation/common/views.hpp>
#include <beluga_estimation/manifolds/euclidean_manifold.hpp>
#include <beluga_estimation/manifolds/product_manifold.hpp>
#include <beluga_estimation/manifolds/se2_manifold.hpp>

namespace beluga_estimation {
namespace {

// Common sub-manifold instances used across tests.
const auto kE2 = Euclidean2Manifold::Instance();
const auto kE3 = Euclidean3Manifold::Instance();
const auto kSE2 = SE2Manifold::Instance();

// Builds a product manifold from the given (sub-manifold, count) blocks.
std::shared_ptr<ProductManifold_<double>> MakeProduct(
    std::initializer_list<std::pair<std::shared_ptr<const Manifold_<double>>, std::size_t>> blocks) {
  ProductManifoldBuilder_<double> builder;
  for (const auto& [manifold, count] : blocks) {
    builder.Add(manifold, count);
  }
  return builder.Build();
}

// --- Construction tests ---

TEST(ProductManifold, SingleEuclidean) {
  auto manifold = MakeProduct({{kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 1u);
  EXPECT_EQ(manifold->AmbientSize(), 2u);
  EXPECT_EQ(manifold->TangentSize(), 2u);
  EXPECT_EQ(manifold->GetSubManifold(0)->AmbientSize(), 2u);
  EXPECT_EQ(manifold->GetSubManifold(0)->TangentSize(), 2u);
}

TEST(ProductManifold, MultipleEuclidean) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 2u);
  EXPECT_EQ(manifold->AmbientSize(), 5u);
  EXPECT_EQ(manifold->TangentSize(), 5u);
  EXPECT_EQ(manifold->GetSubManifold(0)->AmbientSize(), 2u);
  EXPECT_EQ(manifold->GetSubManifold(0)->TangentSize(), 2u);
  EXPECT_EQ(manifold->GetSubManifold(1)->AmbientSize(), 3u);
  EXPECT_EQ(manifold->GetSubManifold(1)->TangentSize(), 3u);
}

TEST(ProductManifold, MixedManifoldTypes) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 2u);
  EXPECT_EQ(manifold->AmbientSize(), 6u);  // SE2=4 + E2=2
  EXPECT_EQ(manifold->TangentSize(), 5u);  // SE2=3 + E2=2

  EXPECT_EQ(manifold->GetSubManifold(0)->AmbientSize(), 4u);
  EXPECT_EQ(manifold->GetSubManifold(0)->TangentSize(), 3u);
  EXPECT_EQ(manifold->GetSubManifold(1)->AmbientSize(), 2u);
  EXPECT_EQ(manifold->GetSubManifold(1)->TangentSize(), 2u);
}

// --- Builder tests ---

TEST(ProductManifold, BuilderCompaction) {
  auto manifold = MakeProduct({{kE2, 1}, {kE2, 1}, {kE2, 3}});

  EXPECT_EQ(manifold->NumSubManifolds(), 5u);
  EXPECT_EQ(manifold->AmbientSize(), 10u);
  EXPECT_EQ(manifold->TangentSize(), 10u);
  for (std::size_t i = 0; i < 5; ++i) {
    EXPECT_EQ(manifold->GetSubManifold(i).get(), kE2.get());
  }
}

TEST(ProductManifold, BuilderCountsRepeatedAddsOfSameInstance) {
  // Adding the same instance twice still counts both sub-manifolds, even
  // though the builder compacts them into a single block.
  auto manifold = MakeProduct({{kE2, 1}, {kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 2u);
}

TEST(ProductManifold, BuilderNonConsecutiveDuplicates) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}, {kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 3u);
}

TEST(ProductManifold, BuilderEmptyThrows) {
  ProductManifoldBuilder_<double> builder;
  EXPECT_THROW(builder.Build(), std::runtime_error);
}

TEST(ProductManifold, BuildConsumesState) {
  ProductManifoldBuilder_<double> builder;
  builder.Add(kE2);
  auto manifold = builder.Build();
  EXPECT_EQ(manifold->NumSubManifolds(), 1u);

  EXPECT_THROW(builder.Build(), std::runtime_error);
}

TEST(ProductManifold, AddWithCount) {
  auto manifold = MakeProduct({{kE2, 3}});

  EXPECT_EQ(manifold->NumSubManifolds(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(manifold->GetSubManifold(i)->AmbientSize(), 2u);
  }
}

// --- Plus / Minus tests ---

TEST(ProductManifold, PlusWithEuclidean) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  std::vector<double> lhs = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> rhs = {10.0, 20.0, 30.0, 40.0, 50.0};
  std::vector<double> result(5, 0.0);

  manifold->Plus(lhs, rhs, result, std::nullopt, std::nullopt);

  EXPECT_DOUBLE_EQ(result[0], 11.0);
  EXPECT_DOUBLE_EQ(result[1], 22.0);
  EXPECT_DOUBLE_EQ(result[2], 33.0);
  EXPECT_DOUBLE_EQ(result[3], 44.0);
  EXPECT_DOUBLE_EQ(result[4], 55.0);
}

TEST(ProductManifold, MinusWithEuclidean) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  std::vector<double> lhs = {11.0, 22.0, 33.0, 44.0, 55.0};
  std::vector<double> rhs = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> result(5, 0.0);

  manifold->Minus(lhs, rhs, result, std::nullopt, std::nullopt);

  EXPECT_DOUBLE_EQ(result[0], 10.0);
  EXPECT_DOUBLE_EQ(result[1], 20.0);
  EXPECT_DOUBLE_EQ(result[2], 30.0);
  EXPECT_DOUBLE_EQ(result[3], 40.0);
  EXPECT_DOUBLE_EQ(result[4], 50.0);
}

TEST(ProductManifold, PlusMinusRoundTrip) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  std::vector<double> state = {1.0, 0.0, 10.0, 20.0, 100.0, 200.0};
  std::vector<double> tangent(5, 0.0);
  tangent[0] = 0.1;
  tangent[3] = 5.0;
  tangent[4] = -3.0;

  std::vector<double> perturbed(6, 0.0);
  manifold->Plus(state, tangent, perturbed, std::nullopt, std::nullopt);

  std::vector<double> recovered(5, 0.0);
  manifold->Minus(perturbed, state, recovered, std::nullopt, std::nullopt);

  EXPECT_NEAR(recovered[0], 0.1, 1e-10);
  EXPECT_NEAR(recovered[1], 0.0, 1e-10);
  EXPECT_NEAR(recovered[2], 0.0, 1e-10);
  EXPECT_NEAR(recovered[3], 5.0, 1e-10);
  EXPECT_NEAR(recovered[4], -3.0, 1e-10);
}

TEST(ProductManifold, RepeatedBlockPlus) {
  auto manifold = MakeProduct({{kE2, 3}});

  std::vector<double> lhs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> rhs = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0};
  std::vector<double> result(6, 0.0);

  manifold->Plus(lhs, rhs, result, std::nullopt, std::nullopt);

  EXPECT_DOUBLE_EQ(result[0], 11.0);
  EXPECT_DOUBLE_EQ(result[1], 22.0);
  EXPECT_DOUBLE_EQ(result[2], 33.0);
  EXPECT_DOUBLE_EQ(result[3], 44.0);
  EXPECT_DOUBLE_EQ(result[4], 55.0);
  EXPECT_DOUBLE_EQ(result[5], 66.0);
}

TEST(ProductManifold, RepeatedBlockMinus) {
  auto manifold = MakeProduct({{kE2, 3}});

  std::vector<double> lhs = {11.0, 22.0, 33.0, 44.0, 55.0, 66.0};
  std::vector<double> rhs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> result(6, 0.0);

  manifold->Minus(lhs, rhs, result, std::nullopt, std::nullopt);

  EXPECT_DOUBLE_EQ(result[0], 10.0);
  EXPECT_DOUBLE_EQ(result[1], 20.0);
  EXPECT_DOUBLE_EQ(result[2], 30.0);
  EXPECT_DOUBLE_EQ(result[3], 40.0);
  EXPECT_DOUBLE_EQ(result[4], 50.0);
  EXPECT_DOUBLE_EQ(result[5], 60.0);
}

TEST(ProductManifold, RepeatedPlusMinusRoundTrip) {
  auto manifold = MakeProduct({{kE2, 3}});

  std::vector<double> state = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<double> tangent = {0.5, 1.5, 2.5, 3.5, 4.5, 5.5};
  std::vector<double> perturbed(6, 0.0);

  manifold->Plus(state, tangent, perturbed, std::nullopt, std::nullopt);

  std::vector<double> recovered(6, 0.0);
  manifold->Minus(perturbed, state, recovered, std::nullopt, std::nullopt);

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(recovered[i], tangent[i], 1e-10);
  }
}

// --- GetSubManifold(span<slice>) tests ---

TEST(ProductManifold, GetSubManifoldFullSelection) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  auto sub = manifold->GetSubManifold(contiguous_slice::Span(0, 5));

  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 5u);
  EXPECT_EQ(sub->TangentSize(), 5u);
}

TEST(ProductManifold, GetSubManifoldPartialSelection) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  auto sub = manifold->GetSubManifold(contiguous_slice::Span(0, 2));

  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->AmbientSize(), 2u);
  EXPECT_EQ(sub->TangentSize(), 2u);
}

TEST(ProductManifold, GetSubManifoldAcrossBlocks) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  std::vector<contiguous_slice> slices = {
      contiguous_slice::Span(0, 1), contiguous_slice::Span(2, 1), contiguous_slice::Span(4, 1)};
  auto sub = manifold->GetSubManifold(slices);

  ASSERT_NE(sub, nullptr);
  EXPECT_EQ(sub->TangentSize(), 3u);
  EXPECT_EQ(sub->AmbientSize(), 3u);
}

TEST(ProductManifold, GetSubManifoldWithRepeatedBlocks) {
  auto manifold = MakeProduct({{kSE2, 2}, {kE2, 3}});

  auto result = manifold->GetSubManifold(contiguous_slice::Span(10, 2));

  ASSERT_NE(result, nullptr);
  auto sub_product = std::dynamic_pointer_cast<const ProductManifold_<double>>(result);
  ASSERT_NE(sub_product, nullptr);
  EXPECT_EQ(sub_product->NumSubManifolds(), 1u);
  EXPECT_EQ(sub_product->GetSubManifold(0)->AmbientSize(), 2u);
}

TEST(ProductManifold, GetSubManifoldInvalidSlice) {
  auto manifold = MakeProduct({{kE2, 1}});

  auto sub = manifold->GetSubManifold(contiguous_slice::Span(10, 1));
  EXPECT_EQ(sub, nullptr);
}

TEST(ProductManifold, GetSubManifoldPartialNonEuclideanReturnsNull) {
  auto manifold = MakeProduct({{kSE2, 1}});

  // A partial selection of a manifold without proper sub-manifolds is invalid.
  auto sub = manifold->GetSubManifold(contiguous_slice::Span(0, 2));
  EXPECT_EQ(sub, nullptr);
}

TEST(ProductManifold, GetSubManifoldEmptySelection) {
  auto manifold = MakeProduct({{kE2, 1}});

  std::vector<contiguous_slice> slices = {};
  auto sub = manifold->GetSubManifold(slices);
  EXPECT_EQ(sub, nullptr);
}

// --- EmbedAmbient tests ---

TEST(ProductManifold, EmbedAmbientSingleBlock) {
  auto manifold = MakeProduct({{kE2, 1}});

  beluga_estimation::Vector2d input(10.0, 20.0);
  beluga_estimation::Vector<double, 1> output;

  manifold->EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(0, 1), beluga_estimation::to_view(output));

  EXPECT_DOUBLE_EQ(output[0], 10.0);
}

TEST(ProductManifold, EmbedAmbientMultipleBlocks) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  beluga_estimation::Vector<double, 5> input(1.0, 2.0, 3.0, 4.0, 5.0);
  beluga_estimation::Vector2d output;
  std::vector<contiguous_slice> slices = {contiguous_slice::Span(1, 1), contiguous_slice::Span(2, 1)};

  manifold->EmbedAmbient(beluga_estimation::to_view(input), slices, beluga_estimation::to_view(output));

  EXPECT_DOUBLE_EQ(output[0], 2.0);
  EXPECT_DOUBLE_EQ(output[1], 3.0);
}

// --- Jacobian forwarding tests ---

TEST(ProductManifold, PlusWithJacobian) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  std::vector<double> lhs = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> rhs = {10.0, 20.0, 30.0, 40.0, 50.0};
  std::vector<double> result(5, 0.0);

  std::vector<double> jac_data(25, 0.0);
  auto jacobian = std_future::mdspan<double, 2>(jac_data.data(), 5, 5);

  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t j = 0; j < 5; ++j) {
      jacobian(i, j) = static_cast<double>(i * 5 + j + 1);
    }
  }

  manifold->Plus(lhs, rhs, result, jacobian, std::nullopt);

  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t j = 0; j < 5; ++j) {
      EXPECT_DOUBLE_EQ(jacobian(i, j), static_cast<double>(i * 5 + j + 1));
    }
  }

  EXPECT_DOUBLE_EQ(result[0], 11.0);
  EXPECT_DOUBLE_EQ(result[1], 22.0);
}

TEST(ProductManifold, MinusWithJacobian) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  std::vector<double> lhs = {11.0, 22.0, 33.0, 44.0, 55.0};
  std::vector<double> rhs = {1.0, 2.0, 3.0, 4.0, 5.0};
  std::vector<double> result(5, 0.0);

  std::vector<double> jac_data(25, 0.0);
  auto jacobian = std_future::mdspan<double, 2>(jac_data.data(), 5, 5);

  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t j = 0; j < 5; ++j) {
      jacobian(i, j) = static_cast<double>(i * 5 + j + 1);
    }
  }

  manifold->Minus(lhs, rhs, result, jacobian, std::nullopt);

  for (std::size_t i = 0; i < 5; ++i) {
    for (std::size_t j = 0; j < 5; ++j) {
      EXPECT_DOUBLE_EQ(jacobian(i, j), static_cast<double>(i * 5 + j + 1));
    }
  }

  EXPECT_DOUBLE_EQ(result[0], 10.0);
  EXPECT_DOUBLE_EQ(result[1], 20.0);
}

// --- Compaction tests ---

TEST(ProductManifold, BuilderCompactionWithCount) {
  // Add(e2, 2) then Add(e2) should merge into a single compacted block of 3.
  auto manifold = MakeProduct({{kE2, 2}, {kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 3u);
  EXPECT_EQ(manifold->AmbientSize(), 6u);
  EXPECT_EQ(manifold->TangentSize(), 6u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(manifold->GetSubManifold(i).get(), kE2.get());
  }
}

TEST(ProductManifold, BuilderCompactionAfterNonConsecutive) {
  // Add(e2), Add(e3), Add(e2) — only consecutive identical pointers merge.
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}, {kE2, 1}});

  EXPECT_EQ(manifold->NumSubManifolds(), 3u);
  EXPECT_EQ(manifold->AmbientSize(), 7u);  // 2 + 3 + 2
  EXPECT_EQ(manifold->TangentSize(), 7u);
}

// --- Repeated block slice tests ---

TEST(ProductManifold, RepeatedBlockSliceAcrossRepetitions) {
  // Product = E2 x E2 x E2 (3 copies, compacted into one block of size=3)
  // tangent layout: rep0=[0,2), rep1=[2,4), rep2=[4,6)
  // ambient layout: rep0=[0,2), rep1=[2,4), rep2=[4,6)
  auto manifold = MakeProduct({{kE2, 3}});

  // Slice [1, 5) spans rep0(1-2), rep1(2-4), rep2(4-5)
  auto sub = manifold->GetSubManifold(contiguous_slice::Span(1, 4));

  ASSERT_NE(sub, nullptr) << "Should produce a valid sub-manifold";
  // Three sub-blocks: e2 with tangent dims [1,2)=1, [0,2)=2, [0,1)=1 → total = 4
  EXPECT_EQ(sub->TangentSize(), 4u);
  EXPECT_EQ(sub->AmbientSize(), 4u);

  // Verify embed-ambient works correctly across repetition boundaries.
  // The sub-product's ambient layout is:
  //   block0 (1D) at ambient_offset=0 → input[0..1)
  //   block1 (2D) at ambient_offset=1 → input[1..3)
  //   block2 (1D) at ambient_offset=3 → input[3..4)
  beluga_estimation::Vector<double, 4> input(10.0, 20.0, 30.0, 40.0);
  beluga_estimation::Vector<double, 4> output{};
  sub->EmbedAmbient(
      beluga_estimation::to_view(input), contiguous_slice::Span(0, 4), beluga_estimation::to_view(output));
  // block0(offset=0, 1D) → input[0] = 10
  // block1(offset=1, 2D) → input[1], input[2] = 20, 30
  // block2(offset=3, 1D) → input[3] = 40
  EXPECT_DOUBLE_EQ(output[0], 10.0);
  EXPECT_DOUBLE_EQ(output[1], 20.0);
  EXPECT_DOUBLE_EQ(output[2], 30.0);
  EXPECT_DOUBLE_EQ(output[3], 40.0);
}

// --- SharedSubManifoldOwnership test ---

TEST(ProductManifold, SharedSubManifoldOwnership) {
  auto m1 = MakeProduct({{kE2, 1}});
  auto m2 = MakeProduct({{kE2, 1}});

  EXPECT_EQ(m1->AmbientSize(), m2->AmbientSize());
  EXPECT_EQ(m1->TangentSize(), m2->TangentSize());
}

// --- Sub-manifold size queries ---

TEST(ProductManifold, AmbientSizeOfSubManifoldBySlices) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // SE(2) block: tangent [0,3) -> ambient 4. E2 block: tangent [3,5) -> ambient 2.
  EXPECT_EQ(manifold->AmbientSize(contiguous_slice::Span(0, 3)), 4u);
  EXPECT_EQ(manifold->AmbientSize(contiguous_slice::Span(3, 2)), 2u);
  // Partial SE(2) selection is unsupported.
  EXPECT_THROW(manifold->AmbientSize(contiguous_slice::Span(0, 2)), std::runtime_error);
}

TEST(ProductManifold, SizeOfSubManifoldByIndices) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // SE(2) block: tangent 3, ambient 4. E2 block: tangent 2, ambient 2.
  std::vector<std::size_t> both = {0, 1};
  EXPECT_EQ(manifold->TangentSize(both), 5u);
  EXPECT_EQ(manifold->AmbientSize(both), 6u);
  std::vector<std::size_t> first = {0};
  EXPECT_EQ(manifold->TangentSize(first), 3u);
  EXPECT_EQ(manifold->AmbientSize(first), 4u);
  std::vector<std::size_t> second = {1};
  EXPECT_EQ(manifold->TangentSize(second), 2u);
  EXPECT_EQ(manifold->AmbientSize(second), 2u);
  EXPECT_THROW((void)manifold->TangentSize(std::vector<std::size_t>{}), std::runtime_error);
  EXPECT_THROW((void)manifold->AmbientSize(std::vector<std::size_t>{}), std::runtime_error);
  std::vector<std::size_t> bad = {0, 7};
  EXPECT_THROW((void)manifold->TangentSize(bad), std::runtime_error);
  EXPECT_THROW((void)manifold->AmbientSize(bad), std::runtime_error);
}

// --- Index-based Lift/Embed tests ---

TEST(ProductManifold, LiftEmbedTangentByIndexRoundTrip) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  // Lifting the second sub-manifold block leaves the first untouched...
  beluga_estimation::VectorXd sub(3);
  sub << 30.0, 40.0, 50.0;
  beluga_estimation::VectorXd full = beluga_estimation::VectorXd::Constant(5, -1.0);  // sentinel values
  manifold->LiftTangent(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], -1.0);  // non-selected dims untouched
  EXPECT_DOUBLE_EQ(full[1], -1.0);
  EXPECT_DOUBLE_EQ(full[2], 30.0);
  EXPECT_DOUBLE_EQ(full[3], 40.0);
  EXPECT_DOUBLE_EQ(full[4], 50.0);

  // ...and embedding the selection back recovers the input exactly.
  beluga_estimation::VectorXd recovered = beluga_estimation::VectorXd::Constant(3, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(full), 1, beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));

  // Embedding the first (2D) block.
  beluga_estimation::VectorXd first_full(5);
  first_full << 10.0, 20.0, 30.0, 40.0, 50.0;
  beluga_estimation::VectorXd first_sub = beluga_estimation::VectorXd::Constant(2, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(first_full), 0, beluga_estimation::to_view(first_sub));

  EXPECT_DOUBLE_EQ(first_sub[0], 10.0);
  EXPECT_DOUBLE_EQ(first_sub[1], 20.0);
}

TEST(ProductManifold, LiftEmbedTangentRepeatedBlockMiddleRepetition) {
  auto manifold = MakeProduct({{kE2, 3}});

  // Lifting the middle repetition leaves the other two untouched...
  beluga_estimation::VectorXd sub(2);
  sub << 30.0, 40.0;
  beluga_estimation::VectorXd full = beluga_estimation::VectorXd::Constant(6, -1.0);  // sentinel values
  manifold->LiftTangent(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], -1.0);  // non-selected dims untouched
  EXPECT_DOUBLE_EQ(full[1], -1.0);
  EXPECT_DOUBLE_EQ(full[2], 30.0);
  EXPECT_DOUBLE_EQ(full[3], 40.0);
  EXPECT_DOUBLE_EQ(full[4], -1.0);
  EXPECT_DOUBLE_EQ(full[5], -1.0);

  // ...and embedding the selection back recovers the input exactly.
  beluga_estimation::VectorXd recovered = beluga_estimation::VectorXd::Constant(2, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(full), 1, beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, LiftEmbedAmbientByIndex) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Lifting the second (E2) block leaves the SE(2) block untouched...
  beluga_estimation::VectorXd sub(2);
  sub << 7.0, 8.0;
  beluga_estimation::VectorXd full = beluga_estimation::VectorXd::Constant(6, -1.0);  // sentinel values
  manifold->LiftAmbient(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], -1.0);  // SE(2) block untouched
  EXPECT_DOUBLE_EQ(full[1], -1.0);
  EXPECT_DOUBLE_EQ(full[2], -1.0);
  EXPECT_DOUBLE_EQ(full[3], -1.0);
  EXPECT_DOUBLE_EQ(full[4], 7.0);
  EXPECT_DOUBLE_EQ(full[5], 8.0);

  // ...and embedding the selection back recovers the input exactly.
  beluga_estimation::VectorXd recovered = beluga_estimation::VectorXd::Constant(2, -1.0);
  manifold->EmbedAmbient(beluga_estimation::to_view(full), 1, beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));

  // Embedding the first (SE(2), 4D) block.
  beluga_estimation::VectorXd se2_full(6);
  se2_full << 1.0, 0.0, 10.0, 20.0, 100.0, 200.0;
  beluga_estimation::VectorXd se2_sub = beluga_estimation::VectorXd::Constant(4, -1.0);
  manifold->EmbedAmbient(beluga_estimation::to_view(se2_full), 0, beluga_estimation::to_view(se2_sub));

  EXPECT_DOUBLE_EQ(se2_sub[0], 1.0);
  EXPECT_DOUBLE_EQ(se2_sub[1], 0.0);
  EXPECT_DOUBLE_EQ(se2_sub[2], 10.0);
  EXPECT_DOUBLE_EQ(se2_sub[3], 20.0);
}

TEST(ProductManifold, LiftTangentInvalidIndexThrows) {
  auto manifold = MakeProduct({{kE2, 1}});

  VectorXd sub = VectorXd::Zero(2);
  VectorXd full = VectorXd::Zero(2);
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full)), std::runtime_error);
  EXPECT_THROW(
      manifold->EmbedTangent(beluga_estimation::to_view(full), 1, beluga_estimation::to_view(sub)), std::runtime_error);
  EXPECT_THROW(
      manifold->LiftAmbient(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full)), std::runtime_error);
  EXPECT_THROW(
      manifold->EmbedAmbient(beluga_estimation::to_view(full), 1, beluga_estimation::to_view(sub)), std::runtime_error);
}

TEST(ProductManifold, IndexSelectsSubManifoldNotTangentDimension) {
  // Index-based overloads select sub-manifolds by their position in the
  // product (0 = first sub-manifold), NOT by tangent-space dimension.
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}, {kE3, 1}});

  // Tangent layout: SE2=[0,3), E2=[3,5), E3=[5,8).
  // Index 1 selects the E2 sub-manifold, whose tangent range is [3,5).
  VectorXd sub(2);
  sub << 30.0, 40.0;
  VectorXd full = VectorXd::Constant(8, -1.0);  // sentinel values

  manifold->LiftTangent(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], -1.0);
  EXPECT_DOUBLE_EQ(full[1], -1.0);
  EXPECT_DOUBLE_EQ(full[2], -1.0);
  EXPECT_DOUBLE_EQ(full[3], 30.0);
  EXPECT_DOUBLE_EQ(full[4], 40.0);
  EXPECT_DOUBLE_EQ(full[5], -1.0);
  EXPECT_DOUBLE_EQ(full[6], -1.0);
  EXPECT_DOUBLE_EQ(full[7], -1.0);
}

// --- Multiple sub-manifold index Lift/Embed tests ---

TEST(ProductManifold, LiftEmbedTangentMultipleIndicesRoundTrip) {
  auto manifold = MakeProduct({{kE2, 1}, {kE3, 1}});

  // Lifting and then embedding back the same selection must recover the
  // input exactly.
  VectorXd sub(5);
  sub << 10.0, 20.0, 30.0, 40.0, 50.0;  // e2(2) ++ e3(3)
  VectorXd full = VectorXd::Constant(5, -1.0);
  std::vector<std::size_t> indices = {0, 1};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));
  VectorXd recovered = VectorXd::Constant(5, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(recovered));

  EXPECT_TRUE(full.isApprox(sub));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, LiftEmbedTangentMultipleIndicesWithGap) {
  auto manifold = MakeProduct({{kE2, 3}});

  // Lifting repetitions 0 and 2 leaves repetition 1 untouched...
  VectorXd sub(4);
  sub << 10.0, 20.0, 30.0, 40.0;
  VectorXd full = VectorXd::Constant(6, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {0, 2};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], 10.0);
  EXPECT_DOUBLE_EQ(full[1], 20.0);
  EXPECT_DOUBLE_EQ(full[2], -1.0);  // non-selected repetition untouched
  EXPECT_DOUBLE_EQ(full[3], -1.0);
  EXPECT_DOUBLE_EQ(full[4], 30.0);
  EXPECT_DOUBLE_EQ(full[5], 40.0);

  // ...and embedding the selection back recovers the input exactly.
  VectorXd recovered = VectorXd::Constant(4, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, LiftTangentMultipleIndicesFollowRequestedOrder) {
  // The output is filled in the order the indices appear, not sorted.
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}, {kE3, 1}});

  // sub = E3 tangent (3) ++ SE2 tangent (3), in requested order {2, 0}.
  VectorXd sub(6);
  sub << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0;
  VectorXd full = VectorXd::Constant(8, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {2, 0};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));

  // SE2 (index 0) at [0,3) gets the last 3 elements.
  EXPECT_DOUBLE_EQ(full[0], 40.0);
  EXPECT_DOUBLE_EQ(full[1], 50.0);
  EXPECT_DOUBLE_EQ(full[2], 60.0);
  // E2 (index 1) untouched.
  EXPECT_DOUBLE_EQ(full[3], -1.0);
  EXPECT_DOUBLE_EQ(full[4], -1.0);
  // E3 (index 2) at [5,8) gets the first 3 elements.
  EXPECT_DOUBLE_EQ(full[5], 10.0);
  EXPECT_DOUBLE_EQ(full[6], 20.0);
  EXPECT_DOUBLE_EQ(full[7], 30.0);
}

TEST(ProductManifold, EmbedTangentMultipleIndicesFollowRequestedOrder) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}, {kE3, 1}});

  VectorXd full(8);
  full << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  VectorXd sub = VectorXd::Constant(6, -1.0);
  std::vector<std::size_t> indices = {2, 0};

  manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(sub));

  // E3 (index 2) at [5,8) first, then SE2 (index 0) at [0,3).
  EXPECT_DOUBLE_EQ(sub[0], 6.0);
  EXPECT_DOUBLE_EQ(sub[1], 7.0);
  EXPECT_DOUBLE_EQ(sub[2], 8.0);
  EXPECT_DOUBLE_EQ(sub[3], 1.0);
  EXPECT_DOUBLE_EQ(sub[4], 2.0);
  EXPECT_DOUBLE_EQ(sub[5], 3.0);
}

TEST(ProductManifold, LiftAmbientMultipleIndicesWithGap) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 2}});

  // Lift sub-manifolds 0 (SE2) and 2 (second E2); E2 at index 1 untouched.
  VectorXd sub(6);
  sub << 1.0, 0.0, 10.0, 20.0, 100.0, 200.0;    // se2(4) ++ e2(2)
  VectorXd full = VectorXd::Constant(8, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {0, 2};

  manifold->LiftAmbient(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], 1.0);
  EXPECT_DOUBLE_EQ(full[1], 0.0);
  EXPECT_DOUBLE_EQ(full[2], 10.0);
  EXPECT_DOUBLE_EQ(full[3], 20.0);
  EXPECT_DOUBLE_EQ(full[4], -1.0);  // non-selected block untouched
  EXPECT_DOUBLE_EQ(full[5], -1.0);
  EXPECT_DOUBLE_EQ(full[6], 100.0);
  EXPECT_DOUBLE_EQ(full[7], 200.0);
}

TEST(ProductManifold, LiftEmbedAmbientMultipleIndicesRoundTrip) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Both blocks selected: the ambient representation survives a
  // lift/embed round trip exactly.
  VectorXd sub(6);
  sub << 1.0, 0.0, 10.0, 20.0, 100.0, 200.0;  // se2(4) ++ e2(2)
  VectorXd full = VectorXd::Constant(6, -1.0);
  std::vector<std::size_t> indices = {0, 1};

  manifold->LiftAmbient(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));
  VectorXd recovered = VectorXd::Constant(6, -1.0);
  manifold->EmbedAmbient(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(recovered));

  EXPECT_TRUE(full.isApprox(sub));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, LiftTangentMatrixMultipleIndices) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Tangent is 3 + 2 = 5. The sub-matrix holds the square covariance blocks
  // of the two sub-manifolds concatenated on the diagonal: SE2 (3x3) then E2
  // (2x2). Only the diagonal blocks are copied; off-diagonal stays untouched.
  MatrixXd sub(5, 5);
  // clang-format off
  sub << 1.0, 2.0, 3.0, 0.0, 0.0,
         4.0, 5.0, 6.0, 0.0, 0.0,
         7.0, 8.0, 9.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 10.0, 11.0,
         0.0, 0.0, 0.0, 12.0, 13.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(5, 5, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {0, 1};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full));

  // SE(2) block at rows/cols [0,3).
  EXPECT_DOUBLE_EQ(full(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(full(2, 2), 9.0);
  // E2 block at rows/cols [3,5).
  EXPECT_DOUBLE_EQ(full(3, 3), 10.0);
  EXPECT_DOUBLE_EQ(full(4, 4), 13.0);
  // Off-diagonal blocks untouched.
  EXPECT_DOUBLE_EQ(full(0, 3), -1.0);
  EXPECT_DOUBLE_EQ(full(3, 0), -1.0);
}

TEST(ProductManifold, EmbedTangentMatrixMultipleIndicesWithGap) {
  auto manifold = MakeProduct({{kE2, 3}});

  // Extract the 2x2 covariance blocks of repetitions 0 and 2 from the 6x6
  // full matrix (rows and cols [0,2) and [4,6)).
  MatrixXd full(6, 6);
  // clang-format off
  full << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
         7.0, 8.0, 9.0, 10.0, 11.0, 12.0,
         13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
         19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
         25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
         31.0, 32.0, 33.0, 34.0, 35.0, 36.0;
  // clang-format on
  MatrixXd sub = MatrixXd::Constant(4, 4, -1.0);
  std::vector<std::size_t> indices = {0, 2};

  manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(sub));

  // Block 0 (rows/cols [0,2)) then block 2 (rows/cols [4,6)), concatenated.
  EXPECT_DOUBLE_EQ(sub(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(sub(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(sub(1, 0), 7.0);
  EXPECT_DOUBLE_EQ(sub(1, 1), 8.0);
  EXPECT_DOUBLE_EQ(sub(2, 2), 29.0);
  EXPECT_DOUBLE_EQ(sub(2, 3), 30.0);
  EXPECT_DOUBLE_EQ(sub(3, 2), 35.0);
  EXPECT_DOUBLE_EQ(sub(3, 3), 36.0);
}

TEST(ProductManifold, LiftTangentEmptyIndicesThrows) {
  auto manifold = MakeProduct({{kE2, 1}});

  VectorXd sub = VectorXd::Zero(2);
  VectorXd full = VectorXd::Zero(2);
  std::vector<std::size_t> indices = {};
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(sub)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->LiftAmbient(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->EmbedAmbient(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(sub)),
      std::runtime_error);
}

TEST(ProductManifold, LiftTangentInvalidIndexInSetThrows) {
  auto manifold = MakeProduct({{kE2, 1}});

  VectorXd sub = VectorXd::Zero(2);
  VectorXd full = VectorXd::Zero(2);
  std::vector<std::size_t> indices = {0, 7};
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->EmbedTangent(beluga_estimation::to_view(full), indices, beluga_estimation::to_view(sub)),
      std::runtime_error);
}

// --- Index-based matrix Lift/Embed tests ---

TEST(ProductManifold, LiftEmbedTangentMatrixByIndex) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Lifting the SE(2) 3x3 covariance block fills rows/cols [0,3)...
  MatrixXd sub(3, 3);
  // clang-format off
  sub << 1.0, 2.0, 3.0,
         4.0, 5.0, 6.0,
         7.0, 8.0, 9.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(5, 5, -1.0);  // sentinel values

  manifold->LiftTangent(beluga_estimation::to_view(sub), 0, beluga_estimation::to_view(full));

  // Block at rows/cols [0,3) filled; the rest untouched.
  EXPECT_DOUBLE_EQ(full(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(full(2, 2), 9.0);
  EXPECT_DOUBLE_EQ(full(3, 3), -1.0);
  EXPECT_DOUBLE_EQ(full(3, 4), -1.0);
  EXPECT_DOUBLE_EQ(full(4, 3), -1.0);
  EXPECT_DOUBLE_EQ(full(4, 4), -1.0);

  // ...and embedding it back recovers the input exactly.
  MatrixXd recovered = MatrixXd::Constant(3, 3, -1.0);
  manifold->EmbedTangent(beluga_estimation::to_view(full), 0, beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, LiftTangentMatrixIndexSelectsSubManifoldNotTangentDimension) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}, {kE3, 1}});

  // Tangent layout: SE2=[0,3), E2=[3,5), E3=[5,8).
  // Index 1 selects the E2 sub-manifold: 2x2 block at rows/cols [3,5).
  MatrixXd sub(2, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(8, 8, -1.0);  // sentinel values

  manifold->LiftTangent(beluga_estimation::to_view(sub), 1, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full(3, 3), 1.0);
  EXPECT_DOUBLE_EQ(full(3, 4), 2.0);
  EXPECT_DOUBLE_EQ(full(4, 3), 3.0);
  EXPECT_DOUBLE_EQ(full(4, 4), 4.0);
  // Outside the E2 block untouched.
  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(2, 2), -1.0);
  EXPECT_DOUBLE_EQ(full(5, 5), -1.0);
}

// --- Cross-block slice Lift/Embed tests ---

TEST(ProductManifold, LiftEmbedTangentCrossBlockSlice) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Tangent layout: SE2 = [0, 3), E2 = [3, 5). Slice [1, 5) spans both.
  VectorXd sub(4);
  sub << 20.0, 30.0, 40.0, 50.0;
  VectorXd full = VectorXd::Constant(5, -1.0);  // sentinel values

  manifold->LiftTangent(
      beluga_estimation::to_view(sub), contiguous_slice::Span(1, 4), beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full[0], -1.0);  // non-selected dimension untouched
  EXPECT_DOUBLE_EQ(full[1], 20.0);
  EXPECT_DOUBLE_EQ(full[2], 30.0);
  EXPECT_DOUBLE_EQ(full[3], 40.0);
  EXPECT_DOUBLE_EQ(full[4], 50.0);

  // ...and embedding the selection back recovers the input exactly.
  VectorXd recovered = VectorXd::Constant(4, -1.0);
  manifold->EmbedTangent(
      beluga_estimation::to_view(full), contiguous_slice::Span(1, 4), beluga_estimation::to_view(recovered));
  EXPECT_TRUE(recovered.isApprox(sub));
}

TEST(ProductManifold, EmbedAmbientPartialSE2Throws) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Selecting a partial SE(2) tangent range is invalid for ambient ops.
  VectorXd full = VectorXd::Zero(6);
  VectorXd sub = VectorXd::Zero(2);
  EXPECT_THROW(
      manifold->EmbedAmbient(
          beluga_estimation::to_view(full), contiguous_slice::Span(0, 2), beluga_estimation::to_view(sub)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->LiftAmbient(
          beluga_estimation::to_view(sub), contiguous_slice::Span(0, 2), beluga_estimation::to_view(full)),
      std::runtime_error);
}

TEST(ProductManifold, AmbientOpsInvertedDimsThrow) {
  auto manifold = MakeProduct({{kE2, 1}});

  // Inverted interval {start=3, end=2} must be rejected, not silently ignored.
  VectorXd full = VectorXd::Zero(2);
  VectorXd sub = VectorXd::Zero(2);
  std::vector<contiguous_slice> slices = {contiguous_slice{std::size_t{3}, std::size_t{2}}};
  EXPECT_THROW(
      manifold->EmbedAmbient(beluga_estimation::to_view(full), slices, beluga_estimation::to_view(sub)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->LiftAmbient(beluga_estimation::to_view(sub), slices, beluga_estimation::to_view(full)),
      std::runtime_error);
  EXPECT_EQ(manifold->GetSubManifold(slices), nullptr);
}

TEST(ProductManifold, AmbientOpsEmptyDimsThrow) {
  auto manifold = MakeProduct({{kE2, 1}});

  VectorXd full = VectorXd::Zero(2);
  VectorXd sub = VectorXd::Zero(2);
  std::vector<contiguous_slice> slices = {};
  EXPECT_THROW(
      manifold->EmbedAmbient(beluga_estimation::to_view(full), slices, beluga_estimation::to_view(sub)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->LiftAmbient(beluga_estimation::to_view(sub), slices, beluga_estimation::to_view(full)),
      std::runtime_error);
}

// --- Axis-based rectangular LiftTangent tests ---

TEST(ProductManifold, LiftTangentMatrixByIndexColumns) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Rows pass through (residual-like, 3 rows); columns map the E2 block
  // (tangent [3,5)).
  MatrixXd sub(3, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0,
         5.0, 6.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(3, 5, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {1};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full), kColsAxis);

  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);  // non-selected columns untouched
  EXPECT_DOUBLE_EQ(full(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 2), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(full(0, 4), 2.0);
  EXPECT_DOUBLE_EQ(full(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(full(1, 4), 4.0);
  EXPECT_DOUBLE_EQ(full(2, 3), 5.0);
  EXPECT_DOUBLE_EQ(full(2, 4), 6.0);
}

TEST(ProductManifold, LiftTangentMatrixByIndexRows) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Columns pass through (2 columns); rows map the E2 block (tangent [3,5)).
  MatrixXd sub(2, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(5, 2, -1.0);  // sentinel values
  std::vector<std::size_t> indices = {1};

  manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full), kRowsAxis);

  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);  // non-selected rows untouched
  EXPECT_DOUBLE_EQ(full(1, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(2, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(3, 0), 1.0);
  EXPECT_DOUBLE_EQ(full(3, 1), 2.0);
  EXPECT_DOUBLE_EQ(full(4, 0), 3.0);
  EXPECT_DOUBLE_EQ(full(4, 1), 4.0);
}

TEST(ProductManifold, LiftTangentMatrixBySliceColumns) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Slice-based axis overload: map the E2 tangent range [3,5) as columns.
  MatrixXd sub(3, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0,
         5.0, 6.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(3, 5, -1.0);
  std::vector<contiguous_slice> slices = {contiguous_slice::Span(3, 2)};

  manifold->LiftTangent(beluga_estimation::to_view(sub), slices, beluga_estimation::to_view(full), kColsAxis);

  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(full(0, 4), 2.0);
  EXPECT_DOUBLE_EQ(full(2, 3), 5.0);
  EXPECT_DOUBLE_EQ(full(2, 4), 6.0);
}

TEST(ProductManifold, LiftTangentAxisEmptyIndicesThrows) {
  auto manifold = MakeProduct({{kE2, 1}});

  MatrixXd sub = MatrixXd::Zero(2, 2);
  MatrixXd full = MatrixXd::Zero(2, 2);
  std::vector<std::size_t> indices = {};
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), indices, beluga_estimation::to_view(full), kColsAxis),
      std::runtime_error);
  std::vector<std::size_t> bad = {0, 7};
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), bad, beluga_estimation::to_view(full), kRowsAxis),
      std::runtime_error);
}

// --- Two-index-set rectangular LiftTangent tests ---

TEST(ProductManifold, LiftTangentMatrixByIndicesRowsAndColumns) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  // Rows map the SE(2) block (tangent [0,3)); columns map the E2 block
  // (tangent [3,5)). The result is a 3x2 tile at full[0:3, 3:5].
  MatrixXd sub(3, 2);
  // clang-format off
  sub << 1.0, 2.0,
         3.0, 4.0,
         5.0, 6.0;
  // clang-format on
  MatrixXd full = MatrixXd::Constant(5, 5, -1.0);  // sentinel values
  std::vector<std::size_t> row_indices = {0};
  std::vector<std::size_t> col_indices = {1};

  manifold->LiftTangent(beluga_estimation::to_view(sub), row_indices, col_indices, beluga_estimation::to_view(full));

  EXPECT_DOUBLE_EQ(full(0, 3), 1.0);
  EXPECT_DOUBLE_EQ(full(0, 4), 2.0);
  EXPECT_DOUBLE_EQ(full(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(full(1, 4), 4.0);
  EXPECT_DOUBLE_EQ(full(2, 3), 5.0);
  EXPECT_DOUBLE_EQ(full(2, 4), 6.0);
  // Non-selected entries untouched.
  EXPECT_DOUBLE_EQ(full(0, 0), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 1), -1.0);
  EXPECT_DOUBLE_EQ(full(3, 3), -1.0);
  EXPECT_DOUBLE_EQ(full(0, 2), -1.0);
}

TEST(ProductManifold, LiftTangentMatrixByIndicesRowsAndColumnsEmptyThrows) {
  auto manifold = MakeProduct({{kE2, 1}});

  MatrixXd sub = MatrixXd::Zero(2, 2);
  MatrixXd full = MatrixXd::Zero(2, 2);
  std::vector<std::size_t> indices = {0};
  std::vector<std::size_t> empty = {};
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), indices, empty, beluga_estimation::to_view(full)),
      std::runtime_error);
  EXPECT_THROW(
      manifold->LiftTangent(beluga_estimation::to_view(sub), empty, indices, beluga_estimation::to_view(full)),
      std::runtime_error);
}

// --- EmbedAmbient view-of-spans tests ---

TEST(ProductManifold, EmbedAmbientViewOfSpans) {
  auto manifold = MakeProduct({{kSE2, 1}, {kE2, 1}});

  VectorXd full(6);
  full << 1.0, 0.0, 10.0, 20.0, 100.0, 200.0;

  // Multiple indices yield one view per selected sub-manifold.
  std::vector<std::size_t> indices = {0, 1};
  auto views = manifold->EmbedAmbient(beluga_estimation::to_view(full), indices);

  ASSERT_EQ(views.size(), 2u);
  EXPECT_EQ(views[0].size(), 4u);  // SE(2) block
  EXPECT_DOUBLE_EQ(views[0][0], 1.0);
  EXPECT_DOUBLE_EQ(views[0][1], 0.0);
  EXPECT_DOUBLE_EQ(views[0][2], 10.0);
  EXPECT_DOUBLE_EQ(views[0][3], 20.0);
  EXPECT_EQ(views[1].size(), 2u);  // E2 block
  EXPECT_DOUBLE_EQ(views[1][0], 100.0);
  EXPECT_DOUBLE_EQ(views[1][1], 200.0);

  // A single index yields exactly one view.
  std::vector<std::size_t> single = {1};
  auto single_view = manifold->EmbedAmbient(beluga_estimation::to_view(full), single);

  ASSERT_EQ(single_view.size(), 1u);
  EXPECT_EQ(single_view[0].size(), 2u);
  EXPECT_DOUBLE_EQ(single_view[0][0], 100.0);
  EXPECT_DOUBLE_EQ(single_view[0][1], 200.0);
}

TEST(ProductManifold, EmbedAmbientViewOfSpansInvalidIndicesThrow) {
  auto manifold = MakeProduct({{kE2, 1}});

  VectorXd full = VectorXd::Zero(2);
  std::vector<std::size_t> empty = {};
  EXPECT_THROW(manifold->EmbedAmbient(beluga_estimation::to_view(full), empty), std::runtime_error);
  std::vector<std::size_t> bad = {0, 7};
  EXPECT_THROW(manifold->EmbedAmbient(beluga_estimation::to_view(full), bad), std::runtime_error);
}

}  // namespace
}  // namespace beluga_estimation
