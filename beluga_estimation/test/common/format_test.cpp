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

#include <chrono>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <beluga_estimation/common/algebra.hpp>
#include <beluga_estimation/common/format.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/mocks/mock_middleware.hpp>

namespace {

using namespace beluga_estimation;

/// Counts how many times a format argument is actually rendered.
int g_render_count = 0;

/// Test-only wrapper whose formatter records that rendering happened.
struct Counted {
  int value;
};

}  // namespace

/// @brief Formatter for @ref Counted, recording every rendering.
template <>
struct fmt::formatter<Counted> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const Counted& counted, FormatContext& context) const {
    ++g_render_count;
    return fmt::formatter<std::string_view>::format(fmt::format("{}", counted.value), context);
  }
};

namespace {

using ::testing::_;
using ::testing::Return;

TimePoint MakeTime(int seconds) {
  return TimePoint{std::chrono::seconds{seconds}};
}

TEST(IdentifierFormattingTest, RendersNameAndTimestamp) {
  EXPECT_EQ(fmt::format("{}", Identifier{"pose", MakeTime(42)}), "pose@42000000000");
}

TEST(IdentifierFormattingTest, RendersInsideLogMessages) {
  const Identifier id{"velocity", MakeTime(1)};
  EXPECT_EQ(fmt::format("variable '{}' has no prior", id), "variable 'velocity@1000000000' has no prior");
}

TEST(IdentifierFormattingTest, JoinsIdentifiers) {
  const std::vector<Identifier> ids{{"pose", MakeTime(0)}, {"velocity", MakeTime(1)}};
  EXPECT_EQ(fmt::format("[{}]", fmt::join(ids, ", ")), "[pose@0, velocity@1000000000]");
}

TEST(IdentifierFormattingTest, JoinsIdentifiersFromSpan) {
  const std::vector<Identifier> ids{{"pose", MakeTime(0)}, {"velocity", MakeTime(1)}};
  const std_future::span<const Identifier> span{ids};
  EXPECT_EQ(fmt::format("[{}]", fmt::join(span, ", ")), "[pose@0, velocity@1000000000]");
}

TEST(IdentifierFormattingTest, JoinsEmptyRange) {
  EXPECT_EQ(fmt::format("[{}]", fmt::join(std::vector<Identifier>{}, ", ")), "[]");
}

TEST(VectorLikeTest, RendersFixedSizeVector) {
  const Vector3d values{1.0, 2.0, 0.5};
  EXPECT_EQ(fmt::format("{}", VectorLike(values)), "[1, 2, 0.5]");
}

TEST(VectorLikeTest, RendersDynamicVector) {
  const VectorX<double> values = VectorX<double>::Zero(2);
  EXPECT_EQ(fmt::format("{}", VectorLike(values)), "[0, 0]");
}

TEST(VectorLikeTest, RendersEigenExpression) {
  const Vector3d first{1.0, 2.0, 3.0};
  const Vector3d second{0.5, 0.5, 0.5};
  EXPECT_EQ(fmt::format("{}", VectorLike(first + second)), "[1.5, 2.5, 3.5]");
}

TEST(VectorLikeTest, RendersRefToVector) {
  VectorX<double> storage = VectorX<double>::Zero(3);
  storage << 1.0, 2.0, 3.0;
  const Ref<VectorX<double>> view{storage};
  EXPECT_EQ(fmt::format("{}", VectorLike(view)), "[1, 2, 3]");
}

TEST(VectorLikeTest, UsesSixSignificantDigits) {
  const Vector2d values{1.0 / 3.0, 123456789.0};
  EXPECT_EQ(fmt::format("{}", VectorLike(values)), "[0.333333, 1.23457e+08]");
}

TEST(MatrixLikeTest, RendersMatrixDiagonal) {
  Matrix3d matrix = Matrix3d::Identity() * 1.5;
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix, MatrixStyle::kDiagonal)), "[1.5, 1.5, 1.5]");
}

TEST(MatrixLikeTest, RendersOnlyDiagonalOfNonDiagonalMatrix) {
  Matrix2d matrix;
  matrix << 1.0, 9.0, 9.0, 2.0;
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix, MatrixStyle::kDiagonal)), "[1, 2]");
}

TEST(MatrixLikeTest, RendersFullMatrixByDefault) {
  Matrix2d matrix;
  matrix << 1.0, 9.0, 9.0, 2.0;
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix)), "[[1, 9], [9, 2]]");
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix, MatrixStyle::kFull)), "[[1, 9], [9, 2]]");
}

TEST(MatrixLikeTest, RendersSingleEntryMatrix) {
  Matrix1d matrix;
  matrix << 2.0;
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix, MatrixStyle::kDiagonal)), "[2]");
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix)), "[[2]]");
}

TEST(MatrixLikeTest, RendersEmptyMatrix) {
  const MatrixX<double> matrix;
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix, MatrixStyle::kDiagonal)), "[]");
  EXPECT_EQ(fmt::format("{}", MatrixLike(matrix)), "[]");
}

TEST(MatrixLikeTest, RendersFullEigenExpression) {
  const Matrix2d covariance = Matrix2d::Identity();
  EXPECT_EQ(fmt::format("{}", MatrixLike(covariance * Matrix2d::Identity())), "[[1, 0], [0, 1]]");
}

TEST(MatrixLikeTest, RendersEigenExpression) {
  const Matrix3d covariance = Matrix3d::Identity();
  EXPECT_EQ(
      fmt::format("{}", MatrixLike(covariance * Matrix3d::Identity() + covariance, MatrixStyle::kDiagonal)),
      "[2, 2, 2]");
}

TEST(FormatLazinessTest, ArgumentsAreNotRenderedWhenDebugLoggingIsDisabled) {
  auto middleware = std::make_shared<MockMiddleware>();
  ON_CALL(*middleware->logger, IsEnabled(LogLevel::kDebug)).WillByDefault(Return(false));
  EXPECT_CALL(*middleware->logger, Log(_, _)).Times(0);

  g_render_count = 0;
  middleware->Debug("value {}", Counted{7});

  EXPECT_EQ(g_render_count, 0);
}

TEST(FormatLazinessTest, ArgumentsAreRenderedWhenDebugLoggingIsEnabled) {
  auto middleware = std::make_shared<MockMiddleware>();
  ON_CALL(*middleware->logger, IsEnabled(LogLevel::kDebug)).WillByDefault(Return(true));
  EXPECT_CALL(*middleware->logger, Log(LogLevel::kDebug, _)).Times(1);

  g_render_count = 0;
  middleware->Debug("value {}", Counted{7});

  EXPECT_EQ(g_render_count, 1);
}

}  // namespace
