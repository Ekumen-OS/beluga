// Copyright 2024 Ekumen, Inc.
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <openvdb/openvdb.h>
#include <openvdb/tools/Diagnostics.h>
#include <openvdb/tools/TopologyToLevelSet.h>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/common.hpp>

#include "beluga_vdb/sensor/likelihood_field_model3.hpp"
#include "beluga_vdb/test/simple_pointcloud_interface.hpp"

namespace {

template <typename GridT, typename T>
auto make_map(const double voxel_size, const std::vector<T>& world_points) {
  // Parameters
  constexpr int kHalfWidth = 3;
  constexpr int kClosingSteps = 0;

  // Create the grid
  const typename GridT::Ptr grid = openvdb::FloatGrid::create();
  grid->setTransform(openvdb::math::Transform::createLinearTransform(voxel_size));

  // Get an accessor for coordinate-based access to voxels
  typename GridT::Accessor accessor = grid->getAccessor();

  // Fill the grid
  const openvdb::math::Transform& transform = grid->transform();
  for (const auto& point : world_points) {
    // Transform to index world
    const openvdb::math::Coord ijk = transform.worldToIndexCellCentered(point);
    // Set the voxel to 1
    accessor.setValue(ijk, 1.0);
    // Activate the voxel
    accessor.setActiveState(ijk);
  }

  // Check grid
  openvdb::tools::CheckLevelSet<GridT> checker(*grid);
  // Check inactive values have a magnitude equal to the background value
  assert(checker.checkInactiveValues() == "");

  // Transform to levelset
  return openvdb::tools::topologyToLevelSet(*grid, kHalfWidth, kClosingSteps);
}

TEST(TestLikelihoodFieldModel3, Point) {
  openvdb::initialize();
  // Parameters
  constexpr double kVoxelSize = 0.07;
  constexpr double kMaxObstacleDistance = 2.0;
  constexpr double kMaxLaserDistance = 100.0;
  constexpr double kZHit = 0.5;
  constexpr double kZRandom = 0.001;
  constexpr double kSigmaHit = 0.22;

  // Create Grid
  const std::vector<openvdb::math::Vec3s> world_points{openvdb::math::Vec3s(1.0F, 1.0F, 1.0F)};
  auto map = make_map<openvdb::FloatGrid, openvdb::math::Vec3s>(kVoxelSize, world_points);

  const auto params =
      beluga_vdb::LikelihoodFieldModel3Param{kMaxObstacleDistance, kMaxLaserDistance, kZHit, kZRandom, kSigmaHit};
  auto sensor_model =
      beluga_vdb::LikelihoodFieldModel3<openvdb::FloatGrid, beluga_vdb::testing::SimpleSparsePointCloud3f>{
          params, *map};

  // Exact point
  auto pointcloud_measurement_exact =
      beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{{1.0F, 1.0F, 1.0F}}};
  auto state_weighting_function_exact = sensor_model(std::move(pointcloud_measurement_exact));
  ASSERT_GT(state_weighting_function_exact(Sophus::SE3d{}), 1.90);

  // Close point (Inside the narrow band)
  auto pointcloud_measurement_close =
      beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{{1.035F, 1.0F, 1.0F}}};
  auto state_weighting_function_close = sensor_model(std::move(pointcloud_measurement_close));
  ASSERT_GT(state_weighting_function_close(Sophus::SE3d{}), 1.8);

  // Far point (Outside the narrow band)
  auto pointcloud_measurement_far =
      beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{{1.1F, 1.1F, 1.1F}}};
  auto state_weighting_function_far = sensor_model(std::move(pointcloud_measurement_far));
  ASSERT_LT(state_weighting_function_far(Sophus::SE3d{}), 1.65);
}

TEST(TestLikelihoodFieldModel3, Cube) {
  openvdb::initialize();
  // Parameters
  constexpr double kVoxelSize = 0.07;
  constexpr double kMaxObstacleDistance = 2.0;
  constexpr double kMaxLaserDistance = 100.0;
  constexpr double kZHit = 0.5;
  constexpr double kZRandom = 0.001;
  constexpr double kSigmaHit = 0.22;

  // Create Grid
  const std::vector<openvdb::math::Vec3s> world_points{
      openvdb::math::Vec3s(1.0F, 1.0F, 1.0F),   openvdb::math::Vec3s(1.0F, 1.0F, -1.0F),
      openvdb::math::Vec3s(1.0F, -1.0F, 1.0F),  openvdb::math::Vec3s(1.0F, -1.0F, -1.0F),
      openvdb::math::Vec3s(-1.0F, -1.0F, 1.0F), openvdb::math::Vec3s(-1.0F, -1.0F, -1.0F),
      openvdb::math::Vec3s(-1.0F, 1.0F, 1.0F),  openvdb::math::Vec3s(-1.0F, 1.0F, -1.0F)};
  auto map = make_map<openvdb::FloatGrid, openvdb::math::Vec3s>(kVoxelSize, world_points);

  const auto params =
      beluga_vdb::LikelihoodFieldModel3Param{kMaxObstacleDistance, kMaxLaserDistance, kZHit, kZRandom, kSigmaHit};
  auto sensor_model =
      beluga_vdb::LikelihoodFieldModel3<openvdb::FloatGrid, beluga_vdb::testing::SimpleSparsePointCloud3f>{
          params, *map};

  // Exact point
  auto pointcloud_measurement_exact = beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{
      {1.0F, 1.0F, 1.0F},
      {1.0F, 1.0F, -1.0F},
      {1.0F, -1.0F, 1.0F},
      {1.0F, -1.0F, -1.0F},
      {-1.0F, -1.0F, 1.0F},
      {-1.0F, -1.0F, -1.0F},
      {-1.0F, 1.0F, 1.0F},
      {-1.0F, 1.0F, -1.0F}}};
  auto state_weighting_function_exact = sensor_model(std::move(pointcloud_measurement_exact));
  ASSERT_GT(state_weighting_function_exact(Sophus::SE3d{}), 8.2);

  // Close point (Inside the narrow band)
  auto pointcloud_measurement_close = beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{
      {1.035F, 1.0F, 1.0F},
      {1.035F, 1.0F, -1.0F},
      {1.035F, -1.0F, 1.0F},
      {1.035F, -1.0F, -1.0F},
      {-1.0F, -1.0F, 1.0F},
      {-1.0F, -1.0F, -1.0F},
      {-1.0F, 1.0F, 1.0F},
      {-1.0F, 1.0F, -1.0F}}};
  auto state_weighting_function_close = sensor_model(std::move(pointcloud_measurement_close));
  ASSERT_GT(state_weighting_function_close(Sophus::SE3d{}), 7.4);

  // Far point (Outside the narrow band)
  auto pointcloud_measurement_far = beluga_vdb::testing::SimpleSparsePointCloud3f{std::vector<Eigen::Vector3<float>>{
      {1.1F, 1.1F, 1.1F},
      {1.1F, 1.1F, -1.1F},
      {1.1F, -1.1F, 1.1F},
      {1.1F, -1.1F, -1.1F},
      {-1.1F, -1.1F, 1.1F},
      {-1.1F, -1.1F, -1.1F},
      {-1.1F, 1.1F, 1.1F},
      {-1.1F, 1.1F, -1.1F}}};
  auto state_weighting_function_far = sensor_model(std::move(pointcloud_measurement_far));
  ASSERT_LT(state_weighting_function_far(Sophus::SE3d{}), 6.85);
}

}  // namespace
