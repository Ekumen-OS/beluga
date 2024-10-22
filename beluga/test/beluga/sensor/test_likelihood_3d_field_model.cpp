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

#include <openvdb/math/Mat.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Diagnostics.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetTracker.h>
#include <openvdb/tools/TopologyToLevelSet.h>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/common.hpp>

#include "beluga/sensor/likelihood_3d_field_model.hpp"
#include "beluga/test/simple_pointcloud_interface.hpp"

namespace {

template <typename GridT, typename T>
auto make_map(const double voxel_size, const std::vector<T>& world_points) {
  // Create the grid
  typename GridT::Ptr grid = openvdb::createLevelSet<GridT>(voxel_size);
  // Get an accessor for coordinate-based access to voxels
  typename GridT::Accessor accessor = grid->getAccessor();

  // Fill the grid
  const openvdb::math::Transform transform;
  for (const auto& point : world_points) {
    // Transform to index world
    const openvdb::math::Coord ijk = transform.worldToIndexCellCentered(point);
    // Set the voxel to 1
    accessor.setValue(ijk, 1.0);
    // Activate the voxel
    accessor.setActiveState(ijk);
  }

  // Prune the grid
  // Set voxels that are outside the narrow band to the background value and prune the grid
  openvdb::tools::LevelSetTracker<GridT> pruner(*grid);
  pruner.setTrimming(openvdb::tools::lstrack::TrimMode::kAll);
  pruner.prune();

  // Check grid
  openvdb::tools::CheckLevelSet<GridT> checker(*grid);
  // Check inactive values have a magnitude equal to the background value
  assert(checker.checkInactiveValues() == "");

  // Associate metadata
  // just a terminal command to remember
  // vdb_print -l pcdgrid.vdb
  grid->setName("map");
  grid->setGridClass(openvdb::GridClass::GRID_LEVEL_SET);
  grid->setIsInWorldSpace(true);  // not quite sure!

  return grid;
}

TEST(Likelihood3DFieldModel, Likelihood3DField) {
  openvdb::initialize();
  // Create Grid
  const double voxel_size = 0.07;
  const std::vector<openvdb::math::Vec3s> world_points{openvdb::math::Vec3s(1.0F, 1.0F, 1.0F)};
  auto map = make_map<openvdb::FloatGrid, openvdb::math::Vec3s>(voxel_size, world_points);

  const auto params = beluga::Likelihood3DFieldModelParam{voxel_size, 2.0, 20.0, 0.5, 0.5, 0.2};
  auto pointcloud_measurement =
      beluga::testing::SparsePointCloud3<float>{std::vector<Eigen::Vector3<float>>{{1.0F, 1.0F, 1.0F}}};
  auto sensor_model =
      beluga::Likelihood3DFieldModel<openvdb::FloatGrid, beluga::testing::SparsePointCloud3<float>>{params, *map};

  auto state_weighting_function = sensor_model(std::move(pointcloud_measurement));
  ASSERT_NEAR(1.0, state_weighting_function(Sophus::SE3d{}), 0.03);
}

}  // namespace
