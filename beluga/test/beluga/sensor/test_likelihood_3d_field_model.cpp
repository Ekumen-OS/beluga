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
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/TopologyToLevelSet.h>

#include <range/v3/range/conversion.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/common.hpp>

namespace {

template <typename T, typename TT>
auto make_map(const double voxel_size, const std::vector<TT>& world_points) {
  // Create the grid
  T::Ptr grid = openvdb::tools::createLevelSet<T>(voxel_size);
  // Get an accessor for coordinate-based access to voxels
  const T::Accessor accessor = grid->getAccessor();

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
  openvdb::tools::LevelSetTracker<T> pruner(grid);
  pruner.setTrimming(openvdb::TrimMode::kAll);
  pruner.prune();

  // Check grid
  openvdb::tools::CheckLevelSet checker(grid);
  // Check inactive values have a magnitude equal to the background value
  assert(checker.checkInactiveValues() == "");

  // Associate metadata
  // vdb_print -l pcdgrid.vdb
  grid->setName("map");
  grid->setGridClass(openvdb::GridClass::GRID_LEVEL_SET);
  grid->setIsInWorldSpace(true);  // not quite sure!

  return grid;
}

TEST(Likelihood3DFieldModel, LikelihoodField) {
  openvdb::initialize();
  // Create Grid
  const double voxel_size = 0.07;
  const std::vector<openvdb::Vec3f> world_points{(1.0F, 1.0F, 1.0F)};
  auto map = make_map<openvdb::FloatGrid, openvdb::Vec3f>(voxel_size, world_points);

  const auto params = beluga::LikelihoodFieldModelParam{2.0, 20.0, 0.5, 0.5, 0.2};
  auto sensor_model = beluga::Likelihood3DFieldModel<openvdb::FloatGrid, beluga_ros::SparsePointCloud3>{params, map};

  ASSERT_THAT(
      sensor_model.likelihood_field().data(),
      testing::Pointwise(testing::DoubleNear(0.003), expected_cubed_likelihood));
}

}  // namespace
