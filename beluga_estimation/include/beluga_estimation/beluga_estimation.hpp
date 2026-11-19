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
 * @brief Includes all the Beluga API.
 */

#include <beluga_estimation/common/assert.hpp>
#include <beluga_estimation/common/error.hpp>
#include <beluga_estimation/common/format.hpp>
#include <beluga_estimation/common/hash.hpp>
#include <beluga_estimation/common/message.hpp>
#include <beluga_estimation/common/registry.hpp>
#include <beluga_estimation/common/result.hpp>
#include <beluga_estimation/common/std_future.hpp>
#include <beluga_estimation/common/types.hpp>
#include <beluga_estimation/compute/budget.hpp>
#include <beluga_estimation/compute/operation.hpp>
#include <beluga_estimation/compute/operations/input/imu_probe.hpp>
#include <beluga_estimation/compute/operations/input/odom_probe.hpp>
#include <beluga_estimation/compute/operations/motion/odometry.hpp>
#include <beluga_estimation/compute/operations/motion/se2_kinematic_state_connector.hpp>
#include <beluga_estimation/compute/operations/output/se2_pose_broadcaster.hpp>
#include <beluga_estimation/compute/operations/solvers/ekf.hpp>
#include <beluga_estimation/compute/operations/variable_prior_initializer.hpp>
#include <beluga_estimation/compute/program.hpp>
#include <beluga_estimation/compute/programs/recursive_filter.hpp>
#include <beluga_estimation/core/constraint.hpp>
#include <beluga_estimation/core/distribution.hpp>
#include <beluga_estimation/core/identifiable.hpp>
#include <beluga_estimation/core/identifier.hpp>
#include <beluga_estimation/core/manifold.hpp>
#include <beluga_estimation/core/on_manifold_estimate.hpp>
#include <beluga_estimation/core/tangent_distribution.hpp>
#include <beluga_estimation/core/variable.hpp>
#include <beluga_estimation/manifolds/product_manifold.hpp>
#include <beluga_estimation/middleware/middleware_interface.hpp>
#include <beluga_estimation/middleware/parameter_value.hpp>
#include <beluga_estimation/middleware/parameters_interface.hpp>
#include <beluga_estimation/middleware/publisher.hpp>
#include <beluga_estimation/middleware/subscriber.hpp>
#include <beluga_estimation/middleware/topics_interface.hpp>
#include <beluga_estimation/msg/header.hpp>
#include <beluga_estimation/msg/imu.hpp>
#include <beluga_estimation/msg/odometry.hpp>
