#ifndef BELUGA_SENSOR_POSE_SENSOR_MODEL_HPP
#define BELUGA_SENSOR_POSE_SENSOR_MODEL_HPP

// external
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

// standard library
#include <algorithm>
#include <cassert>
#include <cmath>
#include <type_traits>

/**
 * \file
 * \brief Implementation of a pose sensor model for absolute pose measurements.
 */

namespace beluga {

/// A 2D absolute pose measurement with associated covariance.
/**
 *
 * \sa PoseSensorModel
 */
struct PoseMeasurement2d {
  Sophus::SE2d pose;           ///< Measured pose in the world frame.
  Eigen::Matrix3d covariance;  ///< 3x3 covariance matrix in [x, y, theta] order
};
/// A 3D absolute pose measurement with associated covariance.
/**
 *
 * \sa PoseSensorModel
 */
struct PoseMeasurement3d {
  Sophus::SE3d pose;                       ///< Measured pose in the world frame.
  Eigen::Matrix<double, 6, 6> covariance;  ///< 6x6 covariance matrix in [x, y, z, roll, pitch, yaw] order
};

/// Parameters used to construct a PoseSensorModel instance.
struct PoseSensorModelParam {
  double min_weight{1e-5};  ///< Minimum weight returned for any particle state. Must be positive.
};

/// Generic pose sensor model for absolute pose measurements (both 2D and 3D).
/**
 * Computes particle importance weights from an absolute pose measurement (e.g. from GPS
 * or visual localization) using a Mahalanobis-distance Gaussian likelihood.
 *
 * This model requires no map. The measurement covariance must be positive-definite.
 *
 * \note This class satisfies \ref SensorModelPage.
 *
 * \tparam StateType Type of the particle state. Must be Sophus::SE2d or Sophus::SE3d.
 */
template <class StateType>
class PoseSensorModel {
 public:
  static_assert(
      std::is_same_v<StateType, Sophus::SE2d> || std::is_same_v<StateType, Sophus::SE3d>,
      "PoseSensorModel only supports Sophus::SE2d and Sophus::SE3d state types.");

  /// State type of a particle.
  using state_type = StateType;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor (selected based on state dimensionality).
  using measurement_type =
      std::conditional_t<std::is_same_v<state_type, Sophus::SE2d>, PoseMeasurement2d, PoseMeasurement3d>;
  /// Parameter type used to configure this instance.
  using param_type = PoseSensorModelParam;

 private:
  static constexpr int kDim = std::is_same_v<state_type, Sophus::SE2d> ? 3 : 6;
  using CovMatrix = Eigen::Matrix<double, kDim, kDim>;

 public:
  /// Constructs a PoseSensorModel instance.
  /**
   * \param params Parameters to configure this instance. See beluga::PoseSensorModelParam for details.
   */
  explicit PoseSensorModel(param_type params) : params_{std::move(params)} {
    assert(params_.min_weight > 0.0);
  }

  /// Returns a state weighting function conditioned on an absolute pose measurement.
  /**
   * \param measurement Absolute pose measurement including covariance.
   * \return a state weighting function satisfying \ref StateWeightingFunctionPage
   */
  [[nodiscard]] auto operator()(measurement_type&& measurement) const {
    const auto llt = measurement.covariance.llt();
    assert(llt.info() == Eigen::ComputationInfo::Success);
    const CovMatrix info = llt.solve(CovMatrix::Identity());
    const state_type pose_inv = measurement.pose.inverse();
    return [this, pose_inv = std::move(pose_inv), info](const state_type& state) -> weight_type {
      const auto error = (pose_inv * state).log();
      return std::max(std::exp(-0.5 * error.dot(info * error)), params_.min_weight);
    };
  }

 private:
  param_type params_;
};

/// Convenience alias for a 2D pose sensor model.
using PoseSensorModel2d = PoseSensorModel<Sophus::SE2d>;

/// Convenience alias for a 3D pose sensor model.
using PoseSensorModel3d = PoseSensorModel<Sophus::SE3d>;

}  // namespace beluga

#endif
