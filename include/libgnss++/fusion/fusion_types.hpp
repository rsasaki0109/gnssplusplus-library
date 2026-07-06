#pragma once

#include <Eigen/Dense>

#include <libgnss++/core/types.hpp>

namespace libgnss {

/// Standard gravity magnitude (m/s^2), used as the loose-coupling filter's
/// default local-ENU gravity model (flat-Earth, no latitude/height correction
/// -- see fusion/fusion_types.hpp module doc in docs/design.md 3.1).
constexpr double kStandardGravityMps2 = 9.80665;

/**
 * @brief Nominal (best-estimate) navigation state of the loosely-coupled
 * error-state EKF.
 *
 * Frame: local ENU tangent plane (flat-Earth, no Earth-rotation/transport-rate
 * terms), body frame FLU (Forward-Left-Up), reached via
 * ImuAxisConvention::apply() at ingest (io/imu.hpp) -- never assumed to match
 * the raw sensor file directly.
 */
struct NominalState {
    GNSSTime time;
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();       ///< body/IMU origin, ENU, meters
    Eigen::Vector3d velocity_enu = Eigen::Vector3d::Zero();       ///< ENU, m/s
    Eigen::Quaterniond attitude_body_to_enu = Eigen::Quaterniond::Identity();  ///< body -> ENU rotation
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();         ///< m/s^2, body frame
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();          ///< rad/s, body frame
};

/**
 * @brief Error-state layout: dx(15) = [dp(3) dv(3) dtheta(3) dba(3) dbg(3)].
 */
namespace fusion_index {
constexpr int POSITION = 0;
constexpr int VELOCITY = 3;
constexpr int ATTITUDE = 6;
constexpr int ACCEL_BIAS = 9;
constexpr int GYRO_BIAS = 12;
constexpr int SIZE = 15;
}  // namespace fusion_index

/**
 * @brief Full loosely-coupled filter state: nominal state + 15x15 error
 * covariance.
 */
struct FusionState {
    NominalState nominal;
    Eigen::Matrix<double, 15, 15> covariance = Eigen::Matrix<double, 15, 15>::Zero();
};

}  // namespace libgnss
