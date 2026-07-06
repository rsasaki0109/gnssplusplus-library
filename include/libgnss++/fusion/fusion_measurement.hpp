#pragma once

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {
namespace fusion_measurement {

/**
 * @brief Same field shape as rtk_measurement::MeasurementSystem
 * (rtk_measurement.hpp), but design_matrix columns index into the 15-wide
 * error state instead of RTKProcessor's NX-wide RTK state.
 */
struct FusionMeasurementSystem {
    Eigen::MatrixXd design_matrix;   ///< (m x 15), H such that residual ~= H * dx
    Eigen::VectorXd residuals;       ///< (m x 1), z - h(nominal state)
    Eigen::MatrixXd covariance;      ///< (m x m), measurement noise R
};

/**
 * @brief GNSS antenna position update, lever-arm compensated.
 *
 * h(x) = position_enu + R * lever_arm_body (antenna ECEF/ENU position is the
 * IMU-origin position plus the body-frame lever arm rotated into ENU by the
 * current attitude). H = [I 0 -R*[lever]x 0 0].
 */
FusionMeasurementSystem buildGnssPositionUpdate(const FusionState& state,
                                                const Eigen::Vector3d& antenna_position_enu,
                                                const Eigen::Matrix3d& position_covariance_enu,
                                                const Eigen::Vector3d& lever_arm_body);

/**
 * @brief GNSS antenna velocity update, lever-arm + angular-rate compensated.
 *
 * h(x) = velocity_enu + R * (angular_rate_body x lever_arm_body) (velocity of
 * the lever-arm-offset antenna point, ENU). angular_rate_body should already
 * be bias-corrected (measured gyro minus the current gyro bias estimate) by
 * the caller: it is treated as an opaque input here, not re-derived from
 * state's gyro bias, so the returned Jacobian has no GYRO_BIAS column (a
 * finite-difference check against this exact function confirms d(h)/d(dbg)
 * is genuinely zero for this parameterization).
 */
FusionMeasurementSystem buildGnssVelocityUpdate(const FusionState& state,
                                                const Eigen::Vector3d& antenna_velocity_enu,
                                                const Eigen::Matrix3d& velocity_covariance_enu,
                                                const Eigen::Vector3d& lever_arm_body,
                                                const Eigen::Vector3d& angular_rate_body);

/**
 * @brief Zero-velocity update (ZUPT): asserts velocity_enu ~= 0.
 *
 * H = [0 I 0 0 0], R = sigma_mps^2 * I3. Caller is responsible for gating
 * this on a stationarity detector (docs/design.md 3.8) before calling.
 */
FusionMeasurementSystem buildZuptUpdate(const FusionState& state, double sigma_mps);

/**
 * @brief Non-holonomic constraint (NHC): asserts body-frame lateral (Left)
 * and vertical (Up) velocity of a lever-arm-offset point are ~= 0 (a wheeled
 * vehicle cannot slide sideways or fly).
 *
 * v_body = R^T * velocity_enu + (angular_rate_body x lever_arm_body);
 * residual rows are [left, up] of (0,0) minus v_body. angular_rate_body
 * should already be bias-corrected.
 */
FusionMeasurementSystem buildNhcUpdate(const FusionState& state,
                                       const Eigen::Vector3d& lever_arm_body,
                                       const Eigen::Vector3d& angular_rate_body,
                                       double sigma_lateral_mps, double sigma_vertical_mps);

}  // namespace fusion_measurement
}  // namespace libgnss
