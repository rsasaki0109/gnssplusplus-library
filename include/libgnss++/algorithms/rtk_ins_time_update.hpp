#pragma once

#include <Eigen/Dense>

namespace libgnss::rtk_ins_time_update {

/**
 * @brief Apply an externally mechanized ECEF position increment to RTK state.
 *
 * Only the leading three position elements and their 3x3 covariance block are
 * changed. Every position-to-ambiguity/ionosphere cross-covariance is retained
 * exactly. Inputs are validated before mutation; false leaves both arguments
 * unchanged.
 */
bool apply(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
           const Eigen::Vector3d& position_delta_ecef,
           const Eigen::Matrix3d& process_noise_ecef,
           double position_q_floor_m2);

/**
 * @brief Apply a coupled position/velocity INS time update.
 *
 * Velocity lives at `velocity_index` so existing RTK ambiguity indices remain
 * unchanged. The 6x6 order is [position, velocity]. A zero-covariance velocity
 * block is initialized from `velocity_initial_covariance_ecef`; subsequent
 * calls accumulate process noise. All unrelated cross-covariances are kept.
 */
bool applyPositionVelocity(
    Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
    const Eigen::Vector3d& position_delta_ecef,
    const Eigen::Vector3d& velocity_ecef,
    const Eigen::Matrix<double, 6, 6>& process_noise_ecef,
    const Eigen::Matrix3d& velocity_initial_covariance_ecef,
    double position_q_floor_m2,
    int velocity_index);

}  // namespace libgnss::rtk_ins_time_update
