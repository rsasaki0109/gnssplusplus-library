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

}  // namespace libgnss::rtk_ins_time_update
