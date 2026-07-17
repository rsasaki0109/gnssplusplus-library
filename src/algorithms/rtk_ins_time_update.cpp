#include <libgnss++/algorithms/rtk_ins_time_update.hpp>

#include <cmath>

#include <Eigen/Eigenvalues>

namespace libgnss::rtk_ins_time_update {

bool apply(Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
           const Eigen::Vector3d& position_delta_ecef,
           const Eigen::Matrix3d& process_noise_ecef,
           double position_q_floor_m2) {
    const Eigen::Index state_size = state.size();
    if (state_size < 3 || covariance.rows() != state_size ||
        covariance.cols() != state_size || !state.allFinite() ||
        !covariance.allFinite() || !position_delta_ecef.allFinite() ||
        !process_noise_ecef.allFinite() || !std::isfinite(position_q_floor_m2) ||
        position_q_floor_m2 < 0.0) {
        return false;
    }

    const Eigen::Matrix3d symmetric_noise =
        0.5 * (process_noise_ecef + process_noise_ecef.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> noise_solver(symmetric_noise);
    if (noise_solver.info() != Eigen::Success ||
        noise_solver.eigenvalues().minCoeff() < -1e-10) {
        return false;
    }

    // Clip only tiny negative eigenvalues introduced by frame rotation or
    // floating-point symmetrization. Materially indefinite input was rejected
    // above and never mutates the caller's state.
    const Eigen::Vector3d clipped_eigenvalues =
        noise_solver.eigenvalues().cwiseMax(0.0);
    const Eigen::Matrix3d regularized_noise =
        noise_solver.eigenvectors() * clipped_eigenvalues.asDiagonal() *
        noise_solver.eigenvectors().transpose();

    const Eigen::Vector3d next_position = state.head<3>() + position_delta_ecef;
    const Eigen::Matrix3d next_position_covariance =
        covariance.topLeftCorner<3, 3>() + regularized_noise +
        position_q_floor_m2 * Eigen::Matrix3d::Identity();
    if (!next_position.allFinite() || !next_position_covariance.allFinite()) {
        return false;
    }

    state.head<3>() = next_position;
    covariance.topLeftCorner<3, 3>() =
        0.5 * (next_position_covariance + next_position_covariance.transpose());
    return true;
}

}  // namespace libgnss::rtk_ins_time_update
