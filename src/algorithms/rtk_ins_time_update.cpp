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

bool applyPositionVelocity(
    Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
    const Eigen::Vector3d& position_delta_ecef,
    const Eigen::Vector3d& velocity_ecef,
    const Eigen::Matrix<double, 6, 6>& process_noise_ecef,
    const Eigen::Matrix3d& velocity_initial_covariance_ecef,
    double position_q_floor_m2,
    int velocity_index) {
    const Eigen::Index state_size = state.size();
    if (state_size < 6 || covariance.rows() != state_size ||
        covariance.cols() != state_size || velocity_index < 3 ||
        velocity_index + 3 > state_size || !state.allFinite() ||
        !covariance.allFinite() || !position_delta_ecef.allFinite() ||
        !velocity_ecef.allFinite() || !process_noise_ecef.allFinite() ||
        !velocity_initial_covariance_ecef.allFinite() ||
        !std::isfinite(position_q_floor_m2) || position_q_floor_m2 < 0.0) {
        return false;
    }

    const Eigen::Matrix<double, 6, 6> symmetric_noise =
        0.5 * (process_noise_ecef + process_noise_ecef.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> noise_solver(
        symmetric_noise);
    const Eigen::Matrix3d symmetric_initial_velocity =
        0.5 * (velocity_initial_covariance_ecef +
               velocity_initial_covariance_ecef.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> velocity_solver(
        symmetric_initial_velocity);
    if (noise_solver.info() != Eigen::Success ||
        velocity_solver.info() != Eigen::Success ||
        noise_solver.eigenvalues().minCoeff() < -1e-10 ||
        velocity_solver.eigenvalues().minCoeff() < -1e-10) {
        return false;
    }

    const auto regularized_noise =
        noise_solver.eigenvectors() *
        noise_solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
        noise_solver.eigenvectors().transpose();
    const Eigen::Matrix3d regularized_initial_velocity =
        velocity_solver.eigenvectors() *
        velocity_solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
        velocity_solver.eigenvectors().transpose();

    const Eigen::Vector3d next_position = state.head<3>() + position_delta_ecef;
    const bool initialize_velocity =
        covariance.block<3, 3>(velocity_index, velocity_index).diagonal().maxCoeff() <= 0.0;
    const int indices[6] = {0, 1, 2, velocity_index, velocity_index + 1, velocity_index + 2};
    Eigen::Matrix<double, 6, 6> next_navigation_covariance;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            next_navigation_covariance(row, col) =
                covariance(indices[row], indices[col]) + regularized_noise(row, col);
        }
    }
    next_navigation_covariance.topLeftCorner<3, 3>() +=
        position_q_floor_m2 * Eigen::Matrix3d::Identity();
    if (initialize_velocity) {
        next_navigation_covariance.bottomRightCorner<3, 3>() +=
            regularized_initial_velocity;
    }
    next_navigation_covariance =
        0.5 * (next_navigation_covariance + next_navigation_covariance.transpose());
    if (!next_position.allFinite() || !next_navigation_covariance.allFinite()) return false;

    state.head<3>() = next_position;
    state.segment<3>(velocity_index) = velocity_ecef;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            covariance(indices[row], indices[col]) = next_navigation_covariance(row, col);
        }
    }
    return true;
}

}  // namespace libgnss::rtk_ins_time_update
