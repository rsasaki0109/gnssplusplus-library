#pragma once

#include "fgo_gtsam_internal.hpp"
#include <Eigen/Eigenvalues>

namespace libgnss::fgo_gtsam_internal {

inline Eigen::Matrix3d missingPositionCovariance() {
    return Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

// A diagonal-only check would allow an indefinite Schur complement to become
// recovery authority. Reject material negative eigenvalues; remove roundoff
// only, without manufacturing an uncertainty floor for an absent marginal.
inline Eigen::Matrix3d checkedPositionCovariance(const Eigen::Matrix3d& value) {
    if (!value.allFinite()) return missingPositionCovariance();
    const Eigen::Matrix3d symmetric = (0.5 * (value + value.transpose())).eval();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(symmetric);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().maxCoeff() <= 0.0 ||
        eigen.eigenvalues().minCoeff() <
            -1e-10 * std::max(1.0, eigen.eigenvalues().maxCoeff())) {
        return missingPositionCovariance();
    }
    if (eigen.eigenvalues().minCoeff() >= 0.0) return symmetric;
    return (eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0.0).asDiagonal() *
            eigen.eigenvectors().transpose()).eval();
}

inline Eigen::Matrix3d antennaPositionCovariance(
    const gtsam::gnss::LeverArm& lever_arm, const gtsam::Pose3& pose,
    const gtsam::Matrix& pose_covariance) {
    if (pose_covariance.rows() != 6 || pose_covariance.cols() != 6 ||
        !pose_covariance.allFinite()) return missingPositionCovariance();
    gtsam::gnss::LeverArm::PoseFrame frame;
    lever_arm.antennaPosition(pose, &frame);
    gtsam::Matrix36 jacobian;
    for (int axis = 0; axis < 3; ++axis) {
        gtsam::Matrix13 unit = gtsam::Matrix13::Zero();
        unit(0, axis) = 1.0;
        jacobian.row(axis) = lever_arm.antennaPoseJacobian(unit, frame);
    }
    return checkedPositionCovariance(
        jacobian * pose_covariance * jacobian.transpose());
}

}  // namespace libgnss::fgo_gtsam_internal
