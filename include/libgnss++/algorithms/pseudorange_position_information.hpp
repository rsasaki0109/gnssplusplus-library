#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace libgnss::pseudorange_information {

struct ScalarResidualProjection {
    double information = 0.;
    double coefficient_residual_dot = 0.;
    double residual_energy = 0.;
};

// Diagnostic moments for r + a*delta + B*n. A local scalar step, when
// identifiable, has sign -coefficient_residual_dot/information. No step is
// returned or applied here; rank-deficient cases must not produce huge fits.
inline ScalarResidualProjection scalarResidualProjection(
    const Eigen::VectorXd& coefficient, const Eigen::MatrixXd& nuisance,
    const Eigen::VectorXd& residual, const Eigen::VectorXd& sigma_m) {
    if (coefficient.size() != nuisance.rows() || coefficient.size() != residual.size() ||
        coefficient.size() != sigma_m.size() || !coefficient.allFinite() ||
        !nuisance.allFinite() || !residual.allFinite() || !sigma_m.allFinite() ||
        (sigma_m.array() <= 0).any())
        throw std::invalid_argument("Invalid residual projection inputs");
    if (!coefficient.size()) return {};
    Eigen::MatrixXd ar(coefficient.size(), 2);
    ar.col(0) = coefficient.array() / sigma_m.array();
    ar.col(1) = residual.array() / sigma_m.array();
    Eigen::MatrixXd b = nuisance.array().colwise() / sigma_m.array();
    if (!ar.allFinite() || !b.allFinite())
        throw std::invalid_argument("Nonfinite whitened residual projection");
    if (b.cols()) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(b, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::MatrixXd u = svd.matrixU().leftCols(svd.rank());
        ar = (ar - u * (u.transpose() * ar)).eval();
    }
    ScalarResidualProjection result{ar.col(0).squaredNorm(),
        ar.col(0).dot(ar.col(1)), ar.col(1).squaredNorm()};
    if (!std::isfinite(result.information) || !std::isfinite(result.coefficient_residual_dot) ||
        !std::isfinite(result.residual_energy))
        throw std::invalid_argument("Nonfinite residual projection moments");
    return result;
}

// Single scalar measurement column after eliminating arbitrary nuisance
// columns (e.g. position plus C7). Gaussian local information only: no prior,
// damping or robust reweighting. Redundant/absent nuisance columns are allowed.
inline double scalarProjectedInformation(const Eigen::VectorXd& coefficient,
                                         const Eigen::MatrixXd& nuisance,
                                         const Eigen::VectorXd& sigma_m) {
    if (coefficient.size() != nuisance.rows() || coefficient.size() != sigma_m.size() ||
        !coefficient.allFinite() || !nuisance.allFinite() || !sigma_m.allFinite() ||
        (sigma_m.array() <= 0).any())
        throw std::invalid_argument("Invalid scalar information inputs");
    if (!coefficient.size()) return 0.;
    Eigen::VectorXd a = coefficient.array() / sigma_m.array();
    Eigen::MatrixXd b = nuisance.array().colwise() / sigma_m.array();
    if (!a.allFinite() || !b.allFinite())
        throw std::invalid_argument("Nonfinite whitened scalar information inputs");
    if (b.cols()) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(b, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::MatrixXd u = svd.matrixU().leftCols(svd.rank());
        a = (a - u * (u.transpose() * a)).eval();
    }
    const double information = a.squaredNorm();
    if (!std::isfinite(information))
        throw std::invalid_argument("Nonfinite scalar information");
    return information;
}

// Diagnostic only: project whitened position Jacobians out of the observed
// clock-column space. No clock prior, damping, trajectory or truth is used.
// This is single-epoch linearized P information, NOT full FGO observability.
inline Eigen::Matrix3d clockProjectedInformation(
    const Eigen::MatrixXd& position_jacobian,
    const Eigen::MatrixXd& clock_jacobian,
    const Eigen::VectorXd& sigma_m) {
    if (position_jacobian.cols() != 3 ||
        position_jacobian.rows() != clock_jacobian.rows() ||
        position_jacobian.rows() != sigma_m.size() ||
        !position_jacobian.allFinite() || !clock_jacobian.allFinite() ||
        !sigma_m.allFinite() || (sigma_m.array() <= 0).any()) {
        throw std::invalid_argument("Invalid pseudorange information inputs");
    }
    if (sigma_m.size() == 0) return Eigen::Matrix3d::Zero();
    Eigen::MatrixXd a = position_jacobian.array().colwise() / sigma_m.array();
    Eigen::MatrixXd b = clock_jacobian.array().colwise() / sigma_m.array();
    if (!a.allFinite() || !b.allFinite()) {
        throw std::invalid_argument("Nonfinite whitened pseudorange Jacobian");
    }
    if (b.cols() > 0) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(b, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // Eigen's numerical rank excludes absent/redundant clock slots.
        const Eigen::MatrixXd u = svd.matrixU().leftCols(svd.rank());
        a = (a - u * (u.transpose() * a)).eval();
    }
    const Eigen::Matrix3d information = a.transpose() * a;
    if (!information.allFinite()) {
        throw std::invalid_argument("Nonfinite pseudorange information");
    }
    return information;
}

} // namespace libgnss::pseudorange_information
