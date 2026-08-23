#include <libgnss++/fusion/fusion_update.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss {
namespace fusion_update {

namespace {

// LDLT::solve() can return a finite but meaningless vector when S has a
// negative or effectively zero pivot. That used to turn a non-PSD propagated
// covariance into a tiny/negative NIS, which then passed the gate and made the
// next state update depend on the exact floating-point path. Use an explicit
// PSD/conditioning check before solving; R is already floored by the caller,
// so this scale-relative floor only rejects genuinely singular innovation
// matrices rather than ordinary small measurement variances.
constexpr double kInnovationEigenvalueRelativeFloor = 1e-12;

}  // namespace

FusionUpdateResult applyDenseUpdate(Eigen::Matrix<double, 15, 1>& error_state,
                                    Eigen::Matrix<double, 15, 15>& covariance,
                                    const fusion_measurement::FusionMeasurementSystem& system,
                                    double max_normalized_innovation_squared_per_observation) {
    FusionUpdateResult result;
    const int m = static_cast<int>(system.residuals.size());
    result.observation_count = m;
    if (m == 0 || system.design_matrix.rows() != m || system.design_matrix.cols() != fusion_index::SIZE ||
        system.covariance.rows() != m || system.covariance.cols() != m) {
        return result;
    }

    const Eigen::MatrixXd& H = system.design_matrix;
    const Eigen::VectorXd& v = system.residuals;
    const Eigen::MatrixXd& R = system.covariance;

    if (!error_state.allFinite() || !covariance.allFinite() || !H.allFinite() ||
        !v.allFinite() || !R.allFinite()) {
        result.rejected_by_invalid_innovation_covariance = true;
        return result;
    }

    const Eigen::MatrixXd p_ht = covariance * H.transpose();  // 15 x m
    Eigen::MatrixXd innovation_covariance = H * p_ht + R;      // m x m
    innovation_covariance = 0.5 * (innovation_covariance + innovation_covariance.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(innovation_covariance);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
        !eig.eigenvectors().allFinite()) {
        result.rejected_by_invalid_innovation_covariance = true;
        return result;
    }
    const double max_eigenvalue = eig.eigenvalues().maxCoeff();
    const double min_eigenvalue = eig.eigenvalues().minCoeff();
    const double eigenvalue_floor =
        kInnovationEigenvalueRelativeFloor * std::max(1.0, std::abs(max_eigenvalue));
    if (!std::isfinite(max_eigenvalue) || !std::isfinite(min_eigenvalue) ||
        max_eigenvalue <= 0.0 || min_eigenvalue <= eigenvalue_floor) {
        result.rejected_by_invalid_innovation_covariance = true;
        return result;
    }

    const Eigen::VectorXd inverse_eigenvalues = eig.eigenvalues().cwiseInverse();
    const Eigen::MatrixXd innovation_inverse =
        eig.eigenvectors() * inverse_eigenvalues.asDiagonal() * eig.eigenvectors().transpose();

    const Eigen::VectorXd weighted_residual = innovation_inverse * v;
    if (!weighted_residual.allFinite()) {
        result.rejected_by_invalid_innovation_covariance = true;
        return result;
    }
    result.normalized_innovation_squared = v.dot(weighted_residual);
    result.normalized_innovation_squared_per_observation =
        result.normalized_innovation_squared / static_cast<double>(m);
    if (!std::isfinite(result.normalized_innovation_squared)) {
        return result;
    }

    if (std::isfinite(max_normalized_innovation_squared_per_observation) &&
        max_normalized_innovation_squared_per_observation > 0.0 &&
        result.normalized_innovation_squared_per_observation >
            max_normalized_innovation_squared_per_observation) {
        result.rejected_by_innovation_gate = true;
        return result;
    }

    // K = P H^T S^-1: solve S * K^T = (P H^T)^T for K^T, then transpose.
    const Eigen::MatrixXd kt = innovation_inverse * p_ht.transpose();  // m x 15
    if (!kt.allFinite()) {
        result.rejected_by_invalid_innovation_covariance = true;
        return result;
    }
    const Eigen::MatrixXd K = kt.transpose();  // 15 x m

    error_state += K * v;

    const Eigen::MatrixXd i_kh =
        Eigen::MatrixXd::Identity(fusion_index::SIZE, fusion_index::SIZE) - K * H;
    Eigen::MatrixXd updated_covariance = i_kh * covariance * i_kh.transpose() + K * R * K.transpose();
    updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
    covariance = updated_covariance;

    result.ok = true;
    return result;
}

}  // namespace fusion_update
}  // namespace libgnss
