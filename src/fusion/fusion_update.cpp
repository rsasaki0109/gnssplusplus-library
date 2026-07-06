#include <libgnss++/fusion/fusion_update.hpp>

#include <cmath>

namespace libgnss {
namespace fusion_update {

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

    const Eigen::MatrixXd p_ht = covariance * H.transpose();  // 15 x m
    Eigen::MatrixXd innovation_covariance = H * p_ht + R;      // m x m
    innovation_covariance = 0.5 * (innovation_covariance + innovation_covariance.transpose());

    Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success) {
        return result;
    }

    const Eigen::VectorXd weighted_residual = ldlt.solve(v);
    if (!weighted_residual.allFinite()) {
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
    const Eigen::MatrixXd kt = ldlt.solve(p_ht.transpose());  // m x 15
    if (!kt.allFinite()) {
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
