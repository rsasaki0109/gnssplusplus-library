#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace libgnss {
namespace rtk_update {

namespace {

struct InnovationStats {
    bool ok = false;
    int observation_count = 0;
    double normalized_innovation_squared = 0.0;
    double normalized_innovation_squared_per_observation = 0.0;
    Eigen::VectorXd hph_diagonal;
};

double residualRms(const Eigen::VectorXd& residuals) {
    if (residuals.size() == 0) {
        return 0.0;
    }
    return std::sqrt(residuals.squaredNorm() / static_cast<double>(residuals.size()));
}

double residualMaxAbs(const Eigen::VectorXd& residuals) {
    double max_abs = 0.0;
    for (int index = 0; index < residuals.size(); ++index) {
        max_abs = std::max(max_abs, std::abs(residuals(index)));
    }
    return max_abs;
}

InnovationStats computeInnovationStats(const Eigen::VectorXd& state,
                                       const Eigen::MatrixXd& covariance,
                                       const Eigen::MatrixXd& design_matrix,
                                       const Eigen::VectorXd& residuals,
                                       const Eigen::MatrixXd& measurement_covariance,
                                       const std::vector<bool>& force_active,
                                       bool export_hph_diagonal,
                                       bool compute_nis = true) {
    InnovationStats stats;
    const int n = static_cast<int>(state.size());
    const int m = static_cast<int>(residuals.size());
    if (m == 0 || design_matrix.rows() != m || measurement_covariance.rows() != m ||
        measurement_covariance.cols() != m) {
        return stats;
    }
    if (!force_active.empty() && static_cast<int>(force_active.size()) != n) {
        return stats;
    }

    std::vector<int> active_states;
    active_states.reserve(n);
    for (int i = 0; i < n; ++i) {
        const bool explicitly_active = !force_active.empty() && force_active[i];
        if ((state(i) != 0.0 || explicitly_active) && covariance(i, i) > 0.0) {
            active_states.push_back(i);
        }
    }
    if (active_states.empty()) {
        return stats;
    }

    for (int row = 0; row < m; ++row) {
        if (residuals(row) != 0.0 || !design_matrix.row(row).isZero(0.0)) {
            ++stats.observation_count;
        }
    }
    if (stats.observation_count == 0) {
        return stats;
    }

    Eigen::MatrixXd active_design(m, static_cast<int>(active_states.size()));
    Eigen::MatrixXd active_covariance(active_states.size(), active_states.size());
    for (int col = 0; col < static_cast<int>(active_states.size()); ++col) {
        const int state_col = active_states[static_cast<size_t>(col)];
        active_design.col(col) = design_matrix.col(state_col);
        for (int row = 0; row < static_cast<int>(active_states.size()); ++row) {
            active_covariance(row, col) =
                covariance(active_states[static_cast<size_t>(row)], state_col);
        }
    }

    const Eigen::MatrixXd innovation_covariance =
        active_design * active_covariance * active_design.transpose() + measurement_covariance;
    if (export_hph_diagonal) {
        stats.hph_diagonal =
            innovation_covariance.diagonal() - measurement_covariance.diagonal();
    }
    if (!compute_nis) {
        stats.ok = true;
        return stats;
    }
    Eigen::LDLT<Eigen::MatrixXd> ldlt(innovation_covariance);
    if (ldlt.info() != Eigen::Success) {
        return stats;
    }
    const Eigen::VectorXd weighted = ldlt.solve(residuals);
    if (!weighted.allFinite()) {
        return stats;
    }
    stats.normalized_innovation_squared = residuals.dot(weighted);
    stats.normalized_innovation_squared_per_observation =
        stats.normalized_innovation_squared / static_cast<double>(stats.observation_count);
    stats.ok = std::isfinite(stats.normalized_innovation_squared) &&
               std::isfinite(stats.normalized_innovation_squared_per_observation);
    return stats;
}

}  // namespace

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count,
                                          double max_normalized_innovation_squared_per_observation,
                                          const std::vector<bool>& force_active,
                                          bool compute_row_stats,
                                          bool reuse_kalman_factorization_for_nis) {
    FilterUpdateResult result;
    result.observation_count = static_cast<int>(measurement_system.residuals.size());
    result.prefit_residual_rms_m = residualRms(measurement_system.residuals);
    result.prefit_residual_max_abs_m = residualMaxAbs(measurement_system.residuals);
    if (result.observation_count < min_observation_count) {
        result.post_suppression_residual_rms_m = result.prefit_residual_rms_m;
        result.post_suppression_residual_max_abs_m = result.prefit_residual_max_abs_m;
        return result;
    }

    result.suppressed_outliers = rtk_measurement::suppressOutlierRows(
        measurement_system.residuals, measurement_system.design_matrix, outlier_threshold,
        measurement_system.row_outlier_thresholds);
    result.post_suppression_residual_rms_m = residualRms(measurement_system.residuals);
    result.post_suppression_residual_max_abs_m = residualMaxAbs(measurement_system.residuals);

    const bool can_reuse_kalman_factorization =
        reuse_kalman_factorization_for_nis &&
        !(std::isfinite(max_normalized_innovation_squared_per_observation) &&
          max_normalized_innovation_squared_per_observation > 0.0);
    if (can_reuse_kalman_factorization) {
        // Preserve the legacy H*P*H' evaluation order used by adaptive
        // row statistics, but skip its LDLT. The Kalman update below returns
        // the weighted innovation from the LU it already requires.
        const auto row_stats = computeInnovationStats(
            state,
            covariance,
            measurement_system.design_matrix,
            measurement_system.residuals,
            measurement_system.covariance,
            force_active,
            compute_row_stats,
            /*compute_nis=*/false);
        result.innovation_observation_count = row_stats.observation_count;
        Eigen::VectorXd weighted_innovation;
        const int info = kalmanFilter(
            state,
            covariance,
            measurement_system.design_matrix,
            measurement_system.residuals,
            measurement_system.covariance,
            force_active,
            &weighted_innovation,
            nullptr);
        result.ok = (info == 0);
        if (result.ok &&
            weighted_innovation.size() == measurement_system.residuals.size() &&
            result.innovation_observation_count > 0) {
            result.normalized_innovation_squared =
                measurement_system.residuals.dot(weighted_innovation);
            result.normalized_innovation_squared_per_observation =
                result.normalized_innovation_squared /
                static_cast<double>(result.innovation_observation_count);
        }
        if (compute_row_stats && result.ok &&
            row_stats.hph_diagonal.size() == measurement_system.residuals.size()) {
            result.row_innovations = measurement_system.residuals;
            result.row_hph_diagonal = row_stats.hph_diagonal;
        }
        return result;
    }

    const auto innovation_stats = computeInnovationStats(state,
                                                         covariance,
                                                         measurement_system.design_matrix,
                                                         measurement_system.residuals,
                                                         measurement_system.covariance,
                                                         force_active,
                                                         compute_row_stats);
    result.innovation_observation_count = innovation_stats.observation_count;
    if (compute_row_stats && innovation_stats.hph_diagonal.size() ==
                                 measurement_system.residuals.size()) {
        result.row_innovations = measurement_system.residuals;
        result.row_hph_diagonal = innovation_stats.hph_diagonal;
    }
    result.normalized_innovation_squared =
        innovation_stats.ok ? innovation_stats.normalized_innovation_squared : 0.0;
    result.normalized_innovation_squared_per_observation =
        innovation_stats.ok ? innovation_stats.normalized_innovation_squared_per_observation : 0.0;
    if (std::isfinite(max_normalized_innovation_squared_per_observation) &&
        max_normalized_innovation_squared_per_observation > 0.0) {
        if (!innovation_stats.ok ||
            innovation_stats.normalized_innovation_squared_per_observation >
                max_normalized_innovation_squared_per_observation) {
            result.rejected_by_innovation_gate = true;
            return result;
        }
    }

    const int info = kalmanFilter(state,
                                  covariance,
                                  measurement_system.design_matrix,
                                  measurement_system.residuals,
                                  measurement_system.covariance,
                                  force_active);
    result.ok = (info == 0);
    return result;
}

}  // namespace rtk_update
}  // namespace libgnss
