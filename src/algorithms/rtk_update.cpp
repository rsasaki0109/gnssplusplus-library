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
                                       const Eigen::MatrixXd& measurement_covariance) {
    InnovationStats stats;
    const int n = static_cast<int>(state.size());
    const int m = static_cast<int>(residuals.size());
    if (m == 0 || design_matrix.rows() != m || measurement_covariance.rows() != m ||
        measurement_covariance.cols() != m) {
        return stats;
    }

    std::vector<int> active_states;
    active_states.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (state(i) != 0.0 && covariance(i, i) > 0.0) {
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
                                          double max_normalized_innovation_squared_per_observation) {
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
        measurement_system.residuals, measurement_system.design_matrix, outlier_threshold);
    result.post_suppression_residual_rms_m = residualRms(measurement_system.residuals);
    result.post_suppression_residual_max_abs_m = residualMaxAbs(measurement_system.residuals);

    const auto innovation_stats = computeInnovationStats(state,
                                                         covariance,
                                                         measurement_system.design_matrix,
                                                         measurement_system.residuals,
                                                         measurement_system.covariance);
    result.innovation_observation_count = innovation_stats.observation_count;
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
                                  measurement_system.covariance);
    result.ok = (info == 0);
    return result;
}

}  // namespace rtk_update
}  // namespace libgnss
