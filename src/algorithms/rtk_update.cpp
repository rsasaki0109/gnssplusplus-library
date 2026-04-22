#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss {
namespace rtk_update {

namespace {

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

}  // namespace

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count) {
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
