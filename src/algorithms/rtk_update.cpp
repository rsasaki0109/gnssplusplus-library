#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>

namespace libgnss {
namespace rtk_update {

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count) {
    FilterUpdateResult result;
    result.observation_count = static_cast<int>(measurement_system.residuals.size());
    if (result.observation_count < min_observation_count) {
        return result;
    }

    result.suppressed_outliers = rtk_measurement::suppressOutlierRows(
        measurement_system.residuals, measurement_system.design_matrix, outlier_threshold);

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
