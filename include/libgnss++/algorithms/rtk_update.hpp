#pragma once

#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <Eigen/Dense>

namespace libgnss {
namespace rtk_update {

struct FilterUpdateResult {
    bool ok = false;
    bool rejected_by_innovation_gate = false;
    int observation_count = 0;
    int innovation_observation_count = 0;
    int suppressed_outliers = 0;
    double prefit_residual_rms_m = 0.0;
    double prefit_residual_max_abs_m = 0.0;
    double post_suppression_residual_rms_m = 0.0;
    double post_suppression_residual_max_abs_m = 0.0;
    double normalized_innovation_squared = 0.0;
    double normalized_innovation_squared_per_observation = 0.0;
};

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count,
                                          double max_normalized_innovation_squared_per_observation = 0.0);

}  // namespace rtk_update
}  // namespace libgnss
