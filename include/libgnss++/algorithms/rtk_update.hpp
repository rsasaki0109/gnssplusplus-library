#pragma once

#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <Eigen/Dense>

namespace libgnss {
namespace rtk_update {

struct FilterUpdateResult {
    bool ok = false;
    int observation_count = 0;
    int suppressed_outliers = 0;
};

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count);

}  // namespace rtk_update
}  // namespace libgnss
