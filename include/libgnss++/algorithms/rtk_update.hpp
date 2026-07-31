#pragma once

#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace rtk_update {

enum class HeavyTailWeightModel {
    STUDENT_T,
    LAPLACIAN,
    HUBER,
};

struct StudentTFrontEndConfig {
    bool enabled = false;
    bool code_only = true;
    HeavyTailWeightModel weight_model =
        HeavyTailWeightModel::STUDENT_T;
    double degrees_of_freedom = 4.0;
    double minimum_weight = 0.05;
    double activation_threshold_sigma = 2.5;
};

struct StudentTFrontEndResult {
    bool applied = false;
    int downweighted_rows = 0;
    double minimum_weight = 1.0;
    double mean_weight = 1.0;
};

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
    // Populated only when compute_row_stats is requested: per-row prefit
    // innovations (post outlier suppression, before the state update) and the
    // matching diagonal of H * P- * H' over the active states. Rows zeroed by
    // suppressOutlierRows carry 0 in both vectors. Consumed by the
    // innovation-based adaptive measurement noise tracker.
    Eigen::VectorXd row_innovations;
    Eigen::VectorXd row_hph_diagonal;
    StudentTFrontEndResult student_t;
};

StudentTFrontEndResult applyStudentTMeasurementWeights(
    const Eigen::VectorXd& residuals,
    Eigen::MatrixXd& measurement_covariance,
    const StudentTFrontEndConfig& config);
StudentTFrontEndResult applyStudentTMeasurementWeights(
    const Eigen::VectorXd& residuals,
    const Eigen::VectorXd& innovation_variances,
    Eigen::MatrixXd& measurement_covariance,
    const StudentTFrontEndConfig& config);

FilterUpdateResult applyMeasurementUpdate(Eigen::VectorXd& state,
                                          Eigen::MatrixXd& covariance,
                                          rtk_measurement::MeasurementSystem& measurement_system,
                                          double outlier_threshold,
                                          int min_observation_count,
                                          double max_normalized_innovation_squared_per_observation = 0.0,
                                          const std::vector<bool>& force_active = {},
                                          bool compute_row_stats = false,
                                          bool reuse_kalman_factorization_for_nis = false,
                                          const StudentTFrontEndConfig& student_t_config = {});

}  // namespace rtk_update
}  // namespace libgnss
