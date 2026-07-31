#pragma once

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace rtk_measurement {

struct StateCoefficient {
    int state_index = -1;
    double coefficient = 0.0;
};

enum class MeasurementKind {
    UNKNOWN = 0,
    PHASE = 1,
    CODE = 2,
    // navi.776 B: rover-only between-satellite single-difference Doppler
    // rows (range-rate domain, m/s). Receiver clock drift cancels in the
    // between-satellite difference; rows touch only the velocity tail
    // states, never ambiguities or AR.
    DOPPLER = 3
};

struct MeasurementRow {
    double residual = 0.0;
    Eigen::Vector3d baseline_coefficients = Eigen::Vector3d::Zero();
    std::vector<StateCoefficient> state_coefficients;
    double reference_variance = 0.0;
    double satellite_variance = 0.0;
    // Bookkeeping for the innovation-based adaptive noise tracker. -1 means
    // this row does not participate in adaptation. adaptive_model_variance
    // carries the unadapted model variance (varerr * inflation factors) so
    // the tracker can clamp relative to the current geometry.
    int adaptive_key = -1;
    double adaptive_model_variance = 0.0;
};

struct MeasurementBlock {
    MeasurementKind kind = MeasurementKind::UNKNOWN;
    int frequency_index = -1;
    std::vector<MeasurementRow> rows;
};

struct MeasurementSystem {
    Eigen::MatrixXd design_matrix;
    Eigen::VectorXd residuals;
    Eigen::MatrixXd covariance;
    // navi.776 B: optional per-row outlier thresholds. Empty (default) keeps
    // the single scalar-threshold behavior. When sized to the row count,
    // entries > 0 override the scalar threshold for that row — Doppler rows
    // live in the m/s domain and must not be judged by the metre-domain
    // threshold.
    std::vector<double> row_outlier_thresholds;
    std::vector<MeasurementKind> row_kinds;
};

struct MeasurementDiagnostics {
    int observation_count = 0;
    int phase_observation_count = 0;
    int code_observation_count = 0;
    int doppler_observation_count = 0;
    double residual_rms_m = 0.0;
    double residual_max_abs_m = 0.0;
};

struct AmbiguityDifference {
    int reference_state_index = -1;
    int satellite_state_index = -1;
};

struct AmbiguityTransform {
    Eigen::VectorXd head_state;
    Eigen::VectorXd dd_float;
    Eigen::MatrixXd ambiguity_covariance;
    Eigen::MatrixXd head_ambiguity_covariance;
};

Eigen::MatrixXd buildDoubleDifferenceCovariance(const std::vector<int>& block_sizes,
                                                const std::vector<double>& reference_variances,
                                                const std::vector<double>& satellite_variances,
                                                int observation_count);

MeasurementSystem assembleMeasurementSystem(const std::vector<MeasurementBlock>& blocks,
                                            int n_states);

MeasurementDiagnostics summarizeMeasurementBlocks(const std::vector<MeasurementBlock>& blocks);

AmbiguityTransform buildAmbiguityTransform(const Eigen::VectorXd& state,
                                          const Eigen::MatrixXd& covariance,
                                          int head_state_count,
                                          const std::vector<AmbiguityDifference>& differences);

int suppressOutlierRows(Eigen::VectorXd& residuals,
                        Eigen::MatrixXd& design_matrix,
                        double threshold,
                        const std::vector<double>& per_row_thresholds = {});

}  // namespace rtk_measurement
}  // namespace libgnss
