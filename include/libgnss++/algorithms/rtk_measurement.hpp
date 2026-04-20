#pragma once

#include <Eigen/Dense>

#include <vector>

namespace libgnss {
namespace rtk_measurement {

struct StateCoefficient {
    int state_index = -1;
    double coefficient = 0.0;
};

struct MeasurementRow {
    double residual = 0.0;
    Eigen::Vector3d baseline_coefficients = Eigen::Vector3d::Zero();
    std::vector<StateCoefficient> state_coefficients;
    double reference_variance = 0.0;
    double satellite_variance = 0.0;
};

struct MeasurementBlock {
    std::vector<MeasurementRow> rows;
};

struct MeasurementSystem {
    Eigen::MatrixXd design_matrix;
    Eigen::VectorXd residuals;
    Eigen::MatrixXd covariance;
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

AmbiguityTransform buildAmbiguityTransform(const Eigen::VectorXd& state,
                                          const Eigen::MatrixXd& covariance,
                                          int head_state_count,
                                          const std::vector<AmbiguityDifference>& differences);

int suppressOutlierRows(Eigen::VectorXd& residuals,
                        Eigen::MatrixXd& design_matrix,
                        double threshold);

}  // namespace rtk_measurement
}  // namespace libgnss
