#include <libgnss++/algorithms/rtk_measurement.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss {
namespace rtk_measurement {

Eigen::MatrixXd buildDoubleDifferenceCovariance(const std::vector<int>& block_sizes,
                                                const std::vector<double>& reference_variances,
                                                const std::vector<double>& satellite_variances,
                                                int observation_count) {
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(observation_count, observation_count);
    int offset = 0;
    for (size_t block = 0; block < block_sizes.size(); ++block) {
        for (int row = 0; row < block_sizes[block]; ++row) {
            for (int col = 0; col < block_sizes[block]; ++col) {
                covariance(offset + row, offset + col) = reference_variances[offset + row];
                if (row == col) {
                    covariance(offset + row, offset + col) += satellite_variances[offset + row];
                }
            }
        }
        offset += block_sizes[block];
    }
    return covariance;
}

MeasurementSystem assembleMeasurementSystem(const std::vector<MeasurementBlock>& blocks,
                                            int n_states) {
    std::vector<int> block_sizes;
    std::vector<double> ref_variances;
    std::vector<double> sat_variances;
    int observation_count = 0;
    for (const auto& block : blocks) {
        block_sizes.push_back(static_cast<int>(block.rows.size()));
        observation_count += static_cast<int>(block.rows.size());
        for (const auto& row : block.rows) {
            ref_variances.push_back(row.reference_variance);
            sat_variances.push_back(row.satellite_variance);
        }
    }

    MeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(observation_count, n_states);
    system.residuals = Eigen::VectorXd::Zero(observation_count);
    system.covariance = buildDoubleDifferenceCovariance(
        block_sizes, ref_variances, sat_variances, observation_count);

    int row_index = 0;
    for (const auto& block : blocks) {
        for (const auto& row : block.rows) {
            system.residuals(row_index) = row.residual;
            system.design_matrix(row_index, 0) = row.baseline_coefficients(0);
            system.design_matrix(row_index, 1) = row.baseline_coefficients(1);
            system.design_matrix(row_index, 2) = row.baseline_coefficients(2);
            for (const auto& coefficient : row.state_coefficients) {
                if (coefficient.state_index < 0 || coefficient.state_index >= n_states) {
                    continue;
                }
                system.design_matrix(row_index, coefficient.state_index) = coefficient.coefficient;
            }
            ++row_index;
        }
    }

    return system;
}

MeasurementDiagnostics summarizeMeasurementBlocks(const std::vector<MeasurementBlock>& blocks) {
    MeasurementDiagnostics diagnostics;
    double sum_sq = 0.0;
    for (const auto& block : blocks) {
        const int block_observations = static_cast<int>(block.rows.size());
        diagnostics.observation_count += block_observations;
        if (block.kind == MeasurementKind::PHASE) {
            diagnostics.phase_observation_count += block_observations;
        } else if (block.kind == MeasurementKind::CODE) {
            diagnostics.code_observation_count += block_observations;
        }
        for (const auto& row : block.rows) {
            sum_sq += row.residual * row.residual;
            diagnostics.residual_max_abs_m =
                std::max(diagnostics.residual_max_abs_m, std::abs(row.residual));
        }
    }
    if (diagnostics.observation_count > 0) {
        diagnostics.residual_rms_m =
            std::sqrt(sum_sq / static_cast<double>(diagnostics.observation_count));
    }
    return diagnostics;
}

AmbiguityTransform buildAmbiguityTransform(const Eigen::VectorXd& state,
                                          const Eigen::MatrixXd& covariance,
                                          int head_state_count,
                                          const std::vector<AmbiguityDifference>& differences) {
    const int nx = state.size();
    const int nb = static_cast<int>(differences.size());

    AmbiguityTransform transform;
    transform.head_state = state.head(head_state_count);
    transform.dd_float = Eigen::VectorXd::Zero(nb);
    transform.ambiguity_covariance = Eigen::MatrixXd::Zero(nb, nb);
    transform.head_ambiguity_covariance = Eigen::MatrixXd::Zero(head_state_count, nb);

    for (int i = 0; i < nb; ++i) {
        const int ref_i = differences[i].reference_state_index;
        const int sat_i = differences[i].satellite_state_index;
        if (ref_i < 0 || sat_i < 0 || ref_i >= nx || sat_i >= nx) {
            continue;
        }

        transform.dd_float(i) = state(ref_i) - state(sat_i);
        for (int row = 0; row < head_state_count; ++row) {
            transform.head_ambiguity_covariance(row, i) =
                covariance(row, ref_i) - covariance(row, sat_i);
        }

        for (int j = i; j < nb; ++j) {
            const int ref_j = differences[j].reference_state_index;
            const int sat_j = differences[j].satellite_state_index;
            if (ref_j < 0 || sat_j < 0 || ref_j >= nx || sat_j >= nx) {
                continue;
            }
            const double value =
                covariance(ref_i, ref_j) - covariance(ref_i, sat_j) -
                covariance(sat_i, ref_j) + covariance(sat_i, sat_j);
            transform.ambiguity_covariance(i, j) = value;
            transform.ambiguity_covariance(j, i) = value;
        }
    }

    return transform;
}

int suppressOutlierRows(Eigen::VectorXd& residuals,
                        Eigen::MatrixXd& design_matrix,
                        double threshold) {
    int suppressed = 0;
    for (int row = 0; row < residuals.size(); ++row) {
        if (std::abs(residuals(row)) > threshold) {
            residuals(row) = 0.0;
            design_matrix.row(row).setZero();
            ++suppressed;
        }
    }
    return suppressed;
}

}  // namespace rtk_measurement
}  // namespace libgnss
