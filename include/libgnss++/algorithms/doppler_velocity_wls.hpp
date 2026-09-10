#pragma once

#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/core/types.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::doppler_velocity_wls {

/**
 * @brief One corrected undifferenced-Doppler equation.
 *
 * The row is intentionally the same receiver-side contract used by the
 * corrected FGO factor:
 *
 *   residual_mps = los.dot(v_receiver_ecef) + receiver_clock_rate_mps.
 *
 * `los` is therefore the factor LOS (negative receiver-to-satellite unit
 * vector), not the geometric receiver-to-satellite vector.  Keeping this
 * representation independent of FGO's factor type lets the initializer and
 * factor share the exact equation without a dependency cycle.
 */
struct ObservationRow {
    Vector3d los = Vector3d::Zero();
    double residual_mps = 0.0;
    double sigma_mps = 0.0;
};

/** Physically predeclared, truth-free gates for the per-epoch solve. */
struct Options {
    int min_rows = 4;
    double max_condition_number = 1.0e8;
    double huber_threshold_sigma = 4.0;
    int max_irls_iterations = 3;
    double max_velocity_mps = 70.0;
    double max_clock_rate_mps = 2000.0;
    double max_normalized_rms = 4.0;
    double max_abs_normalized_residual = 25.0;
};

struct Estimate {
    bool valid = false;
    // `direct` is false only for the bounded temporal completion performed by
    // the FGO problem builder.  The standalone solver always returns direct
    // estimates.
    bool direct = true;
    bool propagated = false;
    bool reset = false;
    Vector3d velocity_ecef_mps = Vector3d::Zero();
    double clock_rate_mps = 0.0;
    Eigen::Matrix<double, 4, 4> covariance =
        Eigen::Matrix<double, 4, 4>::Constant(
            std::numeric_limits<double>::quiet_NaN());
    int rows = 0;
    int inlier_rows = 0;
    int rank = 0;
    int robust_iterations = 0;
    double condition_number = std::numeric_limits<double>::infinity();
    double normalized_rms = std::numeric_limits<double>::infinity();
    double max_abs_normalized_residual =
        std::numeric_limits<double>::infinity();
    std::string reason = "uninitialized";
};

/**
 * @brief Central receiver-side prediction used by both WLS and corrected FGO.
 */
inline double predict(const Vector3d& los,
                      const Vector3d& velocity_ecef_mps,
                      double clock_rate_mps) {
    return doppler_contract::receiverPrediction(
        -los, velocity_ecef_mps, clock_rate_mps);
}

/** Return the four columns of the corrected FGO/WLS row. */
inline Eigen::Matrix<double, 1, 4> designRow(const Vector3d& los) {
    Eigen::Matrix<double, 1, 4> row;
    row << los.x(), los.y(), los.z(), 1.0;
    return row;
}

inline Estimate solve(const std::vector<ObservationRow>& rows,
                      const Options& options = Options()) {
    Estimate result;
    result.rows = static_cast<int>(rows.size());
    auto reject = [&](const char* reason) {
        result.valid = false;
        result.reason = reason;
        return result;
    };

    if (options.min_rows < 4 ||
        !(options.max_condition_number > 1.0) ||
        !(options.huber_threshold_sigma > 0.0) ||
        options.max_irls_iterations < 1 ||
        !(options.max_velocity_mps > 0.0) ||
        !(options.max_clock_rate_mps > 0.0) ||
        !(options.max_normalized_rms > 0.0) ||
        !(options.max_abs_normalized_residual > 0.0)) {
        return reject("invalid-options");
    }
    if (rows.size() < static_cast<std::size_t>(options.min_rows)) {
        return reject("insufficient-rows");
    }

    for (const auto& observation : rows) {
        const double los_norm = observation.los.norm();
        if (!observation.los.allFinite() ||
            !(los_norm > 0.9 && los_norm < 1.1) ||
            !std::isfinite(observation.residual_mps) ||
            !(observation.sigma_mps > 0.0) ||
            !std::isfinite(observation.sigma_mps)) {
            return reject("nonfinite-or-invalid-row");
        }
    }

    const Eigen::Index row_count = static_cast<Eigen::Index>(rows.size());
    Eigen::MatrixXd design(row_count, 4);
    Eigen::VectorXd observations(row_count);
    Eigen::VectorXd sigma(row_count);
    for (Eigen::Index i = 0; i < row_count; ++i) {
        design.row(i) = designRow(rows[static_cast<std::size_t>(i)].los);
        observations(i) = rows[static_cast<std::size_t>(i)].residual_mps;
        sigma(i) = rows[static_cast<std::size_t>(i)].sigma_mps;
    }

    Eigen::VectorXd robust_weights = Eigen::VectorXd::Ones(row_count);
    Eigen::Vector4d state = Eigen::Vector4d::Zero();
    bool solved_once = false;
    for (int iteration = 0; iteration < options.max_irls_iterations;
         ++iteration) {
        Eigen::MatrixXd weighted_design = design;
        Eigen::VectorXd weighted_observations = observations;
        for (Eigen::Index i = 0; i < row_count; ++i) {
            const double weight = std::sqrt(robust_weights(i)) / sigma(i);
            weighted_design.row(i) *= weight;
            weighted_observations(i) *= weight;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            weighted_design, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto singular_values = svd.singularValues();
        if (singular_values.size() != 4 ||
            !singular_values.allFinite()) {
            return reject("nonfinite-svd");
        }
        const double largest = singular_values(0);
        const double smallest = singular_values(3);
        result.rank = static_cast<int>(svd.rank());
        if (!(largest > 0.0) || !(smallest > 0.0) ||
            !std::isfinite(largest) || !std::isfinite(smallest)) {
            return reject("rank-deficient");
        }
        result.condition_number = largest / smallest;
        if (!std::isfinite(result.condition_number) ||
            result.rank < 4 ||
            result.condition_number > options.max_condition_number) {
            return reject("rank-or-condition-gate");
        }

        const Eigen::Vector4d next_state = svd.solve(weighted_observations);
        if (!next_state.allFinite()) {
            return reject("nonfinite-solution");
        }
        const double state_change = (next_state - state).norm();
        solved_once = true;
        state = next_state;
        result.robust_iterations = iteration + 1;

        const Eigen::VectorXd residuals = design * state - observations;
        Eigen::VectorXd next_weights = Eigen::VectorXd::Ones(row_count);
        for (Eigen::Index i = 0; i < row_count; ++i) {
            const double normalized = residuals(i) / sigma(i);
            const double absolute = std::abs(normalized);
            if (!std::isfinite(normalized)) {
                return reject("nonfinite-residual");
            }
            if (absolute > options.huber_threshold_sigma) {
                next_weights(i) = options.huber_threshold_sigma / absolute;
            }
        }

        if (iteration > 0 && state_change < 1e-10) {
            robust_weights = next_weights;
            break;
        }
        robust_weights = next_weights;
    }
    if (!solved_once) {
        return reject("solver-did-not-run");
    }

    // Recompute the final robust solution once with the final weights.  This
    // avoids reporting covariance for a stale pre-Huber linearization while
    // keeping the iteration count and update deterministic.
    {
        Eigen::MatrixXd weighted_design = design;
        Eigen::VectorXd weighted_observations = observations;
        for (Eigen::Index i = 0; i < row_count; ++i) {
            const double weight = std::sqrt(robust_weights(i)) / sigma(i);
            weighted_design.row(i) *= weight;
            weighted_observations(i) *= weight;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            weighted_design, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto singular_values = svd.singularValues();
        if (singular_values.size() != 4 || !singular_values.allFinite() ||
            !(singular_values(3) > 0.0)) {
            return reject("final-rank-gate");
        }
        state = svd.solve(weighted_observations);
        if (!state.allFinite()) {
            return reject("final-nonfinite-solution");
        }
        Eigen::Vector4d inverse_squares;
        for (int i = 0; i < 4; ++i) {
            inverse_squares(i) =
                1.0 / (singular_values(i) * singular_values(i));
        }
        result.covariance = svd.matrixV() * inverse_squares.asDiagonal() *
                            svd.matrixV().transpose();
    }

    const Eigen::VectorXd residuals = design * state - observations;
    double robust_square_sum = 0.0;
    double robust_weight_sum = 0.0;
    result.inlier_rows = 0;
    result.max_abs_normalized_residual = 0.0;
    for (Eigen::Index i = 0; i < row_count; ++i) {
        const double normalized = residuals(i) / sigma(i);
        const double absolute = std::abs(normalized);
        if (!std::isfinite(normalized)) {
            return reject("final-nonfinite-residual");
        }
        result.max_abs_normalized_residual =
            std::max(result.max_abs_normalized_residual, absolute);
        robust_square_sum += robust_weights(i) * normalized * normalized;
        robust_weight_sum += robust_weights(i);
        if (robust_weights(i) >= 0.5) {
            ++result.inlier_rows;
        }
    }
    if (!(robust_weight_sum > 0.0) || !std::isfinite(robust_weight_sum)) {
        return reject("empty-robust-support");
    }
    result.normalized_rms =
        std::sqrt(robust_square_sum / robust_weight_sum);
    if (!std::isfinite(result.normalized_rms) ||
        result.normalized_rms > options.max_normalized_rms ||
        result.max_abs_normalized_residual >
            options.max_abs_normalized_residual) {
        return reject("robust-residual-gate");
    }

    result.velocity_ecef_mps = state.head<3>();
    result.clock_rate_mps = state(3);
    if (!result.velocity_ecef_mps.allFinite() ||
        !std::isfinite(result.clock_rate_mps) ||
        result.velocity_ecef_mps.norm() > options.max_velocity_mps ||
        std::abs(result.clock_rate_mps) > options.max_clock_rate_mps ||
        !result.covariance.allFinite()) {
        return reject("physical-gate");
    }

    result.valid = true;
    result.reason = "direct-wls";
    return result;
}

}  // namespace libgnss::doppler_velocity_wls
