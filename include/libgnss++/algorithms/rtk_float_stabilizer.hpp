#pragma once

#include <Eigen/Dense>

#include <cmath>
#include <deque>
#include <optional>
#include <vector>

namespace libgnss::rtk_float_stabilizer {

struct FixedAnchor {
    double time_s = 0.0;
    Eigen::Vector3d position_ecef = Eigen::Vector3d::Zero();
};

struct Config {
    double fit_window_s = 20.0;
    double max_anchor_age_s = 15.0;
    double min_position_covariance_trace_m2 = 10.0;
    double min_float_disagreement_m = 2.0;
    double max_fit_residual_rms_m = 2.0;
    // 1 = constant-velocity linear fit (default, unchanged); 2 = constant
    // acceleration quadratic fit, which survives turns where the linear
    // residual exceeds the safeguard.
    int fit_degree = 1;
};

inline bool shouldArm(double baseline_m, double max_baseline_m) {
    return !std::isfinite(max_baseline_m) ||
           max_baseline_m <= 0.0 ||
           (std::isfinite(baseline_m) &&
            baseline_m <= max_baseline_m);
}

/**
 * Polynomial (constant-acceleration for fit_degree == 2) fixed-anchor
 * prediction.  Same truth-free safeguards as the linear path: fit window,
 * anchor age, fit-residual RMS and minimum disagreement with the raw
 * non-fixed position.
 */
inline std::optional<Eigen::Vector3d> predictPolynomial(
    const std::deque<FixedAnchor>& anchors,
    double time_s,
    const Eigen::Vector3d& raw_position_ecef,
    const Config& config) {
    const int degree = config.fit_degree;
    if (anchors.size() < static_cast<std::size_t>(degree + 1) ||
        !std::isfinite(time_s) || !raw_position_ecef.allFinite()) {
        return std::nullopt;
    }
    const auto& latest = anchors.back();
    const double anchor_age_s = time_s - latest.time_s;
    if (!std::isfinite(anchor_age_s) || anchor_age_s < 0.0 ||
        anchor_age_s > config.max_anchor_age_s) {
        return std::nullopt;
    }
    std::vector<double> u;
    std::vector<Eigen::Vector3d> positions;
    for (const auto& anchor : anchors) {
        const double age_from_latest_s = latest.time_s - anchor.time_s;
        if (age_from_latest_s < 0.0 ||
            age_from_latest_s > config.fit_window_s ||
            !anchor.position_ecef.allFinite()) {
            continue;
        }
        u.push_back(-age_from_latest_s);
        positions.push_back(anchor.position_ecef);
    }
    const int count = static_cast<int>(u.size());
    if (count < degree + 1) return std::nullopt;

    Eigen::MatrixXd design(count, degree + 1);
    for (int row = 0; row < count; ++row) {
        for (int col = 0; col <= degree; ++col) {
            design(row, col) =
                std::pow(u[static_cast<std::size_t>(row)], col);
        }
    }
    const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    Eigen::MatrixXd coefficients(degree + 1, 3);
    for (int dim = 0; dim < 3; ++dim) {
        Eigen::VectorXd target(count);
        for (int row = 0; row < count; ++row) {
            target(row) = positions[static_cast<std::size_t>(row)](dim);
        }
        coefficients.col(dim) = qr.solve(target);
    }
    double fit_residual_sum_squares_m2 = 0.0;
    for (int row = 0; row < count; ++row) {
        Eigen::Vector3d fitted = Eigen::Vector3d::Zero();
        for (int col = 0; col <= degree; ++col) {
            fitted += coefficients.row(col).transpose() *
                      std::pow(u[static_cast<std::size_t>(row)], col);
        }
        fit_residual_sum_squares_m2 +=
            (positions[static_cast<std::size_t>(row)] - fitted).squaredNorm();
    }
    const double fit_residual_rms_m = std::sqrt(
        fit_residual_sum_squares_m2 / static_cast<double>(count));
    if (!std::isfinite(fit_residual_rms_m) ||
        fit_residual_rms_m > config.max_fit_residual_rms_m) {
        return std::nullopt;
    }
    Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
    for (int col = 0; col <= degree; ++col) {
        predicted +=
            coefficients.row(col).transpose() * std::pow(anchor_age_s, col);
    }
    if (!predicted.allFinite() ||
        (predicted - raw_position_ecef).norm() <
            config.min_float_disagreement_m) {
        return std::nullopt;
    }
    return predicted;
}

inline std::optional<Eigen::Vector3d> predict(
    const std::deque<FixedAnchor>& anchors,
    double time_s,
    const Eigen::Vector3d& float_position_ecef,
    double position_covariance_trace_m2,
    const Config& config = {}) {
    if (anchors.size() < 2 ||
        !std::isfinite(time_s) ||
        !float_position_ecef.allFinite() ||
        !std::isfinite(position_covariance_trace_m2) ||
        position_covariance_trace_m2 <= config.min_position_covariance_trace_m2) {
        return std::nullopt;
    }

    if (config.fit_degree >= 2) {
        return predictPolynomial(
            anchors, time_s, float_position_ecef, config);
    }

    const auto& latest = anchors.back();
    const double anchor_age_s = time_s - latest.time_s;
    if (!std::isfinite(anchor_age_s) ||
        anchor_age_s < 0.0 ||
        anchor_age_s > config.max_anchor_age_s) {
        return std::nullopt;
    }

    double mean_dt_s = 0.0;
    Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
    int count = 0;
    for (const auto& anchor : anchors) {
        const double age_from_latest_s = latest.time_s - anchor.time_s;
        if (age_from_latest_s < 0.0 ||
            age_from_latest_s > config.fit_window_s ||
            !anchor.position_ecef.allFinite()) {
            continue;
        }
        mean_dt_s += anchor.time_s - latest.time_s;
        mean_position += anchor.position_ecef;
        ++count;
    }
    if (count < 2) return std::nullopt;

    mean_dt_s /= static_cast<double>(count);
    mean_position /= static_cast<double>(count);
    double time_variance_s2 = 0.0;
    Eigen::Vector3d position_time_covariance =
        Eigen::Vector3d::Zero();
    for (const auto& anchor : anchors) {
        const double age_from_latest_s = latest.time_s - anchor.time_s;
        if (age_from_latest_s < 0.0 ||
            age_from_latest_s > config.fit_window_s ||
            !anchor.position_ecef.allFinite()) {
            continue;
        }
        const double centered_time_s =
            (anchor.time_s - latest.time_s) - mean_dt_s;
        time_variance_s2 += centered_time_s * centered_time_s;
        position_time_covariance +=
            centered_time_s * (anchor.position_ecef - mean_position);
    }
    if (!(time_variance_s2 > 0.0) ||
        !std::isfinite(time_variance_s2)) {
        return std::nullopt;
    }

    const Eigen::Vector3d velocity_ecef =
        position_time_covariance / time_variance_s2;
    double fit_residual_sum_squares_m2 = 0.0;
    for (const auto& anchor : anchors) {
        const double age_from_latest_s = latest.time_s - anchor.time_s;
        if (age_from_latest_s < 0.0 ||
            age_from_latest_s > config.fit_window_s ||
            !anchor.position_ecef.allFinite()) {
            continue;
        }
        const double centered_time_s =
            (anchor.time_s - latest.time_s) - mean_dt_s;
        const Eigen::Vector3d fitted_position =
            mean_position + velocity_ecef * centered_time_s;
        fit_residual_sum_squares_m2 +=
            (anchor.position_ecef - fitted_position).squaredNorm();
    }
    const double fit_residual_rms_m = std::sqrt(
        fit_residual_sum_squares_m2 / static_cast<double>(count));
    if (!std::isfinite(fit_residual_rms_m) ||
        fit_residual_rms_m > config.max_fit_residual_rms_m) {
        return std::nullopt;
    }

    const Eigen::Vector3d predicted_position =
        latest.position_ecef + velocity_ecef * anchor_age_s;
    if (!predicted_position.allFinite() ||
        (predicted_position - float_position_ecef).norm() <
            config.min_float_disagreement_m) {
        return std::nullopt;
    }
    return predicted_position;
}

}  // namespace libgnss::rtk_float_stabilizer
