#include <libgnss++/algorithms/rtk_carrier_prior.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::rtk_carrier_prior {

namespace {

struct LeastSquares {
    bool ok = false;
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
    Eigen::VectorXd residual;
};

LeastSquares solve(const std::vector<TdcpRow>& rows) {
    LeastSquares out;
    const int n = static_cast<int>(rows.size());
    Eigen::MatrixXd a(n, 4);
    Eigen::VectorXd y(n);
    Eigen::VectorXd w(n);
    for (int i = 0; i < n; ++i) {
        if (!rows[i].los.allFinite() || !std::isfinite(rows[i].delta_m) ||
            !(rows[i].sigma_m > 0.0) || !std::isfinite(rows[i].sigma_m)) {
            return out;
        }
        a.block<1, 3>(i, 0) = -rows[i].los.transpose();
        a(i, 3) = 1.0;
        y(i) = rows[i].delta_m;
        w(i) = 1.0 / (rows[i].sigma_m * rows[i].sigma_m);
    }
    const Eigen::Matrix4d normal = a.transpose() * w.asDiagonal() * a;
    Eigen::LDLT<Eigen::Matrix4d> ldlt(normal);
    if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return out;
    const Eigen::Vector4d rhs = a.transpose() * w.asDiagonal() * y;
    out.x = ldlt.solve(rhs);
    out.covariance = ldlt.solve(Eigen::Matrix4d::Identity());
    if (!out.x.allFinite() || !out.covariance.allFinite()) return out;
    out.residual = y - a * out.x;
    out.ok = true;
    return out;
}

}  // namespace

TdcpSolution solveTdcpDisplacement(const std::vector<TdcpRow>& input_rows,
                                   const TdcpConfig& config) {
    TdcpSolution solution;
    std::vector<TdcpRow> rows = input_rows;
    const int min_rows = std::max(config.min_rows, 5);
    int rejected = 0;
    while (static_cast<int>(rows.size()) >= min_rows) {
        const LeastSquares ls = solve(rows);
        if (!ls.ok) return solution;
        const int n = static_cast<int>(rows.size());
        const int dof = n - 4;
        double sum_sq = 0.0;
        double chi2 = 0.0;
        double worst_normalized = 0.0;
        int worst = 0;
        for (int i = 0; i < n; ++i) {
            sum_sq += ls.residual(i) * ls.residual(i);
            const double normalized = std::abs(ls.residual(i)) / rows[i].sigma_m;
            chi2 += normalized * normalized;
            if (normalized > worst_normalized) {
                worst_normalized = normalized;
                worst = i;
            }
        }
        const double rms = std::sqrt(sum_sq / dof);
        if (rms <= config.max_rms_m && worst_normalized <= config.max_normalized_residual) {
            Eigen::Matrix3d covariance = ls.covariance.topLeftCorner<3, 3>();
            covariance *= std::max(1.0, chi2 / dof);
            solution.ok = covariance.allFinite();
            solution.displacement_ecef = ls.x.head<3>();
            solution.covariance = 0.5 * (covariance + covariance.transpose());
            solution.clock_term_m = ls.x(3);
            solution.rms_m = rms;
            solution.rows_used = n;
            solution.rows_rejected = rejected;
            return solution;
        }
        if (n <= min_rows) return solution;
        rows.erase(rows.begin() + worst);
        ++rejected;
    }
    return solution;
}

CodeConsistency evaluateCodeConsistency(const std::vector<double>& residuals_m,
                                        const std::vector<double>& elevations_rad,
                                        double threshold_m,
                                        double min_elevation_rad) {
    CodeConsistency out;
    if (elevations_rad.size() != residuals_m.size()) return out;
    std::vector<double> values;
    values.reserve(residuals_m.size());
    for (const double r : residuals_m) {
        if (std::isfinite(r)) values.push_back(r);
    }
    out.rows = static_cast<int>(values.size());
    if (values.empty()) return out;
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    const std::size_t mid = sorted.size() / 2;
    out.median_m = (sorted.size() % 2 == 1) ? sorted[mid] : 0.5 * (sorted[mid - 1] + sorted[mid]);
    for (std::size_t i = 0; i < residuals_m.size(); ++i) {
        if (!std::isfinite(residuals_m[i]) || !(elevations_rad[i] >= min_elevation_rad)) continue;
        if (std::abs(residuals_m[i] - out.median_m) > threshold_m) ++out.inconsistent_rows;
    }
    return out;
}

void Monitor::reset() {
    engaged_ = false;
    engaged_since_s_ = 0.0;
    last_bad_s_ = 0.0;
    has_healthy_since_ = false;
    healthy_since_s_ = 0.0;
    has_last_call_ = false;
    last_call_s_ = 0.0;
}

void Monitor::reportContradiction() {
    engaged_ = false;
    has_healthy_since_ = false;
}

bool Monitor::update(const MonitorInput& input) {
    const bool inconsistent = input.inconsistent_rows >= config_.min_inconsistent_rows;
    // Healthy streak: the previous epoch's update was consistent and no epoch
    // was skipped for longer than the gap limit.
    const bool gap =
        has_last_call_ && input.time_s - last_call_s_ > config_.max_streak_gap_s;
    if (gap) has_healthy_since_ = false;
    if (input.previous_epoch_healthy) {
        if (!has_healthy_since_) {
            has_healthy_since_ = true;
            // The previous epoch is only part of the streak when it is not
            // separated from this one by a gap.
            healthy_since_s_ = (has_last_call_ && !gap) ? last_call_s_ : input.time_s;
        }
    } else {
        has_healthy_since_ = false;
    }
    has_last_call_ = true;
    last_call_s_ = input.time_s;

    if (engaged_) {
        if (!input.prior_available ||
            input.time_s - engaged_since_s_ > config_.max_engaged_s) {
            // The chain cannot be advanced (or is out of its error budget):
            // fall back to the legacy re-seed and restart the health streak.
            engaged_ = false;
            has_healthy_since_ = false;
            return false;
        }
        if (inconsistent || !input.previous_epoch_healthy) last_bad_s_ = input.time_s;
        if (input.time_s - last_bad_s_ >= config_.release_hold_s) {
            engaged_ = false;
            return false;
        }
        return true;
    }
    if (input.prior_available && !input.prior_bridged && inconsistent &&
        input.previous_epoch_healthy &&
        healthyStreak(input.time_s) >= config_.min_healthy_streak_s) {
        engaged_ = true;
        engaged_since_s_ = input.time_s;
        last_bad_s_ = input.time_s;
        return true;
    }
    return false;
}

}  // namespace libgnss::rtk_carrier_prior
