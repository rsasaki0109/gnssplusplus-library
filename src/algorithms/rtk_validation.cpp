#include <libgnss++/algorithms/rtk_validation.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss::rtk_validation {

double normalizedDt(double dt_seconds, double fallback_dt_seconds) {
    if (!std::isfinite(dt_seconds) || dt_seconds < 0.5) {
        return fallback_dt_seconds;
    }
    return dt_seconds;
}

double adaptiveJumpLimit(double dt_seconds, double min_jump_m, double rate_m_per_s) {
    return std::max(min_jump_m, rate_m_per_s * normalizedDt(dt_seconds));
}

bool exceedsAdaptiveJump(const Eigen::Vector3d& current_position,
                         const Eigen::Vector3d& reference_position,
                         double dt_seconds,
                         double min_jump_m,
                         double rate_m_per_s) {
    const double jump = (current_position - reference_position).norm();
    return jump > adaptiveJumpLimit(dt_seconds, min_jump_m, rate_m_per_s);
}

bool exceedsAbsoluteJump(const Eigen::Vector3d& current_position,
                         const Eigen::Vector3d& reference_position,
                         bool has_reference_position,
                         double max_jump_m) {
    if (!has_reference_position) {
        return false;
    }
    return (current_position - reference_position).norm() > max_jump_m;
}

bool exceedsFixHistoryJump(const Eigen::Vector3d& fixed_position,
                           const Eigen::Vector3d& last_fixed_position,
                           bool has_last_fixed_position,
                           bool static_mode,
                           int consecutive_fix_count) {
    if (!has_last_fixed_position) {
        return false;
    }
    const double max_jump_m = static_mode ? 0.1 : 0.2;
    if ((fixed_position - last_fixed_position).norm() <= max_jump_m) {
        return false;
    }
    return consecutive_fix_count >= 3;
}

bool canAttemptHoldFix(int consecutive_fix_count,
                       int min_hold_count,
                       bool has_last_fixed_position,
                       bool has_held_dd_integers) {
    return consecutive_fix_count >= min_hold_count &&
           has_last_fixed_position &&
           has_held_dd_integers;
}

}  // namespace libgnss::rtk_validation
