#pragma once

#include <Eigen/Dense>

namespace libgnss::rtk_validation {

double normalizedDt(double dt_seconds, double fallback_dt_seconds = 1.0);

double adaptiveJumpLimit(double dt_seconds, double min_jump_m, double rate_m_per_s);

bool exceedsAdaptiveJump(const Eigen::Vector3d& current_position,
                         const Eigen::Vector3d& reference_position,
                         double dt_seconds,
                         double min_jump_m,
                         double rate_m_per_s);

bool exceedsAbsoluteJump(const Eigen::Vector3d& current_position,
                         const Eigen::Vector3d& reference_position,
                         bool has_reference_position,
                         double max_jump_m);

bool exceedsFixHistoryJump(const Eigen::Vector3d& fixed_position,
                           const Eigen::Vector3d& last_fixed_position,
                           bool has_last_fixed_position,
                           bool static_mode,
                           int consecutive_fix_count);

bool canAttemptHoldFix(int consecutive_fix_count,
                       int min_hold_count,
                       bool has_last_fixed_position,
                       bool has_held_dd_integers);

}  // namespace libgnss::rtk_validation
