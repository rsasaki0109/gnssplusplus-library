#pragma once

#include <Eigen/Dense>
#include <libgnss++/core/solution.hpp>
#include <vector>

namespace libgnss::rtk_validation {

struct NonFixedDriftGuardConfig {
    double max_anchor_gap_s = 120.0;
    double max_anchor_speed_mps = 1.0;
    double max_residual_m = 30.0;
    int min_segment_epochs = 20;
};

struct NonFixedDriftGuardResult {
    std::vector<PositionSolution> solutions;
    int inspected_segments = 0;
    int rejected_segments = 0;
    int rejected_epochs = 0;
};

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

NonFixedDriftGuardResult filterNonFixedStationaryDrift(
    const std::vector<PositionSolution>& solutions,
    const NonFixedDriftGuardConfig& config = NonFixedDriftGuardConfig{});

}  // namespace libgnss::rtk_validation
