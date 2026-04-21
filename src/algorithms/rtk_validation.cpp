#include <libgnss++/algorithms/rtk_validation.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

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

NonFixedDriftGuardResult filterNonFixedStationaryDrift(
    const std::vector<PositionSolution>& solutions,
    const NonFixedDriftGuardConfig& config) {
    NonFixedDriftGuardResult result;
    result.solutions.reserve(solutions.size());

    if (solutions.empty()) {
        return result;
    }

    std::vector<bool> keep(solutions.size(), true);
    size_t index = 0;
    while (index < solutions.size()) {
        if (solutions[index].isFixed()) {
            ++index;
            continue;
        }

        const size_t segment_start = index;
        while (index < solutions.size() && !solutions[index].isFixed()) {
            ++index;
        }
        const size_t segment_end = index;

        if (segment_end - segment_start < static_cast<size_t>(std::max(1, config.min_segment_epochs))) {
            continue;
        }
        if (segment_start == 0 || segment_end >= solutions.size()) {
            continue;
        }

        const PositionSolution& anchor_before = solutions[segment_start - 1];
        const PositionSolution& anchor_after = solutions[segment_end];
        if (!anchor_before.isFixed() || !anchor_after.isFixed()) {
            continue;
        }

        const double anchor_gap_s = anchor_after.time - anchor_before.time;
        if (!std::isfinite(anchor_gap_s) || anchor_gap_s <= 0.0 ||
            anchor_gap_s > config.max_anchor_gap_s) {
            continue;
        }

        const Eigen::Vector3d anchor_delta =
            anchor_after.position_ecef - anchor_before.position_ecef;
        const double anchor_speed_mps = anchor_delta.norm() / anchor_gap_s;
        if (!std::isfinite(anchor_speed_mps) ||
            anchor_speed_mps > config.max_anchor_speed_mps) {
            continue;
        }

        result.inspected_segments++;
        bool rejected_segment = false;
        for (size_t solution_index = segment_start; solution_index < segment_end; ++solution_index) {
            const PositionSolution& candidate = solutions[solution_index];
            const double fraction = (candidate.time - anchor_before.time) / anchor_gap_s;
            const Eigen::Vector3d predicted_position =
                anchor_before.position_ecef + anchor_delta * fraction;
            const double residual_m =
                (candidate.position_ecef - predicted_position).norm();
            if (std::isfinite(residual_m) && residual_m > config.max_residual_m) {
                keep[solution_index] = false;
                rejected_segment = true;
                result.rejected_epochs++;
            }
        }
        if (rejected_segment) {
            result.rejected_segments++;
        }
    }

    for (size_t solution_index = 0; solution_index < solutions.size(); ++solution_index) {
        if (keep[solution_index]) {
            result.solutions.push_back(solutions[solution_index]);
        }
    }
    return result;
}

SppHeightStepGuardResult filterSppHeightSteps(
    const std::vector<PositionSolution>& solutions,
    const SppHeightStepGuardConfig& config) {
    SppHeightStepGuardResult result;
    result.solutions.reserve(solutions.size());

    const PositionSolution* last_kept = nullptr;
    for (const PositionSolution& solution : solutions) {
        bool reject = false;
        if (last_kept != nullptr &&
            solution.status == SolutionStatus::SPP &&
            solution.isValid()) {
            const double dt_seconds = solution.time - last_kept->time;
            const double max_height_step_m =
                adaptiveJumpLimit(dt_seconds, config.min_step_m, config.max_rate_mps);
            const double height_step_m = std::abs(
                solution.position_geodetic.height -
                last_kept->position_geodetic.height);
            reject = std::isfinite(height_step_m) &&
                height_step_m > max_height_step_m;
        }

        if (reject) {
            result.rejected_epochs++;
            continue;
        }
        result.solutions.push_back(solution);
        if (solution.isValid()) {
            last_kept = &result.solutions.back();
        }
    }

    return result;
}

FloatBridgeTailGuardResult filterFloatBridgeTail(
    const std::vector<PositionSolution>& solutions,
    const FloatBridgeTailGuardConfig& config) {
    FloatBridgeTailGuardResult result;
    result.solutions.reserve(solutions.size());

    if (solutions.empty()) {
        return result;
    }

    std::vector<bool> keep(solutions.size(), true);
    size_t index = 0;
    while (index < solutions.size()) {
        if (solutions[index].isFixed()) {
            ++index;
            continue;
        }

        const size_t segment_start = index;
        while (index < solutions.size() && !solutions[index].isFixed()) {
            ++index;
        }
        const size_t segment_end = index;

        if (segment_end - segment_start < static_cast<size_t>(std::max(1, config.min_segment_epochs))) {
            continue;
        }
        if (segment_start == 0 || segment_end >= solutions.size()) {
            continue;
        }

        const PositionSolution& anchor_before = solutions[segment_start - 1];
        const PositionSolution& anchor_after = solutions[segment_end];
        if (!anchor_before.isFixed() || !anchor_after.isFixed()) {
            continue;
        }

        const double anchor_gap_s = anchor_after.time - anchor_before.time;
        if (!std::isfinite(anchor_gap_s) || anchor_gap_s <= 0.0 ||
            anchor_gap_s > config.max_anchor_gap_s) {
            continue;
        }

        const Eigen::Vector3d anchor_delta =
            anchor_after.position_ecef - anchor_before.position_ecef;
        const double anchor_speed_mps = anchor_delta.norm() / anchor_gap_s;
        if (!std::isfinite(anchor_speed_mps) ||
            anchor_speed_mps < config.min_anchor_speed_mps ||
            anchor_speed_mps > config.max_anchor_speed_mps) {
            continue;
        }

        result.inspected_segments++;
        bool rejected_segment = false;
        for (size_t solution_index = segment_start; solution_index < segment_end; ++solution_index) {
            const PositionSolution& candidate = solutions[solution_index];
            if (candidate.status != SolutionStatus::FLOAT) {
                continue;
            }

            const double fraction = (candidate.time - anchor_before.time) / anchor_gap_s;
            const Eigen::Vector3d predicted_position =
                anchor_before.position_ecef + anchor_delta * fraction;
            const double residual_m =
                (candidate.position_ecef - predicted_position).norm();
            if (std::isfinite(residual_m) && residual_m > config.max_residual_m) {
                keep[solution_index] = false;
                rejected_segment = true;
                result.rejected_epochs++;
            }
        }
        if (rejected_segment) {
            result.rejected_segments++;
        }
    }

    for (size_t solution_index = 0; solution_index < solutions.size(); ++solution_index) {
        if (keep[solution_index]) {
            result.solutions.push_back(solutions[solution_index]);
        }
    }
    return result;
}

}  // namespace libgnss::rtk_validation
