#pragma once

#include <Eigen/Dense>
#include <libgnss++/core/solution.hpp>
#include <vector>

namespace libgnss::rtk_validation {

struct NonFixedDriftGuardConfig {
    double max_anchor_gap_s = 120.0;
    double max_anchor_speed_mps = 1.0;
    double max_residual_m = 30.0;
    double min_horizontal_residual_m = 0.0;
    int min_segment_epochs = 20;
    // 0 disables the upper bound. Useful for diagnostic profiles that should
    // avoid pruning very long fallback outages.
    int max_segment_epochs = 0;
};

struct NonFixedDriftGuardResult {
    std::vector<PositionSolution> solutions;
    int inspected_segments = 0;
    int rejected_segments = 0;
    int rejected_epochs = 0;
};

struct SppHeightStepGuardConfig {
    double min_step_m = 30.0;
    double max_rate_mps = 4.0;
};

struct SppHeightStepGuardResult {
    std::vector<PositionSolution> solutions;
    int rejected_epochs = 0;
};

struct FloatBridgeTailGuardConfig {
    double max_anchor_gap_s = 120.0;
    // Horizontal FIX-anchor speed bounds in the local ENU frame.
    double min_anchor_speed_mps = 0.4;
    double max_anchor_speed_mps = 1.0;
    double max_residual_m = 12.0;
    int min_segment_epochs = 20;
};

struct FloatBridgeTailGuardResult {
    std::vector<PositionSolution> solutions;
    int inspected_segments = 0;
    int rejected_segments = 0;
    int rejected_epochs = 0;
};

struct FixedBridgeBurstGuardConfig {
    double max_anchor_gap_s = 30.0;
    double min_boundary_gap_s = 1.0;
    double max_residual_m = 20.0;
    int max_segment_epochs = 12;
};

struct FixedBridgeBurstGuardResult {
    std::vector<PositionSolution> solutions;
    int inspected_segments = 0;
    int rejected_segments = 0;
    int rejected_epochs = 0;
};

/** Kinematic output post-filter (height-step drop), one implementation shared by the batch
 * `gnss_solve` pass and the real-time `gnss_live` output path.
 *
 * Causal by construction: the verdict for an epoch depends only on the last *kept* epoch.
 * Once a FIX has been kept, an epoch whose height differs from the anchor by more than
 * max(min_step_m, max_rate_mps * dt) is dropped (dt <= 0 or non-finite -> 1 s). A dropped epoch
 * never advances the anchor, so the anchor cannot be walked off by a run of bad epochs.
 * Usage per valid epoch: `if (!filter.exceedsHeightStep(s)) { emit(s); filter.accept(s); }`. */
struct KinematicPostFilterConfig {
    double min_step_m = 1.0;
    double max_rate_mps = 4.0;
};

class KinematicPostFilter {
public:
    explicit KinematicPostFilter(const KinematicPostFilterConfig& config = KinematicPostFilterConfig{})
        : config_(config) {}

    /** Delta time to the anchor (normalized), only meaningful when hasAnchor(). */
    double anchorDt(const PositionSolution& candidate) const;
    /** True when the filter is armed (a FIX was kept) and `candidate` steps too far in height. */
    bool exceedsHeightStep(const PositionSolution& candidate) const;
    /** Record a kept epoch as the new anchor. */
    void accept(const PositionSolution& kept);
    void reset();

    bool hasAnchor() const { return have_anchor_; }
    bool armed() const { return have_anchor_ && have_kept_fixed_; }
    const PositionSolution& anchor() const { return anchor_; }
    const KinematicPostFilterConfig& config() const { return config_; }

private:
    KinematicPostFilterConfig config_;
    PositionSolution anchor_;
    bool have_anchor_ = false;
    bool have_kept_fixed_ = false;
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

bool fixedAnchorUsable(bool has_fixed_position,
                       bool has_fixed_time,
                       double age_s,
                       double max_age_s);

bool canAttemptHoldFix(int consecutive_fix_count,
                       int min_hold_count,
                       bool has_last_fixed_position,
                       bool has_held_dd_integers);

/// Whether a FIX from the hold-fix fallback re-applies the held integers. Only
/// the Kalman hold restricted to validated AR fixes skips it.
bool holdAppliesAfterHoldFix(bool hold_ambiguity_kalman_update,
                             bool kalman_update_validated_only);

NonFixedDriftGuardResult filterNonFixedStationaryDrift(
    const std::vector<PositionSolution>& solutions,
    const NonFixedDriftGuardConfig& config = NonFixedDriftGuardConfig{});

SppHeightStepGuardResult filterSppHeightSteps(
    const std::vector<PositionSolution>& solutions,
    const SppHeightStepGuardConfig& config = SppHeightStepGuardConfig{});

FloatBridgeTailGuardResult filterFloatBridgeTail(
    const std::vector<PositionSolution>& solutions,
    const FloatBridgeTailGuardConfig& config = FloatBridgeTailGuardConfig{});

FixedBridgeBurstGuardResult filterFixedBridgeBursts(
    const std::vector<PositionSolution>& solutions,
    const FixedBridgeBurstGuardConfig& config = FixedBridgeBurstGuardConfig{});

}  // namespace libgnss::rtk_validation
