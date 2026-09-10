#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/disjoint_satellite_fix_evidence.hpp>
#include <libgnss++/algorithms/fix_failure_budget.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>
#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>
#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

void RTKProcessor::incrementLockCounts(const std::map<SatelliteId, SatelliteData>& sat_data) {
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1) lock_count_l1_[sat]++;
        if (sd.has_l2) lock_count_l2_[sat]++;
        if (sd.has_l5) lock_count_l5_[sat]++;  // Phase 18 Step 4: only set when enable_l5 populated has_l5
    }
}

void RTKProcessor::handleConsecutiveFloatReset(const ObservationData& rover_obs,
                                               const NavigationData& nav) {
    if (rtk_config_.max_consecutive_float_for_reset <= 0 ||
        consecutive_float_count_ < rtk_config_.max_consecutive_float_for_reset) {
        return;
    }

    // Skip reset under DEMO5_CONTINUOUS policy (simple AR maintains its own state).
    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        consecutive_float_count_ = 0;
        return;
    }

    resetAmbiguityStatesForReacquisition(rover_obs, nav, false);
}

void RTKProcessor::resetAmbiguityStatesForReacquisition(const ObservationData& rover_obs,
                                                        const NavigationData& nav,
                                                        bool clear_hold_state) {
    // navi.776 A2: a full reacquisition invalidates all learned variances.
    adaptive_noise_tracker_.clear();
    if (!filter_initialized_) {
        consecutive_float_count_ = 0;
        consecutive_nonfix_count_ = 0;
        consecutive_high_float_residual_count_ = 0;
        consecutive_high_fixed_residual_count_ = 0;
        adaptive_dynamic_slip_hold_count_ = 0;
        return;
    }

    for (auto& [sat, idx] : filter_state_.n1_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    for (auto& [sat, idx] : filter_state_.n2_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    // Phase 18 Step 2: reset N5 ambiguities (no-op until n5_indices populated).
    for (auto& [sat, idx] : filter_state_.n5_indices) {
        filter_state_.state(idx) = 0.0;
        filter_state_.covariance(idx, idx) = 900.0;
    }
    // Also reset position to SPP to prevent baseline drift.
    resetPositionToSPP(rover_obs, nav);
    if (clear_hold_state) {
        restoreHoldState(HoldStateSnapshot{});
    }
    // Ordinary FLOAT resets retain held DD integers for hold fix. A
    // wrong-basin FIX reset explicitly clears them above.
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
}

bool RTKProcessor::shouldResetAfterFixedResidualGate() {
    const double max_prefit_rms = rtk_config_.max_fixed_prefit_residual_rms_m;
    const int min_outliers = rtk_config_.min_fixed_prefit_outliers;
    const bool enabled = std::isfinite(max_prefit_rms) && max_prefit_rms > 0.0 &&
                         min_outliers > 0;
    const bool exceeded =
        enabled && std::isfinite(current_update_diagnostics_.prefit_residual_rms_m) &&
        current_update_diagnostics_.prefit_residual_rms_m > max_prefit_rms &&
        current_update_diagnostics_.suppressed_outliers >= min_outliers;
    const double max_covariance_trace =
        rtk_config_.max_fixed_overconfidence_covariance_trace_m2;
    const bool covariance_passes =
        !(std::isfinite(max_covariance_trace) && max_covariance_trace > 0.0) ||
        (std::isfinite(debug_telemetry_.float_position_covariance_trace_m2) &&
         debug_telemetry_.float_position_covariance_trace_m2 <= max_covariance_trace);
    if (!exceeded || !covariance_passes) {
        consecutive_high_fixed_residual_count_ = 0;
        return false;
    }
    ++consecutive_high_fixed_residual_count_;
    if (consecutive_high_fixed_residual_count_ <
        std::max(1, rtk_config_.fixed_prefit_reset_streak)) {
        return false;
    }
    if (!rtk_config_.fixed_prefit_quarantine_only) {
        consecutive_high_fixed_residual_count_ = 0;
    }
    return true;
}

bool RTKProcessor::floatResidualExceedsReacquisitionGate() const {
    if (rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return false;
    }
    const double max_prefit_rms = rtk_config_.max_float_prefit_residual_rms_m;
    if (std::isfinite(max_prefit_rms) && max_prefit_rms > 0.0 &&
        std::isfinite(current_update_diagnostics_.prefit_residual_rms_m) &&
        current_update_diagnostics_.prefit_residual_rms_m > max_prefit_rms) {
        return true;
    }
    const double max_prefit_residual = rtk_config_.max_float_prefit_residual_max_m;
    if (std::isfinite(max_prefit_residual) && max_prefit_residual > 0.0 &&
        std::isfinite(current_update_diagnostics_.prefit_residual_max_m) &&
        current_update_diagnostics_.prefit_residual_max_m > max_prefit_residual) {
        return true;
    }
    return false;
}

bool RTKProcessor::floatResidualTrustedJumpPassesGate(
    const PositionSolution& float_solution,
    const Vector3d& saved_last_trusted_position,
    bool saved_has_last_trusted,
    const GNSSTime& saved_last_trusted_time,
    bool saved_has_last_trusted_time) const {
    const double min_trusted_jump =
        rtk_config_.min_float_prefit_residual_trusted_jump_m;
    if (!std::isfinite(min_trusted_jump) || min_trusted_jump <= 0.0) {
        return true;
    }
    if (!saved_has_last_trusted || !saved_has_last_trusted_time ||
        !float_solution.position_ecef.allFinite()) {
        return false;
    }
    const double dt = float_solution.time - saved_last_trusted_time;
    if (!std::isfinite(dt) || dt < 0.0) {
        return false;
    }
    const double trusted_jump =
        (float_solution.position_ecef - saved_last_trusted_position).norm();
    return std::isfinite(trusted_jump) && trusted_jump >= min_trusted_jump;
}

bool RTKProcessor::shouldResetAfterFloatResidualGate(
    const PositionSolution& float_solution,
    const Vector3d& saved_last_trusted_position,
    bool saved_has_last_trusted,
    const GNSSTime& saved_last_trusted_time,
    bool saved_has_last_trusted_time) {
    if (!floatResidualExceedsReacquisitionGate()) {
        consecutive_high_float_residual_count_ = 0;
        return false;
    }
    if (!floatResidualTrustedJumpPassesGate(float_solution,
                                            saved_last_trusted_position,
                                            saved_has_last_trusted,
                                            saved_last_trusted_time,
                                            saved_has_last_trusted_time)) {
        consecutive_high_float_residual_count_ = 0;
        return false;
    }
    consecutive_high_float_residual_count_++;
    const int reset_streak =
        std::max(1, rtk_config_.max_float_prefit_residual_reset_streak);
    if (consecutive_high_float_residual_count_ < reset_streak) {
        return false;
    }
    consecutive_high_float_residual_count_ = 0;
    return true;
}

void RTKProcessor::recordFixedEpoch(const PositionSolution& solution) {
    consecutive_float_count_ = 0;
    consecutive_nonfix_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    if (!rtk_config_.enable_fixed_anchor_float_stabilization ||
        !solution.position_ecef.allFinite()) {
        return;
    }
    const double max_baseline_m =
        rtk_config_.doppler_row_max_baseline_m;
    if (!fixed_anchor_float_stabilizer_armed_) {
        const double baseline_m =
            (solution.position_ecef - base_position_).norm();
        fixed_anchor_float_stabilizer_armed_ =
            rtk_float_stabilizer::shouldArm(
                baseline_m, max_baseline_m);
        if (!fixed_anchor_float_stabilizer_armed_) return;
    }
    const double time_s =
        static_cast<double>(solution.time.week) * 604800.0 +
        solution.time.tow;
    fixed_anchor_float_history_.push_back(
        {time_s, solution.position_ecef});
    while (fixed_anchor_float_history_.size() > 2 &&
           time_s - fixed_anchor_float_history_.front().time_s > 20.0) {
        fixed_anchor_float_history_.pop_front();
    }
}

void RTKProcessor::stabilizeFloatOutput(PositionSolution& solution) const {
    if (!rtk_config_.enable_fixed_anchor_float_stabilization ||
        solution.status != SolutionStatus::FLOAT) {
        return;
    }
    const double time_s =
        static_cast<double>(solution.time.week) * 604800.0 +
        solution.time.tow;
    const auto prediction = rtk_float_stabilizer::predict(
        fixed_anchor_float_history_,
        time_s,
        solution.position_ecef,
        debug_telemetry_.float_position_covariance_trace_m2);
    if (!prediction.has_value()) return;

    solution.position_ecef = *prediction;
    solution.baseline_length =
        (solution.position_ecef - base_position_).norm();
    ecef2geodetic(
        solution.position_ecef,
        solution.position_geodetic.latitude,
        solution.position_geodetic.longitude,
        solution.position_geodetic.height);
}

void RTKProcessor::recordFloatEpoch(
    const ObservationData& rover_obs,
    const NavigationData& nav) {
    consecutive_high_fixed_residual_count_ = 0;
    consecutive_float_count_++;
    consecutive_nonfix_count_++;
    if (rtk_config_.max_consecutive_nonfix_for_reset <= 0 ||
        consecutive_nonfix_count_ < rtk_config_.max_consecutive_nonfix_for_reset ||
        rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return;
    }
    resetAmbiguityStatesForReacquisition(rover_obs, nav, false);
}

void RTKProcessor::recordFallbackEpoch(
    const ObservationData& rover_obs,
    const NavigationData& nav) {
    consecutive_fix_count_ = 0;
    consecutive_float_count_ = 0;
    consecutive_high_float_residual_count_ = 0;
    consecutive_high_fixed_residual_count_ = 0;
    if (!filter_initialized_) {
        consecutive_nonfix_count_ = 0;
        return;
    }
    consecutive_nonfix_count_++;
    if (rtk_config_.max_consecutive_nonfix_for_reset <= 0 ||
        consecutive_nonfix_count_ < rtk_config_.max_consecutive_nonfix_for_reset ||
        rtk_config_.ar_policy == RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        return;
    }
    resetAmbiguityStatesForReacquisition(rover_obs, nav, false);
}

void RTKProcessor::resetPositionToSPP(
    const ObservationData& rover_obs,
    const NavigationData& nav) {
    // M1 time updates are strictly single-use. Consume before any mode return
    // so STATIC/MOVING_BASE or a caller mode change cannot retain a stale IMU
    // increment for a later kinematic epoch.
    const bool has_time_update_this_epoch = has_external_position_time_update_;
    const Vector3d position_delta_ecef = external_position_delta_ecef_;
    const Matrix3d position_process_noise_ecef = external_position_process_noise_ecef_;
    const bool has_velocity_update_this_epoch = has_external_velocity_time_update_;
    const Vector3d velocity_ecef = external_velocity_ecef_;
    const Eigen::Matrix<double, 6, 6> position_velocity_process_noise_ecef =
        external_position_velocity_process_noise_ecef_;
    const Matrix3d velocity_initial_covariance_ecef =
        external_velocity_initial_covariance_ecef_;
    has_external_position_time_update_ = false;
    has_external_velocity_time_update_ = false;
    ins_time_update_applied_last_epoch_ = false;

    if (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) {
        // Static: position accumulates with process noise
        double pos_pnoise = rtk_config_.process_noise_position;  // default 1e-4 m^2/s
        for (int i = 0; i < BASE_STATES; ++i)
            filter_state_.covariance(i, i) += pos_pnoise;
        return;
    }

    const bool moving_base_mode = isMovingBasePositionMode(rtk_config_);

    if (rtk_config_.use_external_position_time_update &&
        has_time_update_this_epoch && !moving_base_mode) {
        const bool applied = rtk_config_.enable_velocity_states &&
                has_velocity_update_this_epoch
            ? rtk_ins_time_update::applyPositionVelocity(
                  filter_state_.state, filter_state_.covariance,
                  position_delta_ecef, velocity_ecef,
                  position_velocity_process_noise_ecef,
                  velocity_initial_covariance_ecef,
                  rtk_config_.ins_time_update_position_q_floor_m2,
                  VELOCITY_STATE_INDEX)
            : rtk_ins_time_update::apply(
                  filter_state_.state, filter_state_.covariance,
                  position_delta_ecef, position_process_noise_ecef,
                  rtk_config_.ins_time_update_position_q_floor_m2);
        if (applied) {
            ++ins_time_update_applied_count_;
            ins_time_update_applied_last_epoch_ = true;
            // Both external mechanisms target the same epoch. Never let a
            // simultaneously queued absolute prior survive this successful
            // update and accidentally seed a later epoch.
            has_external_position_prior_ = false;
            return;
        }
        ++ins_time_update_rejected_count_;
    }

    // Phase 1 GNSS/IMU coupling (docs/design.md): opt-in external position
    // prior (e.g. INS-mechanization-predicted antenna position) in place of
    // the legacy SPP/trusted-position reseed below. The prior is always
    // consumed (has_external_position_prior_ cleared) here, whether or not
    // it is actually applied, so a caller that stops supplying one (e.g. an
    // IMU gap) transparently falls back to the legacy reseed on the very
    // next epoch rather than accidentally reusing a stale value. Guarded on
    // !moving_base_mode: MOVING_BASE tracks a relative baseline against a
    // time-varying base, which an absolute INS-predicted antenna position
    // cannot seed meaningfully.
    const bool has_prior_this_epoch = has_external_position_prior_;
    const Vector3d prior_ecef = external_position_prior_ecef_;
    const Matrix3d prior_cov = external_position_prior_covariance_;
    has_external_position_prior_ = false;
    if (rtk_config_.use_external_position_prior && has_prior_this_epoch && !moving_base_mode) {
        const Vector3d baseline = prior_ecef - base_position_;
        filter_state_.state.head<3>() = baseline;
        const int n = filter_state_.state.size();
        for (int i = 0; i < BASE_STATES; ++i) {
            for (int j = 0; j < n; ++j) {
                filter_state_.covariance(i, j) = 0.0;
                filter_state_.covariance(j, i) = 0.0;
            }
        }
        filter_state_.covariance.block<BASE_STATES, BASE_STATES>(0, 0) = prior_cov;
        return;
    }

    // Dynamic modes: refresh the baseline seed each epoch. Moving-base keeps the
    // relative baseline and only uses absolute rover hints when they exist.
    Vector3d rover_pos;
    double var_pos = moving_base_mode ? 25.0 : 900.0;
    const auto spp_started = std::chrono::steady_clock::now();
    const auto spp = spp_processor_.processEpoch(rover_obs, nav);
    debug_telemetry_.stage_spp_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - spp_started)
            .count();
    if (moving_base_mode) {
        if (rover_obs.receiver_position.norm() > 1e6) {
            rover_pos = rover_obs.receiver_position;
        } else if (has_fixed_solution_) {
            rover_pos = base_position_ + fixed_baseline_;
        } else if (filter_initialized_ && filter_state_.state.size() >= 3) {
            rover_pos = base_position_ + filter_state_.state.head<3>();
        } else if (spp.isValid()) {
            rover_pos = spp.position_ecef;
        } else {
            rover_pos = base_position_;
        }
    } else {
        bool seeded = false;
        if (rtk_config_.use_doppler_float_seed &&
            has_last_trusted_position_ && has_last_trusted_time_ &&
            has_last_doppler_velocity_) {
            const double anchor_age = rover_obs.time - last_trusted_time_;
            const double velocity_age = rover_obs.time - last_doppler_velocity_time_;
            if (std::isfinite(anchor_age) && anchor_age >= 0.0 &&
                anchor_age <= rtk_config_.doppler_float_seed_max_age_s &&
                std::isfinite(velocity_age) && velocity_age >= 0.0 &&
                velocity_age <= 1.0 && last_doppler_velocity_ecef_.allFinite()) {
                rover_pos = last_trusted_position_ +
                            last_doppler_velocity_ecef_ * anchor_age;
                var_pos = std::max(25.0,
                    std::pow(doppler_velocity_sigma_mps_ * std::max(anchor_age, 0.2), 2.0));
                seeded = true;
            }
        }
        if (!seeded && rtk_config_.prefer_trusted_position_seed &&
            has_last_trusted_position_ && has_last_trusted_time_) {
            const double dt = rover_obs.time - last_trusted_time_;
            if (std::isfinite(dt) && dt >= 0.0 && dt <= 1.0) {
                rover_pos = last_trusted_position_;
                var_pos = std::max(25.0, std::pow(3.0 * std::max(dt, 0.2), 2.0));
                seeded = true;
            }
        }

        // WP9: float-trust-policy graceful degradation. Only ever consulted
        // once trust has lapsed (the previous processed epoch did not
        // refresh trust) -- on every epoch of a healthy segment this block
        // is skipped entirely and the pre-WP9 legacy branch below runs
        // unchanged, matching WP8's finding that the wide reset should only
        // need softening during an actual trust drought. LEGACY (the
        // default) never enters this block at all.
        bool wp9_seeded = false;
        if (!seeded &&
            rtk_config_.float_trust_policy != float_trust_policy::FloatTrustPolicy::LEGACY) {
            const bool trust_refreshed_last_epoch =
                has_last_trusted_time_ && has_last_epoch_ &&
                (last_trusted_time_ == last_epoch_time_);
            const bool trust_lapsed = float_trust_policy::hasTrustLapsed(
                has_last_trusted_position_ && has_last_trusted_time_,
                trust_refreshed_last_epoch);
            if (trust_lapsed) {
                double dt_epoch = has_last_epoch_ ? (rover_obs.time - last_epoch_time_) : 0.2;
                if (!std::isfinite(dt_epoch) || dt_epoch <= 0.0) dt_epoch = 0.2;

                if (rtk_config_.float_trust_policy ==
                        float_trust_policy::FloatTrustPolicy::CV_PREDICT &&
                    has_last_solution_position_) {
                    Vector3d velocity = Vector3d::Zero();
                    if (has_prev_trusted_position_ && has_last_trusted_position_ &&
                        has_last_trusted_time_) {
                        velocity = float_trust_policy::estimateVelocityFromTrustedDeltas(
                            last_trusted_position_, prev_trusted_position_,
                            last_trusted_time_ - prev_trusted_time_, 10.0);
                    }
                    rover_pos = float_trust_policy::predictPositionConstantVelocity(
                        last_solution_position_, velocity, dt_epoch);
                    const double previous_var_pos = filter_state_.covariance(0, 0);
                    var_pos = float_trust_policy::growPositionVarianceCvPredict(
                        previous_var_pos, rtk_config_.trust_lapse_qpos_m2_per_s, dt_epoch, 900.0);
                    wp9_seeded = true;
                } else if (rtk_config_.float_trust_policy ==
                               float_trust_policy::FloatTrustPolicy::SCALED_RESET &&
                           spp.isValid()) {
                    const double dt_since_trust = has_last_trusted_time_
                        ? std::max(rover_obs.time - last_trusted_time_, 0.0)
                        : 1.0e6;  // never trusted yet -> effectively at the legacy cap
                    rover_pos = spp.position_ecef;
                    var_pos = float_trust_policy::scaledResetPositionVariance(
                        25.0, rtk_config_.trust_lapse_qpos_m2_per_s, dt_since_trust, 900.0);
                    wp9_seeded = true;
                } else if (rtk_config_.float_trust_policy ==
                               float_trust_policy::FloatTrustPolicy::LAPSE_GATED &&
                           spp.isValid()) {
                    // WP10: only switch off the LEGACY path once the
                    // *continuous* trust lapse exceeds the configured
                    // gate (or, optionally, on a sufficiently NLOS-heavy
                    // epoch regardless of lapse length). Below the gate
                    // (and with the optional NLOS trigger off/unmet),
                    // wp9_seeded is deliberately left false so this falls
                    // straight through to the unmodified legacy fallback
                    // branch below -- bit-identical to LEGACY for short
                    // lapses by construction, not just numerically close.
                    const double dt_since_trust = has_last_trusted_time_
                        ? std::max(rover_obs.time - last_trusted_time_, 0.0)
                        : 1.0e6;  // never trusted yet -> effectively at the legacy cap
                    // current_epoch_nlos_fraction_ still holds the *previous*
                    // epoch's value here (this epoch's own value isn't
                    // computed until collectSatelliteData() runs, later in
                    // processRTKEpoch()) -- a one-epoch-lagged proxy, cheap
                    // and adequate for the multi-second-to-minute NLOS-heavy
                    // dwells (e.g. the canyon) this trigger targets.
                    const bool nlos_frac_trigger =
                        rtk_config_.trust_lapse_gate_nlos_frac >= 0.0 &&
                        std::isfinite(current_epoch_nlos_fraction_) &&
                        current_epoch_nlos_fraction_ > rtk_config_.trust_lapse_gate_nlos_frac;
                    if (float_trust_policy::lapseGateExceeded(
                            dt_since_trust, rtk_config_.trust_lapse_gate_s) ||
                        nlos_frac_trigger) {
                        rover_pos = spp.position_ecef;
                        var_pos = float_trust_policy::scaledResetPositionVariance(
                            25.0, rtk_config_.trust_lapse_qpos_m2_per_s, dt_since_trust, 900.0);
                        wp9_seeded = true;
                    }
                }
            }
        }

        if (!seeded && !wp9_seeded) {
            if (rtk_config_.prefer_rover_position_seed &&
                rover_obs.receiver_position.norm() > 1e6) {
                rover_pos = rover_obs.receiver_position;
            } else if (spp.isValid()) {
                rover_pos = spp.position_ecef;
            } else if (has_last_fixed_position_) {
                rover_pos = last_fixed_position_;
            } else if (rover_obs.receiver_position.norm() > 1e6) {
                rover_pos = rover_obs.receiver_position;
            } else if (has_last_solution_position_) {
                rover_pos = last_solution_position_;
            } else {
                rover_pos = base_position_;
            }
        }
    }
    Vector3d baseline = rover_pos - base_position_;

    filter_state_.state.head<3>() = baseline;
    int n = filter_state_.state.size();
    for (int i = 0; i < BASE_STATES; ++i) {
        for (int j = 0; j < n; ++j) {
            filter_state_.covariance(i, j) = 0.0;
            filter_state_.covariance(j, i) = 0.0;
        }
        filter_state_.covariance(i, i) = var_pos;
    }
}

// ============================================================
// Main processing (RTKLIB relpos equivalent)
// ============================================================

}  // namespace libgnss
