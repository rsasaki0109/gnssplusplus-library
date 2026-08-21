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

PositionSolution RTKProcessor::processRTKEpoch(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    if (rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        has_doppler_continuity_position_ && has_last_doppler_velocity_) {
        const double dt = rover_obs.time - doppler_continuity_time_;
        const double velocity_age = rover_obs.time - last_doppler_velocity_time_;
        if (std::isfinite(dt) && dt > 0.0 && dt <= 2.0 &&
            std::isfinite(velocity_age) && velocity_age >= 0.0 && velocity_age <= 2.0 &&
            last_doppler_velocity_ecef_.allFinite()) {
            doppler_continuity_position_ecef_ += last_doppler_velocity_ecef_ * dt;
            doppler_continuity_time_ = rover_obs.time;
        } else if (!std::isfinite(dt) || dt < 0.0 || dt > 2.0) {
            has_doppler_continuity_position_ = false;
        }
    }
    PositionSolution solution = processRTKEpochInternal(rover_obs, base_obs, nav);
    updateSafeFixShadowStateMachine(rover_obs.time);
    applyLibraryFixedQualityGate(solution);

    // Doppler-derived velocity: SPPProcessor now populates has_velocity on
    // its own solutions (spp.cpp), so the fallback-to-SPP paths inside
    // processRTKEpochInternal (fallback_spp()/the catch block) already carry
    // a velocity through untouched. The DD-RTK float/fixed paths
    // (generateSolution()) do not compute velocity at all, so run the same
    // SPP-style Doppler LS directly on the rover observations here -- no
    // dependency on RTK's DD/ambiguity state, just broadcast ephemeris +
    // Doppler, per docs/design.md.
    if (solution.isValid() &&
        (!solution.has_velocity || rtk_config_.max_fixed_doppler_consensus_m > 0.0)) {
        const Vector3d velocity_linearization_position =
            rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
                    has_doppler_continuity_position_
                ? doppler_continuity_position_ecef_
                : solution.position_ecef;
        const auto velocity_result = spp_velocity::solveVelocityFromObservations(
            rover_obs, nav, velocity_linearization_position, doppler_velocity_sigma_mps_);
        if (velocity_result.ok) {
            solution.velocity_ecef = velocity_result.velocity_ecef;
            solution.velocity_covariance = velocity_result.velocity_covariance;
            solution.has_velocity = true;
            solution.receiver_clock_drift = velocity_result.receiver_clock_drift;
        }
    }

    if (solution.isValid() && solution.has_velocity && solution.velocity_ecef.allFinite()) {
        last_doppler_velocity_ecef_ = solution.velocity_ecef;
        last_doppler_velocity_time_ = rover_obs.time;
        has_last_doppler_velocity_ = true;
    }

    if (rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        solution.status == SolutionStatus::FIXED && solution.position_ecef.allFinite()) {
        // Only a candidate that already passed the independent consensus gate
        // may re-anchor the track. This bounds integration drift without letting
        // a rejected RTK basin contaminate the Doppler continuity state.
        doppler_continuity_position_ecef_ = solution.position_ecef;
        doppler_continuity_time_ = rover_obs.time;
        has_doppler_continuity_position_ = true;
    }

    // A nominal DD path can still finish with an invalid generated FLOAT
    // (for example after a rank-deficient update) without taking
    // fallback_spp() or throwing. Apply the same bounded FLOAT-only bridge
    // here so every no-solution exit is governed by one fail-closed policy.
    if (solution.status == SolutionStatus::NONE ||
        !solution.isValid()) {
        auto continuity = makeSafeFloatContinuity(rover_obs.time);
        if (continuity.isValid()) {
            recordFallbackEpoch(rover_obs, nav);
            external_inertial_fix_evidence_ =
                ExternalInertialFixEvidence{};
            external_disjoint_satellite_fix_evidence_ =
                ExternalDisjointSatelliteFixEvidence{};
            return continuity;
        }
    }
    external_inertial_fix_evidence_ = ExternalInertialFixEvidence{};
    external_disjoint_satellite_fix_evidence_ =
        ExternalDisjointSatelliteFixEvidence{};
    return solution;
}

PositionSolution RTKProcessor::processRTKEpochInternal(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    debug_telemetry_ = EpochDebugTelemetry{};
    independent_failure_budget_evaluated_this_epoch_ = false;
    PositionSolution solution;
    solution.time = rover_obs.time;
    solution.status = SolutionStatus::NONE;
    current_update_diagnostics_ = RTKUpdateDiagnostics{};
    // WP7: record current tow for buildMeasurementBlocks()'s NLOS weight lookup.
    current_epoch_time_ = rover_obs.time;

    try {
        const bool moving_base_mode = isMovingBasePositionMode(rtk_config_);
        if (moving_base_mode && base_obs.receiver_position.norm() > 1e6) {
            setBasePosition(base_obs.receiver_position);
        }
        if (!base_position_known_) {
            auto spp = spp_processor_.processEpoch(rover_obs, nav);
            rememberSolution(spp);
            consecutive_fix_count_ = 0;
            consecutive_float_count_ = 0;
            consecutive_nonfix_count_ = 0;
            consecutive_high_float_residual_count_ = 0;
            consecutive_high_fixed_residual_count_ = 0;
            adaptive_dynamic_slip_hold_count_ = 0;
            return spp;
        }

        auto current_spp = spp_processor_.processEpoch(rover_obs, nav);
        auto fallback_spp = [&]() {
            last_ar_ratio_ = 0.0;
            last_num_fixed_ambiguities_ = 0;
            auto spp = current_spp;
            if (!moving_base_mode && spp.isValid() && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (spp.position_ecef - last_trusted_position_).norm();
                if (spp.num_satellites <= 5 && trusted_jump > 25.0) {
                    spp = PositionSolution{};
                    spp.time = rover_obs.time;
                    spp.status = SolutionStatus::NONE;
                }
            }
            if (!moving_base_mode && spp.isValid() && has_last_solution_position_ && has_last_epoch_) {
                double dt = rover_obs.time - last_epoch_time_;
                if (!std::isfinite(dt) || dt < 0.5) dt = 1.0;
                const double jump =
                    (spp.position_ecef - last_solution_position_).norm();
                const double max_jump = std::max(100.0, 30.0 * dt);
                if (spp.num_satellites <= 4 && jump > max_jump) {
                    spp = PositionSolution{};
                    spp.time = rover_obs.time;
                    spp.status = SolutionStatus::NONE;
                }
            }
            if (!spp.isValid()) {
                spp = makeSafeFloatContinuity(rover_obs.time);
            }
            if (
                spp.isValid() &&
                !debug_telemetry_.safe_float_continuity_used) {
                spp.status = SolutionStatus::SPP;
            }
            if (!debug_telemetry_.safe_float_continuity_used) {
                rememberSolution(spp);
            }
            recordFallbackEpoch(rover_obs, nav);
            return spp;
        };

        auto finitePosition = [](const PositionSolution& sol) {
            return sol.position_ecef.allFinite();
        };

        auto deviatesTooFarFromSPP = [&](const PositionSolution& sol, double threshold_m) {
            if (!current_spp.isValid() || !finitePosition(sol) || !current_spp.position_ecef.allFinite()) {
                return false;
            }
            return (sol.position_ecef - current_spp.position_ecef).norm() > threshold_m;
        };

        if (!filter_initialized_) {
            auto init_sat = collectSatelliteData(rover_obs, base_obs, nav);
            if (init_sat.size() < 4) {
                return fallback_spp();
            }
            if (!initializeFilter(rover_obs, base_obs, nav)) {
                return fallback_spp();
            }
        }

        resetPositionToSPP(rover_obs, nav);

        handleConsecutiveFloatReset(rover_obs, nav);

        auto sat_data = collectSatelliteData(rover_obs, base_obs, nav);
        if (sat_data.size() < 4) {
            return fallback_spp();
        }

        // WP9/WP10: cache this epoch's NLOS fraction (sat_data is only
        // available locally here) for two downstream readers: WP9's
        // rememberSolution() jump-gate relax check (same epoch), and
        // WP10's LAPSE_GATED --trust-lapse-gate-nlos-frac trigger, which
        // reads it (necessarily one-epoch-lagged) from the *next*
        // epoch's resetPositionToSPP(), called before this recomputation.
        // NaN unless one of the two levers is on and a table is loaded --
        // std::isfinite() at every read site gates this to a strict no-op
        // otherwise.
        current_epoch_nlos_fraction_ = std::numeric_limits<double>::quiet_NaN();
        if ((rtk_config_.trust_gate_nlos_relax ||
             rtk_config_.trust_lapse_gate_nlos_frac >= 0.0) &&
            nlos_weight_table_ && !nlos_weight_table_->empty()) {
            int total = 0;
            int nlos = 0;
            for (const auto& kv : sat_data) {
                ++total;
                const double los_prob = nlos_weights::lookupLosProb(
                    *nlos_weight_table_, current_epoch_time_.tow, kv.first.toString(),
                    rtk_config_.nlos_tow_tolerance_s);
                if (los_prob < 0.5) ++nlos;
            }
            current_epoch_nlos_fraction_ = total > 0 ? static_cast<double>(nlos) / total : 0.0;
        }

        double state_dt = 1.0;
        if (has_last_epoch_) {
            state_dt = rover_obs.time - last_epoch_time_;
        }
        updateGlonassHardwareBias(state_dt);

        SatelliteId new_ref = selectReferenceSatellite(sat_data);
        handleReferenceSatelliteChange(new_ref, sat_data);
        current_sat_data_ = sat_data;
        updateBias(sat_data, state_dt);

        // Iterative KF update
        bool filter_ok = false;
        int max_kf_iterations = rtk_config_.kf_iterations;
        if (isDynamicPositionMode(rtk_config_) &&
            has_last_solution_position_ && max_kf_iterations > 2) {
            max_kf_iterations = 2;
        }
        for (int iter = 0; iter < max_kf_iterations; ++iter) {
            const Vector3d baseline_before_iter = filter_state_.state.head<3>();
            filter_ok = updateFilter(sat_data);
            if (!filter_ok) break;
            current_update_diagnostics_.iterations++;
            if (iter >= 1) {
                const double baseline_step =
                    (filter_state_.state.head<3>() - baseline_before_iter).norm();
                if (baseline_step < 1e-3) {
                    break;
                }
            }
        }

        if (filter_ok) {
            incrementLockCounts(sat_data);
        }

        if (filter_ok) {
            int n_sats = static_cast<int>(sat_data.size());
            const Vector3d saved_last_solution_position = last_solution_position_;
            const bool saved_has_last_solution = has_last_solution_position_;
            const GNSSTime saved_last_solution_time = last_epoch_time_;
            const bool saved_has_last_solution_time = has_last_epoch_;
            const Vector3d saved_last_trusted_position = last_trusted_position_;
            const bool saved_has_last_trusted = has_last_trusted_position_;
            const GNSSTime saved_last_trusted_time = last_trusted_time_;
            const bool saved_has_last_trusted_time = has_last_trusted_time_;
            fixed_update_gate_previous_position_ = saved_last_solution_position;
            fixed_update_gate_previous_time_ = saved_last_solution_time;
            has_fixed_update_gate_previous_solution_ =
                saved_has_last_solution && saved_has_last_solution_time;
            auto restoreRememberedState = [&]() {
                last_solution_position_ = saved_last_solution_position;
                has_last_solution_position_ = saved_has_last_solution;
                last_epoch_time_ = saved_last_solution_time;
                has_last_epoch_ = saved_has_last_solution_time;
                last_trusted_position_ = saved_last_trusted_position;
                has_last_trusted_position_ = saved_has_last_trusted;
                last_trusted_time_ = saved_last_trusted_time;
                has_last_trusted_time_ = saved_has_last_trusted_time;
                consecutive_high_float_residual_count_ = 0;
            };
            last_ar_ratio_ = 0.0;
            last_num_fixed_ambiguities_ = 0;
            solution = generateSolution(rover_obs.time, SolutionStatus::FLOAT, n_sats);
            const PositionSolution float_solution = solution;

            const double max_float_spp_divergence_m = rtk_config_.max_float_spp_divergence_m;
            const bool float_exceeds_spp_gate =
                std::isfinite(max_float_spp_divergence_m) &&
                max_float_spp_divergence_m > 0.0 &&
                deviatesTooFarFromSPP(solution, max_float_spp_divergence_m);
            if (!finitePosition(solution) ||
                deviatesTooFarFromSPP(solution, 150.0) ||
                float_exceeds_spp_gate) {
                restoreRememberedState();
                return fallback_spp();
            }

            if (!moving_base_mode &&
                n_sats <= 4 && has_last_trusted_position_ && has_last_trusted_time_) {
                const double trusted_jump =
                    (solution.position_ecef - saved_last_trusted_position).norm();
                if (trusted_jump > 25.0) {
                    restoreRememberedState();
                    return fallback_spp();
                }
            }

            if (!moving_base_mode && saved_has_last_trusted && saved_has_last_trusted_time) {
                double dt = rover_obs.time - saved_last_trusted_time;
                if (!std::isfinite(dt) || dt < 0.5) {
                    dt = 1.0;
                }
                if (dt <= 3.0) {
                    const double trusted_jump =
                        (solution.position_ecef - saved_last_trusted_position).norm();
                    const double max_trusted_jump = std::max(8.0, 12.0 * dt);
                    if (trusted_jump > max_trusted_jump) {
                        restoreRememberedState();
                        return fallback_spp();
                    }
                }
            }

            if (!moving_base_mode && saved_has_last_solution && saved_has_last_solution_time) {
                double dt = rover_obs.time - saved_last_solution_time;
                if (!std::isfinite(dt) || dt < 0.5) dt = 1.0;
                const double jump =
                    (solution.position_ecef - saved_last_solution_position).norm();
                const double max_jump = std::max(120.0, 35.0 * dt);
                if (n_sats <= 5 && jump > max_jump) {
                    restoreRememberedState();
                    return fallback_spp();
                }
            }

            const auto saved_hold_state = captureHoldState();
            bool forced_fixed_reacquisition_reset = false;
            bool fixed_prefit_quarantine = false;
            auto emitReacquisitionFloat = [&]() {
                debug_telemetry_.post_validation_rejected = true;
                debug_telemetry_.reject_reason = "fixed_prefit_wrong_basin";
                has_fixed_solution_ = false;
                restoreHoldState(saved_hold_state);
                restoreRememberedState();
                solution = float_solution;
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
                resetAmbiguityStatesForReacquisition(rover_obs, nav, true);
                has_last_fixed_position_ = false;
                has_last_fixed_time_ = false;
                has_last_trusted_position_ = false;
                has_last_trusted_time_ = false;
                forced_fixed_reacquisition_reset = true;
            };
            has_fixed_solution_ = false;
            struct ARCandidate {
                bool valid = false;
                Vector3d baseline = Vector3d::Zero();
                double ratio = 0.0;
                int num_fixed_ambiguities = 0;
                std::vector<DDPair> dd_pairs;
                std::vector<int> best_subset;
                VectorXd dd_fixed;
                bool independent_failure_budget_passed = false;
                int independent_source_families = 0;
                double joint_failure_probability = 1.0;
            };

            auto capture_candidate = [&]() {
                ARCandidate candidate;
                if (!has_fixed_solution_) {
                    return candidate;
                }
                candidate.valid = true;
                candidate.baseline = fixed_baseline_;
                candidate.ratio = last_ar_ratio_;
                candidate.num_fixed_ambiguities = last_num_fixed_ambiguities_;
                candidate.dd_pairs = last_dd_pairs_;
                candidate.best_subset = last_best_subset_;
                candidate.dd_fixed = last_dd_fixed_;
                candidate.independent_failure_budget_passed =
                    debug_telemetry_
                        .safe_fix_shadow_failure_budget_passed;
                candidate.independent_source_families =
                    debug_telemetry_
                        .safe_fix_shadow_independent_source_families;
                candidate.joint_failure_probability =
                    debug_telemetry_
                        .safe_fix_shadow_joint_failure_probability;
                return candidate;
            };

            auto restore_candidate = [&](const ARCandidate& candidate) {
                has_fixed_solution_ = candidate.valid;
                fixed_baseline_ = candidate.baseline;
                last_ar_ratio_ = candidate.ratio;
                last_num_fixed_ambiguities_ = candidate.num_fixed_ambiguities;
                last_dd_pairs_ = candidate.dd_pairs;
                last_best_subset_ = candidate.best_subset;
                last_dd_fixed_ = candidate.dd_fixed;
                debug_telemetry_
                    .safe_fix_shadow_failure_budget_passed =
                    candidate.independent_failure_budget_passed;
                debug_telemetry_
                    .safe_fix_shadow_independent_source_families =
                    candidate.independent_source_families;
                debug_telemetry_
                    .safe_fix_shadow_joint_failure_probability =
                    candidate.joint_failure_probability;
            };

            const int min_lock = std::max(1, rtk_config_.min_lock_count);
            auto build_pairs_for_mode = [&](RTKConfig::GlonassARMode mode) {
                const auto saved_mode = rtk_config_.glonass_ar_mode;
                rtk_config_.glonass_ar_mode = mode;
                auto dd_pairs = buildDoubleDifferencePairs(sat_data, min_lock);
                rtk_config_.glonass_ar_mode = saved_mode;
                return dd_pairs;
            };

            auto try_ar_mode = [&](RTKConfig::GlonassARMode mode,
                                   const std::vector<DDPair>* prebuilt_pairs) {
                const auto saved_mode = rtk_config_.glonass_ar_mode;
                rtk_config_.glonass_ar_mode = mode;
                has_fixed_solution_ = false;
                last_ar_ratio_ = 0.0;
                last_num_fixed_ambiguities_ = 0;
                const bool resolved =
                    (prebuilt_pairs ? resolveAmbiguities(*prebuilt_pairs) : resolveAmbiguities()) &&
                    has_fixed_solution_;
                ARCandidate candidate;
                if (resolved) {
                    updateIndependentFailureBudgetTelemetry();
                    candidate = capture_candidate();
                }
                rtk_config_.glonass_ar_mode = saved_mode;
                return candidate;
            };

            bool have_fix_candidate = false;
            if (rtk_config_.glonass_ar_mode == RTKConfig::GlonassARMode::AUTOCAL) {
                const auto preview_pairs =
                    build_pairs_for_mode(RTKConfig::GlonassARMode::AUTOCAL);
                const int glonass_pair_count = static_cast<int>(std::count_if(
                    preview_pairs.begin(), preview_pairs.end(), [](const DDPair& pair) {
                        return pair.ref_sat.system == GNSSSystem::GLONASS;
                    }));

                const auto autocal_candidate =
                    glonass_pair_count > 0
                        ? try_ar_mode(RTKConfig::GlonassARMode::AUTOCAL, &preview_pairs)
                        : ARCandidate{};
                const bool confident_autocal =
                    autocal_candidate.valid &&
                    autocal_candidate.ratio >= rtk_config_.ambiguity_ratio_threshold + 0.4 &&
                    autocal_candidate.num_fixed_ambiguities >=
                        std::max(4, rtk_config_.min_satellites_for_ar - 1);
                std::vector<DDPair> classic_pairs;
                const auto classic_candidate = [&]() {
                    if (glonass_pair_count != 0 && confident_autocal) {
                        return ARCandidate{};
                    }
                    if (glonass_pair_count == 0) {
                        return try_ar_mode(RTKConfig::GlonassARMode::OFF, &preview_pairs);
                    }
                    classic_pairs = build_pairs_for_mode(RTKConfig::GlonassARMode::OFF);
                    return try_ar_mode(RTKConfig::GlonassARMode::OFF, &classic_pairs);
                }();
                ARCandidate chosen;
                if (autocal_candidate.valid && classic_candidate.valid) {
                    chosen = (autocal_candidate.ratio + 1e-6 >= classic_candidate.ratio)
                                 ? autocal_candidate
                                 : classic_candidate;
                } else if (autocal_candidate.valid) {
                    chosen = autocal_candidate;
                } else if (classic_candidate.valid) {
                    chosen = classic_candidate;
                }
                if (chosen.valid) {
                    restore_candidate(chosen);
                    have_fix_candidate = true;
                } else {
                    has_fixed_solution_ = false;
                    last_ar_ratio_ = 0.0;
                    last_num_fixed_ambiguities_ = 0;
                }
            } else if (resolveAmbiguities() && has_fixed_solution_) {
                have_fix_candidate = true;
            }

            // A declaration-time independent-budget gate must run before
            // validate/apply/hold mutates any trusted-FIX state.  A later
            // output-only demotion would still let a rejected integer
            // candidate seed hold ambiguities and future trusted positions.
            const bool require_independent_budget =
                rtk_config_.library_fixed_quality_gate.enabled &&
                rtk_config_.library_fixed_quality_gate
                    .require_independent_failure_budget;
            if (require_independent_budget) {
                if (rtk_config_.glonass_ar_mode !=
                    RTKConfig::GlonassARMode::AUTOCAL) {
                    updateIndependentFailureBudgetTelemetry();
                }
                if (!debug_telemetry_
                         .safe_fix_shadow_failure_budget_passed) {
                    have_fix_candidate = false;
                    has_fixed_solution_ = false;
                    if (debug_telemetry_.reject_reason.empty()) {
                        debug_telemetry_.reject_reason =
                            "independent_failure_budget";
                    }
                }
            }

            bool applied_fix_solution = false;
            if (have_fix_candidate && has_fixed_solution_) {
                debug_telemetry_.validation_attempted = true;
                const bool validation_ok = validateFixedSolution(sat_data, rover_obs.time);
                debug_telemetry_.validation_passed = validation_ok;
                if (validation_ok) {
                    Vector3d saved_baseline = filter_state_.state.head<3>();
                    filter_state_.state.head<3>() = fixed_baseline_;
                    solution = generateSolution(rover_obs.time, SolutionStatus::FIXED, n_sats);
                    filter_state_.state.head<3>() = saved_baseline;
                    const double fixed_float_jump =
                        (solution.position_ecef - float_solution.position_ecef).norm();
                    debug_telemetry_.fixed_float_jump_m = fixed_float_jump;
                    bool exceeds_trusted_jump = false;
                    if (saved_has_last_trusted && saved_has_last_trusted_time) {
                        const double dt = rover_obs.time - saved_last_trusted_time;
                        exceeds_trusted_jump = rtk_validation::exceedsAdaptiveJump(
                            solution.position_ecef,
                            saved_last_trusted_position,
                            dt,
                            20.0,
                            25.0);
                    }
                    if (!finitePosition(solution) ||
                        deviatesTooFarFromSPP(solution, 150.0) ||
                        fixed_float_jump > 20.0 ||
                        exceeds_trusted_jump) {
                        debug_telemetry_.post_validation_rejected = true;
                        if (!finitePosition(solution)) {
                            debug_telemetry_.reject_reason = "nonfinite_fixed";
                        } else if (deviatesTooFarFromSPP(solution, 150.0)) {
                            debug_telemetry_.reject_reason = "fixed_spp_distance";
                        } else if (fixed_float_jump > 20.0) {
                            debug_telemetry_.reject_reason = "fixed_float_jump";
                        } else if (exceeds_trusted_jump) {
                            debug_telemetry_.reject_reason = "trusted_jump";
                        }
                        has_fixed_solution_ = false;
                        restoreHoldState(saved_hold_state);
                        last_trusted_position_ = saved_last_trusted_position;
                        has_last_trusted_position_ = saved_has_last_trusted;
                        last_trusted_time_ = saved_last_trusted_time;
                        has_last_trusted_time_ = saved_has_last_trusted_time;
                    } else if (!moving_base_mode && shouldResetAfterFixedResidualGate()) {
                        if (rtk_config_.fixed_prefit_quarantine_only) {
                            debug_telemetry_.post_validation_rejected = true;
                            debug_telemetry_.reject_reason = "fixed_prefit_quarantine";
                            has_fixed_solution_ = false;
                            restoreRememberedState();
                            fixed_prefit_quarantine = true;
                        } else {
                            emitReacquisitionFloat();
                        }
                    } else {
                        updateStatistics(SolutionStatus::FIXED);
                        consecutive_fix_count_++;
                        consecutive_float_count_ = 0;
                        recordFixedEpoch(solution);
                        debug_telemetry_.final_fixed_applied = true;

                        // Save fixed position for next epoch's position reset
                        last_fixed_position_ = base_position_ + fixed_baseline_;
                        has_last_fixed_position_ = true;
                        last_fixed_time_ = rover_obs.time;
                        has_last_fixed_time_ = true;

                        // holdamb: constrain SD ambiguities toward validated DD integers
                        if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                            applyHoldAmbiguity();
                        }
                        applied_fix_solution = true;
                    }
                } else {
                    if (debug_telemetry_.reject_reason.empty()) {
                        debug_telemetry_.reject_reason = "fixed_validation";
                    }
                    has_fixed_solution_ = false;
                    // Mirror the post-validation reject branch above: validateFixedSolution
                    // ran after generateSolution(FIXED) which updated last_trusted_position_
                    // via rememberSolution; if we don't roll those back, downstream epochs
                    // see the rejected fix as "trusted" and fail deviatesTooFarFromSPP /
                    // trusted-jump gates, cascading into fallback_spp and aborting the run.
                    last_trusted_position_ = saved_last_trusted_position;
                    has_last_trusted_position_ = saved_has_last_trusted;
                    last_trusted_time_ = saved_last_trusted_time;
                    has_last_trusted_time_ = saved_has_last_trusted_time;
                }
            }

            if (!moving_base_mode &&
                !applied_fix_solution &&
                !forced_fixed_reacquisition_reset &&
                !fixed_prefit_quarantine &&
                (!require_independent_budget ||
                 debug_telemetry_
                     .safe_fix_shadow_failure_budget_passed) &&
                rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
                rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                                  rtk_config_.min_hold_count,
                                                  saved_hold_state.has_last_fixed_position,
                                                  saved_hold_state.hasHeldIntegers())) {
                restoreHoldState(saved_hold_state);
                if (tryHoldFix(sat_data, rover_obs.time, n_sats, solution)) {
                    if (shouldResetAfterFixedResidualGate()) {
                        emitReacquisitionFloat();
                    } else {
                        updateStatistics(SolutionStatus::FIXED);
                        consecutive_fix_count_++;
                        consecutive_float_count_ = 0;
                        recordFixedEpoch(solution);
                        debug_telemetry_.final_fixed_applied = true;
                        if (consecutive_fix_count_ >= rtk_config_.min_hold_count) {
                            applyHoldAmbiguity();
                        }
                        applied_fix_solution = true;
                    }
                }
            }

            if (!applied_fix_solution && !forced_fixed_reacquisition_reset) {
                restoreHoldState(
                    fixed_prefit_quarantine ? HoldStateSnapshot{} : saved_hold_state);
                solution = float_solution;
                const bool reset_after_high_float =
                    !moving_base_mode &&
                    shouldResetAfterFloatResidualGate(float_solution,
                                                      saved_last_trusted_position,
                                                      saved_has_last_trusted,
                                                      saved_last_trusted_time,
                                                      saved_has_last_trusted_time);
                if (reset_after_high_float) {
                    restoreRememberedState();
                } else {
                    rememberSolution(float_solution);
                }
                updateStatistics(SolutionStatus::FLOAT);
                consecutive_fix_count_ = 0;
                if (reset_after_high_float) {
                    resetAmbiguityStatesForReacquisition(rover_obs, nav);
                } else {
                    recordFloatEpoch(rover_obs, nav);
                    if (fixed_prefit_quarantine) {
                        consecutive_high_fixed_residual_count_ = std::max(
                            1, rtk_config_.fixed_prefit_reset_streak);
                    }
                }
            }
        } else {
            return fallback_spp();
        }
    } catch (const std::exception& e) {
        std::cerr << "RTK exception: " << e.what() << std::endl;
        auto spp = spp_processor_.processEpoch(rover_obs, nav);
        if (!spp.isValid()) {
            spp = makeSafeFloatContinuity(rover_obs.time);
        }
        if (spp.isValid() &&
            !debug_telemetry_.safe_float_continuity_used) {
            spp.status = SolutionStatus::SPP;
            rememberSolution(spp);
        }
        recordFallbackEpoch(rover_obs, nav);
        consecutive_fix_count_ = 0;
        consecutive_float_count_ = 0;
        consecutive_nonfix_count_ = 0;
        consecutive_high_float_residual_count_ = 0;
        consecutive_high_fixed_residual_count_ = 0;
        adaptive_dynamic_slip_hold_count_ = 0;
        return spp;
    }
    stabilizeFloatOutput(solution);
    return solution;
}

// ============================================================
// KF update: DD observation model with H mapping to SD states
// ============================================================

}  // namespace libgnss
