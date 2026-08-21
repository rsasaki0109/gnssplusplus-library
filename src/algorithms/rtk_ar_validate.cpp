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

bool RTKProcessor::validateCpPrFixedCandidate(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    const GNSSTime& current_time) {
    if (!rtk_config_.enable_cp_pr_fixed_gate) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    auto wavelength = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.l1_wavelength
             : freq == 1 ? data.l2_wavelength
                         : data.l5_wavelength;
    };
    auto phaseSd = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.rover_l1_phase - data.base_l1_phase
             : freq == 1 ? data.rover_l2_phase - data.base_l2_phase
                         : data.rover_l5_phase - data.base_l5_phase;
    };
    auto codeSd = [](const SatelliteData& data, int freq) {
        return freq == 0 ? data.rover_l1_code - data.base_l1_code
             : freq == 1 ? data.rover_l2_code - data.base_l2_code
                         : data.rover_l5_code - data.base_l5_code;
    };

    std::vector<rtk_cp_pr_gate::Observation> gate_observations;
    if (last_best_subset_.size() == static_cast<std::size_t>(last_dd_fixed_.size())) {
        gate_observations.reserve(last_best_subset_.size());
        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int pair_index = last_best_subset_[i];
            if (pair_index < 0 || pair_index >= static_cast<int>(last_dd_pairs_.size())) {
                continue;
            }
            const auto& pair = last_dd_pairs_[pair_index];
            if (pair.ref_sat.system == GNSSSystem::GLONASS) {
                continue;
            }
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }
            const double ref_wavelength = wavelength(ref_it->second, pair.freq);
            const double sat_wavelength = wavelength(sat_it->second, pair.freq);
            if (!(ref_wavelength > 0.0) || !(sat_wavelength > 0.0) ||
                std::abs(ref_wavelength - sat_wavelength) > 1e-6) {
                continue;
            }
            rtk_cp_pr_gate::Observation observation;
            observation.dd_pseudorange_m =
                codeSd(ref_it->second, pair.freq) - codeSd(sat_it->second, pair.freq);
            observation.dd_carrier_m =
                ref_wavelength * phaseSd(ref_it->second, pair.freq) -
                sat_wavelength * phaseSd(sat_it->second, pair.freq);
            observation.fixed_ambiguity_m = ref_wavelength * last_dd_fixed_(i);
            gate_observations.push_back(observation);
        }
    }

    rtk_cp_pr_gate::Config gate_config;
    gate_config.innovation_threshold_m = rtk_config_.cp_pr_fixed_gate_threshold_m;
    gate_config.min_pairs = static_cast<std::size_t>(
        std::max(1, rtk_config_.cp_pr_fixed_gate_min_pairs));
    gate_config.max_bad_pairs = static_cast<std::size_t>(
        std::max(0, rtk_config_.cp_pr_fixed_gate_max_bad_pairs));
    gate_config.escalation_epochs = static_cast<std::size_t>(
        std::max(1, rtk_config_.cp_pr_fixed_gate_escalation_epochs));
    const auto gate_result = rtk_cp_pr_gate::evaluate(gate_observations, gate_config);
    if (!gate_result.valid) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    debug_telemetry_.cp_pr_gate_evaluated = true;
    debug_telemetry_.cp_pr_gate_checked_pairs =
        static_cast<int>(gate_result.checked_pairs);
    debug_telemetry_.cp_pr_gate_bad_pairs = static_cast<int>(gate_result.bad_pairs);
    debug_telemetry_.cp_pr_gate_rms_m = gate_result.rms_innovation_m;
    debug_telemetry_.cp_pr_gate_max_m = gate_result.max_abs_innovation_m;
    if (gate_result.consistent) {
        consecutive_cp_pr_gate_rejections_ = 0;
        return true;
    }

    debug_telemetry_.cp_pr_gate_rejected = true;
    ++consecutive_cp_pr_gate_rejections_;
    const bool escalated = consecutive_cp_pr_gate_rejections_ >=
        std::max(1, rtk_config_.cp_pr_fixed_gate_escalation_epochs);
    debug_telemetry_.cp_pr_gate_escalated = escalated;
    if (escalated) {
        std::set<std::pair<SatelliteId, SatelliteId>> used_pairs;
        std::vector<rtk_ddpr_anchor::Observation> anchor_observations;
        for (const auto& pair : last_dd_pairs_) {
            if (pair.freq != 0 || !used_pairs.insert({pair.ref_sat, pair.sat}).second) {
                continue;
            }
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }
            rtk_ddpr_anchor::Observation observation;
            observation.reference_satellite_rover_ecef = ref_it->second.sat_pos;
            observation.target_satellite_rover_ecef = sat_it->second.sat_pos;
            observation.reference_satellite_base_ecef = ref_it->second.sat_pos_base;
            observation.target_satellite_base_ecef = sat_it->second.sat_pos_base;
            observation.dd_pseudorange_m =
                codeSd(ref_it->second, 0) - codeSd(sat_it->second, 0);
            anchor_observations.push_back(observation);
        }
        rtk_ddpr_anchor::Config anchor_config;
        anchor_config.fde_threshold_m = rtk_config_.ddpr_anchor_fde_threshold_m;
        anchor_config.max_fde_removals = static_cast<std::size_t>(
            std::max(0, rtk_config_.ddpr_anchor_max_fde_removals));
        const Vector3d initial_position = base_position_ + filter_state_.state.head<3>();
        const auto anchor_result = rtk_ddpr_anchor::solve(
            anchor_observations, base_position_, initial_position, anchor_config);
        if (anchor_result.valid) {
            last_ddpr_anchor_position_ecef_ = anchor_result.position_ecef;
            last_ddpr_anchor_covariance_ecef_ = anchor_result.covariance_ecef;
            last_ddpr_anchor_time_ = current_time;
            has_last_ddpr_anchor_ = true;
            debug_telemetry_.ddpr_anchor_valid = true;
            debug_telemetry_.ddpr_anchor_observations =
                static_cast<int>(anchor_result.observations_used);
            debug_telemetry_.ddpr_anchor_residual_rms_m = anchor_result.residual_rms_m;
            debug_telemetry_.ddpr_anchor_fixed_distance_m =
                (anchor_result.position_ecef - (base_position_ + fixed_baseline_)).norm();
        }
    }
    debug_telemetry_.reject_reason = "cp_pr_innovation";
    return false;
}

RTKProcessor::HoldStateSnapshot RTKProcessor::captureHoldState() const {
    HoldStateSnapshot snapshot;
    snapshot.last_fixed_position = last_fixed_position_;
    snapshot.has_last_fixed_position = has_last_fixed_position_;
    snapshot.last_fixed_time = last_fixed_time_;
    snapshot.has_last_fixed_time = has_last_fixed_time_;
    snapshot.dd_pairs = last_dd_pairs_;
    snapshot.best_subset = last_best_subset_;
    snapshot.dd_fixed = last_dd_fixed_;
    snapshot.ar_ratio = last_ar_ratio_;
    snapshot.num_fixed_ambiguities = last_num_fixed_ambiguities_;
    return snapshot;
}

void RTKProcessor::restoreHoldState(const HoldStateSnapshot& snapshot) {
    last_fixed_position_ = snapshot.last_fixed_position;
    has_last_fixed_position_ = snapshot.has_last_fixed_position;
    last_fixed_time_ = snapshot.last_fixed_time;
    has_last_fixed_time_ = snapshot.has_last_fixed_time;
    last_dd_pairs_ = snapshot.dd_pairs;
    last_best_subset_ = snapshot.best_subset;
    last_dd_fixed_ = snapshot.dd_fixed;
    last_ar_ratio_ = snapshot.ar_ratio;
    last_num_fixed_ambiguities_ = snapshot.num_fixed_ambiguities;
}

bool RTKProcessor::validateFixedSolution(const std::map<SatelliteId, SatelliteData>& sat_data,
                                         const GNSSTime& current_time) {
    if (!has_fixed_solution_) {
        debug_telemetry_.reject_reason = "no_fixed_solution";
        return false;
    }

    const Vector3d new_pos = base_position_ + fixed_baseline_;
    debug_telemetry_.fixed_candidate_position_ecef = new_pos;
    debug_telemetry_.fixed_candidate_position_valid = new_pos.allFinite();
    if (filter_state_.state.size() >= 3) {
        const Vector3d float_pos = base_position_ + filter_state_.state.head<3>();
        debug_telemetry_.fixed_candidate_float_separation_m =
            (new_pos - float_pos).norm();
    }
    if (has_last_fixed_position_) {
        debug_telemetry_.fixed_candidate_history_jump_m =
            (new_pos - last_fixed_position_).norm();
    }
    if (has_last_fixed_time_) {
        debug_telemetry_.fixed_candidate_history_dt_s = current_time - last_fixed_time_;
    }

    const bool low_ratio_guard_enabled =
        std::isfinite(rtk_config_.low_ratio_guard_threshold) &&
        rtk_config_.low_ratio_guard_threshold > 0.0 &&
        rtk_config_.low_ratio_min_fixed_ambiguities > 0;
    const bool low_count_rescue_enabled =
        std::isfinite(rtk_config_.low_count_rescue_ratio_threshold) &&
        rtk_config_.low_count_rescue_ratio_threshold > 0.0 &&
        rtk_config_.low_count_rescue_min_fixed_ambiguities > 0 &&
        std::isfinite(rtk_config_.low_count_rescue_max_history_speed_mps) &&
        rtk_config_.low_count_rescue_max_history_speed_mps > 0.0;
    const bool low_count_candidate =
        low_ratio_guard_enabled && std::isfinite(last_ar_ratio_) &&
        last_ar_ratio_ < rtk_config_.low_ratio_guard_threshold &&
        last_num_fixed_ambiguities_ < rtk_config_.low_ratio_min_fixed_ambiguities;
    bool low_count_rescue_pass = false;
    if (low_count_candidate && low_count_rescue_enabled) {
        debug_telemetry_.low_count_rescue_evaluated = true;
        const double history_dt = debug_telemetry_.fixed_candidate_history_dt_s;
        const double history_jump = debug_telemetry_.fixed_candidate_history_jump_m;
        const double history_speed =
            (std::isfinite(history_dt) && history_dt > 0.0 &&
             std::isfinite(history_jump))
                ? history_jump / history_dt
                : std::numeric_limits<double>::infinity();
        low_count_rescue_pass =
            last_num_fixed_ambiguities_ >=
                rtk_config_.low_count_rescue_min_fixed_ambiguities &&
            last_ar_ratio_ >= rtk_config_.low_count_rescue_ratio_threshold &&
            history_speed <= rtk_config_.low_count_rescue_max_history_speed_mps;
        debug_telemetry_.low_count_rescue_passed = low_count_rescue_pass;
    }
    if (low_ratio_guard_enabled &&
        (!std::isfinite(last_ar_ratio_) ||
         (low_count_candidate && !low_count_rescue_pass))) {
        debug_telemetry_.reject_reason = "low_ratio_weak_integer_system";
        return false;
    }

    if (!validateCpPrFixedCandidate(sat_data, current_time)) {
        return false;
    }

    const double fixed_candidate_baseline_m = fixed_baseline_.norm();
    const auto window_enabled = [](double max_ratio,
                                   double min_baseline,
                                   double max_baseline,
                                   double min_speed,
                                   double max_speed) {
        return (std::isfinite(max_ratio) && max_ratio > 0.0) ||
               (std::isfinite(min_baseline) && min_baseline > 0.0) ||
               (std::isfinite(max_baseline) && max_baseline > 0.0) ||
               (std::isfinite(min_speed) && min_speed > 0.0) ||
               (std::isfinite(max_speed) && max_speed > 0.0);
    };
    const bool primary_window_enabled = window_enabled(
        rtk_config_.max_fixed_update_gate_ratio,
        rtk_config_.min_fixed_update_gate_baseline_m,
        rtk_config_.max_fixed_update_gate_baseline_m,
        rtk_config_.min_fixed_update_gate_speed_mps,
        rtk_config_.max_fixed_update_gate_speed_mps);
    const bool secondary_window_enabled = window_enabled(
        rtk_config_.max_fixed_update_secondary_gate_ratio,
        rtk_config_.min_fixed_update_secondary_gate_baseline_m,
        rtk_config_.max_fixed_update_secondary_gate_baseline_m,
        rtk_config_.min_fixed_update_secondary_gate_speed_mps,
        rtk_config_.max_fixed_update_secondary_gate_speed_mps);
    const bool any_speed_window_enabled =
        (std::isfinite(rtk_config_.min_fixed_update_gate_speed_mps) &&
         rtk_config_.min_fixed_update_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.max_fixed_update_gate_speed_mps) &&
         rtk_config_.max_fixed_update_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.min_fixed_update_secondary_gate_speed_mps) &&
         rtk_config_.min_fixed_update_secondary_gate_speed_mps > 0.0) ||
        (std::isfinite(rtk_config_.max_fixed_update_secondary_gate_speed_mps) &&
         rtk_config_.max_fixed_update_secondary_gate_speed_mps > 0.0);
    double fixed_update_gate_speed_mps = std::numeric_limits<double>::quiet_NaN();
    if (any_speed_window_enabled &&
        has_fixed_update_gate_previous_solution_ &&
        new_pos.allFinite() &&
        fixed_update_gate_previous_position_.allFinite()) {
        const double dt = current_time - fixed_update_gate_previous_time_;
        if (std::isfinite(dt) && dt > 1e-3) {
            fixed_update_gate_speed_mps =
                (new_pos - fixed_update_gate_previous_position_).norm() / dt;
        }
    }
    const auto window_passes = [&](double max_ratio,
                                   double min_baseline,
                                   double max_baseline,
                                   double min_speed,
                                   double max_speed) {
        const bool ratio_passes =
            !std::isfinite(max_ratio) ||
            max_ratio <= 0.0 ||
            (std::isfinite(last_ar_ratio_) && last_ar_ratio_ <= max_ratio);
        const bool baseline_passes =
            std::isfinite(fixed_candidate_baseline_m) &&
            (!std::isfinite(min_baseline) ||
             min_baseline <= 0.0 ||
             fixed_candidate_baseline_m >= min_baseline) &&
            (!std::isfinite(max_baseline) ||
             max_baseline <= 0.0 ||
             fixed_candidate_baseline_m <= max_baseline);
        const bool speed_window_enabled =
            (std::isfinite(min_speed) && min_speed > 0.0) ||
            (std::isfinite(max_speed) && max_speed > 0.0);
        const bool speed_passes =
            !speed_window_enabled ||
            (std::isfinite(fixed_update_gate_speed_mps) &&
             (!std::isfinite(min_speed) ||
              min_speed <= 0.0 ||
              fixed_update_gate_speed_mps >= min_speed) &&
             (!std::isfinite(max_speed) ||
              max_speed <= 0.0 ||
              fixed_update_gate_speed_mps <= max_speed));
        return ratio_passes && baseline_passes && speed_passes;
    };
    const bool fixed_update_gate_window_passes =
        (!primary_window_enabled && !secondary_window_enabled) ||
        (primary_window_enabled &&
         window_passes(rtk_config_.max_fixed_update_gate_ratio,
                       rtk_config_.min_fixed_update_gate_baseline_m,
                       rtk_config_.max_fixed_update_gate_baseline_m,
                       rtk_config_.min_fixed_update_gate_speed_mps,
                       rtk_config_.max_fixed_update_gate_speed_mps)) ||
        (secondary_window_enabled &&
         window_passes(rtk_config_.max_fixed_update_secondary_gate_ratio,
                       rtk_config_.min_fixed_update_secondary_gate_baseline_m,
                       rtk_config_.max_fixed_update_secondary_gate_baseline_m,
                       rtk_config_.min_fixed_update_secondary_gate_speed_mps,
                       rtk_config_.max_fixed_update_secondary_gate_speed_mps));
    if (fixed_update_gate_window_passes) {
        const double max_fixed_update_nis =
            rtk_config_.max_fixed_update_nis_per_observation;
        if (std::isfinite(max_fixed_update_nis) &&
            max_fixed_update_nis > 0.0 &&
            std::isfinite(
                current_update_diagnostics_.normalized_innovation_squared_per_observation) &&
            current_update_diagnostics_.normalized_innovation_squared_per_observation >
                max_fixed_update_nis) {
            debug_telemetry_.reject_reason = "fixed_update_nis";
            return false;
        }
        const double max_fixed_update_post_rms =
            rtk_config_.max_fixed_update_post_residual_rms_m;
        if (std::isfinite(max_fixed_update_post_rms) &&
            max_fixed_update_post_rms > 0.0 &&
            std::isfinite(current_update_diagnostics_.post_suppression_residual_rms_m) &&
            current_update_diagnostics_.post_suppression_residual_rms_m >
                max_fixed_update_post_rms) {
            debug_telemetry_.reject_reason = "fixed_update_post_rms";
            return false;
        }
    }

    // Reject fixes that jump too much from the previous fix position
    // This catches wrong integers that pass the ratio test
    const bool has_fixed_jump_dt = has_last_fixed_time_ &&
        std::isfinite(current_time - last_fixed_time_);
    const double fixed_jump_dt =
        has_fixed_jump_dt ? current_time - last_fixed_time_ : 0.0;
    const bool fixed_anchor_usable = rtk_validation::fixedAnchorUsable(
        has_last_fixed_position_,
        has_last_fixed_time_,
        fixed_jump_dt,
        rtk_config_.max_fixed_anchor_age_s);
    const bool fixed_residual_overconfidence_suspect =
        rtk_config_.max_fixed_prefit_residual_rms_m > 0.0 &&
        rtk_config_.min_fixed_prefit_outliers > 0 &&
        current_update_diagnostics_.prefit_residual_rms_m >
            rtk_config_.max_fixed_prefit_residual_rms_m &&
        current_update_diagnostics_.suppressed_outliers >=
            rtk_config_.min_fixed_prefit_outliers &&
        rtk_config_.max_fixed_overconfidence_covariance_trace_m2 > 0.0 &&
        debug_telemetry_.float_position_covariance_trace_m2 <=
            rtk_config_.max_fixed_overconfidence_covariance_trace_m2;
    const bool require_doppler_consensus =
        !fixed_anchor_usable || fixed_residual_overconfidence_suspect;
    if (!isMovingBasePositionMode(rtk_config_) &&
        rtk_config_.max_fixed_doppler_consensus_m > 0.0 &&
        require_doppler_consensus &&
        has_doppler_continuity_position_) {
        const double consensus_distance =
            (new_pos - doppler_continuity_position_ecef_).norm();
        debug_telemetry_.fixed_candidate_doppler_consensus_distance_m =
            consensus_distance;
        if (!std::isfinite(consensus_distance) ||
            consensus_distance > rtk_config_.max_fixed_doppler_consensus_m) {
            debug_telemetry_.reject_reason = "fixed_doppler_consensus";
            return false;
        }
    }
    const bool use_adaptive_position_jump =
        rtk_config_.max_position_jump_rate_mps > 0.0 &&
        fixed_anchor_usable &&
        has_fixed_jump_dt;
    if (!isMovingBasePositionMode(rtk_config_) &&
        !use_adaptive_position_jump &&
        rtk_validation::exceedsFixHistoryJump(
            new_pos,
            last_fixed_position_,
            fixed_anchor_usable,
            rtk_config_.position_mode == RTKConfig::PositionMode::STATIC,
            consecutive_fix_count_)) {
        debug_telemetry_.reject_reason = "fix_history_jump";
        return false;
    }
    if (!isMovingBasePositionMode(rtk_config_) && fixed_anchor_usable) {
        double position_jump_limit = rtk_config_.max_position_jump_m;
        if (use_adaptive_position_jump) {
            const double adaptive_limit = rtk_validation::adaptiveJumpLimit(
                fixed_jump_dt,
                rtk_config_.max_position_jump_min_m,
                rtk_config_.max_position_jump_rate_mps);
            position_jump_limit = std::max(position_jump_limit, adaptive_limit);
        }
        if (position_jump_limit > 0.0 &&
            rtk_validation::exceedsAbsoluteJump(
                new_pos, last_fixed_position_, true, position_jump_limit)) {
            debug_telemetry_.reject_reason =
                use_adaptive_position_jump ? "adaptive_position_jump" : "max_position_jump";
            return false;
        }
    }

    // Sanity check: reject fixes where any component is unreasonably large
    if (fixed_baseline_.norm() > rtk_config_.max_baseline_length) {
        debug_telemetry_.reject_reason = "max_baseline";
        return false;
    }

    // IFLC: validate fixed L1/L2 DD ambiguities using geometry-free consistency
    // For each satellite pair, check that fixed_N1*λ1 - fixed_N2*λ2 is consistent
    // with the measured geometry-free phase (which contains iono + N1*λ1 - N2*λ2)
    // If the fixed wide-lane (N1-N2) doesn't match the Melbourne-Wubbena estimate,
    // the fix is likely corrupted by ionosphere
    if (rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC &&
        last_dd_fixed_.size() > 0 && last_dd_pairs_.size() > 0) {
        // Build map: satellite -> (fixed_DD_N1, fixed_DD_N2)
        // from last_dd_pairs_ and last_dd_fixed_
        std::map<SatelliteId, std::pair<SatelliteId, double>> fixed_dd_n1;
        std::map<SatelliteId, std::pair<SatelliteId, double>> fixed_dd_n2;
        for (int i = 0; i < (int)last_best_subset_.size(); ++i) {
            int dd_idx = last_best_subset_[i];
            if (dd_idx >= (int)last_dd_pairs_.size()) continue;
            const auto& pair = last_dd_pairs_[dd_idx];
            if (pair.freq == 0) {
                fixed_dd_n1[pair.sat] = {pair.ref_sat, last_dd_fixed_(i)};
            } else if (pair.freq == 1) {
                fixed_dd_n2[pair.sat] = {pair.ref_sat, last_dd_fixed_(i)};
            }
        }

        // For each satellite with both L1 and L2 fixed DD:
        // Compute fixed wide-lane DD_NW = DD_N1 - DD_N2
        // Compare with Melbourne-Wubbena DD_MW from observations
        int bad_wl_count = 0;
        int checked_count = 0;
        for (const auto& [sat, l1_entry] : fixed_dd_n1) {
            auto n2_it = fixed_dd_n2.find(sat);
            if (n2_it == fixed_dd_n2.end()) continue;
            if (!(n2_it->second.first == l1_entry.first)) continue;
            double dd_n1 = l1_entry.second;
            double dd_n2 = n2_it->second.second;
            double fixed_wl = dd_n1 - dd_n2;  // should be integer

            // Compute DD Melbourne-Wubbena from observations
            auto sat_it = sat_data.find(sat);
            auto ref_it = sat_data.find(l1_entry.first);
            if (sat_it == sat_data.end() || ref_it == sat_data.end()) continue;
            if (!sat_it->second.has_l2 || !ref_it->second.has_l2) continue;

            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;
            const double f1 = ref_sd.l1_frequency_hz;
            const double f2 = ref_sd.l2_frequency_hz;
            const double lambda_wl_m = wideLaneWavelength(f1, f2);
            if (lambda_wl_m <= 0.0) continue;

            double ref_L1_sd = ref_sd.rover_l1_phase - ref_sd.base_l1_phase;
            double ref_L2_sd = ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
            double ref_P1_sd = ref_sd.rover_l1_code - ref_sd.base_l1_code;
            double ref_P2_sd = ref_sd.rover_l2_code - ref_sd.base_l2_code;
            double ref_MW = (ref_L1_sd - ref_L2_sd)
                          - (f1 * ref_P1_sd + f2 * ref_P2_sd) / (lambda_wl_m * (f1 + f2));

            double sat_L1_sd = sd.rover_l1_phase - sd.base_l1_phase;
            double sat_L2_sd = sd.rover_l2_phase - sd.base_l2_phase;
            double sat_P1_sd = sd.rover_l1_code - sd.base_l1_code;
            double sat_P2_sd = sd.rover_l2_code - sd.base_l2_code;
            double sat_MW = (sat_L1_sd - sat_L2_sd)
                          - (f1 * sat_P1_sd + f2 * sat_P2_sd) / (lambda_wl_m * (f1 + f2));

            double dd_MW = ref_MW - sat_MW;
            double mw_int = std::round(dd_MW);

            // Check: fixed wide-lane should match MW estimate
            // MW noise depends on code multipath (typically 0.1-0.5 WL cycles)
            // Use threshold of 1.0 to avoid false rejections from code noise
            if (std::abs(fixed_wl - mw_int) >= 1.0) {
                bad_wl_count++;
            }
            checked_count++;
        }

        // If more than half of checked satellites have wrong WL, reject the fix
        if (checked_count > 0 && bad_wl_count > checked_count / 2) {
            debug_telemetry_.reject_reason = "iflc_wl_consistency";
            return false;
        }
    }

    auto computePostFixResidualRms = [&]() {
        if (last_dd_fixed_.size() == 0 ||
            last_best_subset_.size() != static_cast<size_t>(last_dd_fixed_.size()) ||
            filter_state_.state.size() < 3) {
            return std::numeric_limits<double>::infinity();
        }

        VectorXd fixed_state = filter_state_.state;
        fixed_state.head<3>() = fixed_baseline_;
        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int dd_idx = last_best_subset_[i];
            if (dd_idx < 0 || dd_idx >= static_cast<int>(last_dd_pairs_.size())) {
                return std::numeric_limits<double>::infinity();
            }
            const auto& pair = last_dd_pairs_[dd_idx];
            if (pair.ref_idx < 0 || pair.sat_idx < 0 ||
                pair.ref_idx >= fixed_state.size() || pair.sat_idx >= fixed_state.size()) {
                return std::numeric_limits<double>::infinity();
            }
            fixed_state(pair.sat_idx) = fixed_state(pair.ref_idx) - last_dd_fixed_(i);
        }

        const Vector3d rover_pos = base_position_ + fixed_baseline_;
        double l1_sum_sq = 0.0;
        int l1_count = 0;
        double all_sum_sq = 0.0;
        int all_count = 0;

        for (int i = 0; i < static_cast<int>(last_best_subset_.size()); ++i) {
            const int dd_idx = last_best_subset_[i];
            if (dd_idx < 0 || dd_idx >= static_cast<int>(last_dd_pairs_.size())) {
                continue;
            }
            const auto& pair = last_dd_pairs_[dd_idx];
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }

            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;
            const bool use_l1 = pair.freq == 0;
            if ((use_l1 && (!ref_sd.has_l1 || !sd.has_l1)) ||
                (!use_l1 && (!ref_sd.has_l2 || !sd.has_l2))) {
                continue;
            }

            const double ref_wavelength = use_l1 ? ref_sd.l1_wavelength : ref_sd.l2_wavelength;
            const double sat_wavelength = use_l1 ? sd.l1_wavelength : sd.l2_wavelength;
            if (ref_wavelength <= 0.0 || sat_wavelength <= 0.0 ||
                pair.ref_idx < 0 || pair.sat_idx < 0 ||
                pair.ref_idx >= fixed_state.size() || pair.sat_idx >= fixed_state.size()) {
                continue;
            }

            const double ref_phase = use_l1 ? ref_sd.rover_l1_phase - ref_sd.base_l1_phase
                                            : ref_sd.rover_l2_phase - ref_sd.base_l2_phase;
            const double sat_phase = use_l1 ? sd.rover_l1_phase - sd.base_l1_phase
                                            : sd.rover_l2_phase - sd.base_l2_phase;
            const double rr_ref =
                geodist_range(ref_sd.sat_pos, rover_pos) + tropModel(rover_pos, ref_sd.elevation);
            const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                                  tropModel(base_position_, ref_sd.base_elevation);
            const double rr =
                geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
            const double br = geodist_range(sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, sd.base_elevation);
            const double geom_dd = (rr_ref - br_ref) - (rr - br);
            const double amb_term =
                ref_wavelength * fixed_state(pair.ref_idx) -
                sat_wavelength * fixed_state(pair.sat_idx);
            const bool autocal_glonass =
                usesGlonassAutocal(rtk_config_) &&
                ref_sd.satellite.system == GNSSSystem::GLONASS &&
                sd.satellite.system == GNSSSystem::GLONASS &&
                pair.freq < GLO_HWBIAS_STATES;
            const double df_mhz =
                autocal_glonass
                    ? (((use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz) -
                        (use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz)) / 1e6)
                    : 0.0;
            const double glonass_icb =
                glonassInterChannelBiasMeters(
                    rtk_config_,
                    ref_sd.satellite.system,
                    sd.satellite.system,
                    use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz,
                    use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz,
                    pair.freq);
            const double phase_iono_term =
                usesEstimatedIono(rtk_config_)
                    ? (-ionoFrequencyScale(
                           pair.freq,
                           ref_sd.l1_frequency_hz,
                           use_l1 ? ref_sd.l1_frequency_hz : ref_sd.l2_frequency_hz) *
                           fixed_state(II(pair.ref_sat)) +
                       ionoFrequencyScale(
                           pair.freq,
                           sd.l1_frequency_hz,
                           use_l1 ? sd.l1_frequency_hz : sd.l2_frequency_hz) *
                           fixed_state(II(pair.sat)))
                    : 0.0;

            double residual =
                ref_phase * ref_wavelength - sat_phase * sat_wavelength -
                geom_dd - amb_term - glonass_icb - phase_iono_term;
            if (autocal_glonass) {
                residual -= df_mhz * fixed_state(IL(pair.freq));
            }
            if (!std::isfinite(residual)) {
                continue;
            }

            all_sum_sq += residual * residual;
            all_count++;
            if (use_l1) {
                l1_sum_sq += residual * residual;
                l1_count++;
            }
        }

        if (l1_count > 0) {
            return std::sqrt(l1_sum_sq / static_cast<double>(l1_count));
        }
        if (all_count > 0) {
            return std::sqrt(all_sum_sq / static_cast<double>(all_count));
        }
        return std::numeric_limits<double>::infinity();
    };

    const double postfix_residual_rms =
        (filter_initialized_ && filter_state_.state.size() >= 3)
            ? computePostFixResidualRms()
            : std::numeric_limits<double>::infinity();
    debug_telemetry_.postfix_residual_rms = postfix_residual_rms;

    const double max_postfix_residual_rms = rtk_config_.max_postfix_residual_rms;
    if (filter_initialized_ &&
        filter_state_.state.size() >= 3 &&
        std::isfinite(max_postfix_residual_rms) &&
        max_postfix_residual_rms > 0.0 &&
        postfix_residual_rms > max_postfix_residual_rms) {
        debug_telemetry_.reject_reason = "postfix_rms";
        return false;
    }

    return true;
}

// ============================================================
// Hold ambiguities (RTKLIB holdamb equivalent)
// ============================================================
void RTKProcessor::applyHoldAmbiguity() {
    if (last_dd_fixed_.size() == 0) return;

    int n = filter_state_.state.size();

    auto& x = filter_state_.state;
    auto& P = filter_state_.covariance;

    // Direct state adjustment: move SD ambiguities to satisfy DD integer constraints
    // Only adjust the satellite (non-reference) SD ambiguity for each DD pair
    // This avoids modifying the reference satellite state which is shared across all DDs
    for (int i = 0; i < (int)last_best_subset_.size(); ++i) {
        int dd_idx = last_best_subset_[i];
        if (dd_idx >= (int)last_dd_pairs_.size()) continue;
        if (!usesHoldAmbiguitySystem(rtk_config_, last_dd_pairs_[dd_idx].ref_sat.system)) continue;
        int ri = last_dd_pairs_[dd_idx].ref_idx;
        int si = last_dd_pairs_[dd_idx].sat_idx;
        if (ri >= n || si >= n) continue;
        if (x(ri) == 0.0 || x(si) == 0.0) continue;

        // Set sat SD state so that DD = dd_fixed
        // DD = x[ri] - x[si] = dd_fixed  =>  x[si] = x[ri] - dd_fixed
        double dd_fixed = last_dd_fixed_(i);
        x(si) = x(ri) - dd_fixed;

        // Tighten the satellite ambiguity covariance (leave ref untouched)
        // Reduce cross-correlations with position to prevent position distortion
        constexpr double VAR_HOLDAMB = 0.001;
        for (int j = 0; j < n; ++j) {
            if (j == si) continue;
            P(si, j) = 0.0;
            P(j, si) = 0.0;
        }
        P(si, si) = VAR_HOLDAMB;
    }
}

// ============================================================
// Hold fix: use held DD integers when LAMBDA fails
// ============================================================
bool RTKProcessor::tryHoldFix(const std::map<SatelliteId, SatelliteData>& sat_data,
                               const GNSSTime& time, int n_sats, PositionSolution& solution) {
    debug_telemetry_.hold_fix_attempted = true;
    const auto reject_hold = [&](const char* reason) {
        debug_telemetry_.hold_fix_reject_reason = reason;
        return false;
    };
    if (isMovingBasePositionMode(rtk_config_)) {
        return reject_hold("moving_base");
    }
    if (!rtk_validation::canAttemptHoldFix(consecutive_fix_count_,
                                           rtk_config_.min_hold_count,
                                           has_last_fixed_position_,
                                           last_dd_fixed_.size() > 0)) {
        return reject_hold("preconditions");
    }

    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    std::vector<DDPair> dd_pairs = buildDoubleDifferencePairs(sat_data, 1);
    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !usesHoldAmbiguitySystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    debug_telemetry_.hold_fix_candidate_pairs = nb;
    if (nb < 4) return reject_hold("candidate_pairs_lt4");

    // Match current DD pairs with last held DD pairs
    // Use the held DD integers for matching satellites
    VectorXd dd_fixed(nb);
    int matched = 0;
    for (int i = 0; i < nb; ++i) {
        bool found = false;
        for (int j = 0; j < (int)last_best_subset_.size(); ++j) {
            int dd_idx = last_best_subset_[j];
            if (dd_idx >= (int)last_dd_pairs_.size()) continue;
            if (dd_pairs[i].ref_sat == last_dd_pairs_[dd_idx].ref_sat &&
                dd_pairs[i].sat == last_dd_pairs_[dd_idx].sat &&
                dd_pairs[i].freq == last_dd_pairs_[dd_idx].freq) {
                dd_fixed(i) = last_dd_fixed_(j);
                found = true;
                matched++;
                break;
            }
        }
        if (!found) {
            // For unmatched DD pairs, round the current float DD to nearest integer
            double dd_float = filter_state_.state(dd_pairs[i].ref_idx) -
                            filter_state_.state(dd_pairs[i].sat_idx);
            dd_fixed(i) = std::round(dd_float);
        }
    }

    debug_telemetry_.hold_fix_matched_pairs = matched;
    if (matched < 4) return reject_hold("matched_pairs_lt4");

    std::vector<rtk_measurement::AmbiguityDifference> differences;
    differences.reserve(nb);
    for (const auto& pair : dd_pairs) {
        differences.push_back({pair.ref_idx, pair.sat_idx});
    }

    const auto ambiguity_transform = rtk_measurement::buildAmbiguityTransform(
        filter_state_.state, filter_state_.covariance, na, differences);
    const VectorXd head_state = ambiguity_transform.head_state;
    VectorXd dd_float_v = ambiguity_transform.dd_float;
    MatrixXd Qb = ambiguity_transform.ambiguity_covariance;
    MatrixXd Qab = ambiguity_transform.head_ambiguity_covariance;
    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;
    VectorXd db = dd_float_v - dd_fixed;
    Eigen::LDLT<MatrixXd> Qb_solver(Qb);
    if (Qb_solver.info() != Eigen::Success) return reject_hold("covariance_factorization");
    VectorXd Qb_inv_db = Qb_solver.solve(db);

    VectorXd xa = head_state - Qab * Qb_inv_db;
    Vector3d test_pos = base_position_ + xa.head<3>();

    // Position validation: only accept if close to last fix
    const double max_hold_jump_m =
        (rtk_config_.position_mode == RTKConfig::PositionMode::STATIC) ? 0.1 : 1.0;
    debug_telemetry_.hold_fix_jump_m = (test_pos - last_fixed_position_).norm();
    if (rtk_validation::exceedsAbsoluteJump(
            test_pos, last_fixed_position_, has_last_fixed_position_, max_hold_jump_m)) {
        return reject_hold("position_jump");
    }
    if (filter_state_.state.size() >= 3) {
        const Vector3d float_pos = base_position_ + filter_state_.state.head<3>();
        debug_telemetry_.hold_fix_float_divergence_m = (test_pos - float_pos).norm();
        if (rtk_config_.max_hold_divergence_m > 0.0 &&
            debug_telemetry_.hold_fix_float_divergence_m >
                rtk_config_.max_hold_divergence_m) {
            return reject_hold("float_divergence");
        }
    }

    // Accept hold fix
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = std::max(last_ar_ratio_, rtk_config_.ambiguity_ratio_threshold);
    last_num_fixed_ambiguities_ = matched;
    last_dd_pairs_ = dd_pairs;
    // Rebuild best_subset as all indices
    last_best_subset_.clear();
    for (int i = 0; i < nb; ++i) last_best_subset_.push_back(i);
    last_dd_fixed_ = dd_fixed;

    Vector3d saved_baseline = filter_state_.state.head<3>();
    filter_state_.state.head<3>() = fixed_baseline_;
    solution = generateSolution(time, SolutionStatus::FIXED, n_sats);
    filter_state_.state.head<3>() = saved_baseline;

    last_fixed_position_ = base_position_ + fixed_baseline_;
    last_fixed_time_ = time;
    has_last_fixed_time_ = true;
    debug_telemetry_.hold_fix_applied = true;

    return true;
}

// ============================================================
// Solution
// ============================================================

}  // namespace libgnss
