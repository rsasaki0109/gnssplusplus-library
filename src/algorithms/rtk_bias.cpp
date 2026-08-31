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

bool RTKProcessor::initializeFilter(const ObservationData& rover_obs,
    const ObservationData& base_obs, const NavigationData& nav) {
    (void)base_obs;
    const int state_size = rtk_config_.enable_velocity_states ? NX : LEGACY_NX;
    filter_state_.state = VectorXd::Zero(state_size);
    filter_state_.covariance = MatrixXd::Zero(state_size, state_size);
    filter_state_.iono_indices.clear();
    filter_state_.n1_indices.clear();
    filter_state_.n2_indices.clear();
    filter_state_.n5_indices.clear();  // Phase 18 Step 2: clear L5 index map on filter init
    filter_state_.next_state_idx = REAL_STATES + IONO_STATES;

    const auto spp_started = std::chrono::steady_clock::now();
    const auto spp = spp_processor_.processEpoch(rover_obs, nav);
    debug_telemetry_.stage_spp_ms +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - spp_started)
            .count();
    Vector3d rover_pos;
    if (rtk_config_.prefer_rover_position_seed && rover_obs.receiver_position.norm() > 1e6) {
        rover_pos = rover_obs.receiver_position;
    } else if (spp.isValid()) {
        rover_pos = spp.position_ecef;
    } else if (rover_obs.receiver_position.norm() > 1e6) {
        rover_pos = rover_obs.receiver_position;
    } else {
        rover_pos = base_position_;
    }
    filter_state_.state.head<3>() = rover_pos - base_position_;

    for (int i = 0; i < BASE_STATES; ++i)
        filter_state_.covariance(i, i) = 900.0;

    filter_initialized_ = true;
    return true;
}

void RTKProcessor::updateGlonassHardwareBias(double dt) {
    if (!usesGlonassAutocal(rtk_config_)) {
        return;
    }
    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = 1.0;
    }

    const double initial_values[GLO_HWBIAS_STATES] = {
        rtk_config_.glonass_icb_l1_m_per_mhz,
        rtk_config_.glonass_icb_l2_m_per_mhz,
    };
    for (int freq = 0; freq < GLO_HWBIAS_STATES; ++freq) {
        const int idx = IL(freq);
        if (filter_state_.state(idx) == 0.0 || filter_state_.covariance(idx, idx) <= 0.0) {
            filter_state_.state(idx) = initial_values[freq];
            for (int j = 0; j < filter_state_.state.size(); ++j) {
                filter_state_.covariance(idx, j) = 0.0;
                filter_state_.covariance(j, idx) = 0.0;
            }
            filter_state_.covariance(idx, idx) = kGlonassHWBiasInitialVariance;
        } else {
            filter_state_.covariance(idx, idx) += kGlonassHWBiasProcessNoise * dt;
        }
    }
}

// ============================================================
// Update SD biases (RTKLIB udbias)
// ============================================================
void RTKProcessor::updateTdcpDiagnostics(
    const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s) {
    if (!rtk_config_.enable_tdcp_diagnostics) return;

    const rtk_tdcp_diagnostics::Config config{
        rtk_config_.tdcp_diagnostics_max_gap_s};
    double residual_sum_squares = 0.0;
    double residual_max_abs = 0.0;

    auto process_frequency = [&](auto& history, auto get_measurement) {
        std::set<SatelliteId> seen;
        for (const auto& [sat, data] : sat_data) {
            double phase_m = 0.0;
            double range_rate_mps = 0.0;
            bool loss_of_lock = false;
            if (!get_measurement(data, phase_m, range_rate_mps, loss_of_lock)) continue;
            seen.insert(sat);
            ++debug_telemetry_.tdcp_candidate_count;
            const auto previous = history.find(sat);
            if (previous == history.end()) {
                ++debug_telemetry_.tdcp_rejected_missing_previous;
            } else {
                const auto result = rtk_tdcp_diagnostics::evaluate(
                    previous->second.phase_m, phase_m,
                    previous->second.range_rate_mps, range_rate_mps,
                    dt_s, loss_of_lock, config);
                switch (result.status) {
                    case rtk_tdcp_diagnostics::Status::VALID:
                        ++debug_telemetry_.tdcp_residual_count;
                        residual_sum_squares += result.residual_m * result.residual_m;
                        residual_max_abs = std::max(residual_max_abs, std::abs(result.residual_m));
                        break;
                    case rtk_tdcp_diagnostics::Status::INVALID_GAP:
                        ++debug_telemetry_.tdcp_rejected_gap;
                        break;
                    case rtk_tdcp_diagnostics::Status::LOSS_OF_LOCK:
                        ++debug_telemetry_.tdcp_rejected_loss_of_lock;
                        break;
                    case rtk_tdcp_diagnostics::Status::INVALID_INPUT:
                        ++debug_telemetry_.tdcp_rejected_invalid;
                        break;
                }
            }
            history[sat] = TdcpHistory{phase_m, range_rate_mps};
        }
        for (auto it = history.begin(); it != history.end();) {
            it = seen.contains(it->first) ? std::next(it) : history.erase(it);
        }
    };

    process_frequency(tdcp_history_l1_, [](const SatelliteData& data, double& phase_m,
                                           double& range_rate_mps, bool& loss_of_lock) {
        if (!data.has_l1 || !data.has_l1_doppler || data.l1_wavelength <= 0.0) return false;
        phase_m = (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
        range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
            data.rover_l1_doppler, data.base_l1_doppler, data.l1_wavelength);
        loss_of_lock = data.l1_lli != 0;
        return true;
    });
    process_frequency(tdcp_history_l2_, [](const SatelliteData& data, double& phase_m,
                                           double& range_rate_mps, bool& loss_of_lock) {
        if (!data.has_l2 || !data.has_l2_doppler || data.l2_wavelength <= 0.0) return false;
        phase_m = (data.rover_l2_phase - data.base_l2_phase) * data.l2_wavelength;
        range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
            data.rover_l2_doppler, data.base_l2_doppler, data.l2_wavelength);
        loss_of_lock = data.l2_lli != 0;
        return true;
    });
    if (rtk_config_.enable_l5) {
        process_frequency(tdcp_history_l5_, [](const SatelliteData& data, double& phase_m,
                                               double& range_rate_mps, bool& loss_of_lock) {
            if (!data.has_l5 || !data.has_l5_doppler || data.l5_wavelength <= 0.0) return false;
            phase_m = (data.rover_l5_phase - data.base_l5_phase) * data.l5_wavelength;
            range_rate_mps = rtk_slip_detection::singleDifferenceRangeRateMps(
                data.rover_l5_doppler, data.base_l5_doppler, data.l5_wavelength);
            loss_of_lock = data.l5_lli != 0;
            return true;
        });
    }

    if (debug_telemetry_.tdcp_residual_count > 0) {
        debug_telemetry_.tdcp_residual_rms_m = std::sqrt(
            residual_sum_squares / debug_telemetry_.tdcp_residual_count);
        debug_telemetry_.tdcp_residual_max_abs_m = residual_max_abs;
    }
}

void RTKProcessor::updateBias(const std::map<SatelliteId, SatelliteData>& sat_data, double dt_s) {
    updateTdcpDiagnostics(sat_data, dt_s);
    current_epoch_slips_l1_.clear();
    current_epoch_slips_l2_.clear();
    current_epoch_slips_l5_.clear();
    std::vector<SatelliteId> sats_to_remove;
    for (const auto& [sat, idx] : filter_state_.n1_indices) {
        if (sat_data.find(sat) == sat_data.end()) sats_to_remove.push_back(sat);
    }
    for (const auto& sat : sats_to_remove) {
        removeSatelliteFromState(sat);
        lock_count_l1_.erase(sat);
        lock_count_l2_.erase(sat);
        lock_count_l5_.erase(sat);  // Phase 18 Step 2: erase L5 lock count when sat dropped
        gf_l1l2_history_.erase(sat);
        gf_l1l5_history_.erase(sat);  // Phase 18 Step 5
        doppler_phase_history_l1_m_.erase(sat);
        doppler_phase_history_l2_m_.erase(sat);
        doppler_phase_history_l5_m_.erase(sat);  // Phase 18 Step 5
        code_phase_history_l1_m_.erase(sat);
        code_phase_history_l2_m_.erase(sat);
        code_phase_history_l5_m_.erase(sat);  // Phase 18 Step 5
    }

    const bool dynamic_slip_floor_candidate =
        isDynamicPositionMode(rtk_config_) && rtk_config_.use_dynamic_slip_threshold_floor;
    const int adaptive_nonfix_count =
        std::max(1, rtk_config_.adaptive_dynamic_slip_nonfix_count);
    if (dynamic_slip_floor_candidate &&
        rtk_config_.enable_adaptive_dynamic_slip_thresholds &&
        consecutive_nonfix_count_ >= adaptive_nonfix_count) {
        adaptive_dynamic_slip_hold_count_ =
            std::max(adaptive_dynamic_slip_hold_count_,
                     std::max(0, rtk_config_.adaptive_dynamic_slip_hold_epochs));
    }
    const bool adaptive_dynamic_slip_active =
        dynamic_slip_floor_candidate &&
        rtk_config_.enable_adaptive_dynamic_slip_thresholds &&
        (consecutive_nonfix_count_ >= adaptive_nonfix_count ||
         adaptive_dynamic_slip_hold_count_ > 0);
    const bool apply_dynamic_slip_floor =
        dynamic_slip_floor_candidate && !adaptive_dynamic_slip_active;
    debug_telemetry_.adaptive_dynamic_slip_active = adaptive_dynamic_slip_active;
    debug_telemetry_.consecutive_nonfix_before_bias_update = consecutive_nonfix_count_;
    debug_telemetry_.adaptive_dynamic_slip_hold_remaining =
        adaptive_dynamic_slip_hold_count_;
    if (adaptive_dynamic_slip_active && adaptive_dynamic_slip_hold_count_ > 0) {
        adaptive_dynamic_slip_hold_count_--;
    }

    std::set<SatelliteId> gf_slips;
    std::set<SatelliteId> gf_slips_l1l5;  // Phase 18 Step 5
    if (rtk_config_.enable_cycle_slip_detection) {
        const double gf_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.cycle_slip_threshold, 0.12)
                : rtk_config_.cycle_slip_threshold;
        for (const auto& [sat, sd] : sat_data) {
            if (!sd.has_l1 || !sd.has_l2 || sd.l1_wavelength <= 0.0 || sd.l2_wavelength <= 0.0) continue;
            double gf = (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength -
                        (sd.rover_l2_phase - sd.base_l2_phase) * sd.l2_wavelength;
            auto prev_it = gf_l1l2_history_.find(sat);
            if (prev_it != gf_l1l2_history_.end() &&
                std::abs(gf - prev_it->second) > gf_slip_threshold) {
                gf_slips.insert(sat);
            }
            gf_l1l2_history_[sat] = gf;
        }
        if (rtk_config_.enable_l5) {
            // Phase 18 Step 5: parallel GF L1-L5 detector. Same threshold as L1-L2 since both
            // are dual-carrier ionosphere-free residuals — slip on either L1 or L5 carrier
            // appears as a multi-cycle jump in the GF combination.
            for (const auto& [sat, sd] : sat_data) {
                if (!sd.has_l1 || !sd.has_l5 || sd.l1_wavelength <= 0.0 || sd.l5_wavelength <= 0.0) continue;
                double gf15 = (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength -
                              (sd.rover_l5_phase - sd.base_l5_phase) * sd.l5_wavelength;
                auto prev_it = gf_l1l5_history_.find(sat);
                if (prev_it != gf_l1l5_history_.end() &&
                    std::abs(gf15 - prev_it->second) > gf_slip_threshold) {
                    gf_slips_l1l5.insert(sat);
                }
                gf_l1l5_history_[sat] = gf15;
            }
        }
    }
    debug_telemetry_.gf_slip_count = static_cast<int>(gf_slips.size());
    debug_telemetry_.gf_slip_l1l5_count = static_cast<int>(gf_slips_l1l5.size());

    std::set<SatelliteId> doppler_slips_l1;
    std::set<SatelliteId> doppler_slips_l2;
    std::set<SatelliteId> doppler_slips_l5;  // Phase 18 Step 5
    if (rtk_config_.enable_doppler_slip_detection &&
        std::isfinite(dt_s) &&
        dt_s > 0.0 &&
        dt_s <= 5.0) {
        const double doppler_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.doppler_slip_threshold, 0.20)
                : std::max(rtk_config_.doppler_slip_threshold, 0.10);
        for (const auto& [sat, sd] : sat_data) {
            if (sd.has_l1 && sd.has_l1_doppler && sd.l1_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l1_doppler, sd.base_l1_doppler, sd.l1_wavelength);
                auto previous = doppler_phase_history_l1_m_.find(sat);
                if (previous != doppler_phase_history_l1_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l1.insert(sat);
                }
                doppler_phase_history_l1_m_[sat] = sd_phase_m;
            }
            if (sd.has_l2 && sd.has_l2_doppler && sd.l2_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l2_phase - sd.base_l2_phase) * sd.l2_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l2_doppler, sd.base_l2_doppler, sd.l2_wavelength);
                auto previous = doppler_phase_history_l2_m_.find(sat);
                if (previous != doppler_phase_history_l2_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l2.insert(sat);
                }
                doppler_phase_history_l2_m_[sat] = sd_phase_m;
            }
            // Phase 18 Step 5: doppler-based slip detection on L5.
            if (rtk_config_.enable_l5 &&
                sd.has_l5 && sd.has_l5_doppler && sd.l5_wavelength > 0.0) {
                const double sd_phase_m =
                    (sd.rover_l5_phase - sd.base_l5_phase) * sd.l5_wavelength;
                const double sd_range_rate_mps =
                    rtk_slip_detection::singleDifferenceRangeRateMps(
                        sd.rover_l5_doppler, sd.base_l5_doppler, sd.l5_wavelength);
                auto previous = doppler_phase_history_l5_m_.find(sat);
                if (previous != doppler_phase_history_l5_m_.end() &&
                    rtk_slip_detection::detectDopplerSlip(
                        previous->second,
                        sd_phase_m,
                        sd_range_rate_mps,
                        dt_s,
                        doppler_slip_threshold)) {
                    doppler_slips_l5.insert(sat);
                }
                doppler_phase_history_l5_m_[sat] = sd_phase_m;
            }
        }
    }
    debug_telemetry_.doppler_slip_l1_count = static_cast<int>(doppler_slips_l1.size());
    debug_telemetry_.doppler_slip_l2_count = static_cast<int>(doppler_slips_l2.size());
    debug_telemetry_.doppler_slip_l5_count = static_cast<int>(doppler_slips_l5.size());

    std::set<SatelliteId> code_slips_l1;
    std::set<SatelliteId> code_slips_l2;
    std::set<SatelliteId> code_slips_l5;  // Phase 18 Step 5
    if (rtk_config_.enable_code_slip_detection) {
        const double code_slip_threshold =
            apply_dynamic_slip_floor
                ? std::max(rtk_config_.code_slip_threshold, 5.0)
                : std::max(rtk_config_.code_slip_threshold, 3.0);
        for (const auto& [sat, sd] : sat_data) {
            if (sd.has_l1 && sd.l1_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l1_code,
                        sd.base_l1_code,
                        sd.rover_l1_phase,
                        sd.base_l1_phase,
                        sd.l1_wavelength);
                auto previous = code_phase_history_l1_m_.find(sat);
                if (previous != code_phase_history_l1_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l1.insert(sat);
                }
                code_phase_history_l1_m_[sat] = code_minus_phase_m;
            }
            if (sd.has_l2 && sd.l2_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l2_code,
                        sd.base_l2_code,
                        sd.rover_l2_phase,
                        sd.base_l2_phase,
                        sd.l2_wavelength);
                auto previous = code_phase_history_l2_m_.find(sat);
                if (previous != code_phase_history_l2_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l2.insert(sat);
                }
                code_phase_history_l2_m_[sat] = code_minus_phase_m;
            }
            // Phase 18 Step 5: code-minus-phase slip detection on L5.
            if (rtk_config_.enable_l5 && sd.has_l5 && sd.l5_wavelength > 0.0) {
                const double code_minus_phase_m =
                    rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
                        sd.rover_l5_code,
                        sd.base_l5_code,
                        sd.rover_l5_phase,
                        sd.base_l5_phase,
                        sd.l5_wavelength);
                auto previous = code_phase_history_l5_m_.find(sat);
                if (previous != code_phase_history_l5_m_.end() &&
                    rtk_slip_detection::detectCodeSlip(
                        previous->second,
                        code_minus_phase_m,
                        code_slip_threshold)) {
                    code_slips_l5.insert(sat);
                }
                code_phase_history_l5_m_[sat] = code_minus_phase_m;
            }
        }
    }
    debug_telemetry_.code_slip_l1_count = static_cast<int>(code_slips_l1.size());
    debug_telemetry_.code_slip_l2_count = static_cast<int>(code_slips_l2.size());
    debug_telemetry_.code_slip_l5_count = static_cast<int>(code_slips_l5.size());

    // Phase 2a: CMC-aware DD reference-satellite selection (opt-in). Reuses
    // this epoch's already-computed slip-detection sets (gf/doppler/code)
    // as the CMC baseline's arc-restart signal. No-op (cmc_aware_ref_by_
    // system_ left empty) unless the knob is on.
    if (rtk_config_.cmc_aware_reference_selection) {
        updateCmcAwareReferenceSelection(sat_data, gf_slips, gf_slips_l1l5, code_slips_l1,
                                         doppler_slips_l1);
    } else if (!cmc_aware_ref_by_system_.empty()) {
        cmc_aware_ref_by_system_.clear();
    }

    // Phase 18 Step 4: extend freq loop from {L1, L2} to {L1, L2, L5} when enable_l5.
    // Per-frequency accessors abstract over the SatelliteData layout differences.
    auto has_freq_signal = [](const SatelliteData& sd, int freq) -> bool {
        return freq == 0 ? sd.has_l1 : (freq == 1 ? sd.has_l2 : sd.has_l5);
    };
    auto freq_lli = [](const SatelliteData& sd, int freq) -> int {
        return freq == 0 ? sd.l1_lli : (freq == 1 ? sd.l2_lli : sd.l5_lli);
    };
    auto freq_wavelength = [](const SatelliteData& sd, int freq) -> double {
        return freq == 0 ? sd.l1_wavelength : (freq == 1 ? sd.l2_wavelength : sd.l5_wavelength);
    };
    auto freq_phase_diff = [](const SatelliteData& sd, int freq) -> double {
        if (freq == 0) return sd.rover_l1_phase - sd.base_l1_phase;
        if (freq == 1) return sd.rover_l2_phase - sd.base_l2_phase;
        return sd.rover_l5_phase - sd.base_l5_phase;
    };
    auto freq_code_diff = [](const SatelliteData& sd, int freq) -> double {
        if (freq == 0) return sd.rover_l1_code - sd.base_l1_code;
        if (freq == 1) return sd.rover_l2_code - sd.base_l2_code;
        return sd.rover_l5_code - sd.base_l5_code;
    };
    const int max_freq = rtk_config_.enable_l5 ? 3 : 2;
    for (int freq = 0; freq < max_freq; ++freq) {
        auto& indices = (freq == 0) ? filter_state_.n1_indices :
                        (freq == 1) ? filter_state_.n2_indices : filter_state_.n5_indices;
        auto& lock_counts = (freq == 0) ? lock_count_l1_ :
                            (freq == 1) ? lock_count_l2_ : lock_count_l5_;
        int lli_slip_count = 0;
        int ambiguity_reset_count = 0;

        // Detect cycle slips and reset
        for (const auto& [sat, sd] : sat_data) {
            if (!has_freq_signal(sd, freq)) continue;
            int lli = freq_lli(sd, freq);
            const bool lli_slip = (lli & 0x01) != 0;
            if (lli_slip) {
                lli_slip_count++;
            }
            // Phase 18 Step 5: L5 now participates in GF (L1-L5) / doppler-L5 / code-L5 slip checks.
            bool slip = lli_slip;
            if (freq == 0) {
                slip = slip ||
                       gf_slips.find(sat) != gf_slips.end() ||
                       gf_slips_l1l5.find(sat) != gf_slips_l1l5.end() ||
                       code_slips_l1.find(sat) != code_slips_l1.end() ||
                       doppler_slips_l1.find(sat) != doppler_slips_l1.end();
            } else if (freq == 1) {
                slip = slip ||
                       gf_slips.find(sat) != gf_slips.end() ||
                       code_slips_l2.find(sat) != code_slips_l2.end() ||
                       doppler_slips_l2.find(sat) != doppler_slips_l2.end();
            } else {  // freq == 2 (L5)
                slip = slip ||
                       gf_slips_l1l5.find(sat) != gf_slips_l1l5.end() ||
                       code_slips_l5.find(sat) != code_slips_l5.end() ||
                       doppler_slips_l5.find(sat) != doppler_slips_l5.end();
            }
            const int detector_votes =
                (freq == 0
                     ? static_cast<int>(
                           gf_slips.find(sat) != gf_slips.end() ||
                           gf_slips_l1l5.find(sat) !=
                               gf_slips_l1l5.end())
                     : freq == 2
                           ? static_cast<int>(
                                 gf_slips_l1l5.find(sat) !=
                                 gf_slips_l1l5.end())
                           : 0) +
                (freq == 0
                     ? static_cast<int>(
                           code_slips_l1.find(sat) !=
                           code_slips_l1.end())
                     : freq == 2
                           ? static_cast<int>(
                                 code_slips_l5.find(sat) !=
                                 code_slips_l5.end())
                           : 0) +
                (freq == 0
                     ? static_cast<int>(
                           doppler_slips_l1.find(sat) !=
                           doppler_slips_l1.end())
                     : freq == 2
                           ? static_cast<int>(
                                 doppler_slips_l5.find(sat) !=
                                 doppler_slips_l5.end())
                           : 0);
            const bool confirmed_arc_slip =
                lli_slip || detector_votes >= 2;
            if (confirmed_arc_slip && freq == 0) {
                current_epoch_slips_l1_.insert(sat);
            } else if (confirmed_arc_slip && freq == 1) {
                current_epoch_slips_l2_.insert(sat);
            } else if (confirmed_arc_slip && freq == 2) {
                current_epoch_slips_l5_.insert(sat);
            }
            auto idx_it = indices.find(sat);
            if (idx_it != indices.end() && slip) {
                ambiguity_reset_count++;
                // navi.776 A2: a slip invalidates the learned phase variance
                // for this satellite/frequency; code memory survives.
                if (rtk_config_.enable_adaptive_measurement_noise) {
                    adaptive_noise_tracker_.resetKey(
                        freq * MAXSAT + satelliteSlot(sat),
                        rtk_measurement::MeasurementKind::PHASE);
                }
                int idx = idx_it->second;
                filter_state_.state(idx) = 0.0;
                filter_state_.covariance(idx, idx) = 0.0;
                int n = filter_state_.state.size();
                for (int j = 0; j < n; ++j) {
                    if (j != idx) { filter_state_.covariance(j, idx) = 0; filter_state_.covariance(idx, j) = 0; }
                }
                lock_counts[sat] = -rtk_config_.min_lock_count;
                if (usesEstimatedIono(rtk_config_)) {
                    auto iono_it = filter_state_.iono_indices.find(sat);
                    if (iono_it != filter_state_.iono_indices.end()) {
                        int iono_idx = iono_it->second;
                        filter_state_.state(iono_idx) = 0.0;
                        filter_state_.covariance(iono_idx, iono_idx) = 0.0;
                        for (int j = 0; j < n; ++j) {
                            if (j != iono_idx) {
                                filter_state_.covariance(j, iono_idx) = 0.0;
                                filter_state_.covariance(iono_idx, j) = 0.0;
                            }
                        }
                    }
                }
            }
        }
        if (freq == 0) {
            debug_telemetry_.lli_slip_l1_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l1_count = ambiguity_reset_count;
        } else if (freq == 1) {
            debug_telemetry_.lli_slip_l2_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l2_count = ambiguity_reset_count;
        } else {  // freq == 2 (Phase 18 Step 5)
            debug_telemetry_.lli_slip_l5_count = lli_slip_count;
            debug_telemetry_.ambiguity_reset_l5_count = ambiguity_reset_count;
        }

        for (GNSSSystem system : kRTKSupportedSystems) {
            if (!isEnabledRTKSystem(rtk_config_, system)) continue;
            std::map<SatelliteId, double> bias;
            double offset = 0.0;
            int offset_count = 0;

            for (const auto& [sat, sd] : sat_data) {
                if (sat.system != system) continue;
                if (!has_freq_signal(sd, freq)) continue;
                const double wavelength = freq_wavelength(sd, freq);
                if (wavelength <= 0.0) continue;
                const double cp = freq_phase_diff(sd, freq);
                const double pr = freq_code_diff(sd, freq);
                double b = cp - pr / wavelength;
                bias[sat] = b;
                auto idx_it = indices.find(sat);
                if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) {
                    offset += b - filter_state_.state(idx_it->second);
                    offset_count++;
                }
            }

            if (offset_count > 0) {
                double avg_offset = offset / offset_count;
                for (auto& [sat, idx] : indices) {
                    if (sat.system == system && filter_state_.state(idx) != 0.0) {
                        filter_state_.state(idx) += avg_offset;
                    }
                }
            }

            for (const auto& [sat, b] : bias) {
                auto idx_it = indices.find(sat);
                if (idx_it != indices.end() && filter_state_.state(idx_it->second) != 0.0) continue;
                if (freq == 0) getOrCreateN1Index(sat, b);
                else if (freq == 1) getOrCreateN2Index(sat, b);
                else getOrCreateN5Index(sat, b);
                lock_counts[sat] = 0;
            }
        }

        // Add process noise
        if (rtk_config_.process_noise_ambiguity > 0) {
            for (const auto& [sat, idx] : indices) {
                if (filter_state_.state(idx) != 0.0) {
                    filter_state_.covariance(idx, idx) += rtk_config_.process_noise_ambiguity;
                }
            }
        }
    }

    if (usesEstimatedIono(rtk_config_)) {
        for (const auto& [sat, sd] : sat_data) {
            if (!sd.has_l1 || !sd.has_l2) continue;
            if (sat.system == GNSSSystem::GLONASS) continue;
            if (sd.l1_frequency_hz <= 0.0 || sd.l2_frequency_hz <= 0.0) continue;
            const double gamma =
                ionoFrequencyScale(1, sd.l1_frequency_hz, sd.l2_frequency_hz);
            const double denom = gamma - 1.0;
            if (!std::isfinite(denom) || std::abs(denom) < 1e-6) continue;
            const double sd_p1 = sd.rover_l1_code - sd.base_l1_code;
            const double sd_p2 = sd.rover_l2_code - sd.base_l2_code;
            const double iono_l1_m = (sd_p2 - sd_p1) / denom;
            const int idx = getOrCreateIonoIndex(sat, iono_l1_m);
            if (filter_state_.covariance(idx, idx) > 0.0 &&
                rtk_config_.process_noise_iono > 0.0) {
                filter_state_.covariance(idx, idx) += rtk_config_.process_noise_iono;
            }
        }
    }
}

// ============================================================
// Position update (RTKLIB udpos)
// ============================================================

}  // namespace libgnss
