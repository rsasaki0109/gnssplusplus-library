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

std::map<SatelliteId, RTKProcessor::SatelliteData> RTKProcessor::collectSatelliteData(
    const ObservationData& rover_obs, const ObservationData& base_obs, const NavigationData& nav) {
    std::map<SatelliteId, SatelliteData> result;
    std::map<SatelliteId, std::vector<const Observation*>> rover_l1, rover_l2, base_l1, base_l2;
    // L5 may also be collected for the shadow-only L1/L5 WL->NL diagnostic.
    // In that mode L5 remains available as independent evidence but is not
    // added to the production filter's states or measurement blocks.
    std::map<SatelliteId, std::vector<const Observation*>> rover_l5, base_l5;
    const bool l5_collection_enabled =
        rtk_config_.enable_l5 ||
        rtk_config_.lambda_l1_l5_wlnl_shadow ||
        rtk_config_.lambda_l2_l5_wlnl_shadow;
    for (const auto& obs : rover_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            rover_l1[obs.satellite].push_back(&obs);
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            // When L5 enabled, exclude L5-class obs from L2 slot so they don't
            // displace true L2C signals or pollute the L2 wavelength.
            if (l5_collection_enabled &&
                isL5RTKSignal(obs.satellite.system, obs.signal)) {
                rover_l5[obs.satellite].push_back(&obs);
                if (!rtk_config_.enable_l5) {
                    // Preserve the legacy L5-off signal-selection inputs.
                    rover_l2[obs.satellite].push_back(&obs);
                }
            } else {
                rover_l2[obs.satellite].push_back(&obs);
            }
        }
    }
    for (const auto& obs : base_obs.observations) {
        if (!isEnabledRTKSystem(rtk_config_, obs.satellite.system)) continue;
        if (!isUsableRTKSatellite(obs.satellite)) continue;
        if (isPrimaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            base_l1[obs.satellite].push_back(&obs);
        }
        if (isSecondaryRTKSignal(obs.satellite.system, obs.signal) &&
            obs.has_carrier_phase && obs.has_pseudorange) {
            if (l5_collection_enabled &&
                isL5RTKSignal(obs.satellite.system, obs.signal)) {
                base_l5[obs.satellite].push_back(&obs);
                if (!rtk_config_.enable_l5) {
                    base_l2[obs.satellite].push_back(&obs);
                }
            } else {
                base_l2[obs.satellite].push_back(&obs);
            }
        }
    }
    Vector3d rover_pos_for_clk = rover_obs.receiver_position;
    if (filter_initialized_ && filter_state_.state.size() >= 3) {
        Vector3d predicted_rover = base_position_ + filter_state_.state.head<3>();
        if (predicted_rover.norm() > 1e6) {
            rover_pos_for_clk = predicted_rover;
        }
    }

    double rover_clk_bias = 0.0;
    {
        double sum_residual = 0.0; int count = 0;
        for (const auto& [sat, candidates] : rover_l1) {
            const Observation* r_obs = selectPreferredObservation(sat.system, candidates, true);
            if (r_obs == nullptr) continue;
            Vector3d sat_pos, sat_vel; double clk, clk_drift;
            double approx_travel = r_obs->pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime approx_tx = rover_obs.time - approx_travel;
            if (!nav.calculateSatelliteState(sat, approx_tx, sat_pos, sat_vel, clk, clk_drift)) continue;
            double geometric_range = (sat_pos - rover_pos_for_clk).norm();
            sum_residual += r_obs->pseudorange - geometric_range + constants::SPEED_OF_LIGHT * clk;
            count++;
        }
        if (count > 0) rover_clk_bias = sum_residual / count;
    }
    for (const auto& [sat, rover_l1_candidates] : rover_l1) {
        auto b_it = base_l1.find(sat);
        if (b_it == base_l1.end()) continue;

        const Observation* r_obs = nullptr;
        const Observation* b_obs = nullptr;
        if (!selectMatchedObservationPair(
                sat.system, rover_l1_candidates, b_it->second, true, r_obs, b_obs)) {
            continue;
        }

        double pr_travel = r_obs->pseudorange / constants::SPEED_OF_LIGHT;
        GNSSTime t_approx = rover_obs.time - pr_travel;
        Vector3d sat_pos, sat_vel; double clk, clk_drift;
        if (!nav.calculateSatelliteState(sat, t_approx, sat_pos, sat_vel, clk, clk_drift)) continue;
        GNSSTime t_refined = t_approx - clk;
        if (!nav.calculateSatelliteState(sat, t_refined, sat_pos, sat_vel, clk, clk_drift)) continue;
        // Use unrotated sat_pos; Sagnac is handled analytically by geodist_range
        Vector3d corrected_sat_pos = sat_pos;
        Vector3d base_sat_pos = sat_pos;
        {
            double base_pr = b_obs->pseudorange;
            double base_travel = base_pr / constants::SPEED_OF_LIGHT;
            GNSSTime base_tx = base_obs.time - base_travel;
            Vector3d bsp, bsv; double bclk, bclkd;
            if (nav.calculateSatelliteState(sat, base_tx, bsp, bsv, bclk, bclkd)) {
                GNSSTime base_refined = base_tx - bclk;
                if (nav.calculateSatelliteState(sat, base_refined, bsp, bsv, bclk, bclkd)) {
                    base_sat_pos = bsp;
                }
            }
        }
        SatelliteData sd; sd.satellite = sat; sd.sat_pos = corrected_sat_pos;
        sd.sat_pos_base = base_sat_pos; sd.has_ephemeris = true;
        sd.sat_vel = sat_vel; sd.sat_clock_drift = clk_drift; sd.has_sat_velocity = true;
        auto geom = nav.calculateGeometry(rover_pos_for_clk, corrected_sat_pos);
        sd.elevation = geom.elevation;
        auto base_geom = nav.calculateGeometry(base_position_, base_sat_pos);
        sd.base_elevation = base_geom.elevation;
        if (sd.elevation < rtk_config_.elevation_mask) continue;
        const Ephemeris* eph = nav.getEphemeris(sat, t_refined);
        sd.l1_signal = r_obs->signal;
        sd.l1_frequency_hz = signalFrequencyHz(sd.l1_signal, eph);
        sd.l1_wavelength = signalWavelengthMeters(sd.l1_signal, eph);
        if (sd.l1_wavelength <= 0.0) continue;
        sd.rover_l1_phase = r_obs->carrier_phase; sd.rover_l1_code = r_obs->pseudorange;
        sd.base_l1_phase = b_obs->carrier_phase; sd.base_l1_code = b_obs->pseudorange;
        sd.rover_l1_doppler = r_obs->doppler;
        sd.base_l1_doppler = b_obs->doppler;
        sd.rover_l1_snr = r_obs->snr;
        sd.base_l1_snr = b_obs->snr;
        sd.has_l1 = true; sd.l1_lli = r_obs->lli | b_obs->lli;
        sd.has_l1_doppler = r_obs->has_doppler && b_obs->has_doppler;
        auto r_l2 = rover_l2.find(sat); auto b_l2 = base_l2.find(sat);
        if (r_l2 != rover_l2.end() && b_l2 != base_l2.end()) {
            const Observation* r_l2_obs = nullptr;
            const Observation* b_l2_obs = nullptr;
            if (selectMatchedObservationPair(
                    sat.system, r_l2->second, b_l2->second, false, r_l2_obs, b_l2_obs)) {
                sd.l2_signal = r_l2_obs->signal;
                sd.l2_frequency_hz = signalFrequencyHz(sd.l2_signal, eph);
                sd.l2_wavelength = signalWavelengthMeters(sd.l2_signal, eph);
                sd.rover_l2_phase = r_l2_obs->carrier_phase; sd.rover_l2_code = r_l2_obs->pseudorange;
                sd.base_l2_phase = b_l2_obs->carrier_phase; sd.base_l2_code = b_l2_obs->pseudorange;
                sd.rover_l2_doppler = r_l2_obs->doppler;
                sd.base_l2_doppler = b_l2_obs->doppler;
                sd.rover_l2_snr = r_l2_obs->snr;
                sd.base_l2_snr = b_l2_obs->snr;
                sd.has_l2 = sd.l2_wavelength > 0.0;
                sd.l2_lli = r_l2_obs->lli | b_l2_obs->lli;
                sd.has_l2_doppler = r_l2_obs->has_doppler && b_l2_obs->has_doppler;
            }
        }
        // Phase 18 Step 3: L5 pairing (no-op until rtk_config_.enable_l5 is set).
        if (l5_collection_enabled) {
            auto r_l5 = rover_l5.find(sat); auto b_l5 = base_l5.find(sat);
            if (r_l5 != rover_l5.end() && b_l5 != base_l5.end()) {
                const Observation* r_l5_obs = nullptr;
                const Observation* b_l5_obs = nullptr;
                // Both rover/base L5 candidate lists contain only L5-class signals,
                // so selectMatchedObservationPair with primary=false picks the highest-
                // priority L5 variant common to both (e.g. matched GPS_L5 / GAL_E5A).
                if (selectMatchedObservationPair(
                        sat.system, r_l5->second, b_l5->second, false, r_l5_obs, b_l5_obs)) {
                    sd.l5_signal = r_l5_obs->signal;
                    sd.l5_frequency_hz = signalFrequencyHz(sd.l5_signal, eph);
                    sd.l5_wavelength = signalWavelengthMeters(sd.l5_signal, eph);
                    if (sd.l5_wavelength > 0.0) {
                        sd.rover_l5_phase = r_l5_obs->carrier_phase;
                        sd.rover_l5_code = r_l5_obs->pseudorange;
                        sd.base_l5_phase = b_l5_obs->carrier_phase;
                        sd.base_l5_code = b_l5_obs->pseudorange;
                        sd.rover_l5_doppler = r_l5_obs->doppler;
                        sd.base_l5_doppler = b_l5_obs->doppler;
                        sd.rover_l5_snr = r_l5_obs->snr;
                        sd.base_l5_snr = b_l5_obs->snr;
                        sd.has_l5 = true;
                        sd.l5_lli = r_l5_obs->lli | b_l5_obs->lli;
                        sd.has_l5_doppler = r_l5_obs->has_doppler && b_l5_obs->has_doppler;
                    }
                }
            }
        }
        result[sat] = sd;
    }
    return result;
}

SatelliteId RTKProcessor::selectReferenceSatellite(const std::map<SatelliteId, SatelliteData>& sat_data) {
    SatelliteId best_dual, best_l1;
    double max_dual_el = -1.0;
    double max_l1_el = -1.0;
    for (const auto& [sat, sd] : sat_data) {
        if (sd.has_l1 && sd.has_l2 && sd.elevation > max_dual_el) {
            max_dual_el = sd.elevation;
            best_dual = sat;
        }
        if (sd.has_l1 && sd.elevation > max_l1_el) {
            max_l1_el = sd.elevation;
            best_l1 = sat;
        }
    }
    return max_dual_el >= 0.0 ? best_dual : best_l1;
}

// ============================================================
// Reference satellite change: no-op for SD parameterization
// ============================================================
void RTKProcessor::handleReferenceSatelliteChange(const SatelliteId& new_ref,
    const std::map<SatelliteId, SatelliteData>& sat_data) {
    (void)sat_data;
    current_ref_satellite_ = new_ref;
    has_ref_satellite_ = true;
}

bool RTKProcessor::selectSystemReferenceSatellite(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    GNSSSystem system,
    int min_lock_count,
    SatelliteId& ref_sat) const {
    return rtk_selection::selectSystemReferenceSatellite(
        buildSelectionSnapshot(sat_data), system, min_lock_count, ref_sat);
}

std::vector<rtk_selection::SatelliteSelectionData> RTKProcessor::buildSelectionSnapshot(
    const std::map<SatelliteId, SatelliteData>& sat_data) const {
    std::vector<rtk_selection::SatelliteSelectionData> snapshot;
    snapshot.reserve(sat_data.size());
    for (const auto& [sat, sd] : sat_data) {
        rtk_selection::SatelliteSelectionData item;
        item.satellite = sat;
        item.has_l1 = sd.has_l1;
        item.has_l2 = sd.has_l2;
        item.has_l5 = sd.has_l5;  // Phase 18 Step 4
        item.l1_wavelength = sd.l1_wavelength;
        item.l2_wavelength = sd.l2_wavelength;
        item.l5_wavelength = sd.l5_wavelength;  // Phase 18 Step 4
        item.elevation = sd.elevation;
        auto n1_it = filter_state_.n1_indices.find(sat);
        item.n1_active = n1_it != filter_state_.n1_indices.end() &&
                         filter_state_.state(n1_it->second) != 0.0;
        auto n2_it = filter_state_.n2_indices.find(sat);
        item.n2_active = n2_it != filter_state_.n2_indices.end() &&
                         filter_state_.state(n2_it->second) != 0.0;
        auto n5_it = filter_state_.n5_indices.find(sat);
        item.n5_active = n5_it != filter_state_.n5_indices.end() &&
                         filter_state_.state(n5_it->second) != 0.0;
        auto l1_it = lock_count_l1_.find(sat);
        item.lock_count_l1 = l1_it != lock_count_l1_.end() ? l1_it->second : 0;
        auto l2_it = lock_count_l2_.find(sat);
        item.lock_count_l2 = l2_it != lock_count_l2_.end() ? l2_it->second : 0;
        auto l5_it = lock_count_l5_.find(sat);
        item.lock_count_l5 = l5_it != lock_count_l5_.end() ? l5_it->second : 0;
        snapshot.push_back(item);
    }

    // WP8: hard NLOS exclusion. Both buildMeasurementBlocks() (float KF)
    // and buildDoubleDifferencePairs() (AR/LAMBDA candidate set) call this
    // one shared function to get their satellite candidate list, so
    // filtering it here applies identically to both consumers. No-op
    // (bit-identical) unless nlos_weight_mode == EXCLUDE and a weight
    // table is loaded, exactly mirroring the WP7 sigma-inflation hook's own
    // absent-flag guard.
    if (rtk_config_.nlos_weight_mode == nlos_weights::NlosWeightMode::EXCLUDE &&
        nlos_weight_table_ && !nlos_weight_table_->empty()) {
        std::set<SatelliteId> excluded;
        for (const auto& item : snapshot) {
            const double los_prob = nlos_weights::lookupLosProb(
                *nlos_weight_table_, current_epoch_time_.tow, item.satellite.toString(),
                rtk_config_.nlos_tow_tolerance_s);
            if (nlos_weights::nlosShouldExclude(
                    los_prob, rtk_config_.nlos_weight_mode, rtk_config_.nlos_exclude_threshold)) {
                excluded.insert(item.satellite);
            }
        }
        const bool guard_allows = nlos_weights::nlosExclusionGuardAllows(
            static_cast<int>(snapshot.size()), static_cast<int>(excluded.size()),
            rtk_config_.nlos_min_sats);
        if (guard_allows) {
            // Second guard: never exclude a system's *last* remaining
            // reference-satellite candidate -- that would zero out the
            // whole system's DD set rather than just shrinking it, even if
            // the epoch-wide min-sats floor above was satisfied.
            for (GNSSSystem system : kRTKSupportedSystems) {
                SatelliteId full_ref;
                if (!rtk_selection::selectSystemReferenceSatellite(snapshot, system, 0, full_ref)) {
                    continue;  // no reference candidate at all pre-exclusion; nothing to protect
                }
                if (excluded.count(full_ref) == 0) continue;
                std::vector<rtk_selection::SatelliteSelectionData> trial;
                trial.reserve(snapshot.size());
                for (const auto& item : snapshot) {
                    if (excluded.count(item.satellite) == 0) trial.push_back(item);
                }
                SatelliteId trial_ref;
                if (!rtk_selection::selectSystemReferenceSatellite(trial, system, 0, trial_ref)) {
                    excluded.erase(full_ref);
                }
            }
            if (!excluded.empty()) {
                std::vector<rtk_selection::SatelliteSelectionData> filtered;
                filtered.reserve(snapshot.size());
                for (auto& item : snapshot) {
                    if (excluded.count(item.satellite) == 0) filtered.push_back(std::move(item));
                }
                snapshot = std::move(filtered);
            }
        }
    }

    return snapshot;
}

// ============================================================
// Phase 2a: CMC-aware DD reference-satellite selection (opt-in)
// ============================================================
void RTKProcessor::updateCmcAwareReferenceSelection(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    const std::set<SatelliteId>& gf_slips,
    const std::set<SatelliteId>& gf_slips_l1l5,
    const std::set<SatelliteId>& code_slips_l1,
    const std::set<SatelliteId>& doppler_slips_l1) {
    if (!cmc_suspect_tracker_) {
        cmc_suspect_tracker_ =
            std::make_unique<rtk_cmc_reference::CmcSuspectTracker>(rtk_config_.cmc_ref_level_m);
    }

    // 1) Per-satellite CMC suspect classification for this epoch, driven by
    // the SD L1 code-minus-phase deviation from each satellite's own
    // running baseline. Reuses this epoch's already-computed slip sets as
    // the "arc restarted" signal (same reasoning as FGOProcessor's
    // rover_arc_restarted: a fresh ambiguity invalidates the old baseline).
    std::set<SatelliteId> seen;
    std::set<SatelliteId> suspects;
    for (const auto& [sat, sd] : sat_data) {
        if (!sd.has_l1 || sd.l1_wavelength <= 0.0) continue;
        seen.insert(sat);
        const double cmc_m = rtk_slip_detection::singleDifferenceCodeMinusPhaseM(
            sd.rover_l1_code, sd.base_l1_code, sd.rover_l1_phase, sd.base_l1_phase, sd.l1_wavelength);
        const bool arc_restarted = gf_slips.count(sat) > 0 || gf_slips_l1l5.count(sat) > 0 ||
                                   code_slips_l1.count(sat) > 0 || doppler_slips_l1.count(sat) > 0 ||
                                   (sd.l1_lli & 0x01) != 0;
        if (cmc_suspect_tracker_->classify(sat, cmc_m, arc_restarted)) {
            suspects.insert(sat);
            ++cmc_ref_suspect_epoch_count_;
        }
    }
    cmc_suspect_tracker_->pruneMissing(seen);

    // 2) Per-system hysteresis reference selection, fed by this epoch's
    // suspect classification above.
    cmc_aware_ref_by_system_.clear();
    const auto snapshot = buildSelectionSnapshot(sat_data);
    const double return_min_elev_rad = rtk_config_.cmc_ref_return_min_elev_deg * M_PI / 180.0;
    const double switch_away_max_elev_drop_rad =
        rtk_config_.cmc_ref_switch_max_elev_drop_deg * M_PI / 180.0;
    const double switch_away_min_elev_rad = rtk_config_.cmc_ref_switch_min_elev_deg * M_PI / 180.0;

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId natural_ref;
        if (!rtk_selection::selectSystemReferenceSatellite(snapshot, system, 0, natural_ref)) {
            continue;  // no candidate at all this epoch -- nothing to select or track
        }

        std::vector<rtk_cmc_reference::ReferenceHysteresis::Candidate> candidates;
        double natural_ref_elevation_rad = 0.0;
        for (const auto& item : snapshot) {
            if (item.satellite.system != system || !item.has_l1 || !item.n1_active) continue;
            rtk_cmc_reference::ReferenceHysteresis::Candidate candidate;
            candidate.satellite = item.satellite;
            candidate.elevation_rad = item.elevation;
            candidate.dual_frequency = item.has_l2 && item.n2_active;
            candidate.suspect = suspects.count(item.satellite) > 0;
            if (item.satellite == natural_ref) natural_ref_elevation_rad = item.elevation;
            candidates.push_back(candidate);
        }

        auto& hysteresis = cmc_ref_hysteresis_by_system_[system];
        SatelliteId chosen_ref;
        bool switched = false;
        if (hysteresis.update(candidates, natural_ref, natural_ref_elevation_rad,
                              rtk_config_.cmc_ref_switch_epochs, return_min_elev_rad, chosen_ref,
                              switched, switch_away_max_elev_drop_rad,
                              switch_away_min_elev_rad)) {
            cmc_aware_ref_by_system_[system] = chosen_ref;
            if (switched) ++cmc_ref_switch_count_;
        }
    }
}

std::vector<RTKProcessor::DDPair> RTKProcessor::buildDoubleDifferencePairs(
    const std::map<SatelliteId, SatelliteData>& sat_data,
    int min_lock_count) const {
    std::vector<DDPair> dd_pairs;
    const auto snapshot = buildSelectionSnapshot(sat_data);

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        const SatelliteId* forced_ref = nullptr;
        if (rtk_config_.cmc_aware_reference_selection) {
            const auto forced_it = cmc_aware_ref_by_system_.find(system);
            if (forced_it != cmc_aware_ref_by_system_.end()) forced_ref = &forced_it->second;
        }
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            snapshot,
            system,
            min_lock_count,
            requiresMatchedCarrierWavelength(rtk_config_, system),
            forced_ref);
        for (const auto& pair : system_pairs) {
            const auto& indices = (pair.freq == 0) ? filter_state_.n1_indices :
                                  (pair.freq == 1) ? filter_state_.n2_indices :
                                                     filter_state_.n5_indices;  // Phase 18 Step 4
            auto ref_idx = indices.find(pair.ref_sat);
            auto sat_idx = indices.find(pair.sat);
            if (ref_idx == indices.end() || sat_idx == indices.end()) {
                continue;
            }
            dd_pairs.push_back({pair.ref_sat, ref_idx->second, sat_idx->second, pair.sat, pair.freq});
        }
    }

    return dd_pairs;
}

// ============================================================
// Filter initialization
// ============================================================

}  // namespace libgnss
