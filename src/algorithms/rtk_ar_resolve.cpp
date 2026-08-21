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

bool RTKProcessor::resolveAmbiguities() {
    if (!filter_initialized_) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::FILTER_NOT_INIT;
        return false;
    }
    if (usesEstimatedIono(rtk_config_)) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::ESTIMATED_IONO_MODE;
        return false;
    }

    const auto& sat_data = current_sat_data_;

    const int min_lock = std::max(1, rtk_config_.min_lock_count);
    return resolveAmbiguities(buildDoubleDifferencePairs(sat_data, min_lock));
}

bool RTKProcessor::resolveAmbiguities(std::vector<DDPair> dd_pairs) {
    if (!filter_initialized_) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::FILTER_NOT_INIT;
        return false;
    }
    if (usesEstimatedIono(rtk_config_)) {
        debug_telemetry_.ar_skip_reason = ARSkipReason::ESTIMATED_IONO_MODE;
        return false;
    }

    const auto& sat_data = current_sat_data_;
    debug_telemetry_.ar_attempted = true;
    debug_telemetry_.input_pair_count = static_cast<int>(dd_pairs.size());

    const int na = usesGlonassAutocal(rtk_config_) ? REAL_STATES : BASE_STATES;

    dd_pairs.erase(
        std::remove_if(dd_pairs.begin(), dd_pairs.end(),
                       [&](const DDPair& pair) {
                           return !isAmbiguityResolutionSystem(rtk_config_, pair.ref_sat.system);
                       }),
        dd_pairs.end());

    int nb = dd_pairs.size();
    debug_telemetry_.pair_count = nb;
    if (nb < 4) {
        debug_telemetry_.reject_reason = "too_few_pairs";
        debug_telemetry_.ar_skip_reason = ARSkipReason::DD_PAIRS_LT_4_BEFORE_VAR_FILTER;
        return false;
    }

    // WP10 (WP8 recommendation 2): --nlos-min-los-sats AR-acceptance gate.
    // Gates AR only -- it never touches buildSelectionSnapshot()/
    // buildMeasurementBlocks(), so the float-KF update for this epoch is
    // completely unaffected either way. No-op unless nlos_min_los_sats > 0
    // and a weight table is loaded (mirrors WP8 EXCLUDE's own absent-flag
    // guard style).
    if (rtk_config_.nlos_min_los_sats > 0 && nlos_weight_table_ &&
        !nlos_weight_table_->empty()) {
        std::set<SatelliteId> candidate_sats;
        for (const auto& pair : dd_pairs) {
            candidate_sats.insert(pair.ref_sat);
            candidate_sats.insert(pair.sat);
        }
        int los_count = 0;
        for (const auto& sat : candidate_sats) {
            const double los_prob = nlos_weights::lookupLosProb(
                *nlos_weight_table_, current_epoch_time_.tow, sat.toString(),
                rtk_config_.nlos_tow_tolerance_s);
            if (los_prob >= 0.5) ++los_count;
        }
        if (!nlos_weights::nlosMinLosSatsGateAllows(los_count, rtk_config_.nlos_min_los_sats)) {
            debug_telemetry_.reject_reason = "too_few_los_sats";
            debug_telemetry_.ar_skip_reason = ARSkipReason::TOO_FEW_LOS_SATS;
            return false;
        }
    }

    std::vector<rtk_measurement::AmbiguityDifference> differences;
    differences.reserve(nb);
    for (const auto& pair : dd_pairs) {
        differences.push_back({pair.ref_idx, pair.sat_idx});
    }

    const auto ambiguity_transform = rtk_measurement::buildAmbiguityTransform(
        filter_state_.state, filter_state_.covariance, na, differences);
    VectorXd head_state = ambiguity_transform.head_state;
    VectorXd dd_float = ambiguity_transform.dd_float;
    MatrixXd Qb = ambiguity_transform.ambiguity_covariance;
    MatrixXd Qab = ambiguity_transform.head_ambiguity_covariance;

    Qb = (Qb + Qb.transpose()) / 2.0;
    for (int i = 0; i < nb; ++i)
        if (Qb(i, i) < 1e-6) Qb(i, i) = 1e-6;

    // Variance check
    double max_var = 0;
    for (int i = 0; i < nb; ++i) max_var = std::max(max_var, Qb(i, i));
    debug_telemetry_.max_ambiguity_variance = max_var;

    // Exclude DD pairs with outlier variance (relative to median)
    // This removes newly-appearing satellites that haven't converged
    {
        std::vector<rtk_ar_selection::PairDescriptor> descriptors;
        descriptors.reserve(nb);
        for (int i = 0; i < nb; ++i) {
            descriptors.push_back({dd_pairs[i].ref_sat.system, Qb(i, i)});
        }
        std::vector<int> good_pairs =
            rtk_ar_selection::filterPairsByRelativeVariance(descriptors);

        if (!good_pairs.empty() && (int)good_pairs.size() < nb) {
            int nb_new = good_pairs.size();
            VectorXd new_dd_float(nb_new);
            MatrixXd new_Qb(nb_new, nb_new);
            MatrixXd new_Qab(na, nb_new);
            std::vector<DDPair> new_dd_pairs;
            for (int i = 0; i < nb_new; ++i) {
                new_dd_float(i) = dd_float(good_pairs[i]);
                new_dd_pairs.push_back(dd_pairs[good_pairs[i]]);
                for (int j = 0; j < nb_new; ++j)
                    new_Qb(i, j) = Qb(good_pairs[i], good_pairs[j]);
                for (int j = 0; j < na; ++j)
                    new_Qab(j, i) = Qab(j, good_pairs[i]);
            }
            new_Qb = (new_Qb + new_Qb.transpose()) / 2.0;

            dd_float = new_dd_float;
            Qb = new_Qb;
            Qab = new_Qab;
            dd_pairs.clear();
            dd_pairs = new_dd_pairs;
            nb = nb_new;
            max_var = 0;
            for (int i = 0; i < nb; ++i) max_var = std::max(max_var, Qb(i, i));
            debug_telemetry_.pair_count = nb;
            debug_telemetry_.max_ambiguity_variance = max_var;
            if (nb < 4) {
                // Diagnostic-only: mark root cause; do NOT early-return.
                // Preserve original solver flow so LAMBDA still attempts AR
                // (and fails the usual way). LAMBDA_FAILED assignment below
                // is guarded so this more-specific reason is not overwritten.
                debug_telemetry_.ar_skip_reason = ARSkipReason::DD_PAIRS_LT_4_AFTER_VAR_FILTER;
            }
        }
    }

    std::vector<int> causal_arc_ready_subset;
    std::vector<double> causal_arc_smoothed_dd(
        nb, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> causal_arc_satellite_variance(
        nb, std::numeric_limits<double>::infinity());
    std::vector<double> causal_arc_reference_variance(
        nb, std::numeric_limits<double>::infinity());
    if (rtk_config_.lambda_causal_arc_readiness_shadow) {
        debug_telemetry_.lambda_causal_arc_readiness_attempted = true;
        debug_telemetry_.lambda_causal_arc_total_pairs = nb;
        causal_arc_ready_subset.reserve(nb);
        const double time_s =
            static_cast<double>(current_epoch_time_.week) * 604800.0 +
            current_epoch_time_.tow;
        const auto slipped = [&](const SatelliteId& sat, int freq) {
            if (freq == 0) {
                return current_epoch_slips_l1_.count(sat) > 0;
            }
            if (freq == 1) {
                return current_epoch_slips_l2_.count(sat) > 0;
            }
            return current_epoch_slips_l5_.count(sat) > 0;
        };
        for (int index = 0; index < nb; ++index) {
            const auto& pair = dd_pairs[index];
            if (pair.sat_idx < 0 ||
                pair.sat_idx >= filter_state_.state.size() ||
                pair.ref_idx < 0 ||
                pair.ref_idx >= filter_state_.state.size()) {
                continue;
            }
            const auto satellite_arc =
                ambiguity_arc_bank_.updateSignal(
                    pair.sat, pair.freq, time_s,
                    filter_state_.state(pair.sat_idx),
                    slipped(pair.sat, pair.freq));
            const auto reference_arc =
                ambiguity_arc_bank_.updateSignal(
                    pair.ref_sat, pair.freq, time_s,
                    filter_state_.state(pair.ref_idx),
                    slipped(pair.ref_sat, pair.freq));
            if (satellite_arc.reset) {
                ++debug_telemetry_.lambda_causal_arc_resets;
            }
            if (reference_arc.reset) {
                ++debug_telemetry_.lambda_causal_arc_resets;
            }
            if (satellite_arc.ready && reference_arc.ready &&
                std::isfinite(satellite_arc.smoothed_value) &&
                std::isfinite(reference_arc.smoothed_value) &&
                std::isfinite(
                    satellite_arc.smoothed_value_variance) &&
                std::isfinite(
                    reference_arc.smoothed_value_variance) &&
                satellite_arc.smoothed_value_variance >= 0.0 &&
                reference_arc.smoothed_value_variance >= 0.0) {
                causal_arc_ready_subset.push_back(index);
                causal_arc_smoothed_dd[index] =
                    reference_arc.smoothed_value -
                    satellite_arc.smoothed_value;
                causal_arc_satellite_variance[index] =
                    satellite_arc.smoothed_value_variance;
                causal_arc_reference_variance[index] =
                    reference_arc.smoothed_value_variance;
            }
        }
        debug_telemetry_.lambda_causal_arc_ready_pairs =
            static_cast<int>(causal_arc_ready_subset.size());
    }

    // Try full set first
    VectorXd dd_fixed;
    double ratio = 0.0;
    bool fixed = false;

    // Use lower ratio threshold when holdamb is active (more confidence in solution)
    double effective_ratio_threshold = rtk_config_.ambiguity_ratio_threshold;
    if (rtk_config_.enable_satellite_count_ratio_threshold) {
        std::set<SatelliteId> ratio_satellites;
        for (const auto& pair : dd_pairs) {
            ratio_satellites.insert(pair.ref_sat);
            ratio_satellites.insert(pair.sat);
        }
        const int satellite_count = static_cast<int>(ratio_satellites.size());
        debug_telemetry_.ratio_satellite_count = satellite_count;
        if (satellite_count >= 20) {
            effective_ratio_threshold = 1.5;
        } else if (satellite_count >= 15) {
            effective_ratio_threshold = 2.0;
        } else if (satellite_count >= 10) {
            effective_ratio_threshold = 2.5;
        } else {
            effective_ratio_threshold = 3.0;
        }
    }
    if (rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS) {
        if (consecutive_fix_count_ >= rtk_config_.min_hold_count && has_last_fixed_position_) {
            // WP7 dead-knob fix: honor the configured hold-ambiguity ratio
            // threshold (--hold-ratio-threshold) instead of a hardcoded 2.0.
            // Default hold_ambiguity_ratio_threshold is 2.0 (rtk.hpp), so this
            // is bit-identical unless a caller explicitly overrides the flag.
            effective_ratio_threshold =
                std::isfinite(rtk_config_.hold_ambiguity_ratio_threshold) &&
                rtk_config_.hold_ambiguity_ratio_threshold > 0.0
                    ? rtk_config_.hold_ambiguity_ratio_threshold
                    : 2.0;
        }
    }
    debug_telemetry_.effective_ratio_threshold = effective_ratio_threshold;
    const int min_subset_pairs_for_ar = std::max(4, rtk_config_.min_subset_pairs_for_ar);
    debug_telemetry_.min_subset_pair_count = min_subset_pairs_for_ar;
    const double min_full_ratio_for_subset_ar =
        std::isfinite(rtk_config_.min_full_ratio_for_subset_ar)
            ? std::max(0.0, rtk_config_.min_full_ratio_for_subset_ar)
            : 0.0;
    debug_telemetry_.min_full_ratio_for_subset_ar = min_full_ratio_for_subset_ar;

    struct SubsetDiversity {
        int distinct_sats = 0;
        int distinct_systems = 0;
        int distinct_frequencies = 0;
        int dual_frequency_sats = 0;
    };
    auto compute_subset_diversity = [&](const std::vector<int>& subset) {
        SubsetDiversity diversity;
        std::set<SatelliteId> sats;
        std::set<GNSSSystem> systems;
        std::set<int> frequencies;
        std::map<SatelliteId, std::set<int>> frequencies_by_sat;
        for (int index : subset) {
            if (index < 0 || index >= static_cast<int>(dd_pairs.size())) {
                continue;
            }
            const auto& pair = dd_pairs[index];
            sats.insert(pair.sat);
            systems.insert(pair.sat.system);
            frequencies.insert(pair.freq);
            frequencies_by_sat[pair.sat].insert(pair.freq);
        }
        for (const auto& [sat, sat_frequencies] : frequencies_by_sat) {
            (void)sat;
            if (sat_frequencies.size() >= 2) {
                diversity.dual_frequency_sats++;
            }
        }
        diversity.distinct_sats = static_cast<int>(sats.size());
        diversity.distinct_systems = static_cast<int>(systems.size());
        diversity.distinct_frequencies = static_cast<int>(frequencies.size());
        return diversity;
    };
    auto passes_subset_diversity_gate = [&](const SubsetDiversity& diversity) {
        if (rtk_config_.min_subset_sats_for_ar > 0 &&
            diversity.distinct_sats < rtk_config_.min_subset_sats_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_systems_for_ar > 0 &&
            diversity.distinct_systems < rtk_config_.min_subset_systems_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_frequencies_for_ar > 0 &&
            diversity.distinct_frequencies < rtk_config_.min_subset_frequencies_for_ar) {
            return false;
        }
        if (rtk_config_.min_subset_dual_frequency_sats_for_ar > 0 &&
            diversity.dual_frequency_sats < rtk_config_.min_subset_dual_frequency_sats_for_ar) {
            return false;
        }
        return true;
    };

    // === Wide-lane AR pre-step (default-off) ===
    // Frozen copies of full-size matrices for use in build_search_problem
    // (best_candidate may update dd_float/Qb/Qab to subset-sized values later)
    const VectorXd base_dd_float = dd_float;
    const MatrixXd base_Qb = Qb;
    const MatrixXd base_Qab = Qab;
    const VectorXd base_head_state = head_state;

    std::vector<int> full_subset(nb);
    for (int i = 0; i < nb; ++i) {
        full_subset[i] = i;
    }
    std::vector<int> initial_candidate_subset = full_subset;

    struct WideLaneConstraint {
        int l1_index = -1;
        int l2_index = -1;
        double fixed_integer = 0.0;
    };
    std::vector<WideLaneConstraint> wide_lane_constraints;
    int wide_lane_total = 0;
    int wide_lane_fixed = 0;
    int wide_lane_rejected = 0;
    double wide_lane_min_distance = std::numeric_limits<double>::infinity();
    double wide_lane_max_distance = 0.0;

    auto compute_wide_lane_float = [&](int l1_index, int l2_index, double& wide_lane_float) {
        if (l1_index < 0 || l2_index < 0 ||
            l1_index >= nb || l2_index >= nb ||
            dd_pairs[l1_index].freq != 0 || dd_pairs[l2_index].freq != 1) {
            return false;
        }

        const auto ref_it = sat_data.find(dd_pairs[l1_index].ref_sat);
        const auto sat_it = sat_data.find(dd_pairs[l1_index].sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            return false;
        }
        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        if (!ref_sd.has_l1 || !ref_sd.has_l2 || !sd.has_l1 || !sd.has_l2) {
            return false;
        }

        const double f1 = ref_sd.l1_frequency_hz;
        const double f2 = ref_sd.l2_frequency_hz;
        const double lambda_wl_m = wideLaneWavelength(f1, f2);
        if (f1 <= 0.0 || f2 <= 0.0 || lambda_wl_m <= 0.0) {
            return false;
        }

        auto single_difference_wide_lane = [&](const SatelliteData& data) {
            const double phi1_m =
                (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
            const double phi2_m =
                (data.rover_l2_phase - data.base_l2_phase) * data.l2_wavelength;
            const double code_term =
                (f1 * (data.rover_l1_code - data.base_l1_code) +
                 f2 * (data.rover_l2_code - data.base_l2_code)) / (f1 + f2);
            return ((f1 * phi1_m - f2 * phi2_m) / (f1 - f2) - code_term) / lambda_wl_m;
        };

        wide_lane_float = single_difference_wide_lane(ref_sd) -
                          single_difference_wide_lane(sd);
        return std::isfinite(wide_lane_float);
    };

    // Phase 18 Step 6: L1-L5 wide-lane Melbourne-Wübbena combination.
    // Parallel to the L1-L2 path; uses the dd_pairs entry with freq==2 as the L5 leg.
    auto compute_wide_lane_l5_float = [&](int l1_index, int l5_index, double& wide_lane_float) {
        if (l1_index < 0 || l5_index < 0 ||
            l1_index >= nb || l5_index >= nb ||
            dd_pairs[l1_index].freq != 0 || dd_pairs[l5_index].freq != 2) {
            return false;
        }
        const auto ref_it = sat_data.find(dd_pairs[l1_index].ref_sat);
        const auto sat_it = sat_data.find(dd_pairs[l1_index].sat);
        if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
            return false;
        }
        const auto& ref_sd = ref_it->second;
        const auto& sd = sat_it->second;
        if (!ref_sd.has_l1 || !ref_sd.has_l5 || !sd.has_l1 || !sd.has_l5) {
            return false;
        }

        const double f1 = ref_sd.l1_frequency_hz;
        const double f5 = ref_sd.l5_frequency_hz;
        const double lambda_wl_m = wideLaneWavelength(f1, f5);  // ~0.751 m for GPS L1-L5
        if (f1 <= 0.0 || f5 <= 0.0 || lambda_wl_m <= 0.0) {
            return false;
        }

        auto single_difference_wide_lane_l5 = [&](const SatelliteData& data) {
            const double phi1_m =
                (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
            const double phi5_m =
                (data.rover_l5_phase - data.base_l5_phase) * data.l5_wavelength;
            const double code_term =
                (f1 * (data.rover_l1_code - data.base_l1_code) +
                 f5 * (data.rover_l5_code - data.base_l5_code)) / (f1 + f5);
            return ((f1 * phi1_m - f5 * phi5_m) / (f1 - f5) - code_term) / lambda_wl_m;
        };

        wide_lane_float = single_difference_wide_lane_l5(ref_sd) -
                          single_difference_wide_lane_l5(sd);
        return std::isfinite(wide_lane_float);
    };

    // Shadow-only L1/L5 WL->NL cascade. Unlike enable_l5, this path does not
    // add L5 ambiguity states or L5 measurements to the production KF. It
    // derives an independent L5 DD ambiguity from the carrier observation and
    // current float geometry, resolves N1-N5 first, then resolves N1 on the
    // matching L1 subset. Both integer searches must independently pass
    // covariance-inflated FFRT, and the WL integer must agree with MW.
    if (rtk_config_.lambda_l1_l5_wlnl_shadow ||
        rtk_config_.lambda_l1_l2_wlnl_shadow ||
        rtk_config_.lambda_l2_l5_wlnl_shadow) {
        // 0=L1/L5, 1=L1/L2, 2=L2/L5.
        std::vector<int> wlnl_modes;
        if (rtk_config_.lambda_l1_l5_wlnl_shadow) {
            wlnl_modes.push_back(0);
        }
        if (rtk_config_.lambda_l1_l2_wlnl_shadow) {
            wlnl_modes.push_back(1);
        }
        if (rtk_config_.lambda_l2_l5_wlnl_shadow) {
            wlnl_modes.push_back(2);
        }
        const auto wlnl_started = std::chrono::steady_clock::now();
        for (const int wlnl_mode : wlnl_modes) {
        const bool use_l2_wlnl = wlnl_mode == 1;
        const bool use_l2_primary = wlnl_mode == 2;
        auto& wlnl_attempted = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_attempted
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_attempted
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_attempted;
        auto& wlnl_pair_count = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_pair_count
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_pair_count
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_pair_count;
        auto& wlnl_wl_bsr = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_wl_bsr
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_wl_bsr
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_wl_bsr;
        auto& wlnl_wl_ratio = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_wl_ratio
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_wl_ratio
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_wl_ratio;
        auto& wlnl_wl_ffrt_min_ratio = use_l2_wlnl
            ? debug_telemetry_
                  .lambda_l1_l2_wlnl_shadow_wl_ffrt_min_ratio
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_wl_ffrt_min_ratio
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_wl_ffrt_min_ratio;
        auto& wlnl_wl_ffrt_passed = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_wl_ffrt_passed
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_wl_ffrt_passed
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_wl_ffrt_passed;
        auto& wlnl_mw_disagreements = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_mw_disagreements
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_mw_disagreements
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_mw_disagreements;
        auto& wlnl_raw_mw_disagreements = use_l2_wlnl
            ? debug_telemetry_
                  .lambda_l1_l2_wlnl_shadow_raw_mw_disagreements
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_raw_mw_disagreements
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_raw_mw_disagreements;
        auto& wlnl_causal_arc_ready_pairs = use_l2_wlnl
            ? debug_telemetry_
                  .lambda_l1_l2_wlnl_shadow_causal_arc_ready_pairs
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_causal_arc_ready_pairs
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_causal_arc_ready_pairs;
        auto& wlnl_causal_arc_resets = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_causal_arc_resets
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_causal_arc_resets
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_causal_arc_resets;
        auto& wlnl_nl_bsr = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_nl_bsr
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_nl_bsr
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_nl_bsr;
        auto& wlnl_nl_ratio = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_nl_ratio
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_nl_ratio
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_nl_ratio;
        auto& wlnl_nl_ffrt_min_ratio = use_l2_wlnl
            ? debug_telemetry_
                  .lambda_l1_l2_wlnl_shadow_nl_ffrt_min_ratio
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_nl_ffrt_min_ratio
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_nl_ffrt_min_ratio;
        auto& wlnl_nl_ffrt_passed = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_nl_ffrt_passed
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_nl_ffrt_passed
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_nl_ffrt_passed;
        auto& wlnl_candidate_pair_count = use_l2_wlnl
            ? debug_telemetry_
                  .lambda_l1_l2_wlnl_shadow_candidate_pair_count
            : use_l2_primary
                ? debug_telemetry_
                      .lambda_l2_l5_wlnl_shadow_candidate_pair_count
                : debug_telemetry_
                      .lambda_l1_l5_wlnl_shadow_candidate_pair_count;
        auto& wlnl_best_ecef_x = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_x
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_best_ecef_x
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_x;
        auto& wlnl_best_ecef_y = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_y
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_best_ecef_y
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_y;
        auto& wlnl_best_ecef_z = use_l2_wlnl
            ? debug_telemetry_.lambda_l1_l2_wlnl_shadow_best_ecef_z
            : use_l2_primary
                ? debug_telemetry_.lambda_l2_l5_wlnl_shadow_best_ecef_z
                : debug_telemetry_.lambda_l1_l5_wlnl_shadow_best_ecef_z;
        const auto has_secondary =
            [&](const SatelliteData& data) {
                return use_l2_wlnl ? data.has_l2 : data.has_l5;
            };
        const auto primary_wavelength =
            [&](const SatelliteData& data) {
                return use_l2_primary
                    ? data.l2_wavelength
                    : data.l1_wavelength;
            };
        const auto primary_frequency =
            [&](const SatelliteData& data) {
                return use_l2_primary
                    ? data.l2_frequency_hz
                    : data.l1_frequency_hz;
            };
        const auto primary_phase_sd_m =
            [&](const SatelliteData& data) {
                return use_l2_primary
                    ? (data.rover_l2_phase - data.base_l2_phase) *
                          data.l2_wavelength
                    : (data.rover_l1_phase - data.base_l1_phase) *
                          data.l1_wavelength;
            };
        const auto primary_code_sd =
            [&](const SatelliteData& data) {
                return use_l2_primary
                    ? data.rover_l2_code - data.base_l2_code
                    : data.rover_l1_code - data.base_l1_code;
            };
        const auto secondary_wavelength =
            [&](const SatelliteData& data) {
                return use_l2_wlnl
                    ? data.l2_wavelength
                    : data.l5_wavelength;
            };
        const auto secondary_frequency =
            [&](const SatelliteData& data) {
                return use_l2_wlnl
                    ? data.l2_frequency_hz
                    : data.l5_frequency_hz;
            };
        const auto secondary_phase_sd_m =
            [&](const SatelliteData& data) {
                return use_l2_wlnl
                    ? (data.rover_l2_phase - data.base_l2_phase) *
                          data.l2_wavelength
                    : (data.rover_l5_phase - data.base_l5_phase) *
                          data.l5_wavelength;
            };
        const auto secondary_code_sd =
            [&](const SatelliteData& data) {
                return use_l2_wlnl
                    ? data.rover_l2_code - data.base_l2_code
                    : data.rover_l5_code - data.base_l5_code;
            };
        wlnl_attempted = true;
        std::vector<int> l1_indices;
        std::vector<double> n5_float_values;
        std::vector<double> mw_values;
        std::vector<double> raw_mw_values;
        std::vector<double> causal_reference_variances;
        std::vector<double> causal_satellite_variances;
        std::vector<SatelliteId> causal_references;
        const Vector3d shadow_rover_position =
            base_position_ + base_head_state.head<3>();
        for (int i = 0; i < nb; ++i) {
            const auto& pair = dd_pairs[i];
            if (pair.freq != (use_l2_primary ? 1 : 0) ||
                pair.ref_sat.system == GNSSSystem::GLONASS) {
                continue;
            }
            const auto ref_it = sat_data.find(pair.ref_sat);
            const auto sat_it = sat_data.find(pair.sat);
            if (ref_it == sat_data.end() || sat_it == sat_data.end()) {
                continue;
            }
            const auto& ref_sd = ref_it->second;
            const auto& sd = sat_it->second;
            if (!has_secondary(ref_sd) || !has_secondary(sd) ||
                secondary_wavelength(ref_sd) <= 0.0 ||
                secondary_wavelength(sd) <= 0.0 ||
                std::abs(
                    secondary_wavelength(ref_sd) -
                    secondary_wavelength(sd)) > 1e-6) {
                continue;
            }
            const double rr_ref =
                geodist_range(ref_sd.sat_pos, shadow_rover_position) +
                tropModel(shadow_rover_position, ref_sd.elevation);
            const double br_ref =
                geodist_range(ref_sd.sat_pos_base, base_position_) +
                tropModel(base_position_, ref_sd.base_elevation);
            const double rr =
                geodist_range(sd.sat_pos, shadow_rover_position) +
                tropModel(shadow_rover_position, sd.elevation);
            const double br =
                geodist_range(sd.sat_pos_base, base_position_) +
                tropModel(base_position_, sd.base_elevation);
            const double geometry_dd = (rr_ref - br_ref) - (rr - br);
            const double phase_dd_m =
                secondary_phase_sd_m(ref_sd) -
                secondary_phase_sd_m(sd);
            const double n5_float =
                (phase_dd_m - geometry_dd) /
                secondary_wavelength(ref_sd);

            // Reuse the observation-domain MW implementation by pairing the
            // L1 entry with a synthetic index only for validation. The helper
            // requires an L5 DDPair, which is intentionally absent in shadow
            // mode, so calculate the identical L1/L5 MW expression directly.
            const double f1 = primary_frequency(ref_sd);
            const double f5 = secondary_frequency(ref_sd);
            const double lambda_wl_m = wideLaneWavelength(f1, f5);
            if (!std::isfinite(n5_float) || f1 <= 0.0 || f5 <= 0.0 ||
                lambda_wl_m <= 0.0) {
                continue;
            }
            auto single_difference_mw = [&](const SatelliteData& data) {
                const double phi1_m = primary_phase_sd_m(data);
                const double phi5_m =
                    secondary_phase_sd_m(data);
                const double code_term =
                    (f1 * primary_code_sd(data) +
                     f5 * secondary_code_sd(data)) /
                    (f1 + f5);
                return ((f1 * phi1_m - f5 * phi5_m) / (f1 - f5) -
                        code_term) /
                       lambda_wl_m;
            };
            const double reference_mw =
                single_difference_mw(ref_sd);
            const double satellite_mw =
                single_difference_mw(sd);
            const double mw = reference_mw - satellite_mw;
            if (!std::isfinite(mw)) {
                continue;
            }
            double validation_mw = mw;
            double causal_reference_variance =
                std::numeric_limits<double>::infinity();
            double causal_satellite_variance =
                std::numeric_limits<double>::infinity();
            if (rtk_config_
                    .lambda_l1_l5_wlnl_causal_arc_smoothing) {
                const bool satellite_slip =
                    (use_l2_primary
                         ? current_epoch_slips_l2_.count(pair.sat) > 0
                         : current_epoch_slips_l1_.count(pair.sat) > 0) ||
                    (use_l2_wlnl
                         ? current_epoch_slips_l2_.count(pair.sat) > 0
                         : current_epoch_slips_l5_.count(pair.sat) > 0);
                const bool reference_slip =
                    (use_l2_primary
                         ? current_epoch_slips_l2_.count(pair.ref_sat) > 0
                         : current_epoch_slips_l1_.count(pair.ref_sat) > 0) ||
                    (use_l2_wlnl
                         ? current_epoch_slips_l2_.count(pair.ref_sat) > 0
                         : current_epoch_slips_l5_.count(pair.ref_sat) > 0);
                const double time_s =
                    static_cast<double>(current_epoch_time_.week) *
                        604800.0 +
                    current_epoch_time_.tow;
                const auto satellite_arc =
                    l1_l5_mw_arc_bank_.updateSignal(
                        pair.sat,
                        use_l2_primary ? 25 : (use_l2_wlnl ? 12 : 15),
                        time_s, satellite_mw,
                        satellite_slip);
                const auto reference_arc =
                    l1_l5_mw_arc_bank_.updateSignal(
                        pair.ref_sat,
                        use_l2_primary ? 25 : (use_l2_wlnl ? 12 : 15),
                        time_s, reference_mw,
                        reference_slip);
                if (satellite_arc.reset) {
                    ++wlnl_causal_arc_resets;
                }
                if (reference_arc.reset) {
                    ++wlnl_causal_arc_resets;
                }
                if (satellite_arc.ready && reference_arc.ready) {
                    validation_mw =
                        reference_arc.smoothed_value -
                        satellite_arc.smoothed_value;
                    causal_reference_variance =
                        reference_arc.smoothed_value_variance;
                    causal_satellite_variance =
                        satellite_arc.smoothed_value_variance;
                    ++wlnl_causal_arc_ready_pairs;
                } else {
                    validation_mw =
                        std::numeric_limits<double>::quiet_NaN();
                }
            }
            l1_indices.push_back(i);
            n5_float_values.push_back(n5_float);
            mw_values.push_back(validation_mw);
            raw_mw_values.push_back(mw);
            causal_reference_variances.push_back(
                causal_reference_variance);
            causal_satellite_variances.push_back(
                causal_satellite_variance);
            causal_references.push_back(pair.ref_sat);
        }

        const int pair_count = static_cast<int>(l1_indices.size());
        wlnl_pair_count = pair_count;
        if (pair_count >= 4) {
            VectorXd wl_float(pair_count);
            MatrixXd wl_covariance(pair_count, pair_count);
            VectorXd model_wl_float(pair_count);
            MatrixXd model_wl_covariance(pair_count, pair_count);
            for (int row = 0; row < pair_count; ++row) {
                model_wl_float(row) =
                    base_dd_float(l1_indices[row]) - n5_float_values[row];
                const bool causal_observation =
                    rtk_config_
                        .lambda_l1_l5_wlnl_causal_arc_smoothing &&
                    std::isfinite(mw_values[row]) &&
                    std::isfinite(causal_reference_variances[row]) &&
                    std::isfinite(causal_satellite_variances[row]);
                wl_float(row) =
                    causal_observation
                        ? mw_values[row]
                        : base_dd_float(l1_indices[row]) -
                              n5_float_values[row];
                for (int column = 0; column < pair_count; ++column) {
                    model_wl_covariance(row, column) =
                        base_Qb(l1_indices[row], l1_indices[column]);
                    const bool causal_column =
                        rtk_config_
                            .lambda_l1_l5_wlnl_causal_arc_smoothing &&
                        std::isfinite(mw_values[column]) &&
                        std::isfinite(causal_reference_variances[column]) &&
                        std::isfinite(causal_satellite_variances[column]);
                    if (causal_observation && causal_column) {
                        wl_covariance(row, column) =
                            causal_references[row] ==
                                    causal_references[column]
                                ? std::max(
                                      1e-6,
                                      std::min(
                                          causal_reference_variances[row],
                                          causal_reference_variances[column]))
                                : 0.0;
                    } else {
                        wl_covariance(row, column) =
                            base_Qb(
                                l1_indices[row], l1_indices[column]);
                    }
                }
                if (causal_observation) {
                    wl_covariance(row, row) += std::max(
                        1e-6, causal_satellite_variances[row]);
                } else {
                    // Conservative independent L5 carrier/geometry
                    // uncertainty.
                    wl_covariance(row, row) += 0.0025;
                }
                // N1/N5-derived covariance retains the N1 cross-covariance
                // needed by the subsequent Gaussian conditioning.  The
                // causal MW covariance above is only for the independent WL
                // integer search.
                model_wl_covariance(row, row) += 0.0025;
            }
            wl_covariance =
                (wl_covariance + wl_covariance.transpose()) * 0.5;
            model_wl_covariance =
                (model_wl_covariance +
                 model_wl_covariance.transpose()) *
                0.5;

            LambdaCandidateDiagnostics wl_search;
            if (lambdaSearchTopK(wl_float, wl_covariance, 2, wl_search) &&
                wl_search.squared_residuals.size() >= 2 &&
                wl_search.candidates.cols() >= 1) {
                const double covariance_scale =
                    std::max(
                        1.0,
                        rtk_config_
                            .lambda_l1_l5_wlnl_shadow_covariance_scale);
                const double wl_bsr = bootstrappedSuccessRate(
                    wl_search.conditional_variances, covariance_scale);
                const double wl_ratio =
                    wl_search.squared_residuals(0) > 0.0
                        ? wl_search.squared_residuals(1) /
                              wl_search.squared_residuals(0)
                        : 0.0;
                wlnl_wl_bsr = wl_bsr;
                wlnl_wl_ratio = wl_ratio;
                FixedFailureRateRatioThreshold wl_ffrt;
                const bool wl_table_supported =
                    fixedFailureRateRatioThreshold(
                        pair_count, wl_bsr, 0.001, wl_ffrt);
                if (wl_table_supported) {
                    wlnl_wl_ffrt_min_ratio =
                        wl_ffrt.minimum_second_to_best_ratio;
                }
                int mw_disagreements = 0;
                int raw_mw_disagreements = 0;
                for (int row = 0; row < pair_count; ++row) {
                    const double mw = mw_values[row];
                    const double raw_mw = raw_mw_values[row];
                    const bool causal_validation =
                        rtk_config_
                            .lambda_l1_l5_wlnl_causal_arc_smoothing;
                    if (!std::isfinite(mw) ||
                        (!causal_validation &&
                         distanceToNearestInteger(mw) >= 0.25) ||
                        std::abs(
                            wl_search.candidates(row, 0) -
                            std::round(mw)) > 0.5) {
                        ++mw_disagreements;
                    }
                    if (distanceToNearestInteger(raw_mw) >= 0.25 ||
                        std::abs(
                            wl_search.candidates(row, 0) -
                            std::round(raw_mw)) > 0.5) {
                        ++raw_mw_disagreements;
                    }
                }
                wlnl_mw_disagreements = mw_disagreements;
                wlnl_raw_mw_disagreements = raw_mw_disagreements;
                const bool wl_passed =
                    wl_table_supported && wl_ffrt.accepts_any_candidate &&
                    std::isfinite(wl_ratio) &&
                    wl_ratio >= wl_ffrt.minimum_second_to_best_ratio &&
                    mw_disagreements == 0;
                wlnl_wl_ffrt_passed = wl_passed;

                if (wl_passed) {
                    const auto nl_problem =
                        rtk_ar_evaluation::extractSubset(
                            base_dd_float, base_Qb, base_Qab, l1_indices);
                    VectorXd conditioned_head_state;
                    MatrixXd conditioned_Qab;
                    VectorXd conditioned_n1_float;
                    MatrixXd conditioned_Qb;
                    const bool conditioned =
                        rtk_ar_selection::
                            conditionNarrowLaneOnFixedWideLane(
                                base_head_state,
                                nl_problem.Qab,
                                nl_problem.dd_float,
                                nl_problem.Qb,
                                model_wl_float,
                                model_wl_covariance,
                                wl_search.candidates.col(0),
                                conditioned_head_state,
                                conditioned_Qab,
                                conditioned_n1_float,
                                conditioned_Qb);
                    LambdaCandidateDiagnostics nl_search;
                    if (conditioned &&
                        lambdaSearchTopK(
                            conditioned_n1_float, conditioned_Qb, 2,
                            nl_search) &&
                        nl_search.squared_residuals.size() >= 2 &&
                        nl_search.candidates.cols() >= 1) {
                        const double nl_bsr = bootstrappedSuccessRate(
                            nl_search.conditional_variances,
                            covariance_scale);
                        const double nl_ratio =
                            nl_search.squared_residuals(0) > 0.0
                                ? nl_search.squared_residuals(1) /
                                      nl_search.squared_residuals(0)
                                : 0.0;
                        wlnl_nl_bsr = nl_bsr;
                        wlnl_nl_ratio = nl_ratio;
                        FixedFailureRateRatioThreshold nl_ffrt;
                        const bool nl_table_supported =
                            fixedFailureRateRatioThreshold(
                                pair_count, nl_bsr, 0.001, nl_ffrt);
                        if (nl_table_supported) {
                            wlnl_nl_ffrt_min_ratio =
                                nl_ffrt.minimum_second_to_best_ratio;
                        }
                        const bool nl_passed =
                            nl_table_supported &&
                            nl_ffrt.accepts_any_candidate &&
                            std::isfinite(nl_ratio) &&
                            nl_ratio >=
                                nl_ffrt.minimum_second_to_best_ratio;
                        wlnl_nl_ffrt_passed = nl_passed;
                        if (nl_passed) {
                            wlnl_candidate_pair_count = pair_count;
                        }
                        VectorXd best_head;
                        if (nl_passed &&
                            rtk_ar_evaluation::solveFixedHeadState(
                                conditioned_head_state, conditioned_Qab,
                                conditioned_Qb, conditioned_n1_float,
                                nl_search.candidates.col(0), best_head) &&
                            best_head.size() >= 3) {
                            const Vector3d ecef =
                                base_position_ + best_head.head<3>();
                            wlnl_best_ecef_x = ecef.x();
                            wlnl_best_ecef_y = ecef.y();
                            wlnl_best_ecef_z = ecef.z();
                        }
                    }
                }
            }
        }
        }
        debug_telemetry_.lambda_l1_l5_wlnl_shadow_runtime_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - wlnl_started)
                .count();
    }

    if (rtk_config_.enable_wide_lane_ar) {
        const double wide_lane_threshold =
            std::max(0.0, rtk_config_.wide_lane_acceptance_threshold);
        for (int i = 0; i < nb; ++i) {
            if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) {
                continue;
            }
            int l2_pair = -1;
            for (int j = 0; j < nb; ++j) {
                if (dd_pairs[j].freq == 1 &&
                    dd_pairs[j].sat == dd_pairs[i].sat &&
                    dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) {
                    l2_pair = j;
                    break;
                }
            }
            if (l2_pair < 0) {
                continue;
            }

            wide_lane_total++;
            double wide_lane_float = 0.0;
            if (!compute_wide_lane_float(i, l2_pair, wide_lane_float)) {
                continue;
            }
            const double fixed_integer = std::round(wide_lane_float);
            const double wl_distance = distanceToNearestInteger(wide_lane_float);
            wide_lane_min_distance = std::min(wide_lane_min_distance, wl_distance);
            wide_lane_max_distance = std::max(wide_lane_max_distance, wl_distance);
            if (wl_distance >= wide_lane_threshold) {
                wide_lane_rejected++;
                continue;
            }

            wide_lane_constraints.push_back({i, l2_pair, fixed_integer});
            wide_lane_fixed++;
        }
        // Phase 18 Step 6: parallel L1-L5 wide-lane fixing for sats lacking an L1-L2
        // pair (or in addition to it). Constraint structure is identical — l2_index
        // simply now points at a freq=2 dd_pair instead of freq=1.
        if (rtk_config_.enable_l5) {
            for (int i = 0; i < nb; ++i) {
                if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) {
                    continue;
                }
                int l5_pair = -1;
                for (int j = 0; j < nb; ++j) {
                    if (dd_pairs[j].freq == 2 &&
                        dd_pairs[j].sat == dd_pairs[i].sat &&
                        dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) {
                        l5_pair = j;
                        break;
                    }
                }
                if (l5_pair < 0) {
                    continue;
                }

                wide_lane_total++;
                double wide_lane_float = 0.0;
                if (!compute_wide_lane_l5_float(i, l5_pair, wide_lane_float)) {
                    continue;
                }
                const double fixed_integer = std::round(wide_lane_float);
                const double wl_distance = distanceToNearestInteger(wide_lane_float);
                wide_lane_min_distance = std::min(wide_lane_min_distance, wl_distance);
                wide_lane_max_distance = std::max(wide_lane_max_distance, wl_distance);
                if (wl_distance >= wide_lane_threshold) {
                    wide_lane_rejected++;
                    continue;
                }

                wide_lane_constraints.push_back({i, l5_pair, fixed_integer});
                wide_lane_fixed++;
            }
        }
        if (wide_lane_total > 0) {
            std::clog << "[RTK-AR] WL fixed " << wide_lane_fixed
                      << "/" << wide_lane_total << "\n";
        }
    }
    debug_telemetry_.wide_lane_total = wide_lane_total;
    debug_telemetry_.wide_lane_fixed = wide_lane_fixed;
    debug_telemetry_.wide_lane_rejected = wide_lane_rejected;
    if (std::isfinite(wide_lane_min_distance)) {
        debug_telemetry_.wide_lane_min_distance = wide_lane_min_distance;
        debug_telemetry_.wide_lane_max_distance = wide_lane_max_distance;
    }

    struct SearchProblem {
        VectorXd head_state;
        VectorXd dd_float;
        MatrixXd Qb;
        MatrixXd Qab;
    };
    auto build_search_problem = [&](const std::vector<int>& subset) {
        SearchProblem problem;
        problem.head_state = base_head_state;
        if (subset.size() == static_cast<size_t>(nb)) {
            problem.dd_float = base_dd_float;
            problem.Qb = base_Qb;
            problem.Qab = base_Qab;
        } else {
            const auto subset_matrices =
                rtk_ar_evaluation::extractSubset(base_dd_float, base_Qb, base_Qab, subset);
            problem.dd_float = subset_matrices.dd_float;
            problem.Qb = subset_matrices.Qb;
            problem.Qab = subset_matrices.Qab;
        }

        if (!wide_lane_constraints.empty()) {
            std::map<int, int> local_index_by_full_index;
            for (int local = 0; local < static_cast<int>(subset.size()); ++local) {
                local_index_by_full_index[subset[local]] = local;
            }

            for (const auto& constraint : wide_lane_constraints) {
                const auto l1_it = local_index_by_full_index.find(constraint.l1_index);
                const auto l2_it = local_index_by_full_index.find(constraint.l2_index);
                if (l1_it == local_index_by_full_index.end() ||
                    l2_it == local_index_by_full_index.end()) {
                    continue;
                }
                applyAmbiguityConstraintUpdate(problem.head_state,
                                               problem.dd_float,
                                               problem.Qb,
                                               problem.Qab,
                                               l1_it->second,
                                               l2_it->second,
                                               constraint.fixed_integer,
                                               1e-4);
            }
        }

        if (wide_lane_constraints.empty()) {
            return problem;
        }

        problem.Qb = (problem.Qb + problem.Qb.transpose()) * 0.5;
        for (int i = 0; i < problem.Qb.rows(); ++i) {
            if (problem.Qb(i, i) < 1e-6) {
                problem.Qb(i, i) = 1e-6;
            }
        }
        return problem;
    };

    // Standard LAMBDA path
    if (!fixed) {
        const auto full_problem = build_search_problem(full_subset);
        const bool full_solved =
            lambdaMethod(full_problem.dd_float, full_problem.Qb, dd_fixed, ratio);
        debug_telemetry_.full_lambda_solved = full_solved;
        const int shadow_count = rtk_config_.lambda_candidate_shadow_count;
        if (shadow_count > 0) {
            debug_telemetry_.lambda_shadow_attempted = true;
            const auto shadow_started = std::chrono::steady_clock::now();
            LambdaCandidateDiagnostics shadow;
            if (lambdaSearchTopK(
                    full_problem.dd_float, full_problem.Qb, shadow_count, shadow)) {
                debug_telemetry_.lambda_shadow_solved = true;
                debug_telemetry_.lambda_shadow_candidate_count =
                    static_cast<int>(shadow.squared_residuals.size());
                debug_telemetry_.lambda_shadow_bsr =
                    shadow.bootstrapped_success_rate;
                debug_telemetry_.lambda_shadow_bsr_qscale2 =
                    bootstrappedSuccessRate(
                        shadow.conditional_variances, 2.0);
                debug_telemetry_.lambda_shadow_bsr_qscale4 =
                    bootstrappedSuccessRate(
                        shadow.conditional_variances, 4.0);
                debug_telemetry_.lambda_shadow_bsr_qscale8 =
                    bootstrappedSuccessRate(
                        shadow.conditional_variances, 8.0);
                debug_telemetry_.lambda_shadow_bsr_qscale16 =
                    bootstrappedSuccessRate(
                        shadow.conditional_variances, 16.0);
                FixedFailureRateRatioThreshold ffrt;
                if (fixedFailureRateRatioThreshold(
                        static_cast<int>(full_problem.dd_float.size()),
                        shadow.bootstrapped_success_rate, 0.001, ffrt)) {
                    debug_telemetry_.lambda_shadow_ffrt_table_supported = true;
                    debug_telemetry_.lambda_shadow_ffrt_accepts_any =
                        ffrt.accepts_any_candidate;
                    debug_telemetry_.lambda_shadow_ffrt_min_ratio =
                        ffrt.minimum_second_to_best_ratio;
                }
                if (shadow.squared_residuals.size() >= 1) {
                    const double best_cost = shadow.squared_residuals(0);
                    debug_telemetry_.lambda_shadow_best_cost = best_cost;
                    const int logged_candidates = std::min(
                        8, static_cast<int>(
                               shadow.squared_residuals.size()));
                    for (int candidate = 0;
                         candidate < logged_candidates; ++candidate) {
                        debug_telemetry_.lambda_shadow_candidate_costs(
                            candidate) =
                            shadow.squared_residuals(candidate);
                    }
                    double weight_sum = 0.0;
                    double squared_weight_sum = 0.0;
                    for (int i = 0; i < shadow.squared_residuals.size(); ++i) {
                        const double weight = std::exp(
                            -0.5 * (shadow.squared_residuals(i) - best_cost));
                        weight_sum += weight;
                        squared_weight_sum += weight * weight;
                    }
                    if (weight_sum > 0.0) {
                        debug_telemetry_.lambda_shadow_best_mass =
                            1.0 / weight_sum;
                        debug_telemetry_.lambda_shadow_effective_candidates =
                            weight_sum * weight_sum / squared_weight_sum;
                    }
                }
                if (shadow.squared_residuals.size() >= 2) {
                    debug_telemetry_.lambda_shadow_second_cost =
                        shadow.squared_residuals(1);
                    debug_telemetry_.lambda_shadow_best_second_disagreements =
                        static_cast<int>(
                            (shadow.candidates.col(0).array() !=
                             shadow.candidates.col(1).array()).count());
                    const double shadow_ratio =
                        shadow.squared_residuals(0) > 0.0
                            ? shadow.squared_residuals(1) /
                                  shadow.squared_residuals(0)
                            : 0.0;
                    debug_telemetry_.lambda_shadow_ffrt_passed =
                        debug_telemetry_.lambda_shadow_ffrt_accepts_any &&
                        std::isfinite(shadow_ratio) &&
                        shadow_ratio >=
                            debug_telemetry_.lambda_shadow_ffrt_min_ratio;
                }

                VectorXd best_head;
                if (shadow.candidates.cols() >= 1 &&
                    rtk_ar_evaluation::solveFixedHeadState(
                        full_problem.head_state, full_problem.Qab,
                        full_problem.Qb, full_problem.dd_float,
                        shadow.candidates.col(0), best_head) &&
                    best_head.size() >= 3) {
                    const Vector3d best_ecef =
                        base_position_ + best_head.head<3>();
                    debug_telemetry_.lambda_shadow_best_ecef_x = best_ecef.x();
                    debug_telemetry_.lambda_shadow_best_ecef_y = best_ecef.y();
                    debug_telemetry_.lambda_shadow_best_ecef_z = best_ecef.z();
                    debug_telemetry_.lambda_shadow_candidate_ecef_m.col(0) =
                        best_ecef;
                    const Vector3d best_correction =
                        best_head.head<3>() -
                        full_problem.head_state.head<3>();
                    debug_telemetry_.lambda_shadow_best_correction_x =
                        best_correction.x();
                    debug_telemetry_.lambda_shadow_best_correction_y =
                        best_correction.y();
                    debug_telemetry_.lambda_shadow_best_correction_z =
                        best_correction.z();
                    double max_spread_m = 0.0;
                    for (int candidate = 1;
                         candidate < shadow.candidates.cols(); ++candidate) {
                        VectorXd candidate_head;
                        if (!rtk_ar_evaluation::solveFixedHeadState(
                                full_problem.head_state, full_problem.Qab,
                                full_problem.Qb, full_problem.dd_float,
                                shadow.candidates.col(candidate),
                                candidate_head) ||
                            candidate_head.size() < 3) {
                            continue;
                        }
                        const double spread_m =
                            (candidate_head.head<3>() -
                             best_head.head<3>()).norm();
                        debug_telemetry_.lambda_shadow_candidate_ecef_m
                            .col(candidate) =
                            base_position_ + candidate_head.head<3>();
                        max_spread_m = std::max(max_spread_m, spread_m);
                        if (candidate == 1) {
                            debug_telemetry_
                                .lambda_shadow_second_position_delta_m =
                                spread_m;
                            const Vector3d second_ecef =
                                base_position_ +
                                candidate_head.head<3>();
                            debug_telemetry_.lambda_shadow_second_ecef_x =
                                second_ecef.x();
                            debug_telemetry_.lambda_shadow_second_ecef_y =
                                second_ecef.y();
                            debug_telemetry_.lambda_shadow_second_ecef_z =
                                second_ecef.z();
                            const Vector3d second_correction =
                                candidate_head.head<3>() -
                                full_problem.head_state.head<3>();
                            debug_telemetry_
                                .lambda_shadow_second_correction_x =
                                second_correction.x();
                            debug_telemetry_
                                .lambda_shadow_second_correction_y =
                                second_correction.y();
                            debug_telemetry_
                                .lambda_shadow_second_correction_z =
                                second_correction.z();
                        }
                    }
                    debug_telemetry_.lambda_shadow_position_spread_max_m =
                        max_spread_m;
                }

                if (rtk_config_.lambda_causal_arc_readiness_shadow &&
                    causal_arc_ready_subset.size() >= 4) {
                    std::vector<int> causal_arc_search_subset =
                        causal_arc_ready_subset;
                    if (rtk_config_.lambda_causal_arc_smoothed_search &&
                        rtk_config_
                                .lambda_causal_arc_smoothed_max_pairs >
                            0 &&
                        static_cast<int>(
                            causal_arc_search_subset.size()) >
                            rtk_config_
                                .lambda_causal_arc_smoothed_max_pairs) {
                        std::stable_sort(
                            causal_arc_search_subset.begin(),
                            causal_arc_search_subset.end(),
                            [&](int left, int right) {
                                const double left_variance =
                                    causal_arc_satellite_variance[left] +
                                    causal_arc_reference_variance[left];
                                const double right_variance =
                                    causal_arc_satellite_variance[right] +
                                    causal_arc_reference_variance[right];
                                if (left_variance != right_variance) {
                                    return left_variance < right_variance;
                                }
                                return left < right;
                            });
                        causal_arc_search_subset.resize(
                            rtk_config_
                                .lambda_causal_arc_smoothed_max_pairs);
                        std::sort(
                            causal_arc_search_subset.begin(),
                            causal_arc_search_subset.end());
                    }
                    debug_telemetry_
                        .lambda_causal_arc_subset_pair_count =
                        static_cast<int>(
                            causal_arc_search_subset.size());
                    const auto arc_problem =
                        build_search_problem(causal_arc_search_subset);
                    VectorXd arc_search_float = arc_problem.dd_float;
                    MatrixXd arc_search_covariance = arc_problem.Qb;
                    if (rtk_config_.lambda_causal_arc_smoothed_search) {
                        const int arc_count = static_cast<int>(
                            causal_arc_search_subset.size());
                        arc_search_float.resize(arc_count);
                        arc_search_covariance =
                            MatrixXd::Zero(arc_count, arc_count);
                        const auto shared_variance =
                            [](double left, double right) {
                                return std::max(
                                    1e-6, std::min(left, right));
                            };
                        for (int row = 0; row < arc_count; ++row) {
                            const int source_row =
                                causal_arc_search_subset[row];
                            arc_search_float(row) =
                                causal_arc_smoothed_dd[source_row];
                            const auto& row_pair =
                                dd_pairs[source_row];
                            for (int column = 0;
                                 column < arc_count; ++column) {
                                const int source_column =
                                    causal_arc_search_subset[column];
                                const auto& column_pair =
                                    dd_pairs[source_column];
                                if (row_pair.freq != column_pair.freq) {
                                    continue;
                                }
                                double covariance = 0.0;
                                if (row_pair.ref_sat ==
                                    column_pair.ref_sat) {
                                    covariance += shared_variance(
                                        causal_arc_reference_variance[
                                            source_row],
                                        causal_arc_reference_variance[
                                            source_column]);
                                }
                                if (row_pair.ref_sat ==
                                    column_pair.sat) {
                                    covariance -= shared_variance(
                                        causal_arc_reference_variance[
                                            source_row],
                                        causal_arc_satellite_variance[
                                            source_column]);
                                }
                                if (row_pair.sat ==
                                    column_pair.ref_sat) {
                                    covariance -= shared_variance(
                                        causal_arc_satellite_variance[
                                            source_row],
                                        causal_arc_reference_variance[
                                            source_column]);
                                }
                                if (row_pair.sat ==
                                    column_pair.sat) {
                                    covariance += shared_variance(
                                        causal_arc_satellite_variance[
                                            source_row],
                                        causal_arc_satellite_variance[
                                            source_column]);
                                }
                                arc_search_covariance(row, column) =
                                    covariance;
                            }
                            arc_search_covariance(row, row) =
                                std::max(
                                    1e-6,
                                    arc_search_covariance(row, row));
                        }
                        arc_search_covariance =
                            (arc_search_covariance +
                             arc_search_covariance.transpose()) *
                            0.5;
                        // Distinct DD rows can momentarily describe the same
                        // signal relation.  Keep the empirical shared-signal
                        // covariance conservative and strictly positive
                        // definite instead of allowing an exact, overconfident
                        // zero-noise relation to enter LAMBDA/FFRT.
                        arc_search_covariance.diagonal().array() += 1e-4;
                        for (int row = 0;
                             row < arc_search_covariance.rows(); ++row) {
                            double off_diagonal_sum = 0.0;
                            for (int column = 0;
                                 column < arc_search_covariance.cols();
                                 ++column) {
                                if (column != row) {
                                    off_diagonal_sum += std::abs(
                                        arc_search_covariance(row, column));
                                }
                            }
                            arc_search_covariance(row, row) =
                                std::max(
                                    arc_search_covariance(row, row),
                                    off_diagonal_sum + 1e-4);
                        }
                    }
                    LambdaCandidateDiagnostics arc_shadow;
                    if (lambdaSearchTopK(
                            arc_search_float, arc_search_covariance, 2,
                            arc_shadow) &&
                        arc_shadow.squared_residuals.size() >= 2) {
                        debug_telemetry_
                            .lambda_causal_arc_subset_solved = true;
                        const double arc_ratio =
                            arc_shadow.squared_residuals(0) > 0.0
                                ? arc_shadow.squared_residuals(1) /
                                      arc_shadow.squared_residuals(0)
                                : 0.0;
                        debug_telemetry_
                            .lambda_causal_arc_subset_ratio = arc_ratio;
                        VectorXd arc_ffrt_variances =
                            arc_shadow.conditional_variances;
                        if (arc_ffrt_variances.array()
                                .isFinite()
                                .all()) {
                            arc_ffrt_variances =
                                arc_ffrt_variances
                                    .array()
                                    .max(1e-4)
                                    .matrix();
                            debug_telemetry_
                                .lambda_causal_arc_subset_variance_min =
                                arc_ffrt_variances.minCoeff();
                            debug_telemetry_
                                .lambda_causal_arc_subset_variance_max =
                                arc_ffrt_variances.maxCoeff();
                        }
                        const double arc_bsr =
                            bootstrappedSuccessRate(
                                arc_ffrt_variances,
                                rtk_config_
                                    .lambda_causal_arc_readiness_covariance_scale);
                        debug_telemetry_.lambda_causal_arc_subset_bsr =
                            arc_bsr;
                        FixedFailureRateRatioThreshold arc_ffrt;
                        if (fixedFailureRateRatioThreshold(
                                static_cast<int>(
                                    causal_arc_search_subset.size()),
                                arc_bsr, 0.001, arc_ffrt)) {
                            debug_telemetry_
                                .lambda_causal_arc_subset_ffrt_min_ratio =
                                arc_ffrt.minimum_second_to_best_ratio;
                            debug_telemetry_
                                .lambda_causal_arc_subset_ffrt_passed =
                                arc_ffrt.accepts_any_candidate &&
                                std::isfinite(arc_ratio) &&
                                arc_ratio >=
                                    arc_ffrt.minimum_second_to_best_ratio;
                        }
                        VectorXd arc_best_head;
                        VectorXd arc_second_head;
                        const bool arc_best_solved =
                            rtk_ar_evaluation::solveFixedHeadState(
                                arc_problem.head_state, arc_problem.Qab,
                                arc_problem.Qb, arc_problem.dd_float,
                                arc_shadow.candidates.col(0),
                                arc_best_head);
                        const bool arc_second_solved =
                            rtk_ar_evaluation::solveFixedHeadState(
                                arc_problem.head_state, arc_problem.Qab,
                                arc_problem.Qb, arc_problem.dd_float,
                                arc_shadow.candidates.col(1),
                                arc_second_head);
                        if (arc_best_solved &&
                            arc_best_head.size() >= 3) {
                            const Vector3d arc_best_ecef =
                                base_position_ +
                                arc_best_head.head<3>();
                            debug_telemetry_
                                .lambda_causal_arc_subset_best_ecef_x =
                                arc_best_ecef.x();
                            debug_telemetry_
                                .lambda_causal_arc_subset_best_ecef_y =
                                arc_best_ecef.y();
                            debug_telemetry_
                                .lambda_causal_arc_subset_best_ecef_z =
                                arc_best_ecef.z();
                            const auto& external_disjoint =
                                external_disjoint_satellite_fix_evidence_;
                            if (external_disjoint
                                    .partition_a_candidate_ecef
                                    .allFinite()) {
                                debug_telemetry_
                                    .lambda_causal_arc_subset_partition_a_separation_m =
                                    (arc_best_ecef -
                                     external_disjoint
                                         .partition_a_candidate_ecef)
                                        .norm();
                            }
                            if (external_disjoint
                                    .partition_b_candidate_ecef
                                    .allFinite()) {
                                debug_telemetry_
                                    .lambda_causal_arc_subset_partition_b_separation_m =
                                    (arc_best_ecef -
                                     external_disjoint
                                         .partition_b_candidate_ecef)
                                        .norm();
                            }
                        }
                        if (arc_best_solved && arc_second_solved &&
                            arc_best_head.size() >= 3 &&
                            arc_second_head.size() >= 3) {
                            debug_telemetry_
                                .lambda_causal_arc_subset_second_position_delta_m =
                                (arc_best_head.head<3>() -
                                 arc_second_head.head<3>())
                                    .norm();
                        }
                    }
                }

                const double src_threshold =
                    rtk_config_.lambda_src_par_shadow_success_rate;
                if (src_threshold > 0.0) {
                    debug_telemetry_.lambda_src_par_shadow_attempted = true;
                    const auto src_started =
                        std::chrono::steady_clock::now();
                    const double covariance_scale =
                        rtk_config_.lambda_src_par_shadow_covariance_scale;
                    const int subset_size = successRateCriterionSubsetSize(
                        shadow.conditional_variances, covariance_scale,
                        src_threshold);
                    debug_telemetry_.lambda_src_par_shadow_subset_size =
                        subset_size;
                    if (subset_size >= 4) {
                        const VectorXd src_float =
                            shadow.decorrelated_float.tail(subset_size);
                        const MatrixXd src_covariance =
                            shadow.decorrelated_covariance.bottomRightCorner(
                                subset_size, subset_size);
                        LambdaCandidateDiagnostics src;
                        if (lambdaSearchTopK(
                                src_float, src_covariance, 2, src)) {
                            debug_telemetry_.lambda_src_par_shadow_solved =
                                true;
                            const double src_bsr =
                                bootstrappedSuccessRate(
                                    src.conditional_variances,
                                    covariance_scale);
                            debug_telemetry_.lambda_src_par_shadow_bsr =
                                src_bsr;
                            const double src_ratio =
                                src.squared_residuals(0) > 0.0
                                    ? src.squared_residuals(1) /
                                          src.squared_residuals(0)
                                    : 0.0;
                            debug_telemetry_.lambda_src_par_shadow_ratio =
                                src_ratio;
                            FixedFailureRateRatioThreshold src_ffrt;
                            if (fixedFailureRateRatioThreshold(
                                    subset_size, src_bsr, 0.001,
                                    src_ffrt)) {
                                debug_telemetry_
                                    .lambda_src_par_shadow_ffrt_min_ratio =
                                    src_ffrt
                                        .minimum_second_to_best_ratio;
                                debug_telemetry_
                                    .lambda_src_par_shadow_ffrt_passed =
                                    src_ffrt.accepts_any_candidate &&
                                    std::isfinite(src_ratio) &&
                                    src_ratio >=
                                        src_ffrt
                                            .minimum_second_to_best_ratio;
                            }

                            const MatrixXd head_z_covariance =
                                full_problem.Qab *
                                shadow.decorrelation_transform;
                            const MatrixXd src_head_covariance =
                                head_z_covariance.rightCols(subset_size);
                            VectorXd src_best_head;
                            VectorXd src_second_head;
                            const bool best_solved =
                                rtk_ar_evaluation::solveFixedHeadState(
                                    full_problem.head_state,
                                    src_head_covariance, src_covariance,
                                    src_float, src.candidates.col(0),
                                    src_best_head);
                            const bool second_solved =
                                rtk_ar_evaluation::solveFixedHeadState(
                                    full_problem.head_state,
                                    src_head_covariance, src_covariance,
                                    src_float, src.candidates.col(1),
                                    src_second_head);
                            if (best_solved && src_best_head.size() >= 3) {
                                const Vector3d src_best_ecef =
                                    base_position_ +
                                    src_best_head.head<3>();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_ecef_x =
                                    src_best_ecef.x();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_ecef_y =
                                    src_best_ecef.y();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_ecef_z =
                                    src_best_ecef.z();
                                const Vector3d src_best_correction =
                                    src_best_head.head<3>() -
                                    full_problem.head_state.head<3>();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_correction_x =
                                    src_best_correction.x();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_correction_y =
                                    src_best_correction.y();
                                debug_telemetry_
                                    .lambda_src_par_shadow_best_correction_z =
                                    src_best_correction.z();
                            }
                            if (best_solved && second_solved &&
                                src_best_head.size() >= 3 &&
                                src_second_head.size() >= 3) {
                                debug_telemetry_
                                    .lambda_src_par_shadow_second_position_delta_m =
                                    (src_second_head.head<3>() -
                                     src_best_head.head<3>()).norm();
                            }
                        }
                    }
                    debug_telemetry_.lambda_src_par_shadow_runtime_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            src_started).count();
                }

                const int satellite_par_max_drops =
                    rtk_config_
                        .lambda_satellite_par_shadow_max_drop_steps;
                if (satellite_par_max_drops > 0 &&
                    (!rtk_config_
                          .lambda_satellite_par_only_after_full_ffrt_failure ||
                     !debug_telemetry_.lambda_shadow_ffrt_passed)) {
                    debug_telemetry_
                        .lambda_satellite_par_shadow_attempted = true;
                    const auto satellite_par_started =
                        std::chrono::steady_clock::now();
                    std::vector<rtk_ar_selection::PairDescriptor>
                        satellite_descriptors;
                    satellite_descriptors.reserve(nb);
                    auto pair_snr = [&](const DDPair& pair) {
                        const auto ref_it = sat_data.find(pair.ref_sat);
                        const auto sat_it = sat_data.find(pair.sat);
                        if (ref_it == sat_data.end() ||
                            sat_it == sat_data.end()) {
                            return std::numeric_limits<double>::quiet_NaN();
                        }
                        const auto& ref = ref_it->second;
                        const auto& sat = sat_it->second;
                        if (pair.freq == 0) {
                            return std::min(
                                {ref.rover_l1_snr, ref.base_l1_snr,
                                 sat.rover_l1_snr, sat.base_l1_snr});
                        }
                        if (pair.freq == 1) {
                            return std::min(
                                {ref.rover_l2_snr, ref.base_l2_snr,
                                 sat.rover_l2_snr, sat.base_l2_snr});
                        }
                        if (pair.freq == 2) {
                            return std::min(
                                {ref.rover_l5_snr, ref.base_l5_snr,
                                 sat.rover_l5_snr, sat.base_l5_snr});
                        }
                        return std::numeric_limits<double>::quiet_NaN();
                    };
                    for (int index = 0; index < nb; ++index) {
                        double elevation =
                            std::numeric_limits<double>::quiet_NaN();
                        double wavelength =
                            std::numeric_limits<double>::quiet_NaN();
                        double azimuth =
                            std::numeric_limits<double>::quiet_NaN();
                        const auto sat_it =
                            sat_data.find(dd_pairs[index].sat);
                        if (sat_it != sat_data.end()) {
                            elevation = sat_it->second.elevation;
                            if (dd_pairs[index].freq == 0) {
                                wavelength =
                                    sat_it->second.l1_wavelength;
                            } else if (dd_pairs[index].freq == 1) {
                                wavelength =
                                    sat_it->second.l2_wavelength;
                            } else if (dd_pairs[index].freq == 2) {
                                wavelength =
                                    sat_it->second.l5_wavelength;
                            }
                            if (full_problem.head_state.size() >= 3) {
                                const Vector3d rover_position =
                                    base_position_ +
                                    full_problem.head_state.head<3>();
                                const auto geodetic =
                                    spp_utils::ecefToGeodetic(
                                        rover_position);
                                const Vector3d los_enu = ecef2enu(
                                    sat_it->second.sat_pos -
                                        rover_position,
                                    geodetic.latitude,
                                    geodetic.longitude);
                                azimuth =
                                    std::atan2(los_enu.x(), los_enu.y());
                                if (azimuth < 0.0) {
                                    azimuth += 2.0 * M_PI;
                                }
                            }
                        }
                        satellite_descriptors.push_back(
                            {dd_pairs[index].sat.system,
                             full_problem.Qb(index, index),
                             dd_pairs[index].sat,
                             elevation,
                             pair_snr(dd_pairs[index]),
                             distanceToNearestInteger(
                                 full_problem.dd_float(index))});
                        auto& descriptor =
                            satellite_descriptors.back();
                        if (std::isfinite(wavelength) &&
                            wavelength > 0.0) {
                            descriptor.posterior_abs_residual_m =
                                wavelength *
                                descriptor
                                    .fractional_distance_cycles;
                        }
                        descriptor.azimuth_rad = azimuth;
                    }
                    auto satellite_subsets =
                        rtk_config_
                                .lambda_satellite_par_shadow_quality_diverse
                            ? rtk_ar_selection::
                                  buildSatelliteQualityDiverseDropSubsets(
                                      satellite_descriptors,
                                      min_subset_pairs_for_ar,
                                      satellite_par_max_drops,
                                      32)
                            : rtk_ar_selection::
                                  buildSatelliteQualityDropSubsets(
                                      satellite_descriptors,
                                  min_subset_pairs_for_ar,
                                  satellite_par_max_drops);
                    std::vector<int> persistent_subset;
                    if (rtk_config_
                            .lambda_satellite_par_persistent_subset &&
                        !satellite_par_persistent_satellites_.empty()) {
                        debug_telemetry_
                            .lambda_satellite_par_persistent_subset_attempted =
                            true;
                        bool selected_signal_slipped = false;
                        for (const auto& satellite :
                             satellite_par_persistent_satellites_) {
                            selected_signal_slipped =
                                selected_signal_slipped ||
                                current_epoch_slips_l1_.count(satellite) > 0 ||
                                current_epoch_slips_l2_.count(satellite) > 0 ||
                                current_epoch_slips_l5_.count(satellite) > 0;
                        }
                        if (!selected_signal_slipped) {
                            for (int index = 0; index < nb; ++index) {
                                if (satellite_par_persistent_satellites_
                                        .count(dd_pairs[index].sat) > 0) {
                                    persistent_subset.push_back(index);
                                }
                            }
                        } else {
                            satellite_par_persistent_satellites_.clear();
                        }
                        if (static_cast<int>(persistent_subset.size()) >=
                            min_subset_pairs_for_ar) {
                            const bool already_first =
                                !satellite_subsets.empty() &&
                                satellite_subsets.front() ==
                                    persistent_subset;
                            if (!already_first) {
                                satellite_subsets.insert(
                                    satellite_subsets.begin(),
                                    persistent_subset);
                            }
                        }
                    }
                    std::set<SatelliteId> full_satellites;
                    for (const auto& pair : dd_pairs) {
                        full_satellites.insert(pair.sat);
                    }
                    for (const auto& subset : satellite_subsets) {
                        debug_telemetry_
                            .lambda_satellite_par_shadow_subsets_evaluated++;
                        const auto satellite_problem =
                            build_search_problem(subset);
                        LambdaCandidateDiagnostics satellite_candidate;
                        if (!lambdaSearchTopK(
                                satellite_problem.dd_float,
                                satellite_problem.Qb, 2,
                                satellite_candidate)) {
                            continue;
                        }
                        debug_telemetry_
                            .lambda_satellite_par_shadow_solved = true;
                        const double covariance_scale =
                            rtk_config_
                                .lambda_satellite_par_shadow_covariance_scale;
                        const double satellite_bsr =
                            bootstrappedSuccessRate(
                                satellite_candidate.conditional_variances,
                                covariance_scale);
                        const double satellite_ratio =
                            satellite_candidate.squared_residuals(0) > 0.0
                                ? satellite_candidate.squared_residuals(1) /
                                      satellite_candidate
                                          .squared_residuals(0)
                                : 0.0;
                        FixedFailureRateRatioThreshold satellite_ffrt;
                        if (!fixedFailureRateRatioThreshold(
                                static_cast<int>(subset.size()),
                                satellite_bsr, 0.001,
                                satellite_ffrt) ||
                            !satellite_ffrt.accepts_any_candidate ||
                            !std::isfinite(satellite_ratio) ||
                            satellite_ratio <
                                satellite_ffrt
                                    .minimum_second_to_best_ratio) {
                            continue;
                        }
                        VectorXd satellite_best_head;
                        VectorXd satellite_second_head;
                        const bool best_solved =
                            rtk_ar_evaluation::solveFixedHeadState(
                                satellite_problem.head_state,
                                satellite_problem.Qab,
                                satellite_problem.Qb,
                                satellite_problem.dd_float,
                                satellite_candidate.candidates.col(0),
                                satellite_best_head);
                        const bool second_solved =
                            rtk_ar_evaluation::solveFixedHeadState(
                                satellite_problem.head_state,
                                satellite_problem.Qab,
                                satellite_problem.Qb,
                                satellite_problem.dd_float,
                                satellite_candidate.candidates.col(1),
                                satellite_second_head);
                        if (!best_solved ||
                            satellite_best_head.size() < 3) {
                            continue;
                        }
                        debug_telemetry_
                            .lambda_satellite_par_shadow_subset_size =
                            static_cast<int>(subset.size());
                        std::set<SatelliteId> subset_satellites;
                        for (int index : subset) {
                            subset_satellites.insert(
                                dd_pairs[index].sat);
                        }
                        debug_telemetry_
                            .lambda_satellite_par_shadow_dropped_satellites =
                            static_cast<int>(
                                full_satellites.size() -
                                subset_satellites.size());
                        debug_telemetry_
                            .lambda_satellite_par_shadow_bsr =
                            satellite_bsr;
                        debug_telemetry_
                            .lambda_satellite_par_shadow_ratio =
                            satellite_ratio;
                        debug_telemetry_
                            .lambda_satellite_par_shadow_ffrt_min_ratio =
                            satellite_ffrt
                                .minimum_second_to_best_ratio;
                        debug_telemetry_
                            .lambda_satellite_par_shadow_ffrt_passed = true;
                        if (rtk_config_
                                .lambda_satellite_par_persistent_subset) {
                            debug_telemetry_
                                .lambda_satellite_par_persistent_subset_used =
                                subset == persistent_subset &&
                                !persistent_subset.empty();
                            satellite_par_persistent_satellites_.clear();
                            for (int index : subset) {
                                satellite_par_persistent_satellites_.insert(
                                    dd_pairs[index].sat);
                            }
                        }
                        const Vector3d satellite_best_ecef =
                            base_position_ +
                            satellite_best_head.head<3>();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_ecef_x =
                            satellite_best_ecef.x();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_ecef_y =
                            satellite_best_ecef.y();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_ecef_z =
                            satellite_best_ecef.z();
                        const Vector3d satellite_correction =
                            satellite_best_head.head<3>() -
                            satellite_problem.head_state.head<3>();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_correction_x =
                            satellite_correction.x();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_correction_y =
                            satellite_correction.y();
                        debug_telemetry_
                            .lambda_satellite_par_shadow_best_correction_z =
                            satellite_correction.z();
                        if (second_solved &&
                            satellite_second_head.size() >= 3) {
                            debug_telemetry_
                                .lambda_satellite_par_shadow_second_position_delta_m =
                                (satellite_second_head.head<3>() -
                                 satellite_best_head.head<3>())
                                    .norm();
                        }
                        // Sequential PAR stops at the first (largest)
                        // satellite subset passing the fail-closed FFRT.
                        break;
                    }
                    debug_telemetry_
                        .lambda_satellite_par_shadow_runtime_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            satellite_par_started)
                            .count();
                }
            }
            debug_telemetry_.lambda_shadow_runtime_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - shadow_started).count();
        }
        if (full_solved) {
            debug_telemetry_.full_ratio = ratio;
            // WP7 dead-knob fix: passesArFilter is a no-op AND term when
            // enable_ar_filter is false (its default), so this preserves the
            // exact base ratio >= threshold gate unless --arfilter is set.
            if (ratio >= effective_ratio_threshold &&
                rtk_ar_evaluation::passesArFilter(
                    rtk_config_.enable_ar_filter, ratio, effective_ratio_threshold,
                    rtk_config_.ar_filter_margin)) {
                fixed = true;

                // WL-NL cross-validation: verify LAMBDA integers match WL-NL
                if (fixed && rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC) {
                    int mismatch = 0, checked = 0;
                    for (int i = 0; i < nb; ++i) {
                        if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) continue;
                        int l2p = -1;
                        for (int j = 0; j < nb; ++j)
                            if (dd_pairs[j].freq == 1 &&
                                dd_pairs[j].sat == dd_pairs[i].sat &&
                                dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) { l2p = j; break; }
                        if (l2p < 0) continue;
                        // LAMBDA N_wl vs MW N_wl
                        double lambda_nwl = dd_fixed(i) - dd_fixed(l2p);

                        auto rit = sat_data.find(dd_pairs[i].ref_sat);
                        auto sit = sat_data.find(dd_pairs[i].sat);
                        if (rit == sat_data.end() || sit == sat_data.end()) continue;
                        if (!rit->second.has_l2 || !sit->second.has_l2) continue;
                        const double f1 = rit->second.l1_frequency_hz;
                        const double f2 = rit->second.l2_frequency_hz;
                        const double lam_wl = wideLaneWavelength(f1, f2);
                        const double c1 = ionoFreeCoeff1(f1, f2);
                        const double c2 = ionoFreeCoeff2(f1, f2);
                        const double lam_nl = narrowLaneWavelength(f1, f2);
                        if (lam_wl <= 0.0 || lam_nl <= 0.0) continue;
                        auto mw_sd = [&](const SatelliteData& d) {
                            return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                                 - (f1 * (d.rover_l1_code - d.base_l1_code) +
                                    f2 * (d.rover_l2_code - d.base_l2_code))
                                   / ((f1 + f2) * lam_wl);
                        };
                        double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
                        double mw_nwl = std::round(dd_mw);
                        if (std::abs(dd_mw - mw_nwl) < 0.25) {
                            checked++;
                            // Check WL consistency
                            if (std::abs(lambda_nwl - mw_nwl) > 0.5) mismatch++;
                            // Also check NL: use IF to derive expected N2
                            double if_val = c1 * rit->second.l1_wavelength * dd_float(i) +
                                            c2 * rit->second.l2_wavelength * dd_float(l2p);
                            double n2_if = (if_val - c1 * rit->second.l1_wavelength * mw_nwl) / lam_nl;
                            double n2_lambda = dd_fixed(l2p);
                            // If IF-derived N2 differs from LAMBDA N2, it's wrong
                            if (std::abs(n2_if - n2_lambda) > 0.5) mismatch++;
                        }
                    }
                    // If majority of checked satellites have WL mismatch, reject LAMBDA fix
                    if (checked >= 3 && mismatch > checked / 2) {
                        fixed = false;
                    }
                }
            }
        }
    }

    // WL-NL fallback: when LAMBDA fails on long baseline, try MW wide-lane
    // followed by a narrow-lane integer check. Historically this path was
    // tied to IFLC; enable_wlnl_fallback lets iono-off runs test it without
    // also forcing the wide-lane constraint pre-step.
    if (!fixed &&
        (rtk_config_.ionoopt == RTKConfig::IonoOpt::IFLC ||
         rtk_config_.enable_wlnl_fallback) &&
        max_var < 1.0) {
        // Only attempt when KF has converged (max_var < 1 = several epochs in)
        VectorXd wlnl_fixed = dd_float;
        int wl_ok = 0, wl_total = 0;
        std::set<int> wlnl_fixed_indices;
        const double wlnl_acceptance_threshold =
            std::max(0.0, rtk_config_.wide_lane_acceptance_threshold);

        for (int i = 0; i < nb; ++i) {
            if (dd_pairs[i].freq != 0 || dd_pairs[i].ref_sat.system == GNSSSystem::GLONASS) continue;
            wl_total++;
            int l2p = -1;
            for (int j = 0; j < nb; ++j)
                if (dd_pairs[j].freq == 1 &&
                    dd_pairs[j].sat == dd_pairs[i].sat &&
                    dd_pairs[j].ref_sat == dd_pairs[i].ref_sat) { l2p = j; break; }
            if (l2p < 0) continue;

            auto rit = sat_data.find(dd_pairs[i].ref_sat);
            auto sit = sat_data.find(dd_pairs[i].sat);
            if (rit == sat_data.end() || sit == sat_data.end()) continue;
            if (!rit->second.has_l2 || !sit->second.has_l2) continue;
            const double f1 = rit->second.l1_frequency_hz;
            const double f2 = rit->second.l2_frequency_hz;
            const double lam_wl = wideLaneWavelength(f1, f2);
            const double c1_if = ionoFreeCoeff1(f1, f2);
            const double c2_if = ionoFreeCoeff2(f1, f2);
            const double lam_nl = narrowLaneWavelength(f1, f2);
            if (lam_wl <= 0.0 || lam_nl <= 0.0) continue;

            // MW wide-lane
            auto mw_sd = [&](const SatelliteData& d) {
                return (d.rover_l1_phase - d.base_l1_phase) - (d.rover_l2_phase - d.base_l2_phase)
                     - (f1 * (d.rover_l1_code - d.base_l1_code) +
                        f2 * (d.rover_l2_code - d.base_l2_code))
                       / ((f1 + f2) * lam_wl);
            };
            double dd_mw = mw_sd(rit->second) - mw_sd(sit->second);
            double nw = std::round(dd_mw);
            if (std::abs(dd_mw - nw) > wlnl_acceptance_threshold) continue;

            // IF → NL
            double if_dd = c1_if * rit->second.l1_wavelength * dd_float(i) +
                           c2_if * rit->second.l2_wavelength * dd_float(l2p);
            double n2f = (if_dd - c1_if * rit->second.l1_wavelength * nw) / lam_nl;
            double n2 = std::round(n2f);
            if (std::abs(n2f - n2) > wlnl_acceptance_threshold) continue;

            wlnl_fixed(i) = nw + n2;
            wlnl_fixed(l2p) = n2;
            wlnl_fixed_indices.insert(i);
            wlnl_fixed_indices.insert(l2p);
            wl_ok++;
        }

        if (wl_ok >= 3) {
            // Use resolved pairs for position
            std::vector<int> resolved(
                wlnl_fixed_indices.begin(), wlnl_fixed_indices.end());
            if ((int)resolved.size() >= 4) {
                int ns = resolved.size();
                VectorXd sf(ns), sx(ns);
                MatrixXd sQb(ns, ns), sQab(na, ns);
                for (int i = 0; i < ns; ++i) {
                    sf(i) = dd_float(resolved[i]);
                    sx(i) = wlnl_fixed(resolved[i]);
                    for (int j = 0; j < ns; ++j) sQb(i,j) = Qb(resolved[i], resolved[j]);
                    for (int j = 0; j < na; ++j) sQab(j,i) = Qab(j, resolved[i]);
                }
                sQb = (sQb + sQb.transpose()) / 2.0;
                for (int i = 0; i < ns; ++i) if (sQb(i,i) < 1e-6) sQb(i,i) = 1e-6;
                Eigen::LDLT<MatrixXd> slv(sQb);
                if (slv.info() == Eigen::Success) {
                    VectorXd xa = head_state - sQab * slv.solve(sf - sx);
                    fixed_baseline_ = xa.head<3>();
                    has_fixed_solution_ = true;
                    dd_float = sf;
                    Qb = sQb;
                    Qab = sQab;
                    dd_fixed = sx;
                    fixed = true;
                    ratio = 999.9;
                    debug_telemetry_.used_wlnl_fallback = true;
                    initial_candidate_subset = resolved;
                    last_dd_pairs_ = dd_pairs;
                    last_best_subset_ = resolved;
                    last_dd_fixed_ = dd_fixed;
                }
            }
        }
    }

    // Partial AR: try removing worst satellites if full set fails
    rtk_ar_evaluation::CandidateState best_candidate;
    best_candidate.fixed = fixed;
    best_candidate.ratio = ratio;
    best_candidate.subset = initial_candidate_subset;
    best_candidate.dd_float = dd_float;
    best_candidate.Qb = Qb;
    best_candidate.Qab = Qab;
    best_candidate.dd_fixed = dd_fixed;

    const bool search_preferred_subsets =
        rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
        rtk_ar_evaluation::shouldSearchPreferredSubsets(
            fixed, ratio, effective_ratio_threshold);
    const bool search_drop_subsets =
        rtk_config_.ar_policy != RTKConfig::ARPolicy::DEMO5_CONTINUOUS &&
        rtk_ar_evaluation::shouldSearchDropSubsets(
            fixed, ratio, effective_ratio_threshold, max_var);

    if (nb > min_subset_pairs_for_ar && (search_preferred_subsets || search_drop_subsets)) {
        auto passes_full_ratio_gate = [&]() {
            if (min_full_ratio_for_subset_ar <= 0.0) {
                return true;
            }
            return debug_telemetry_.full_lambda_solved &&
                std::isfinite(debug_telemetry_.full_ratio) &&
                debug_telemetry_.full_ratio >= min_full_ratio_for_subset_ar;
        };

        auto try_subset = [&](const std::vector<int>& subset) {
            const int ns = subset.size();
            if (ns < min_subset_pairs_for_ar) {
                return false;
            }
            const auto subset_diversity = compute_subset_diversity(subset);
            debug_telemetry_.subset_candidates_evaluated++;
            if (!passes_subset_diversity_gate(subset_diversity)) {
                debug_telemetry_.subset_candidates_rejected_by_diversity++;
                return false;
            }
            if (!passes_full_ratio_gate()) {
                debug_telemetry_.subset_candidates_rejected_by_full_ratio++;
                return false;
            }

            const auto subset_problem = build_search_problem(subset);
            VectorXd sub_fixed;
            double sub_ratio = 0.0;
            if (!lambdaMethod(subset_problem.dd_float, subset_problem.Qb, sub_fixed, sub_ratio)) {
                return false;
            }
            if (sub_ratio < effective_ratio_threshold) {
                return false;
            }
            if (!rtk_ar_evaluation::passesArFilter(
                    rtk_config_.enable_ar_filter, sub_ratio, effective_ratio_threshold,
                    rtk_config_.ar_filter_margin)) {
                return false;
            }

            if (rtk_ar_evaluation::preferCandidate(
                    best_candidate.ratio, best_candidate.fixed, sub_ratio)) {
                rtk_ar_evaluation::SubsetMatrices subset_matrices;
                subset_matrices.dd_float = subset_problem.dd_float;
                subset_matrices.Qb = subset_problem.Qb;
                subset_matrices.Qab = subset_problem.Qab;
                rtk_ar_evaluation::adoptCandidate(
                    best_candidate, subset, subset_matrices, sub_fixed, sub_ratio);
                return true;
            }
            return false;
        };

        std::vector<rtk_ar_selection::PairDescriptor> descriptors;
        descriptors.reserve(nb);
        for (int i = 0; i < nb; ++i) {
            descriptors.push_back({dd_pairs[i].ref_sat.system, Qb(i, i)});
        }

        const std::vector<std::vector<int>> preferred_subsets =
            rtk_config_.enable_paper_constellation_fallback_ar
                ? rtk_ar_selection::buildPaperConstellationFallbackSubsets(descriptors)
                : rtk_ar_selection::buildPreferredSubsets(descriptors);
        const bool compare_preferred_subsets =
            rtk_config_.enable_paper_constellation_fallback_ar ||
            usesGlonassAutocal(rtk_config_) ||
            std::any_of(dd_pairs.begin(), dd_pairs.end(), [](const DDPair& pair) {
                return pair.ref_sat.system == GNSSSystem::GLONASS ||
                       pair.ref_sat.system == GNSSSystem::BeiDou;
            });
        if (compare_preferred_subsets && search_preferred_subsets) {
            for (const auto& subset : preferred_subsets) {
                if (subset.size() < static_cast<size_t>(min_subset_pairs_for_ar) ||
                    subset.size() >= static_cast<size_t>(nb)) {
                    continue;
                }
                try_subset(subset);
            }
        }

        if (search_drop_subsets) {
            // A full-set fix already passed the Ratio/AR-filter gates. Keep the
            // legacy six-step refinement ceiling in that case so an opt-in
            // deeper search cannot replace a healthy solution with a much
            // smaller, higher-Ratio subset and perturb subsequent filter
            // states. Deeper Partial AR remains available when the full set
            // failed, which is the recovery case the wider search targets.
            const int max_progressive_drop_steps =
                rtk_ar_evaluation::progressiveDropStepLimit(
                    rtk_config_.max_subset_drop_steps_for_ar, fixed);
            const auto progressive_subsets =
                rtk_ar_selection::buildProgressiveVarianceDropSubsets(
                    descriptors,
                    min_subset_pairs_for_ar,
                    max_progressive_drop_steps);
            for (const auto& subset : progressive_subsets) {
                try_subset(subset);
            }

            if (rtk_config_.enable_bsr_guided_decimation &&
                Qb.rows() == nb && Qb.cols() == nb) {
                const int max_drop = std::max(
                    0, std::min(rtk_config_.bsr_guided_max_drop_steps,
                                nb - min_subset_pairs_for_ar));
                const int worst_axes = std::max(1, rtk_config_.bsr_guided_worst_axes);
                const auto bsr_subsets =
                    rtk_ar_selection::buildBSRGuidedDropSubsets(
                        descriptors, Qb, min_subset_pairs_for_ar,
                        max_drop, worst_axes);
                for (const auto& subset : bsr_subsets) {
                    debug_telemetry_.bsr_guided_candidates_evaluated++;
                    if (try_subset(subset)) {
                        debug_telemetry_.bsr_guided_candidates_accepted++;
                    }
                }
            }
        }

        if (best_candidate.fixed) {
            fixed = true;
            dd_fixed = best_candidate.dd_fixed;
            ratio = best_candidate.ratio;
            Qab = best_candidate.Qab;
            Qb = best_candidate.Qb;
            dd_float = best_candidate.dd_float;
        } else if (!fixed && best_candidate.subset.size() < static_cast<size_t>(nb)) {
            Qab = best_candidate.Qab;
            Qb = best_candidate.Qb;
            dd_float = best_candidate.dd_float;
        }
    }

    debug_telemetry_.selected_ratio = ratio;
    debug_telemetry_.selected_pair_count = static_cast<int>(best_candidate.subset.size());
    const auto selected_diversity = compute_subset_diversity(best_candidate.subset);
    debug_telemetry_.selected_distinct_sats = selected_diversity.distinct_sats;
    debug_telemetry_.selected_distinct_systems = selected_diversity.distinct_systems;
    debug_telemetry_.selected_distinct_frequencies = selected_diversity.distinct_frequencies;
    debug_telemetry_.selected_dual_frequency_sats = selected_diversity.dual_frequency_sats;
    debug_telemetry_.selected_used_subset = best_candidate.subset.size() < static_cast<size_t>(nb);
    std::set<SatelliteId> selected_references;
    for (const int index : best_candidate.subset) {
        if (index >= 0 && index < static_cast<int>(dd_pairs.size())) {
            selected_references.insert(dd_pairs[static_cast<size_t>(index)].ref_sat);
        }
    }
    for (const auto& reference : selected_references) {
        if (!debug_telemetry_.selected_reference_satellites.empty()) {
            debug_telemetry_.selected_reference_satellites += ';';
        }
        debug_telemetry_.selected_reference_satellites += reference.toString();
    }
    if (!fixed) {
        debug_telemetry_.selected_fixed = false;
        if (debug_telemetry_.reject_reason.empty()) {
            debug_telemetry_.reject_reason = "lambda_not_fixed";
        }
        // First-cause wins: a more specific skip reason set earlier
        // (e.g. DD_PAIRS_LT_4_AFTER_VAR_FILTER) should not be overwritten.
        if (debug_telemetry_.ar_skip_reason == ARSkipReason::NONE) {
            debug_telemetry_.ar_skip_reason = ARSkipReason::LAMBDA_FAILED;
        }
        return false;
    }
    debug_telemetry_.selected_fixed = true;

    // Fixed solution: xa = y[:na] - Qab * Qb^{-1} * (dd_float - dd_fixed)
    const auto fixed_problem = build_search_problem(best_candidate.subset);
    VectorXd xa;
    if (!rtk_ar_evaluation::solveFixedHeadState(fixed_problem.head_state,
                                                fixed_problem.Qab,
                                                fixed_problem.Qb,
                                                fixed_problem.dd_float,
                                                dd_fixed, xa)) {
        debug_telemetry_.reject_reason = "fixed_head_solve";
        if (debug_telemetry_.ar_skip_reason == ARSkipReason::NONE) {
            debug_telemetry_.ar_skip_reason = ARSkipReason::RATIO_COMPUTATION_FAILED;
        }
        return false;
    }
    fixed_baseline_ = xa.head<3>();
    has_fixed_solution_ = true;
    last_ar_ratio_ = ratio;
    last_num_fixed_ambiguities_ = dd_fixed.size();
    // Clear any diagnostic-only skip reason set earlier (e.g. DD_PAIRS_LT_4_AFTER_VAR_FILTER
    // at line 1803, which marks the cause without early-returning so LAMBDA can still try).
    debug_telemetry_.ar_skip_reason = ARSkipReason::NONE;
    debug_telemetry_.selected_fixed_ambiguities = static_cast<int>(dd_fixed.size());

    // Store fix info for hold and validation
    last_dd_pairs_ = dd_pairs;
    last_best_subset_ = best_candidate.subset;
    last_dd_fixed_ = dd_fixed;

    return true;
}

// ============================================================
// Validate fixed solution
// ============================================================

}  // namespace libgnss
