#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
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
// AR search state shared by resolveAmbiguities() and its extracted
// helpers; compute_subset_diversity remains a member because DDPair is
// a private nested type.

struct ArSubsetDiversity {
    int distinct_sats = 0;
    int distinct_systems = 0;
    int distinct_frequencies = 0;
    int dual_frequency_sats = 0;
    };

struct ArWideLaneConstraint {
    int l1_index = -1;
    int l2_index = -1;
    double fixed_integer = 0.0;
    };

struct ArSearchProblem {
    VectorXd head_state;
    VectorXd dd_float;
    MatrixXd Qb;
    MatrixXd Qab;
    };


bool passes_subset_diversity_gate(const RTKProcessor::RTKConfig& config,
                             const ArSubsetDiversity& diversity) {
    if (config.min_subset_sats_for_ar > 0 &&
        diversity.distinct_sats < config.min_subset_sats_for_ar) {
        return false;
    }
    if (config.min_subset_systems_for_ar > 0 &&
        diversity.distinct_systems < config.min_subset_systems_for_ar) {
        return false;
    }
    if (config.min_subset_frequencies_for_ar > 0 &&
        diversity.distinct_frequencies < config.min_subset_frequencies_for_ar) {
        return false;
    }
    if (config.min_subset_dual_frequency_sats_for_ar > 0 &&
        diversity.dual_frequency_sats < config.min_subset_dual_frequency_sats_for_ar) {
        return false;
    }
    return true;
};

ArSearchProblem build_search_problem(
    const VectorXd& base_dd_float,
    const MatrixXd& base_Qb,
    const MatrixXd& base_Qab,
    const VectorXd& base_head_state,
    int nb,
    const std::vector<ArWideLaneConstraint>& wide_lane_constraints,
    const std::vector<int>& subset) {
    ArSearchProblem problem;
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

ArSubsetDiversity RTKProcessor::compute_subset_diversity(
    const std::vector<DDPair>& dd_pairs,
                             const std::vector<int>& subset) const {
    ArSubsetDiversity diversity;
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
    return resolveAmbiguities(buildDoubleDifferencePairs(sat_data, min_lock, true));
}

void RTKProcessor::tryWlNlFallback(int na,
                                   int nb,
                                   double max_var,
                                   const std::vector<DDPair>& dd_pairs,
                                   const VectorXd& head_state,
                                   VectorXd& dd_float,
                                   MatrixXd& Qb,
                                   MatrixXd& Qab,
                                   VectorXd& dd_fixed,
                                   bool& fixed,
                                   double& ratio,
                                   std::vector<int>& initial_candidate_subset) {
    const auto& sat_data = current_sat_data_;
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

        // 3 is the legacy floor; min_fix_satellites can only raise it. The check
        // runs before any state is written, so a refused fallback leaves no trace.
        if (wl_ok >= std::max(3, rtk_config_.min_fix_satellites)) {
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

    std::vector<ArWideLaneConstraint> wide_lane_constraints;
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

    if (rtk_config_.enable_wide_lane_ar) {
        const double wide_lane_threshold =
            std::max(0.0, rtk_config_.wide_lane_acceptance_threshold);
        // Opt-in causal MW arc smoothing: update every dual-frequency
        // satellite's single-difference MW arc once per epoch, then form the
        // DD WL float from the smoothed arcs instead of one noisy epoch.
        const bool smooth_wide_lane = rtk_config_.wide_lane_min_arc_samples > 0;
        std::map<SatelliteId, causal_ambiguity_arc::Result> wide_lane_arcs;
        if (smooth_wide_lane) {
            constexpr int kL1L2WideLaneArcId = 12;
            const double time_s =
                static_cast<double>(current_epoch_time_.week) * 604800.0 +
                current_epoch_time_.tow;
            for (const auto& [sat, data] : sat_data) {
                if (sat.system == GNSSSystem::GLONASS ||
                    !data.has_l1 || !data.has_l2) {
                    continue;
                }
                const double f1 = data.l1_frequency_hz;
                const double f2 = data.l2_frequency_hz;
                const double lambda_wl_m = wideLaneWavelength(f1, f2);
                if (f1 <= 0.0 || f2 <= 0.0 || lambda_wl_m <= 0.0) {
                    continue;
                }
                const double phi1_m =
                    (data.rover_l1_phase - data.base_l1_phase) * data.l1_wavelength;
                const double phi2_m =
                    (data.rover_l2_phase - data.base_l2_phase) * data.l2_wavelength;
                const double code_term =
                    (f1 * (data.rover_l1_code - data.base_l1_code) +
                     f2 * (data.rover_l2_code - data.base_l2_code)) / (f1 + f2);
                const double sd_mw =
                    ((f1 * phi1_m - f2 * phi2_m) / (f1 - f2) - code_term) / lambda_wl_m;
                const bool slip = current_epoch_slips_l1_.count(sat) > 0 ||
                                  current_epoch_slips_l2_.count(sat) > 0;
                wide_lane_arcs[sat] = wide_lane_mw_arc_bank_.updateSignal(
                    sat, kL1L2WideLaneArcId, time_s, sd_mw, slip);
            }
        }
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
            if (smooth_wide_lane) {
                const auto ref_arc = wide_lane_arcs.find(dd_pairs[i].ref_sat);
                const auto sat_arc = wide_lane_arcs.find(dd_pairs[i].sat);
                // Arc length only: the SD MW carries a non-integer receiver
                // WL bias, so the bank's SD rounding-stability flag is not
                // meaningful here; integer closeness is tested on the DD.
                const int min_samples = rtk_config_.wide_lane_min_arc_samples;
                if (ref_arc == wide_lane_arcs.end() || sat_arc == wide_lane_arcs.end() ||
                    !ref_arc->second.valid || !sat_arc->second.valid ||
                    ref_arc->second.samples < min_samples ||
                    sat_arc->second.samples < min_samples) {
                    wide_lane_rejected++;
                    continue;
                }
                wide_lane_float =
                    ref_arc->second.smoothed_value - sat_arc->second.smoothed_value;
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


    // Standard LAMBDA path
    if (!fixed) {
        const auto full_problem = build_search_problem(base_dd_float, base_Qb, base_Qab, base_head_state, nb, wide_lane_constraints, full_subset);
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
                if (shadow.candidates.cols() >= 1 &&
                    shadow.candidates.rows() == full_problem.dd_float.size() &&
                    full_problem.dd_float.size() == static_cast<int>(dd_pairs.size())) {
                    double max_abs_z = -1.0;
                    int max_index = -1;
                    int above_three = 0;
                    for (int i = 0; i < full_problem.dd_float.size(); ++i) {
                        const double sigma = std::sqrt(full_problem.Qb(i, i));
                        if (!(sigma > 0.0)) continue;
                        const double z = std::abs(
                            full_problem.dd_float(i) - shadow.candidates(i, 0)) / sigma;
                        if (z > 3.0) ++above_three;
                        if (z > max_abs_z) {
                            max_abs_z = z;
                            max_index = i;
                        }
                    }
                    if (max_index >= 0) {
                        const auto& pair = dd_pairs[static_cast<size_t>(max_index)];
                        const auto& locks = pair.freq == 0 ? lock_count_l1_
                                          : pair.freq == 1 ? lock_count_l2_
                                                           : lock_count_l5_;
                        const auto lock_of = [&](const SatelliteId& satellite) {
                            const auto it = locks.find(satellite);
                            return it == locks.end() ? 0 : it->second;
                        };
                        debug_telemetry_.lambda_shadow_max_abs_z = max_abs_z;
                        debug_telemetry_.lambda_shadow_max_z_pair =
                            pair.ref_sat.toString() + "-" + pair.sat.toString() + "/L" +
                            std::to_string(pair.freq + 1);
                        debug_telemetry_.lambda_shadow_max_z_lock_count =
                            std::min(lock_of(pair.ref_sat), lock_of(pair.sat));
                        debug_telemetry_.lambda_shadow_pairs_abs_z_gt3 = above_three;
                    }
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
    // followed by a narrow-lane integer check (see tryWlNlFallback).
    tryWlNlFallback(na, nb, max_var, dd_pairs, head_state, dd_float, Qb, Qab,
                    dd_fixed, fixed, ratio, initial_candidate_subset);
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
            const auto subset_diversity = compute_subset_diversity(dd_pairs, subset);
            debug_telemetry_.subset_candidates_evaluated++;
            if (!passes_subset_diversity_gate(rtk_config_, subset_diversity)) {
                debug_telemetry_.subset_candidates_rejected_by_diversity++;
                return false;
            }
            if (!passes_full_ratio_gate()) {
                debug_telemetry_.subset_candidates_rejected_by_full_ratio++;
                return false;
            }

            const auto subset_problem = build_search_problem(base_dd_float, base_Qb, base_Qab, base_head_state, nb, wide_lane_constraints, subset);
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

return finishAmbiguityResolution(nb, ratio, fixed, dd_pairs,
                                dd_fixed, best_candidate,
                                base_dd_float, base_Qb, base_Qab,
                                base_head_state,
                                wide_lane_constraints);
}

bool RTKProcessor::finishAmbiguityResolution(
    int nb,
    double ratio,
    bool fixed,
    const std::vector<DDPair>& dd_pairs,
    const VectorXd& dd_fixed,
    const rtk_ar_evaluation::CandidateState& best_candidate,
    const VectorXd& base_dd_float,
    const MatrixXd& base_Qb,
    const MatrixXd& base_Qab,
    const VectorXd& base_head_state,
    const std::vector<ArWideLaneConstraint>& wide_lane_constraints) {
    debug_telemetry_.selected_ratio = ratio;
    debug_telemetry_.selected_pair_count = static_cast<int>(best_candidate.subset.size());
    const auto selected_diversity = compute_subset_diversity(dd_pairs, best_candidate.subset);
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
    if (rtk_config_.min_fix_satellites > 0 &&
        selected_diversity.distinct_sats < rtk_config_.min_fix_satellites) {
        // Opt-in: too few double-difference directions to trust the fixed position.
        // distinct_sats counts the non-reference satellites of the selected pairs
        // (all frequencies of a satellite count once).
        debug_telemetry_.selected_fixed = false;
        debug_telemetry_.reject_reason = "fixed_satellites_below_min";
        has_fixed_solution_ = false;
        return false;
    }
    debug_telemetry_.selected_fixed = true;

    // Fixed solution: xa = y[:na] - Qab * Qb^{-1} * (dd_float - dd_fixed)
    const auto fixed_problem = build_search_problem(base_dd_float, base_Qb, base_Qab, base_head_state,
                         nb, wide_lane_constraints, best_candidate.subset);
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
