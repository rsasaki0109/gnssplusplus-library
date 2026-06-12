#include <libgnss++/algorithms/ppp_ar.hpp>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace libgnss::ppp_ar {

constexpr std::array<double, 60> kClaslibRatioThresholdsAlpha10 = {
    39.86,9.00,5.39,4.11,3.45,3.05,2.78,2.59,2.44,2.32,
    2.23,2.15,2.08,2.02,1.97,1.93,1.89,1.85,1.82,1.79,
    1.77,1.74,1.72,1.70,1.68,1.67,1.65,1.63,1.62,1.61,
    1.59,1.58,1.57,1.56,1.55,1.54,1.53,1.52,1.51,1.51,
    1.50,1.49,1.48,1.48,1.47,1.46,1.46,1.45,1.45,1.44,
    1.44,1.43,1.43,1.42,1.42,1.41,1.41,1.40,1.40,1.40
};

SatelliteId clasRealSatellite(const SatelliteId& satellite) {
    if (satellite.prn > 100) {
        return SatelliteId(
            satellite.system,
            static_cast<uint8_t>(std::max(1, static_cast<int>(satellite.prn) - 100)));
    }
    return satellite;
}

std::pair<GNSSSystem, int> ambiguityDdGroup(const SatelliteId& satellite) {
    return {satellite.system, satellite.prn > 100 ? 1 : 0};
}

double claslibRatioThresholdForNb(int nb) {
    if (nb <= 0) {
        return std::numeric_limits<double>::infinity();
    }
    const size_t index =
        static_cast<size_t>(std::min(nb, static_cast<int>(kClaslibRatioThresholdsAlpha10.size())) - 1);
    return kClaslibRatioThresholdsAlpha10[index];
}

double safeVarianceFloor(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance < floor_value) {
        return floor_value;
    }
    return variance;
}

EligibleAmbiguities collectEligibleAmbiguities(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    int min_lock_count) {
    EligibleAmbiguities eligible;
    for (const auto& [satellite, state_index] : filter_state.ambiguity_indices) {
        ++eligible.total_ambiguities;
        const auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        const auto& ambiguity = ambiguity_it->second;
        if (ambiguity.needs_reinitialization || ambiguity.lock_count < min_lock_count) {
            if (ambiguity.needs_reinitialization) {
                ++eligible.skipped_reinitialization;
            } else {
                ++eligible.skipped_lock;
            }
            continue;
        }
        if (!std::isfinite(ambiguity.ambiguity_scale_m) || ambiguity.ambiguity_scale_m <= 0.0) {
            ++eligible.skipped_scale;
            continue;
        }
        if (state_index < filter_state.amb_index || state_index >= filter_state.total_states) {
            ++eligible.skipped_index;
            continue;
        }
        eligible.satellites.push_back(satellite);
        eligible.state_indices.push_back(state_index);
        eligible.scales.push_back(ambiguity.ambiguity_scale_m);
    }
    return eligible;
}

WlnlWideLaneFixSummary applyWideLaneFixes(
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    bool debug_enabled) {
    WlnlWideLaneFixSummary summary;
    for (const auto& satellite : satellites) {
        auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        auto& ambiguity = ambiguity_it->second;
        summary.max_mw_count = std::max(summary.max_mw_count, ambiguity.mw_count);
        if (ambiguity.wl_is_fixed) {
            ++summary.fixed_count;
            continue;
        }
        if (ambiguity.mw_count < config.wl_min_averaging_epochs) {
            continue;
        }
        const double mw_mean = ambiguity.mw_mean_cycles;
        const int wl_int = static_cast<int>(std::round(mw_mean));
        const double frac = mw_mean - wl_int;
        if (std::abs(frac) >= 0.25) {
            if (debug_enabled) {
                std::cerr << "[PPP-WLREJ] " << satellite.toString()
                          << " mw_mean=" << mw_mean << " frac=" << frac
                          << " count=" << ambiguity.mw_count << "\n";
            }
            continue;
        }

        ambiguity.wl_fixed_integer = wl_int;
        ambiguity.wl_is_fixed = true;
        ++summary.fixed_count;
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] WL fix "
                      << satellite.toString()
                      << " mw_mean=" << mw_mean
                      << " int=" << wl_int
                      << " frac=" << frac
                      << " count=" << ambiguity.mw_count << "\n";
        }
    }
    return summary;
}

WlnlPreparation prepareWlnlCandidates(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    bool use_ssr_products,
    bool debug_enabled) {
    WlnlPreparation preparation;
    preparation.min_lock_count = use_ssr_products
        ? std::max(1, config.wl_min_averaging_epochs)
        : config.convergence_min_epochs;
    preparation.eligible_ambiguities = collectEligibleAmbiguities(
        filter_state, ambiguity_states, preparation.min_lock_count);
    preparation.wl_summary = applyWideLaneFixes(
        config, ambiguity_states, preparation.eligible_ambiguities.satellites, debug_enabled);
    return preparation;
}

DdFixAttempt tryDirectDdFix(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<double>& scales,
    const std::set<SatelliteId>& excluded_real_satellites,
    bool debug_enabled) {
    DdFixAttempt attempt;

    std::vector<int> active_indices;
    active_indices.reserve(satellites.size());
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        if (excluded_real_satellites.count(clasRealSatellite(satellites[static_cast<size_t>(i)])) == 0) {
            active_indices.push_back(i);
        }
    }

    if (static_cast<int>(active_indices.size()) < config.min_satellites_for_ar) {
        return attempt;
    }

    struct DdPair {
        int ref_idx = -1;
        int sat_idx = -1;
    };

    std::vector<DdPair> dd_pairs;
    std::map<std::pair<GNSSSystem, int>, int> system_ref_map;
    for (const int candidate_index : active_indices) {
        const auto group = ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]);
        if (system_ref_map.find(group) == system_ref_map.end()) {
            system_ref_map[group] = candidate_index;
        }
    }

    for (const int candidate_index : active_indices) {
        const auto group = ambiguityDdGroup(satellites[static_cast<size_t>(candidate_index)]);
        const int ref_index = system_ref_map[group];
        if (ref_index == candidate_index) {
            continue;
        }
        dd_pairs.push_back({ref_index, candidate_index});
    }

    attempt.nb = static_cast<int>(dd_pairs.size());
    if (attempt.nb < config.min_satellites_for_ar) {
        return attempt;
    }

    VectorXd dd_float = VectorXd::Zero(attempt.nb);
    MatrixXd dd_cov = MatrixXd::Zero(attempt.nb, attempt.nb);

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const double ref_cycles =
            filter_state.state(state_indices[static_cast<size_t>(ri)]) /
            scales[static_cast<size_t>(ri)];
        const double sat_cycles =
            filter_state.state(state_indices[static_cast<size_t>(si)]) /
            scales[static_cast<size_t>(si)];
        dd_float(k) = ref_cycles - sat_cycles;

        for (int l = 0; l < attempt.nb; ++l) {
            const int rj = dd_pairs[static_cast<size_t>(l)].ref_idx;
            const int sj = dd_pairs[static_cast<size_t>(l)].sat_idx;
            auto covarianceCycles = [&](int a, int b) {
                return filter_state.covariance(
                    state_indices[static_cast<size_t>(a)],
                    state_indices[static_cast<size_t>(b)]) /
                    (scales[static_cast<size_t>(a)] * scales[static_cast<size_t>(b)]);
            };
            dd_cov(k, l) =
                covarianceCycles(ri, rj) - covarianceCycles(ri, sj) -
                covarianceCycles(si, rj) + covarianceCycles(si, sj);
        }
    }

    // Per-pair DD L1 float diagnostic: dumps the float DD ambiguity, its
    // distance to the nearest integer, and the formal sigma so the
    // integer-informativeness of the float solution can be measured (a mean
    // |frac| near 0.25 means the floats are uniformly spread = carry no integer
    // information). Gated by GNSS_PPP_AR_DDDUMP, default OFF.
    if (pppEnvOverrides().ar_dddump && excluded_real_satellites.empty()) {
        for (int k = 0; k < attempt.nb; ++k) {
            const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
            const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
            const double frac = dd_float(k) - std::round(dd_float(k));
            std::cerr << "[PPP-AR-DD] ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " dd_float=" << dd_float(k)
                      << " frac=" << frac
                      << " sigma=" << std::sqrt(std::max(0.0, dd_cov(k, k)))
                      << "\n";
        }
    }

    VectorXd dd_fixed = VectorXd::Zero(attempt.nb);
    if (!lambdaSearch(dd_float, dd_cov, dd_fixed, attempt.ratio)) {
        return attempt;
    }

    attempt.required_ratio = config.ar_ratio_threshold;
    if (config.use_clas_osr_filter) {
        attempt.required_ratio =
            std::max(attempt.required_ratio, claslibRatioThresholdForNb(attempt.nb));
    }
    if (!std::isfinite(attempt.ratio) || attempt.ratio < attempt.required_ratio) {
        return attempt;
    }

    attempt.state = filter_state;
    attempt.ambiguities = ambiguity_states;

    const int na = filter_state.amb_index;
    const VectorXd dd_residual = dd_float - dd_fixed;
    MatrixXd dd_cov_copy = dd_cov;
    Eigen::LDLT<MatrixXd> ldlt(dd_cov_copy);
    if (ldlt.info() != Eigen::Success) {
        return DdFixAttempt{};
    }

    const VectorXd db = ldlt.solve(dd_residual);
    if (!db.allFinite()) {
        return DdFixAttempt{};
    }

    const MatrixXd& Pcross =
        pre_anchor_covariance.rows() > 0 ? pre_anchor_covariance : filter_state.covariance;
    MatrixXd Qab = MatrixXd::Zero(na, attempt.nb);
    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        for (int i = 0; i < na; ++i) {
            Qab(i, k) =
                (Pcross(i, state_indices[static_cast<size_t>(ri)]) /
                 scales[static_cast<size_t>(ri)]) -
                (Pcross(i, state_indices[static_cast<size_t>(si)]) /
                 scales[static_cast<size_t>(si)]);
        }
    }

    const VectorXd delta = Qab * db;
    attempt.state.state.head(na) -= delta;

    if (debug_enabled && excluded_real_satellites.empty()) {
        double qab_pos_norm = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            qab_pos_norm += Qab.row(filter_state.pos_index + axis).squaredNorm();
        }
        qab_pos_norm = std::sqrt(qab_pos_norm);
        std::cerr << "[PPP-AR] dd_resid=" << dd_residual.norm()
                  << " db=" << db.norm()
                  << " Qab=" << Qab.norm()
                  << " Qab_pos=" << qab_pos_norm
                  << " delta_all=" << delta.norm()
                  << " delta_pos=" << delta.segment(filter_state.pos_index, 3).norm()
                  << " delta_clk=" << std::abs(delta(filter_state.clock_index))
                  << "\n";
    }

    for (const auto& [group, ref_idx] : system_ref_map) {
        (void)group;
        const int ref_state = state_indices[static_cast<size_t>(ref_idx)];
        auto& ref_ambiguity = attempt.ambiguities[satellites[static_cast<size_t>(ref_idx)]];
        ref_ambiguity.is_fixed = true;
        ref_ambiguity.fixed_value = attempt.state.state(ref_state);
    }

    for (int k = 0; k < attempt.nb; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const double ref_value_m =
            attempt.state.state(state_indices[static_cast<size_t>(ri)]);
        const double scale_si = scales[static_cast<size_t>(si)];
        const double ref_cycles = ref_value_m / scales[static_cast<size_t>(ri)];
        const double fixed_sat_cycles = ref_cycles - dd_fixed(k);
        const double fixed_sat_m = fixed_sat_cycles * scale_si;

        const int sat_state = state_indices[static_cast<size_t>(si)];
        attempt.state.state(sat_state) = fixed_sat_m;
        attempt.state.covariance.row(sat_state).setZero();
        attempt.state.covariance.col(sat_state).setZero();
        attempt.state.covariance(sat_state, sat_state) = 1e-6;

        auto& ambiguity_state = attempt.ambiguities[satellites[static_cast<size_t>(si)]];
        ambiguity_state.float_value = fixed_sat_m;
        ambiguity_state.fixed_value = fixed_sat_m;
        ambiguity_state.is_fixed = true;
    }

    attempt.fixed = true;
    return attempt;
}

DdFixAttempt tryDirectDdFixWithPar(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& pre_anchor_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const std::map<SatelliteId, double>& real_satellite_elevations,
    bool debug_enabled) {
    auto try_dd_fix = [&](const std::set<SatelliteId>& excluded_real_satellites) {
        return tryDirectDdFix(
            config,
            filter_state,
            pre_anchor_covariance,
            ambiguity_states,
            eligible_ambiguities.satellites,
            eligible_ambiguities.state_indices,
            eligible_ambiguities.scales,
            excluded_real_satellites,
            debug_enabled);
    };

    DdFixAttempt best_attempt = try_dd_fix({});
    if (best_attempt.fixed || !config.use_clas_osr_filter || real_satellite_elevations.empty()) {
        return best_attempt;
    }

    std::vector<SatelliteId> par_candidates;
    par_candidates.reserve(real_satellite_elevations.size());
    for (const auto& [satellite, _] : real_satellite_elevations) {
        par_candidates.push_back(satellite);
    }
    std::sort(par_candidates.begin(), par_candidates.end(),
              [&](const SatelliteId& lhs, const SatelliteId& rhs) {
                  return real_satellite_elevations.at(lhs) < real_satellite_elevations.at(rhs);
              });

    std::set<SatelliteId> excluded_real_satellites;
    const int max_exclusions = std::min(
        4,
        std::max(0, static_cast<int>(par_candidates.size()) - config.min_satellites_for_ar));

    for (int iteration = 0; iteration < max_exclusions && !best_attempt.fixed; ++iteration) {
        SatelliteId best_exclusion;
        bool has_best_exclusion = false;
        double best_ratio = best_attempt.ratio;
        double best_required_ratio = best_attempt.required_ratio;

        for (const auto& real_satellite : par_candidates) {
            if (excluded_real_satellites.count(real_satellite) != 0) {
                continue;
            }

            auto trial_exclusions = excluded_real_satellites;
            trial_exclusions.insert(real_satellite);
            DdFixAttempt trial_attempt = try_dd_fix(trial_exclusions);
            if (trial_attempt.fixed) {
                best_attempt = std::move(trial_attempt);
                if (debug_enabled) {
                    std::cerr << "[PPP-AR] PAR fixed: excluded="
                              << real_satellite.toString()
                              << " nb=" << best_attempt.nb
                              << " ratio=" << best_attempt.ratio
                              << " threshold=" << best_attempt.required_ratio << "\n";
                }
                break;
            }

            if (trial_attempt.ratio > best_ratio) {
                best_ratio = trial_attempt.ratio;
                best_required_ratio = trial_attempt.required_ratio;
                best_exclusion = real_satellite;
                has_best_exclusion = true;
            }
        }

        if (best_attempt.fixed || !has_best_exclusion) {
            break;
        }

        excluded_real_satellites.insert(best_exclusion);
        if (debug_enabled) {
            std::cerr << "[PPP-AR] PAR exclude candidate: "
                      << best_exclusion.toString()
                      << " ratio=" << best_ratio
                      << " threshold=" << best_required_ratio << "\n";
        }
    }

    return best_attempt;
}

struct WlnlDdPair {
    int ref_idx = -1;
    int sat_idx = -1;
};

struct StateDdRow {
    SatelliteId ref_satellite;
    SatelliteId sat_satellite;
    int ref_state = -1;
    int sat_state = -1;
    double ref_scale_m = 0.0;
    double sat_scale_m = 0.0;
    double wlnl_fixed_cycles = 0.0;
    const char* band = "";
};

double ambiguityWavelengthL1(const ppp_shared::PPPAmbiguityInfo& ambiguity) {
    return ambiguity.wavelength_l1 > 0.0 ? ambiguity.wavelength_l1
                                         : ambiguity.ambiguity_scale_m;
}

bool validStateDdEndpoint(int state_index, double scale_m, int total_states) {
    return state_index >= 0 &&
           state_index < total_states &&
           std::isfinite(scale_m) &&
           scale_m > 0.0;
}

bool conditionWlnlFilterStateDd(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::vector<WlnlDdPair>& dd_pairs,
    const VectorXd& dd_nl_fixed,
    WlnlFixAttempt& attempt,
    bool debug_enabled) {
    std::vector<StateDdRow> rows;
    rows.reserve(dd_pairs.size() * 2U);

    for (int k = 0; k < static_cast<int>(dd_pairs.size()); ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        if (ri < 0 || si < 0 ||
            ri >= static_cast<int>(satellites.size()) ||
            si >= static_cast<int>(satellites.size()) ||
            ri >= static_cast<int>(state_indices.size()) ||
            si >= static_cast<int>(state_indices.size())) {
            continue;
        }

        const SatelliteId& ref_sat = satellites[static_cast<size_t>(ri)];
        const SatelliteId& sat = satellites[static_cast<size_t>(si)];
        const auto ref_amb_it = ambiguity_states.find(ref_sat);
        const auto sat_amb_it = ambiguity_states.find(sat);
        if (ref_amb_it == ambiguity_states.end() ||
            sat_amb_it == ambiguity_states.end() ||
            !ref_amb_it->second.wl_is_fixed ||
            !sat_amb_it->second.wl_is_fixed) {
            continue;
        }

        const double dd_nl_cycles = std::round(dd_nl_fixed(k));
        const double dd_wl_cycles =
            static_cast<double>(ref_amb_it->second.wl_fixed_integer -
                                sat_amb_it->second.wl_fixed_integer);
        const double l1_fixed_cycles = 0.5 * (dd_nl_cycles + dd_wl_cycles);
        const double l2_fixed_cycles = 0.5 * (dd_nl_cycles - dd_wl_cycles);

        const double ref_l1_scale = ambiguityWavelengthL1(ref_amb_it->second);
        const double sat_l1_scale = ambiguityWavelengthL1(sat_amb_it->second);
        if (validStateDdEndpoint(
                state_indices[static_cast<size_t>(ri)],
                ref_l1_scale,
                filter_state.total_states) &&
            validStateDdEndpoint(
                state_indices[static_cast<size_t>(si)],
                sat_l1_scale,
                filter_state.total_states)) {
            rows.push_back({
                ref_sat,
                sat,
                state_indices[static_cast<size_t>(ri)],
                state_indices[static_cast<size_t>(si)],
                ref_l1_scale,
                sat_l1_scale,
                l1_fixed_cycles,
                "L1",
            });
        }

        const auto ref_l2_state_it = filter_state.ambiguity_l2_indices.find(ref_sat);
        const auto sat_l2_state_it = filter_state.ambiguity_l2_indices.find(sat);
        const double ref_l2_scale = ref_amb_it->second.wavelength_l2;
        const double sat_l2_scale = sat_amb_it->second.wavelength_l2;
        if (ref_l2_state_it != filter_state.ambiguity_l2_indices.end() &&
            sat_l2_state_it != filter_state.ambiguity_l2_indices.end() &&
            validStateDdEndpoint(
                ref_l2_state_it->second,
                ref_l2_scale,
                filter_state.total_states) &&
            validStateDdEndpoint(
                sat_l2_state_it->second,
                sat_l2_scale,
                filter_state.total_states)) {
            rows.push_back({
                ref_sat,
                sat,
                ref_l2_state_it->second,
                sat_l2_state_it->second,
                ref_l2_scale,
                sat_l2_scale,
                l2_fixed_cycles,
                "L2",
            });
        }
    }

    const int nb = static_cast<int>(rows.size());
    attempt.state_dd_count = nb;
    if (nb < 4) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL-RESAMB] insufficient state DD rows: "
                      << nb << "\n";
        }
        return false;
    }

    const MatrixXd& P_constraint =
        constraint_covariance.rows() == filter_state.total_states &&
                constraint_covariance.cols() == filter_state.total_states
            ? constraint_covariance
            : filter_state.covariance;

    VectorXd dd_float = VectorXd::Zero(nb);
    MatrixXd dd_cov = MatrixXd::Zero(nb, nb);
    MatrixXd Qab = MatrixXd::Zero(filter_state.amb_index, nb);
    for (int row = 0; row < nb; ++row) {
        const auto& row_entry = rows[static_cast<size_t>(row)];
        const double ref_cycles =
            filter_state.state(row_entry.ref_state) / row_entry.ref_scale_m;
        const double sat_cycles =
            filter_state.state(row_entry.sat_state) / row_entry.sat_scale_m;
        dd_float(row) = ref_cycles - sat_cycles;

        for (int col = 0; col < nb; ++col) {
            const auto& col_entry = rows[static_cast<size_t>(col)];
            dd_cov(row, col) =
                P_constraint(row_entry.ref_state, col_entry.ref_state) /
                    (row_entry.ref_scale_m * col_entry.ref_scale_m) -
                P_constraint(row_entry.ref_state, col_entry.sat_state) /
                    (row_entry.ref_scale_m * col_entry.sat_scale_m) -
                P_constraint(row_entry.sat_state, col_entry.ref_state) /
                    (row_entry.sat_scale_m * col_entry.ref_scale_m) +
                P_constraint(row_entry.sat_state, col_entry.sat_state) /
                    (row_entry.sat_scale_m * col_entry.sat_scale_m);
        }

        for (int head = 0; head < filter_state.amb_index; ++head) {
            Qab(head, row) =
                P_constraint(head, row_entry.ref_state) / row_entry.ref_scale_m -
                P_constraint(head, row_entry.sat_state) / row_entry.sat_scale_m;
        }
    }

    dd_cov = 0.5 * (dd_cov + dd_cov.transpose());
    if (!dd_float.allFinite() || !dd_cov.allFinite() || !Qab.allFinite()) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL-RESAMB] non-finite state DD system\n";
        }
        return false;
    }

    VectorXd state_lambda_fixed = VectorXd::Zero(nb);
    VectorXd wlnl_fixed = VectorXd::Zero(nb);
    for (int row = 0; row < nb; ++row) {
        wlnl_fixed(row) = rows[static_cast<size_t>(row)].wlnl_fixed_cycles;
        const double fractional =
            std::abs(wlnl_fixed(row) - std::round(wlnl_fixed(row)));
        attempt.state_wlnl_max_fractional_cycles =
            std::max(attempt.state_wlnl_max_fractional_cycles, fractional);
        if (fractional > 1e-6) {
            ++attempt.state_wlnl_noninteger_count;
        }
    }

    double state_ratio = 0.0;
    if (!lambdaSearch(dd_float, dd_cov, state_lambda_fixed, state_ratio)) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL-RESAMB] state lambda failed: nb="
                      << nb << "\n";
        }
        return false;
    }
    attempt.state_lambda_solved = true;
    attempt.state_lambda_ratio = state_ratio;
    attempt.state_required_ratio = config.ar_ratio_threshold;
    if (config.use_clas_osr_filter) {
        attempt.state_required_ratio =
            std::max(attempt.state_required_ratio, claslibRatioThresholdForNb(nb));
    }

    VectorXd dd_fixed = VectorXd::Zero(nb);
    int mismatch_count = 0;
    double max_abs_delta = 0.0;
    for (int row = 0; row < nb; ++row) {
        dd_fixed(row) = std::round(state_lambda_fixed(row));
        const double delta = dd_fixed(row) - wlnl_fixed(row);
        max_abs_delta = std::max(max_abs_delta, std::abs(delta));
        if (std::abs(delta) > 0.25) {
            ++mismatch_count;
        }
        state_lambda_fixed(row) = dd_fixed(row);
    }
    attempt.state_wlnl_mismatch_count = mismatch_count;
    attempt.state_wlnl_max_abs_delta_cycles = max_abs_delta;

    Eigen::LDLT<MatrixXd> ldlt(dd_cov);
    if (ldlt.info() != Eigen::Success) {
        return false;
    }
    const VectorXd dd_residual = dd_float - dd_fixed;
    const VectorXd solved_residual = ldlt.solve(dd_residual);
    if (ldlt.info() != Eigen::Success || !solved_residual.allFinite()) {
        return false;
    }

    attempt.constrained_state = filter_state;
    const VectorXd delta = Qab * solved_residual;
    attempt.constrained_state.state.head(filter_state.amb_index) -= delta;
    attempt.state_dd_residual_norm = dd_residual.norm();
    attempt.state_position_shift_m =
        delta.segment(filter_state.pos_index, 3).norm();

    const MatrixXd qbi =
        ldlt.solve(MatrixXd::Identity(nb, nb));
    if (ldlt.info() == Eigen::Success && qbi.allFinite()) {
        MatrixXd head_cov =
            P_constraint.topLeftCorner(filter_state.amb_index,
                                       filter_state.amb_index) -
            Qab * qbi * Qab.transpose();
        head_cov = 0.5 * (head_cov + head_cov.transpose());
        attempt.constrained_state.covariance
            .topLeftCorner(filter_state.amb_index, filter_state.amb_index) =
            head_cov;
    }

    for (int row = 0; row < nb; ++row) {
        const auto& entry = rows[static_cast<size_t>(row)];
        const double ref_cycles =
            attempt.constrained_state.state(entry.ref_state) / entry.ref_scale_m;
        const double fixed_sat_cycles = ref_cycles - dd_fixed(row);
        attempt.constrained_state.state(entry.sat_state) =
            fixed_sat_cycles * entry.sat_scale_m;
    }

    attempt.has_constrained_state = true;
    attempt.state_lambda_used = true;
    attempt.nb = nb;

    if (debug_enabled) {
        std::cerr << "[PPP-WLNL-RESAMB] state fixed: nb=" << nb
                  << " ratio=" << state_ratio
                  << " threshold=" << attempt.state_required_ratio
                  << " dd_resid=" << attempt.state_dd_residual_norm
                  << " pos_shift=" << attempt.state_position_shift_m
                  << " wlnl_mismatch=" << attempt.state_wlnl_mismatch_count
                  << " wlnl_max_delta="
                  << attempt.state_wlnl_max_abs_delta_cycles
                  << " wlnl_noninteger="
                  << attempt.state_wlnl_noninteger_count
                  << " wlnl_max_frac="
                  << attempt.state_wlnl_max_fractional_cycles << "\n";
        for (int row = 0; row < std::min(nb, 6); ++row) {
            const auto& entry = rows[static_cast<size_t>(row)];
            std::cerr << "[PPP-WLNL-RESAMB] row " << row
                      << " " << entry.band
                      << " ref=" << entry.ref_satellite.toString()
                      << " sat=" << entry.sat_satellite.toString()
                      << " float=" << dd_float(row)
                      << " fixed=" << dd_fixed(row)
                      << " state_lambda="
                      << (attempt.state_lambda_solved
                              ? state_lambda_fixed(row)
                              : std::numeric_limits<double>::quiet_NaN())
                      << " sigma="
                      << std::sqrt(std::max(0.0, dd_cov(row, row)))
                      << "\n";
        }
    }

    return true;
}

WlnlFixAttempt tryWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled) {
    WlnlFixAttempt attempt;

    std::vector<int> wl_fixed_indices;
    wl_fixed_indices.reserve(satellites.size());
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        const auto ambiguity_it = ambiguity_states.find(satellites[static_cast<size_t>(i)]);
        if (ambiguity_it == ambiguity_states.end() || !ambiguity_it->second.wl_is_fixed) {
            continue;
        }
        wl_fixed_indices.push_back(i);
    }

    std::map<WlnlGroupKey, int> system_ref_map;
    for (const int idx : wl_fixed_indices) {
        const auto nl_it = nl_info.find(satellites[static_cast<size_t>(idx)]);
        if (nl_it == nl_info.end() || !nl_it->second.valid) {
            continue;
        }
        const auto& group = nl_it->second.group;
        if (system_ref_map.find(group) == system_ref_map.end()) {
            system_ref_map[group] = idx;
        } else {
            const auto& current_ref =
                ambiguity_states.at(satellites[static_cast<size_t>(system_ref_map[group])]);
            const auto& candidate = ambiguity_states.at(satellites[static_cast<size_t>(idx)]);
            if (candidate.lock_count > current_ref.lock_count) {
                system_ref_map[group] = idx;
            }
        }
    }

    std::vector<WlnlDdPair> dd_pairs;
    for (const int idx : wl_fixed_indices) {
        const auto nl_it = nl_info.find(satellites[static_cast<size_t>(idx)]);
        if (nl_it == nl_info.end() || !nl_it->second.valid) {
            continue;
        }
        const auto ref_it = system_ref_map.find(nl_it->second.group);
        if (ref_it == system_ref_map.end() || ref_it->second == idx) {
            continue;
        }
        dd_pairs.push_back({ref_it->second, idx});
    }

    const int dd_pair_count = static_cast<int>(dd_pairs.size());
    attempt.nb = dd_pair_count;
    if (dd_pair_count < 4) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient DD NL pairs: "
                      << dd_pair_count << "\n";
        }
        return attempt;
    }

    VectorXd dd_nl_float = VectorXd::Zero(dd_pair_count);
    MatrixXd dd_nl_cov = MatrixXd::Identity(dd_pair_count, dd_pair_count) * 1.0;

    int valid_dd = 0;
    for (int k = 0; k < dd_pair_count; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto ref_it = nl_info.find(satellites[static_cast<size_t>(ri)]);
        const auto sat_it = nl_info.find(satellites[static_cast<size_t>(si)]);
        if (ref_it == nl_info.end() || !ref_it->second.valid ||
            sat_it == nl_info.end() || !sat_it->second.valid) {
            dd_nl_float(k) = 0.0;
            dd_nl_cov(k, k) = 1e10;
            continue;
        }
        dd_nl_float(k) = ref_it->second.nl_ambiguity_cycles - sat_it->second.nl_ambiguity_cycles;
        ++valid_dd;

        if (debug_enabled && k < 5) {
            const double frac = dd_nl_float(k) - std::round(dd_nl_float(k));
            std::cerr << "[PPP-WLNL] DD NL pair " << k
                      << " ref=" << satellites[static_cast<size_t>(ri)].toString()
                      << " sat=" << satellites[static_cast<size_t>(si)].toString()
                      << " nl=" << dd_nl_float(k) << " frac=" << frac << "\n";
        }
    }

    if (valid_dd < 4) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient valid DD NL: " << valid_dd << "\n";
        }
        return attempt;
    }

    VectorXd dd_nl_fixed = VectorXd::Zero(dd_pair_count);
    if (!lambdaSearch(dd_nl_float, dd_nl_cov, dd_nl_fixed, attempt.ratio)) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL lambda search failed, nb="
                      << dd_pair_count << "\n";
        }
        return attempt;
    }
    if (!std::isfinite(attempt.ratio) || attempt.ratio < config.ar_ratio_threshold) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL ratio reject: nb=" << dd_pair_count
                      << " ratio=" << attempt.ratio
                      << " threshold=" << config.ar_ratio_threshold << "\n";
        }
        return attempt;
    }

    if (pppEnvOverrides().clas_resamb &&
        !conditionWlnlFilterStateDd(
            config,
            filter_state,
            constraint_covariance,
            ambiguity_states,
            satellites,
            state_indices,
            dd_pairs,
            dd_nl_fixed,
            attempt,
            debug_enabled)) {
        return WlnlFixAttempt{};
    }

    for (const auto& [group, ref_idx] : system_ref_map) {
        (void)group;
        const auto ref_nl_it = nl_info.find(satellites[static_cast<size_t>(ref_idx)]);
        if (ref_nl_it == nl_info.end() || !ref_nl_it->second.valid) {
            continue;
        }
        auto& ambiguity = ambiguity_states[satellites[static_cast<size_t>(ref_idx)]];
        ambiguity.is_fixed = true;
        ambiguity.nl_is_fixed = true;
        ambiguity.nl_fixed_cycles = std::round(ref_nl_it->second.nl_ambiguity_cycles);
    }

    for (int k = 0; k < dd_pair_count; ++k) {
        const int ri = dd_pairs[static_cast<size_t>(k)].ref_idx;
        const int si = dd_pairs[static_cast<size_t>(k)].sat_idx;
        const auto& ref_ambiguity = ambiguity_states.at(satellites[static_cast<size_t>(ri)]);
        if (!ref_ambiguity.nl_is_fixed) {
            continue;
        }
        auto& sat_ambiguity = ambiguity_states[satellites[static_cast<size_t>(si)]];
        sat_ambiguity.is_fixed = true;
        sat_ambiguity.nl_is_fixed = true;
        sat_ambiguity.nl_fixed_cycles = ref_ambiguity.nl_fixed_cycles - dd_nl_fixed(k);
    }

    attempt.fixed = true;
    return attempt;
}

std::map<SatelliteId, WlnlNlInfo> buildWlnlNlInfoMap(
    const std::vector<SatelliteId>& satellites,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const WlnlNlInfoProvider& provider) {
    std::map<SatelliteId, WlnlNlInfo> nl_info;
    for (const auto& satellite : satellites) {
        const auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end() || !ambiguity_it->second.wl_is_fixed) {
            continue;
        }
        WlnlNlInfo info;
        if (!provider || !provider(satellite, info) || !info.valid) {
            continue;
        }
        nl_info[satellite] = std::move(info);
    }
    return nl_info;
}

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled) {
    const auto nl_info = buildWlnlNlInfoMap(
        eligible_ambiguities.satellites,
        ambiguity_states,
        provider);
    return tryWlnlFix(
        config,
        filter_state,
        constraint_covariance,
        ambiguity_states,
        eligible_ambiguities.satellites,
        eligible_ambiguities.state_indices,
        nl_info,
        debug_enabled);
}

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled) {
    return resolveWlnlFix(
        config,
        filter_state,
        MatrixXd{},
        ambiguity_states,
        eligible_ambiguities,
        provider,
        debug_enabled);
}

std::vector<FixedNlObservation> buildFixedNlObservations(
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const FixedNlObservationProvider& provider) {
    std::vector<FixedNlObservation> observations;
    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (!ambiguity.is_fixed || !ambiguity.wl_is_fixed || !ambiguity.nl_is_fixed) {
            continue;
        }
        FixedNlObservation observation;
        if (!provider || !provider(satellite, ambiguity, observation)) {
            continue;
        }
        observations.push_back(std::move(observation));
    }
    return observations;
}

std::vector<FixedCarrierObservation> buildFixedCarrierObservations(
    size_t candidate_count,
    const FixedCarrierObservationProvider& provider) {
    std::vector<FixedCarrierObservation> observations;
    observations.reserve(candidate_count);
    for (size_t index = 0; index < candidate_count; ++index) {
        FixedCarrierObservation observation;
        if (!provider || !provider(index, observation)) {
            continue;
        }
        observations.push_back(std::move(observation));
    }
    return observations;
}

bool solveFixedNlPosition(
    const std::vector<FixedNlObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    const GNSSTime& time,
    const TropMappingFunction& trop_mapping_function,
    Vector3d& fixed_position,
    double* position_shift_norm_m,
    double* final_clock_m) {
    if (fixed_observations.size() < 4) {
        return false;
    }

    Vector3d position = initial_position;
    double clock_m = initial_clock_m;

    for (int iter = 0; iter < 5; ++iter) {
        const int nobs = static_cast<int>(fixed_observations.size());
        MatrixXd H = MatrixXd::Zero(nobs, 4);
        VectorXd residuals = VectorXd::Zero(nobs);

        for (int i = 0; i < nobs; ++i) {
            const auto& fixed_observation = fixed_observations[static_cast<size_t>(i)];
            const double geo = geodist(fixed_observation.sat_pos, position);
            const Vector3d los = (fixed_observation.sat_pos - position).normalized();
            const double elevation = std::asin(los.dot(position.normalized()));
            const double trop_delay =
                fixed_observation.use_trop_model && trop_mapping_function
                    ? trop_mapping_function(position, elevation, time) * trop_zenith
                    : 0.0;

            const double predicted = geo + clock_m
                                     - constants::SPEED_OF_LIGHT * fixed_observation.sat_clk
                                     + trop_delay
                                     + fixed_observation.extra_prediction_m
                                     + fixed_observation.fixed_nl_cycles *
                                           fixed_observation.lambda_nl_m;
            residuals(i) = fixed_observation.nl_phase_m - predicted;
            H(i, 0) = -los.x();
            H(i, 1) = -los.y();
            H(i, 2) = -los.z();
            H(i, 3) = 1.0;
        }

        const MatrixXd HTH = H.transpose() * H;
        const VectorXd dx = HTH.ldlt().solve(H.transpose() * residuals);
        if (!dx.allFinite()) {
            return false;
        }
        position += dx.head(3);
        clock_m += dx(3);

        if (dx.head(3).norm() < 1e-4) {
            break;
        }
    }

    fixed_position = position;
    if (position_shift_norm_m != nullptr) {
        *position_shift_norm_m = (position - initial_position).norm();
    }
    if (final_clock_m != nullptr) {
        *final_clock_m = clock_m;
    }
    return true;
}

bool solveFixedCarrierPosition(
    const std::vector<FixedCarrierObservation>& fixed_observations,
    const Vector3d& initial_position,
    double initial_clock_m,
    double trop_zenith,
    bool estimate_troposphere,
    Vector3d& fixed_position) {
    if (fixed_observations.size() < 4) {
        return false;
    }

    Vector3d position = initial_position;
    double clock_m = initial_clock_m;

    for (int iter = 0; iter < 6; ++iter) {
        const int nobs = static_cast<int>(fixed_observations.size());
        MatrixXd H = MatrixXd::Zero(nobs, 4);
        MatrixXd W = MatrixXd::Zero(nobs, nobs);
        VectorXd residuals = VectorXd::Zero(nobs);

        for (int i = 0; i < nobs; ++i) {
            const auto& fixed_observation = fixed_observations[static_cast<size_t>(i)];
            const Vector3d range_vector = fixed_observation.satellite_position - position;
            const double geometric_range = range_vector.norm();
            if (!std::isfinite(geometric_range) || geometric_range <= 1.0) {
                return false;
            }
            const Vector3d line_of_sight = range_vector / geometric_range;
            const double trop_delay =
                estimate_troposphere
                    ? fixed_observation.trop_mapping * trop_zenith
                    : fixed_observation.modeled_trop_delay_m;
            const double predicted =
                geometric_range + clock_m + fixed_observation.system_clock_offset_m
                - constants::SPEED_OF_LIGHT * fixed_observation.satellite_clock_bias_s
                + trop_delay - fixed_observation.ionosphere_m + fixed_observation.ambiguity_m;
            residuals(i) = fixed_observation.carrier_phase_if - predicted;
            H(i, 0) = -line_of_sight.x();
            H(i, 1) = -line_of_sight.y();
            H(i, 2) = -line_of_sight.z();
            H(i, 3) = 1.0;
            W(i, i) = 1.0 / safeVarianceFloor(fixed_observation.variance_cp, 1e-8);
        }

        const MatrixXd normal = H.transpose() * W * H;
        const VectorXd rhs = H.transpose() * W * residuals;
        const VectorXd dx = normal.ldlt().solve(rhs);
        if (!dx.allFinite()) {
            return false;
        }
        position += dx.head(3);
        clock_m += dx(3);
        if (dx.head(3).norm() < 1e-4) {
            break;
        }
    }

    fixed_position = position;
    return true;
}

}  // namespace libgnss::ppp_ar
