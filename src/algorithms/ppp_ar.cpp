#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>

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
    int min_lock_count,
    const GNSSTime& time,
    double slip_ar_exclusion_seconds) {
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
        if (time.week > 0 && ambiguity.has_last_slip_time &&
            slip_ar_exclusion_seconds > 0.0 &&
            (time - ambiguity.last_slip_time) < slip_ar_exclusion_seconds) {
            ++eligible.skipped_slip_window;
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
    const GNSSTime& time,
    bool debug_enabled) {
    WlnlPreparation preparation;
    preparation.min_lock_count = use_ssr_products
        ? std::max(1, config.wl_min_averaging_epochs)
        : config.convergence_min_epochs;
    // MRTKLIB literal-port track (direct state-DD path): ddmat admits a
    // satellite once lock[f] > 0, where a slip resets lock to -minlock
    // (clas.toml lock_count = 5), i.e. 6 valid epochs after a reset. The
    // WL-averaging floor (20 epochs) belongs to the WL/NL cascade, which
    // the direct path does not use.
    if (config.clas_mrtklib_float_parity &&
        config.use_clas_osr_filter && config.kinematic_mode &&
        !config.low_dynamics_mode && config.use_dynamics_model) {
        preparation.min_lock_count = 6;
    }
    // MRTKLIB parity (kinematic CLAS): post-slip AR re-eligibility is purely
    // lock-count based -- a slip sets lock = -minlock and the satellite
    // becomes DD-eligible again once lock > 0 (minlock+1 = 6 epochs = 1.2 s at
    // 5 Hz; mrtk_ppp_rtk.c:875/1718/2561). resetAmbiguity() already zeroes
    // lock_count on every slip, so the min_lock_count check above provides the
    // same behaviour; the additional 10 s wall-clock exclusion has no MRTKLIB
    // counterpart and starves urban AR (50 epochs at 5 Hz vs MRTKLIB's 6).
    // Non-kinematic paths keep the historical 10 s window.
    const double slip_ar_exclusion_seconds =
        (config.use_clas_osr_filter && config.kinematic_mode) ? 0.0 : 10.0;
    preparation.eligible_ambiguities = collectEligibleAmbiguities(
        filter_state, ambiguity_states, preparation.min_lock_count, time,
        slip_ar_exclusion_seconds);
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

// Resolve a satellite's L2 ambiguity state endpoint. The generic per-freq
// path registers L2 states in ambiguity_l2_indices, but the CLAS per-freq
// path stores them as prn+100 pseudo-satellites inside ambiguity_indices --
// leaving ambiguity_l2_indices empty and (before this fallback) silently
// dropping every L2 row from the state-DD system: measured on tokyo_run2
// dynamics, all 13k+ resamb attempts and every hold constraint were L1-only,
// i.e. LAMBDA searched half of MRTKLIB's nb with no L2 integer constraint.
bool resolveL2StateEndpoint(
    const ppp_shared::PPPState& filter_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const SatelliteId& satellite,
    double wavelength_l2_hint,
    bool allow_pseudo_states,
    int& state_index,
    double& scale_m) {
    const auto l2_it = filter_state.ambiguity_l2_indices.find(satellite);
    if (l2_it != filter_state.ambiguity_l2_indices.end()) {
        state_index = l2_it->second;
        scale_m = wavelength_l2_hint;
        return validStateDdEndpoint(state_index, scale_m,
                                    filter_state.total_states);
    }
    if (!allow_pseudo_states) {
        return false;
    }
    const SatelliteId l2_satellite(
        satellite.system,
        static_cast<uint8_t>(
            std::min(255, static_cast<int>(satellite.prn) + 100)));
    const auto idx_it = filter_state.ambiguity_indices.find(l2_satellite);
    const auto amb_it = ambiguity_states.find(l2_satellite);
    if (idx_it == filter_state.ambiguity_indices.end() ||
        amb_it == ambiguity_states.end()) {
        return false;
    }
    state_index = idx_it->second;
    scale_m = amb_it->second.ambiguity_scale_m;
    return validStateDdEndpoint(state_index, scale_m,
                                filter_state.total_states);
}

bool solveStateDdRows(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    const std::vector<StateDdRow>& rows,
    bool have_wlnl_datum,
    int min_rows,
    WlnlFixAttempt& attempt,
    bool debug_enabled);

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

        // Dynamics-gated: the L2 pseudo-state rows change LAMBDA's system;
        // the white-noise CLAS path keeps its historical L1-only resamb
        // (verified: with L2 rows enabled the white-noise ratio never clears
        // the table and its fix rate collapses to zero).
        const bool allow_l2_pseudo =
            config.use_clas_osr_filter && config.kinematic_mode &&
            config.use_dynamics_model && !config.low_dynamics_mode;
        int ref_l2_state = -1;
        int sat_l2_state = -1;
        double ref_l2_scale = 0.0;
        double sat_l2_scale = 0.0;
        if (resolveL2StateEndpoint(filter_state, ambiguity_states, ref_sat,
                                   ref_amb_it->second.wavelength_l2,
                                   allow_l2_pseudo,
                                   ref_l2_state, ref_l2_scale) &&
            resolveL2StateEndpoint(filter_state, ambiguity_states, sat,
                                   sat_amb_it->second.wavelength_l2,
                                   allow_l2_pseudo,
                                   sat_l2_state, sat_l2_scale)) {
            rows.push_back({
                ref_sat,
                sat,
                ref_l2_state,
                sat_l2_state,
                ref_l2_scale,
                sat_l2_scale,
                l2_fixed_cycles,
                "L2",
            });
        }
    }

    return solveStateDdRows(config, filter_state, constraint_covariance,
                            rows, /*have_wlnl_datum=*/true, /*min_rows=*/4,
                            attempt, debug_enabled);
}

// LAMBDA solve + F-table ratio test + conditional (xa) update over a
// prepared set of ambiguity state-DD rows (shared by the WL/NL-datum path
// and the MRTKLIB-parity direct ddmat-style path).
bool solveStateDdRows(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    const std::vector<StateDdRow>& rows,
    bool have_wlnl_datum,
    int min_rows,
    WlnlFixAttempt& attempt,
    bool debug_enabled) {
    const int nb = static_cast<int>(rows.size());
    attempt.state_dd_count = nb;
    // WL/NL-datum path: historical 4-row floor. MRTKLIB's minamb = 6 counts
    // DD rows in a state where every satellite carries valid L1 and L2 bias
    // states; the WL-datum rows are sparser (L2 endpoints are frequently
    // invalid), so 6 there would be effectively stricter than MRTKLIB --
    // measured on tokyo_run2 white-noise it eliminated every fix (row count
    // peaked at 5). The direct ddmat-style path passes MRTKLIB's minamb (6,
    // clas.toml partial_ar.min_ambiguities).
    if (nb < min_rows) {
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
    if (have_wlnl_datum) {
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
    if (config.use_clas_osr_filter && config.kinematic_mode) {
        // MRTKLIB parity: the ratio threshold is the nb-dependent F-test
        // table ONLY (mrtk_ppp_rtk.c:2062-2063, thres = qf[alpha][nb-1] with
        // clas.toml alpha = "10%"); opt.thresar plays no role when alpha is
        // configured. Taking max(3.0, table) was stricter than MRTKLIB for
        // every nb >= 7 (table drops to 2.78 at nb=7, 2.15 at nb=12).
        attempt.state_required_ratio = claslibRatioThresholdForNb(nb);
    } else if (config.use_clas_osr_filter) {
        attempt.state_required_ratio =
            std::max(attempt.state_required_ratio, claslibRatioThresholdForNb(nb));
    }

    VectorXd dd_fixed = VectorXd::Zero(nb);
    int mismatch_count = 0;
    double max_abs_delta = 0.0;
    for (int row = 0; row < nb; ++row) {
        dd_fixed(row) = std::round(state_lambda_fixed(row));
        if (have_wlnl_datum) {
            const double delta = dd_fixed(row) - wlnl_fixed(row);
            max_abs_delta = std::max(max_abs_delta, std::abs(delta));
            if (std::abs(delta) > 0.25) {
                ++mismatch_count;
            }
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

    if (pppEnvOverrides().clas_resamb) {
        if (!std::isfinite(state_ratio) ||
            state_ratio < attempt.state_required_ratio) {
            if (debug_enabled) {
                std::cerr << "[PPP-WLNL-RESAMB] state ratio reject: ratio="
                          << state_ratio
                          << " threshold=" << attempt.state_required_ratio
                          << "\n";
            }
            return false;
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
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad,
    const std::set<SatelliteId>& excluded_real_satellites) {
    WlnlFixAttempt attempt;

    std::vector<int> wl_fixed_indices;
    wl_fixed_indices.reserve(satellites.size());
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        const SatelliteId real_satellite =
            clasRealSatellite(satellites[static_cast<size_t>(i)]);
        if (excluded_real_satellites.count(real_satellite) != 0) {
            continue;
        }
        if (config.use_clas_osr_filter &&
            config.kinematic_mode &&
            satellite_elevations_rad != nullptr) {
            const auto elevation_it = satellite_elevations_rad->find(satellites[static_cast<size_t>(i)]);
            if (elevation_it == satellite_elevations_rad->end() ||
                elevation_it->second < kMrtklibArElevationMaskRad) {
                continue;
            }
        }
        const auto ambiguity_it = ambiguity_states.find(satellites[static_cast<size_t>(i)]);
        if (ambiguity_it == ambiguity_states.end() || !ambiguity_it->second.wl_is_fixed) {
            continue;
        }
        wl_fixed_indices.push_back(i);
    }

    const bool qzss_wlnl_ref_by_elevation =
        pppEnvOverrides().clas_qzss_s_prn_fix &&
        satellite_elevations_rad != nullptr;

    std::map<WlnlGroupKey, int> system_ref_map;
    for (const int idx : wl_fixed_indices) {
        const auto nl_it = nl_info.find(satellites[static_cast<size_t>(idx)]);
        if (nl_it == nl_info.end() || !nl_it->second.valid) {
            continue;
        }
        const auto& group = nl_it->second.group;
        if (system_ref_map.find(group) == system_ref_map.end()) {
            system_ref_map[group] = idx;
            continue;
        }
        const int current_idx = system_ref_map[group];
        const SatelliteId& current_satellite =
            satellites[static_cast<size_t>(current_idx)];
        const SatelliteId& candidate_satellite =
            satellites[static_cast<size_t>(idx)];
        const bool use_qzss_elevation_ref =
            qzss_wlnl_ref_by_elevation && group.first == GNSSSystem::QZSS;
        if (use_qzss_elevation_ref) {
            const auto current_el_it = satellite_elevations_rad->find(current_satellite);
            const auto candidate_el_it = satellite_elevations_rad->find(candidate_satellite);
            const double current_el =
                current_el_it != satellite_elevations_rad->end() ? current_el_it->second
                                                                 : -1.0;
            const double candidate_el =
                candidate_el_it != satellite_elevations_rad->end() ? candidate_el_it->second
                                                                   : -1.0;
            if (candidate_el > current_el) {
                system_ref_map[group] = idx;
            }
            continue;
        }
        const auto& current_ref =
            ambiguity_states.at(current_satellite);
        const auto& candidate = ambiguity_states.at(candidate_satellite);
        if (candidate.lock_count > current_ref.lock_count) {
            system_ref_map[group] = idx;
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
    // MRTKLIB parity (kinematic CLAS): resamb_LAMBDA requires minamb = 6 DD
    // ROWS counting L1 and L2 separately (mrtk_ppp_rtk.c:2015, clas.toml
    // partial_ar.min_ambiguities = 6), i.e. 3 dual-band DD pairs. The
    // state-DD builder below enforces the 6-row floor; here 3 NL pairs
    // suffice. Non-kinematic paths keep the historical 4-pair floor.
    const bool kinematic_clas_nb_parity =
        config.use_clas_osr_filter && config.kinematic_mode;
    const int min_dd_pairs = kinematic_clas_nb_parity ? 3 : 4;
    if (dd_pair_count < min_dd_pairs) {
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

    if (valid_dd < min_dd_pairs) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] insufficient valid DD NL: " << valid_dd << "\n";
        }
        return attempt;
    }

    // On the kinematic CLAS path the MRTKLIB-parity fix decision is the
    // state-DD LAMBDA (resamb_LAMBDA analog) below; the WL/NL ratio test with
    // its identity DD covariance is too crude to be the gate there.
    const bool kinematic_clas_state_fix =
        config.use_clas_osr_filter && config.kinematic_mode;

    VectorXd dd_nl_fixed = VectorXd::Zero(dd_pair_count);
    if (!lambdaSearch(dd_nl_float, dd_nl_cov, dd_nl_fixed, attempt.ratio)) {
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] NL lambda search failed, nb="
                      << dd_pair_count << "\n";
        }
        return attempt;
    }
    const bool nl_ratio_ok =
        std::isfinite(attempt.ratio) && attempt.ratio >= config.ar_ratio_threshold;
    if (!nl_ratio_ok && !kinematic_clas_state_fix) {
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
    if (kinematic_clas_state_fix) {
        // MRTKLIB resamb_LAMBDA parity: the fixed solution xa comes from
        // LAMBDA on the filter ambiguity-state DDs, gated by the CLASLIB
        // nb-dependent ratio test. The WL/NL-derived integers are not required
        // to agree: (NL+WL)/2 can violate integer parity and the two ambiguity
        // datums differ, so a consistency gate would block xa structurally.
        WlnlFixAttempt state_attempt;
        const bool state_fix_ok =
            conditionWlnlFilterStateDd(
                config,
                filter_state,
                constraint_covariance,
                ambiguity_states,
                satellites,
                state_indices,
                dd_pairs,
                dd_nl_fixed,
                state_attempt,
                debug_enabled) &&
            state_attempt.has_constrained_state &&
            std::isfinite(state_attempt.state_lambda_ratio) &&
            state_attempt.state_lambda_ratio >=
                state_attempt.state_required_ratio;
        if (state_fix_ok) {
            attempt.constrained_state = state_attempt.constrained_state;
            attempt.has_constrained_state = true;
            // The state-DD LAMBDA is the fix decision; publish its ratio/nb
            // (MRTKLIB sol.ratio comes from the same resamb test).
            attempt.ratio = state_attempt.state_lambda_ratio;
            attempt.nb = state_attempt.state_dd_count;
            attempt.state_lambda_ratio = state_attempt.state_lambda_ratio;
            attempt.state_position_shift_m = state_attempt.state_position_shift_m;
            attempt.state_dd_residual_norm = state_attempt.state_dd_residual_norm;
        } else if (!nl_ratio_ok) {
            if (debug_enabled) {
                std::cerr << "[PPP-WLNL] state-DD reject: nl_ratio="
                          << attempt.ratio
                          << " state_ratio=" << state_attempt.state_lambda_ratio
                          << " state_threshold="
                          << state_attempt.state_required_ratio << "\n";
            }
            return WlnlFixAttempt{};
        }
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

WlnlFixAttempt runParExclusionLoop(
    WlnlFixAttempt best_attempt,
    const std::function<WlnlFixAttempt(const std::set<SatelliteId>&)>& try_fix,
    const std::map<SatelliteId, double>& satellite_elevations_rad,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled);

WlnlFixAttempt tryWlnlFixWithPar(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::vector<SatelliteId>& satellites,
    const std::vector<int>& state_indices,
    const std::map<SatelliteId, WlnlNlInfo>& nl_info,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad) {
    auto try_fix = [&](const std::set<SatelliteId>& excluded_real_satellites) {
        return tryWlnlFix(
            config,
            filter_state,
            constraint_covariance,
            ambiguity_states,
            satellites,
            state_indices,
            nl_info,
            debug_enabled,
            satellite_elevations_rad,
            excluded_real_satellites);
    };

    WlnlFixAttempt best_attempt = try_fix({});
    if (best_attempt.fixed ||
        !config.use_clas_osr_filter ||
        !config.kinematic_mode ||
        satellite_elevations_rad == nullptr ||
        satellite_elevations_rad->empty()) {
        return best_attempt;
    }
    return runParExclusionLoop(std::move(best_attempt),
                               try_fix,
                               *satellite_elevations_rad,
                               ambiguity_states,
                               debug_enabled);
}

// MRTKLIB partial-AR exclusion loop (mrtk_ppp_rtk.c:2237-2277), shared by
// the WL/NL-datum and direct state-DD fix paths.
WlnlFixAttempt runParExclusionLoop(
    WlnlFixAttempt best_attempt,
    const std::function<WlnlFixAttempt(const std::set<SatelliteId>&)>& try_fix,
    const std::map<SatelliteId, double>& satellite_elevations_rad,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled) {
    struct ParCandidate {
        SatelliteId real_satellite;
        double elevation_rad = -1.0;
        int mw_count = 0;
    };
    std::vector<ParCandidate> par_candidates;
    par_candidates.reserve(satellite_elevations_rad.size());
    for (const auto& [real_satellite, elevation_rad] : satellite_elevations_rad) {
        par_candidates.push_back({real_satellite, elevation_rad, 0});
    }
    for (auto& candidate : par_candidates) {
        const auto ambiguity_it = ambiguity_states.find(candidate.real_satellite);
        if (ambiguity_it != ambiguity_states.end()) {
            candidate.mw_count = ambiguity_it->second.mw_count;
        }
    }
    std::sort(par_candidates.begin(), par_candidates.end(),
              [](const ParCandidate& lhs, const ParCandidate& rhs) {
                  if (lhs.elevation_rad != rhs.elevation_rad) {
                      return lhs.elevation_rad < rhs.elevation_rad;
                  }
                  return lhs.mw_count < rhs.mw_count;
              });

    std::set<SatelliteId> excluded_real_satellites;
    // MRTKLIB partial AR floor (clas.toml): max_excluded_sats = 4,
    // min_ambiguities = 6 DD rows = 3 dual-band pairs = 4 real satellites.
    constexpr int kMrtklibParMinRealSatellites = 4;
    const int max_exclusions = std::min(
        4,
        std::max(0, static_cast<int>(par_candidates.size()) -
                        kMrtklibParMinRealSatellites));

    for (int iteration = 0; iteration < max_exclusions && !best_attempt.fixed; ++iteration) {
        SatelliteId best_exclusion;
        bool has_best_exclusion = false;
        double best_ratio = best_attempt.ratio;

        for (const auto& candidate : par_candidates) {
            if (excluded_real_satellites.count(candidate.real_satellite) != 0) {
                continue;
            }

            auto trial_exclusions = excluded_real_satellites;
            trial_exclusions.insert(candidate.real_satellite);
            WlnlFixAttempt trial_attempt = try_fix(trial_exclusions);
            if (trial_attempt.fixed) {
                best_attempt = std::move(trial_attempt);
                if (debug_enabled) {
                    std::cerr << "[PPP-WLNL] PAR fixed: excluded="
                              << candidate.real_satellite.toString()
                              << " nb=" << best_attempt.nb
                              << " ratio=" << best_attempt.ratio << "\n";
                }
                break;
            }

            if (trial_attempt.ratio > best_ratio) {
                best_ratio = trial_attempt.ratio;
                best_exclusion = candidate.real_satellite;
                has_best_exclusion = true;
            }
        }

        if (best_attempt.fixed || !has_best_exclusion) {
            break;
        }

        excluded_real_satellites.insert(best_exclusion);
        if (debug_enabled) {
            std::cerr << "[PPP-WLNL] PAR exclude candidate: "
                      << best_exclusion.toString()
                      << " ratio=" << best_ratio << "\n";
        }
    }

    return best_attempt;
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

// MRTKLIB resamb_LAMBDA / ddmat parity (mrtk_ppp_rtk.c:1470-1537, 1701):
// the DD system is built directly from the filter's SD ambiguity states --
// every eligible (locked, unslipped, el >= elmaskar, observed-this-epoch)
// satellite contributes an L1 and an L2 DD row against the group reference,
// with NO wide-lane fix prerequisite. The reference satellite is the first
// eligible one in PRN order per (system, frequency) group, exactly like
// ddmat's scan. LAMBDA + the alpha-10% F-table ratio test + the conditional
// xa update then run on that system via solveStateDdRows.
WlnlFixAttempt tryDirectStateDdFix(
    const ppp_shared::PPPConfig& config,
    const ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad,
    const std::set<SatelliteId>& excluded_real_satellites) {
    WlnlFixAttempt attempt;
    if (satellite_elevations_rad == nullptr) {
        return attempt;
    }
    const auto& satellites = eligible_ambiguities.satellites;
    const auto& state_indices = eligible_ambiguities.state_indices;
    const auto& scales = eligible_ambiguities.scales;

    struct GroupMember {
        int index;
        SatelliteId satellite;
        uint8_t real_prn;
    };
    std::map<std::pair<GNSSSystem, int>, std::vector<GroupMember>> groups;
    for (int i = 0; i < static_cast<int>(satellites.size()); ++i) {
        const SatelliteId real_satellite =
            clasRealSatellite(satellites[static_cast<size_t>(i)]);
        if (excluded_real_satellites.count(real_satellite) != 0) {
            continue;
        }
        // ddmat: vsat (observed and used this epoch) + azel >= elmaskar.
        const auto el_it = satellite_elevations_rad->find(real_satellite);
        if (el_it == satellite_elevations_rad->end() ||
            el_it->second < kMrtklibArElevationMaskRad) {
            continue;
        }
        if (i >= static_cast<int>(state_indices.size()) ||
            i >= static_cast<int>(scales.size())) {
            continue;
        }
        groups[ambiguityDdGroup(satellites[static_cast<size_t>(i)])]
            .push_back({i,
                        satellites[static_cast<size_t>(i)],
                        real_satellite.prn});
    }

    std::vector<StateDdRow> rows;
    for (auto& [group, members] : groups) {
        if (members.size() < 2) {
            continue;
        }
        // ddmat scans states in PRN order; the first eligible satellite is
        // the group reference.
        std::sort(members.begin(), members.end(),
                  [](const GroupMember& lhs, const GroupMember& rhs) {
                      return lhs.real_prn < rhs.real_prn;
                  });
        const GroupMember& ref = members.front();
        for (size_t j = 1; j < members.size(); ++j) {
            rows.push_back({
                ref.satellite,
                members[j].satellite,
                state_indices[static_cast<size_t>(ref.index)],
                state_indices[static_cast<size_t>(members[j].index)],
                scales[static_cast<size_t>(ref.index)],
                scales[static_cast<size_t>(members[j].index)],
                0.0,
                group.second == 0 ? "L1" : "L2",
            });
        }
    }

    // MRTKLIB minamb: clas.toml partial_ar.min_ambiguities = 6.
    if (!solveStateDdRows(config, filter_state, constraint_covariance, rows,
                          /*have_wlnl_datum=*/false, /*min_rows=*/6,
                          attempt, debug_enabled)) {
        return WlnlFixAttempt{};
    }
    if (!attempt.has_constrained_state ||
        !std::isfinite(attempt.state_lambda_ratio) ||
        attempt.state_lambda_ratio < attempt.state_required_ratio) {
        if (debug_enabled) {
            std::cerr << "[PPP-RESAMB-DIRECT] ratio reject: nb="
                      << attempt.state_dd_count
                      << " ratio=" << attempt.state_lambda_ratio
                      << " threshold=" << attempt.state_required_ratio << "\n";
        }
        return WlnlFixAttempt{};
    }

    attempt.ratio = attempt.state_lambda_ratio;
    attempt.nb = attempt.state_dd_count;
    // Mark participants fixed so the hold path (buildWlnlHoldConstraints
    // requires wl_is_fixed && nl_is_fixed) sees them; the hold constraint
    // values come from the constrained fixed state, not these flags.
    for (const auto& row : rows) {
        for (const SatelliteId& satellite :
             {row.ref_satellite, row.sat_satellite}) {
            auto amb_it = ambiguity_states.find(satellite);
            if (amb_it != ambiguity_states.end()) {
                amb_it->second.is_fixed = true;
                amb_it->second.wl_is_fixed = true;
                amb_it->second.nl_is_fixed = true;
            }
        }
    }
    attempt.fixed = true;
    if (debug_enabled) {
        std::cerr << "[PPP-RESAMB-DIRECT] fixed: nb=" << attempt.nb
                  << " ratio=" << attempt.ratio
                  << " pos_shift=" << attempt.state_position_shift_m << "\n";
    }
    return attempt;
}

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    const MatrixXd& constraint_covariance,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad) {
    // MRTKLIB literal-port track: with float parity on the dynamics path the
    // fix comes from the direct ddmat-style state-DD system (no WL/NL
    // cascade prerequisite), wrapped in the same partial-AR exclusion loop.
    if (config.clas_mrtklib_float_parity &&
        config.use_clas_osr_filter && config.kinematic_mode &&
        !config.low_dynamics_mode && config.use_dynamics_model) {
        // MRTKLIB B2 position-variance gate (mrtk_ppp_rtk.c:2223-2233,
        // thresar[1] default 0.9999 m^2): skip AR entirely until the float
        // position has converged. Without it a fresh filter attempts LAMBDA
        // on a flat ambiguity covariance (ratio ~1, thousands of wasted
        // searches) and the rare ratio passes are wrong-integer fixes.
        {
            const MatrixXd& P_gate =
                constraint_covariance.rows() == filter_state.total_states
                    ? constraint_covariance
                    : filter_state.covariance;
            double posvar = 0.0;
            for (int axis = 0; axis < 3; ++axis) {
                posvar += P_gate(filter_state.pos_index + axis,
                                 filter_state.pos_index + axis);
            }
            posvar /= 3.0;
            constexpr double kMrtklibArPosVarGate = 0.9999;  // thresar[1]
            if (!(posvar <= kMrtklibArPosVarGate)) {
                if (debug_enabled) {
                    std::cerr << "[PPP-RESAMB-DIRECT] skip AR posvar="
                              << posvar << "\n";
                }
                return WlnlFixAttempt{};
            }
        }
        auto try_direct = [&](const std::set<SatelliteId>& excluded) {
            return tryDirectStateDdFix(
                config, filter_state, constraint_covariance, ambiguity_states,
                eligible_ambiguities, debug_enabled, satellite_elevations_rad,
                excluded);
        };
        WlnlFixAttempt best_attempt = try_direct({});
        if (best_attempt.fixed ||
            satellite_elevations_rad == nullptr ||
            satellite_elevations_rad->empty()) {
            return best_attempt;
        }
        return runParExclusionLoop(std::move(best_attempt),
                                   try_direct,
                                   *satellite_elevations_rad,
                                   ambiguity_states,
                                   debug_enabled);
    }
    const auto nl_info = buildWlnlNlInfoMap(
        eligible_ambiguities.satellites,
        ambiguity_states,
        provider);
    if (config.use_clas_osr_filter && config.kinematic_mode &&
        satellite_elevations_rad != nullptr) {
        return tryWlnlFixWithPar(
            config,
            filter_state,
            constraint_covariance,
            ambiguity_states,
            eligible_ambiguities.satellites,
            eligible_ambiguities.state_indices,
            nl_info,
            debug_enabled,
            satellite_elevations_rad);
    }
    return tryWlnlFix(
        config,
        filter_state,
        constraint_covariance,
        ambiguity_states,
        eligible_ambiguities.satellites,
        eligible_ambiguities.state_indices,
        nl_info,
        debug_enabled,
        satellite_elevations_rad);
}

WlnlFixAttempt resolveWlnlFix(
    const ppp_shared::PPPConfig& config,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const EligibleAmbiguities& eligible_ambiguities,
    const WlnlNlInfoProvider& provider,
    bool debug_enabled,
    const std::map<SatelliteId, double>* satellite_elevations_rad) {
    return resolveWlnlFix(
        config,
        filter_state,
        MatrixXd{},
        ambiguity_states,
        eligible_ambiguities,
        provider,
        debug_enabled,
        satellite_elevations_rad);
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

namespace {

constexpr double kClaslibSnrMaskDbHz[9] = {
    10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0};

double clasSnrMaskThresholdDbHz(int freq_index, double elevation_rad) {
    if (freq_index < 0 || freq_index >= 3) {
        return 0.0;
    }
    const double elevation_deg = elevation_rad * 180.0 / M_PI;
    double bin = (elevation_deg + 5.0) / 10.0;
    int index = static_cast<int>(std::floor(bin));
    bin -= index;
    if (index < 1) {
        return kClaslibSnrMaskDbHz[freq_index];
    }
    if (index > 8) {
        return kClaslibSnrMaskDbHz[8];
    }
    return (1.0 - bin) * kClaslibSnrMaskDbHz[index - 1] +
           bin * kClaslibSnrMaskDbHz[index];
}

}  // namespace

bool clasKinematicSnrMasked(int freq_index, double elevation_rad, double snr_dbhz) {
    if (!(snr_dbhz > 0.0)) {
        return false;
    }
    return snr_dbhz < clasSnrMaskThresholdDbHz(freq_index, elevation_rad);
}

void clearWlnlHoldState(WlnlHoldState& hold) {
    hold.active = false;
    hold.consecutive_fix_count = 0;
    hold.constraints.clear();
}

bool wlnlHoldStillValid(
    const WlnlHoldState& hold,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states) {
    if (!hold.active || hold.constraints.empty()) {
        return false;
    }
    for (const auto& constraint : hold.constraints) {
        for (const SatelliteId& satellite :
             {constraint.ref_satellite, constraint.sat_satellite}) {
            const auto ambiguity_it = ambiguity_states.find(satellite);
            if (ambiguity_it == ambiguity_states.end() ||
                ambiguity_it->second.needs_reinitialization) {
                return false;
            }
        }
    }
    return true;
}

bool buildWlnlHoldConstraints(
    const ppp_shared::PPPState& fixed_state,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const std::map<SatelliteId, double>& satellite_elevations_rad,
    std::vector<WlnlHoldConstraint>& constraints,
    bool allow_l2_pseudo_states) {
    constraints.clear();

    std::vector<SatelliteId> fixed_satellites;
    fixed_satellites.reserve(ambiguity_states.size());
    for (const auto& [satellite, ambiguity] : ambiguity_states) {
        if (!ambiguity.wl_is_fixed || !ambiguity.nl_is_fixed) {
            continue;
        }
        const auto state_it = fixed_state.ambiguity_indices.find(satellite);
        if (state_it == fixed_state.ambiguity_indices.end()) {
            continue;
        }
        fixed_satellites.push_back(satellite);
    }

    std::map<std::pair<GNSSSystem, int>, int> group_ref_map;
    for (int index = 0; index < static_cast<int>(fixed_satellites.size()); ++index) {
        const SatelliteId& satellite = fixed_satellites[static_cast<size_t>(index)];
        const auto ambiguity_it = ambiguity_states.find(satellite);
        if (ambiguity_it == ambiguity_states.end()) {
            continue;
        }
        const auto group = ambiguityDdGroup(satellite);
        if (group_ref_map.find(group) == group_ref_map.end()) {
            group_ref_map[group] = index;
            continue;
        }
        const int current_idx = group_ref_map[group];
        const SatelliteId& current_satellite =
            fixed_satellites[static_cast<size_t>(current_idx)];
        const auto current_el_it = satellite_elevations_rad.find(current_satellite);
        const auto candidate_el_it = satellite_elevations_rad.find(satellite);
        const double current_el =
            current_el_it != satellite_elevations_rad.end() ? current_el_it->second : -1.0;
        const double candidate_el =
            candidate_el_it != satellite_elevations_rad.end() ? candidate_el_it->second : -1.0;
        if (candidate_el > current_el) {
            group_ref_map[group] = index;
        }
    }

    auto add_constraint = [&](int ref_index, int sat_index) {
        const SatelliteId& ref_satellite =
            fixed_satellites[static_cast<size_t>(ref_index)];
        const SatelliteId& sat_satellite =
            fixed_satellites[static_cast<size_t>(sat_index)];
        const auto ref_amb_it = ambiguity_states.find(ref_satellite);
        const auto sat_amb_it = ambiguity_states.find(sat_satellite);
        const auto ref_state_it = fixed_state.ambiguity_indices.find(ref_satellite);
        const auto sat_state_it = fixed_state.ambiguity_indices.find(sat_satellite);
        if (ref_amb_it == ambiguity_states.end() ||
            sat_amb_it == ambiguity_states.end() ||
            ref_state_it == fixed_state.ambiguity_indices.end() ||
            sat_state_it == fixed_state.ambiguity_indices.end()) {
            return;
        }

        const double ref_l1_scale = ambiguityWavelengthL1(ref_amb_it->second);
        const double sat_l1_scale = ambiguityWavelengthL1(sat_amb_it->second);
        if (!validStateDdEndpoint(
                ref_state_it->second, ref_l1_scale, fixed_state.total_states) ||
            !validStateDdEndpoint(
                sat_state_it->second, sat_l1_scale, fixed_state.total_states)) {
            return;
        }

        // MRTKLIB holdamb(): the constraint target is the fixed-solution DD
        // (xa[ref]-xa[i]), i.e. the constrained state's ambiguity DD.
        const double fixed_dd_m =
            fixed_state.state(ref_state_it->second) -
            fixed_state.state(sat_state_it->second);

        WlnlHoldConstraint constraint;
        constraint.ref_state = ref_state_it->second;
        constraint.sat_state = sat_state_it->second;
        constraint.fixed_dd_m = fixed_dd_m;
        constraint.ambiguity_scale_m = ref_l1_scale;
        constraint.ref_satellite = ref_satellite;
        constraint.sat_satellite = sat_satellite;
        const auto ref_el_it = satellite_elevations_rad.find(ref_satellite);
        const auto sat_el_it = satellite_elevations_rad.find(sat_satellite);
        constraint.ref_elevation_rad =
            ref_el_it != satellite_elevations_rad.end() ? ref_el_it->second : 0.0;
        constraint.sat_elevation_rad =
            sat_el_it != satellite_elevations_rad.end() ? sat_el_it->second : 0.0;
        constraints.push_back(constraint);

        int ref_l2_state = -1;
        int sat_l2_state = -1;
        double ref_l2_scale = 0.0;
        double sat_l2_scale = 0.0;
        if (resolveL2StateEndpoint(fixed_state, ambiguity_states, ref_satellite,
                                   ref_amb_it->second.wavelength_l2,
                                   allow_l2_pseudo_states,
                                   ref_l2_state, ref_l2_scale) &&
            resolveL2StateEndpoint(fixed_state, ambiguity_states, sat_satellite,
                                   sat_amb_it->second.wavelength_l2,
                                   allow_l2_pseudo_states,
                                   sat_l2_state, sat_l2_scale)) {
            const double l2_fixed_dd_m =
                fixed_state.state(ref_l2_state) -
                fixed_state.state(sat_l2_state);
            WlnlHoldConstraint l2_constraint;
            l2_constraint.ref_state = ref_l2_state;
            l2_constraint.sat_state = sat_l2_state;
            l2_constraint.fixed_dd_m = l2_fixed_dd_m;
            l2_constraint.ambiguity_scale_m = ref_l2_scale;
            l2_constraint.ref_satellite = ref_satellite;
            l2_constraint.sat_satellite = sat_satellite;
            l2_constraint.ref_elevation_rad = constraint.ref_elevation_rad;
            l2_constraint.sat_elevation_rad = constraint.sat_elevation_rad;
            constraints.push_back(l2_constraint);
        }
    };

    for (int index = 0; index < static_cast<int>(fixed_satellites.size()); ++index) {
        const SatelliteId& satellite = fixed_satellites[static_cast<size_t>(index)];
        const auto group = ambiguityDdGroup(satellite);
        const auto ref_it = group_ref_map.find(group);
        if (ref_it == group_ref_map.end() || ref_it->second == index) {
            continue;
        }
        add_constraint(ref_it->second, index);
    }

    return !constraints.empty();
}

bool applyWlnlHoldAmbiguity(
    ppp_shared::PPPState& filter_state,
    const std::vector<WlnlHoldConstraint>& constraints,
    double hold_variance_cycles2,
    double hold_elevation_mask_rad) {
    std::vector<WlnlHoldConstraint> selected;
    selected.reserve(constraints.size());
    for (const auto& constraint : constraints) {
        if (constraint.ref_state < 0 ||
            constraint.sat_state < 0 ||
            constraint.ref_elevation_rad < hold_elevation_mask_rad ||
            constraint.sat_elevation_rad < hold_elevation_mask_rad) {
            continue;
        }
        selected.push_back(constraint);
    }
    if (selected.empty()) {
        return false;
    }

    rtk_measurement::MeasurementSystem system;
    system.design_matrix =
        MatrixXd::Zero(static_cast<int>(selected.size()), filter_state.total_states);
    system.residuals = VectorXd::Zero(static_cast<int>(selected.size()));
    system.covariance = MatrixXd::Zero(
        static_cast<int>(selected.size()), static_cast<int>(selected.size()));

    for (int row = 0; row < static_cast<int>(selected.size()); ++row) {
        const auto& constraint = selected[static_cast<size_t>(row)];
        const double float_dd =
            filter_state.state(constraint.ref_state) -
            filter_state.state(constraint.sat_state);
        system.residuals(row) = constraint.fixed_dd_m - float_dd;
        system.design_matrix(row, constraint.ref_state) = 1.0;
        system.design_matrix(row, constraint.sat_state) = -1.0;
        const double scale_m =
            constraint.ambiguity_scale_m > 0.0 ? constraint.ambiguity_scale_m : 0.19;
        system.covariance(row, row) = hold_variance_cycles2 * scale_m * scale_m;
    }

    auto hold_update = rtk_update::applyMeasurementUpdate(
        filter_state.state,
        filter_state.covariance,
        system,
        std::numeric_limits<double>::infinity(),
        1);
    return hold_update.ok;
}

}  // namespace libgnss::ppp_ar
