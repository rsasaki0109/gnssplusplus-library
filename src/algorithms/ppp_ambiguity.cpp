#include "ppp_internal.hpp"

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

bool PPPProcessor::resolveAmbiguities(const ObservationData& obs, const NavigationData& nav) {
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;

    if (!ppp_config_.enable_ambiguity_resolution || (!precise_products_loaded_ && !ssr_products_loaded_)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] skipped: enabled=" << ppp_config_.enable_ambiguity_resolution
                      << " precise=" << precise_products_loaded_ << "\n";
        }
        return false;
    }

    if (pppDebugEnabled()) {
        int total_amb = 0, ready_amb = 0;
        for (const auto& [satellite, state_index] : filter_state_.ambiguity_indices) {
            ++total_amb;
            const auto amb_it = ambiguity_states_.find(satellite);
            if (amb_it != ambiguity_states_.end()) {
                const auto& a = amb_it->second;
                if (!a.needs_reinitialization && a.lock_count >= ppp_config_.convergence_min_epochs &&
                    std::isfinite(a.ambiguity_scale_m) && a.ambiguity_scale_m > 0.0) {
                    ++ready_amb;
                }
            }
        }
        std::cerr << "[PPP-AR-DBG] total_amb=" << total_amb
                  << " ready=" << ready_amb
                  << " min_epochs=" << ppp_config_.convergence_min_epochs << "\n";
    }

    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        // In ionosphere-free mode the filter state holds a single IFLC
        // ambiguity per satellite, so recover the narrow-lane integer directly
        // from that state (decoupled CNES/Laurichesse cascade) rather than from
        // observation-domain narrow-lane phase, which would re-absorb the
        // current position error and fix to wrong integers.
        if (ppp_config_.use_ionosphere_free) {
            return resolveAmbiguitiesDecoupledIf(obs, nav);
        }
        // Per-frequency / CLAS OSR path: NL float values are computed from
        // corrected dual-freq observations.
        return resolveAmbiguitiesWLNL(obs, nav);
    }
    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_PER_FREQ &&
        !ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere) {
        return resolveAmbiguitiesPerFreq(obs, nav);
    }
    // DD_IFLC and DD_PER_FREQ fall through to existing DD-AR code below

    const int ar_min_lock = ssr_products_loaded_ ?
        std::min(ppp_config_.convergence_min_epochs, 10) :
        ppp_config_.convergence_min_epochs;
    const auto eligible_ambiguities = ppp_ar::collectEligibleAmbiguities(
        filter_state_, ambiguity_states_, ar_min_lock);

    if (static_cast<int>(eligible_ambiguities.satellites.size()) <
        ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] skipped: candidates="
                      << eligible_ambiguities.satellites.size()
                      << " min=" << ppp_config_.min_satellites_for_ar << "\n";
        }
        return false;
    }

    std::map<SatelliteId, double> real_satellite_elevations;
    if (ppp_config_.use_clas_osr_filter) {
        for (const auto& satellite : eligible_ambiguities.satellites) {
            const SatelliteId real_satellite = ppp_ar::clasRealSatellite(satellite);
            if (real_satellite_elevations.find(real_satellite) != real_satellite_elevations.end()) {
                continue;
            }

            Vector3d sat_pos;
            Vector3d sat_vel;
            double sat_clk = 0.0;
            double sat_drift = 0.0;
            if (!nav.calculateSatelliteState(real_satellite, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                continue;
            }
            if (ssr_products_loaded_) {
                Vector3d orbit_corr;
                double clock_corr = 0.0;
                if (ssr_products_.interpolateCorrection(real_satellite, obs.time, orbit_corr, clock_corr,
                                                        nullptr, nullptr, nullptr, nullptr)) {
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_corr = ssrRacToEcef(sat_pos, sat_vel, orbit_corr);
                    }
                    sat_pos += orbit_corr;
                }
            }
            const Vector3d receiver_position =
                filter_state_.state.segment(filter_state_.pos_index, 3);
            const Vector3d line_of_sight = sat_pos - receiver_position;
            const double line_of_sight_norm = line_of_sight.norm();
            if (line_of_sight_norm <= 0.0 || receiver_position.norm() <= 0.0) {
                continue;
            }
            const double elevation = std::asin(
                line_of_sight.normalized().dot(receiver_position.normalized()));
            real_satellite_elevations[real_satellite] = elevation;
        }
    }

    ppp_ar::DdFixAttempt best_attempt = ppp_ar::tryDirectDdFixWithPar(
        ppp_config_,
        filter_state_,
        pre_anchor_covariance_,
        ambiguity_states_,
        eligible_ambiguities,
        real_satellite_elevations,
        pppDebugEnabled());

    if (!best_attempt.fixed) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-AR] DD ratio reject: ratio=" << best_attempt.ratio
                      << " threshold=" << best_attempt.required_ratio << "\n";
        }
        return false;
    }

    last_ar_ratio_ = best_attempt.ratio;
    last_fixed_ambiguities_ = best_attempt.nb;
    filter_state_ = std::move(best_attempt.state);
    ambiguity_states_ = std::move(best_attempt.ambiguities);

    if (pppDebugEnabled()) {
        std::cerr << "[PPP-AR] DD fixed: nb=" << best_attempt.nb
                  << " ratio=" << best_attempt.ratio
                  << " threshold=" << best_attempt.required_ratio << "\n";
    }
    return true;
}

bool PPPProcessor::resolveAmbiguitiesDecoupledIf(const ObservationData& obs,
                                                 const NavigationData& nav) {
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;

    // 1) Collect eligible IFLC ambiguities (locked, finite scale).
    const int ar_min_lock = ssr_products_loaded_
        ? std::max(1, ppp_config_.wl_min_averaging_epochs)
        : ppp_config_.convergence_min_epochs;
    const auto eligible = ppp_ar::collectEligibleAmbiguities(
        filter_state_, ambiguity_states_, ar_min_lock);

    // 2) Fix wide-lane integers (N1-N2) from averaged Melbourne-Wubbena.
    ppp_ar::applyWideLaneFixes(
        ppp_config_, ambiguity_states_, eligible.satellites, pppDebugEnabled());

    // 3) Build per-satellite narrow-lane geometry from the IFLC signal pair.
    //    B_IF = lambda_NL * N1 + (c * f2 / (f1^2 - f2^2)) * N_WL, so the
    //    narrow-lane integer is recovered as
    //    N1 = (B_IF - if_wl_coeff * N_WL) / lambda_NL.
    struct NlCandidate {
        SatelliteId satellite;
        int state_index = -1;
        double lambda_nl_m = 0.0;
        double if_wl_coeff_m = 0.0;
        int wl_integer = 0;
    };
    std::vector<NlCandidate> candidates;
    candidates.reserve(eligible.satellites.size());
    for (size_t i = 0; i < eligible.satellites.size(); ++i) {
        const SatelliteId& sat = eligible.satellites[i];
        const auto amb_it = ambiguity_states_.find(sat);
        if (amb_it == ambiguity_states_.end() || !amb_it->second.wl_is_fixed) {
            continue;
        }
        const Observation* primary =
            findObservationForSignals(obs, sat, primarySignals(sat.system));
        const Observation* secondary =
            findObservationForSignals(obs, sat, secondarySignals(sat.system));
        if (primary == nullptr || secondary == nullptr) {
            continue;
        }
        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        const double f1 = signalFrequencyHz(primary->signal, eph);
        const double f2 = signalFrequencyHz(secondary->signal, eph);
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            continue;
        }
        NlCandidate candidate;
        candidate.satellite = sat;
        candidate.state_index = eligible.state_indices[i];
        candidate.lambda_nl_m = constants::SPEED_OF_LIGHT / (f1 + f2);
        candidate.if_wl_coeff_m = constants::SPEED_OF_LIGHT * f2 / (f1 * f1 - f2 * f2);
        candidate.wl_integer = amb_it->second.wl_fixed_integer;
        candidates.push_back(candidate);
    }

    if (pppDebugEnabled()) {
        std::map<GNSSSystem, int> elig_by_sys;
        std::map<GNSSSystem, int> wlfix_by_sys;
        for (const auto& sat : eligible.satellites) {
            elig_by_sys[sat.system]++;
            const auto it = ambiguity_states_.find(sat);
            if (it != ambiguity_states_.end() && it->second.wl_is_fixed) {
                wlfix_by_sys[sat.system]++;
            }
        }
        std::cerr << "[PPP-IFDEC-DBG] eligible/wlfix:";
        for (const auto& [sys, n] : elig_by_sys) {
            std::cerr << " sys" << static_cast<int>(sys) << "=" << n << "/" << wlfix_by_sys[sys];
        }
        std::cerr << " cand=" << candidates.size() << "\n";
    }

    if (static_cast<int>(candidates.size()) < ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-IFDEC] insufficient WL-fixed candidates: "
                      << candidates.size() << " min=" << ppp_config_.min_satellites_for_ar << "\n";
        }
        return false;
    }

    // 4) Form intra-system double differences against the first satellite of
    //    each system group.
    std::map<std::pair<GNSSSystem, int>, int> reference_map;
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto group = ppp_ar::ambiguityDdGroup(candidates[i].satellite);
        reference_map.emplace(group, static_cast<int>(i));
    }
    struct DdPair {
        int ref_idx = -1;
        int sat_idx = -1;
    };
    std::vector<DdPair> dd_pairs;
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto group = ppp_ar::ambiguityDdGroup(candidates[i].satellite);
        const int ref_idx = reference_map[group];
        if (ref_idx == static_cast<int>(i)) {
            continue;
        }
        dd_pairs.push_back({ref_idx, static_cast<int>(i)});
    }

    const int nb = static_cast<int>(dd_pairs.size());
    if (nb < ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-IFDEC] insufficient DD pairs: " << nb << "\n";
        }
        return false;
    }

    // 5) Narrow-lane DD float and covariance, all from the filter state.
    VectorXd n1_float = VectorXd::Zero(nb);
    MatrixXd n1_cov = MatrixXd::Zero(nb, nb);
    for (int k = 0; k < nb; ++k) {
        const NlCandidate& ref = candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(k)].ref_idx)];
        const NlCandidate& sat = candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(k)].sat_idx)];
        const double state_dd_m =
            filter_state_.state(ref.state_index) - filter_state_.state(sat.state_index);
        const double wl_dd = static_cast<double>(ref.wl_integer - sat.wl_integer);
        n1_float(k) = (state_dd_m - sat.if_wl_coeff_m * wl_dd) / sat.lambda_nl_m;

        for (int l = 0; l < nb; ++l) {
            const NlCandidate& ref2 =
                candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(l)].ref_idx)];
            const NlCandidate& sat2 =
                candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(l)].sat_idx)];
            const double cov_m2 =
                filter_state_.covariance(ref.state_index, ref2.state_index) -
                filter_state_.covariance(ref.state_index, sat2.state_index) -
                filter_state_.covariance(sat.state_index, ref2.state_index) +
                filter_state_.covariance(sat.state_index, sat2.state_index);
            n1_cov(k, l) = cov_m2 / (sat.lambda_nl_m * sat2.lambda_nl_m);
        }
    }

    // 6) Integer least squares on the narrow-lane double differences.
    VectorXd n1_fixed = VectorXd::Zero(nb);
    double ratio = 0.0;
    if (!lambdaSearch(n1_float, n1_cov, n1_fixed, ratio)) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-IFDEC] lambda search failed nb=" << nb << "\n";
        }
        return false;
    }
    if (!std::isfinite(ratio) || ratio < ppp_config_.ar_ratio_threshold) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-IFDEC] ratio reject: nb=" << nb << " ratio=" << ratio
                      << " threshold=" << ppp_config_.ar_ratio_threshold << "\n";
        }
        return false;
    }

    // 7) Apply the fixed narrow-lane integers as a tight Kalman pseudo-measurement
    //    on the IFLC ambiguity states.  The IFLC state is ionosphere-free, so the
    //    constraint propagates cleanly to position without poisoning later epochs.
    const int nx = filter_state_.total_states;
    MatrixXd design = MatrixXd::Zero(nb, nx);
    VectorXd innovation = VectorXd::Zero(nb);
    MatrixXd measurement_cov = MatrixXd::Zero(nb, nb);
    constexpr double hold_sigma_m = 1.0e-3;  // 1 mm constraint
    for (int k = 0; k < nb; ++k) {
        const NlCandidate& ref = candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(k)].ref_idx)];
        const NlCandidate& sat = candidates[static_cast<size_t>(dd_pairs[static_cast<size_t>(k)].sat_idx)];
        const double wl_dd = static_cast<double>(ref.wl_integer - sat.wl_integer);
        const double fixed_dd_m = sat.lambda_nl_m * n1_fixed(k) + sat.if_wl_coeff_m * wl_dd;
        const double float_dd_m =
            filter_state_.state(ref.state_index) - filter_state_.state(sat.state_index);
        innovation(k) = fixed_dd_m - float_dd_m;
        design(k, ref.state_index) = 1.0;
        design(k, sat.state_index) = -1.0;
        measurement_cov(k, k) = hold_sigma_m * hold_sigma_m;
    }

    const MatrixXd innovation_cov =
        design * filter_state_.covariance * design.transpose() + measurement_cov;
    const MatrixXd innovation_inverse =
        innovation_cov.ldlt().solve(MatrixXd::Identity(nb, nb));
    if (!innovation_inverse.allFinite()) {
        return false;
    }
    const MatrixXd gain = filter_state_.covariance * design.transpose() * innovation_inverse;
    filter_state_.state += gain * innovation;
    const MatrixXd identity = MatrixXd::Identity(nx, nx);
    const MatrixXd kh = gain * design;
    filter_state_.covariance =
        (identity - kh) * filter_state_.covariance * (identity - kh).transpose() +
        gain * measurement_cov * gain.transpose();

    for (const auto& candidate : candidates) {
        ambiguity_states_[candidate.satellite].is_fixed = true;
        ambiguity_states_[candidate.satellite].nl_is_fixed = true;
    }

    last_ar_ratio_ = ratio;
    last_fixed_ambiguities_ = nb;
    if (pppDebugEnabled()) {
        std::cerr << "[PPP-IFDEC] NL fixed: nb=" << nb << " ratio=" << ratio << "\n";
    }
    return true;
}

bool PPPProcessor::resolveAmbiguitiesPerFreq(const ObservationData& obs,
                                             const NavigationData& nav) {
    (void)nav;
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    // Only attempt AR once the float has converged: a wide, ill-conditioned
    // ambiguity covariance makes the LAMBDA integer search explore an enormous
    // volume (effectively unbounded run time).
    if (!converged_) {
        return false;
    }

    // Restrict AR to satellites observed this epoch (the persistent ambiguity
    // map accumulates set satellites whose stale states must not enter the DD).
    std::set<SatelliteId> observed_now;
    for (const auto& m : obs.observations) {
        if (m.valid && m.has_carrier_phase) {
            observed_now.insert(m.satellite);
        }
    }

    // 1) Collect satellites that have both per-frequency ambiguity states, an
    //    ionosphere state, finite wavelengths, and sufficient lock.
    struct Cand {
        SatelliteId sat;
        int l1_index = -1;
        int l2_index = -1;
        double lambda1 = 0.0;
        double lambda2 = 0.0;
        int wl_int = 0;
        bool wl_fixed = false;
    };
    const int min_lock = ssr_products_loaded_
        ? std::min(ppp_config_.convergence_min_epochs, 10)
        : ppp_config_.convergence_min_epochs;
    std::vector<Cand> cands;
    for (const auto& [sat, l1_index] : filter_state_.ambiguity_indices) {
        if (observed_now.count(sat) == 0) {
            continue;
        }
        const auto l2_it = filter_state_.ambiguity_l2_indices.find(sat);
        if (l2_it == filter_state_.ambiguity_l2_indices.end()) {
            continue;
        }
        const auto amb_it = ambiguity_states_.find(sat);
        if (amb_it == ambiguity_states_.end() ||
            amb_it->second.needs_reinitialization ||
            amb_it->second.lock_count < min_lock) {
            continue;
        }
        const double lam1 = amb_it->second.wavelength_l1;
        const double lam2 = amb_it->second.wavelength_l2;
        if (!(lam1 > 0.0) || !(lam2 > 0.0)) {
            continue;
        }
        if (l1_index < 0 || l1_index >= filter_state_.total_states ||
            l2_it->second < 0 || l2_it->second >= filter_state_.total_states) {
            continue;
        }
        Cand c;
        c.sat = sat;
        c.l1_index = l1_index;
        c.l2_index = l2_it->second;
        c.lambda1 = lam1;
        c.lambda2 = lam2;
        cands.push_back(c);
    }
    if (static_cast<int>(cands.size()) < ppp_config_.min_satellites_for_ar) {
        return false;
    }

    // 2) Wide-lane integer per satellite, double-differenced against the first
    //    candidate of each system. WL_s = x[L1]/lambda1 - x[L2]/lambda2 (cycles).
    std::map<GNSSSystem, int> ref_of_system;
    for (size_t i = 0; i < cands.size(); ++i) {
        ref_of_system.emplace(cands[i].sat.system, static_cast<int>(i));
    }
    struct DdPair { int ref = -1; int sat = -1; int wl_int = 0; };
    std::vector<DdPair> wl_pairs;
    const VectorXd& x = filter_state_.state;
    const MatrixXd& P = filter_state_.covariance;
    auto wl_cycles = [&](const Cand& c) {
        return x(c.l1_index) / c.lambda1 - x(c.l2_index) / c.lambda2;
    };
    constexpr double kMaxFracWl = 0.20;
    constexpr double kMaxStdWl = 1.0;
    int wl_fix_count = 0;
    double wl_frac_sum = 0.0;
    for (size_t i = 0; i < cands.size(); ++i) {
        const int ref_idx = ref_of_system[cands[i].sat.system];
        if (ref_idx == static_cast<int>(i)) {
            continue;
        }
        const Cand& ref = cands[static_cast<size_t>(ref_idx)];
        const Cand& sat = cands[i];
        const double wl_dd = wl_cycles(ref) - wl_cycles(sat);
        const double n = std::round(wl_dd);
        const double frac = std::abs(n - wl_dd);
        // Variance of the WL DD (cycles^2) from the 4 contributing states.
        const double cr1 = 1.0 / ref.lambda1, cr2 = -1.0 / ref.lambda2;
        const double cs1 = -1.0 / sat.lambda1, cs2 = 1.0 / sat.lambda2;
        const int ir1 = ref.l1_index, ir2 = ref.l2_index;
        const int is1 = sat.l1_index, is2 = sat.l2_index;
        double var = cr1 * cr1 * P(ir1, ir1) + cr2 * cr2 * P(ir2, ir2) +
                     cs1 * cs1 * P(is1, is1) + cs2 * cs2 * P(is2, is2) +
                     2.0 * (cr1 * cr2 * P(ir1, ir2) + cr1 * cs1 * P(ir1, is1) +
                            cr1 * cs2 * P(ir1, is2) + cr2 * cs1 * P(ir2, is1) +
                            cr2 * cs2 * P(ir2, is2) + cs1 * cs2 * P(is1, is2));
        const double sd = var > 0.0 ? std::sqrt(var) : 9.9;
        if (env_overrides_.pfdump) {
            std::cerr << "[PFWL] " << ref.sat.toString() << "-" << sat.sat.toString()
                      << " wl=" << wl_dd << " frac=" << frac << " std=" << sd << "\n";
        }
        if (frac < kMaxFracWl && sd < kMaxStdWl) {
            wl_pairs.push_back({ref_idx, static_cast<int>(i), static_cast<int>(n)});
            wl_fix_count++;
            wl_frac_sum += frac;
        }
    }
    if (env_overrides_.pfdump) {
        std::cerr << "[PFWL-SUM] cand=" << cands.size() << " wl_fixed=" << wl_fix_count
                  << " mean_frac=" << (wl_fix_count ? wl_frac_sum / wl_fix_count : 0.0) << "\n";
    }
    if (wl_pairs.empty()) {
        return false;
    }

    // 3) Apply the fixed wide-lane integers as a batched Kalman pseudo-measurement
    //    (cycles). Tight but not rigid so a wrong WL does not lock the filter.
    const int nx = filter_state_.total_states;
    const int nwl = static_cast<int>(wl_pairs.size());
    MatrixXd Hwl = MatrixXd::Zero(nwl, nx);
    VectorXd vwl = VectorXd::Zero(nwl);
    MatrixXd Rwl = MatrixXd::Zero(nwl, nwl);
    constexpr double kWlConstraintSigmaCyc = 0.10;
    for (int k = 0; k < nwl; ++k) {
        const Cand& ref = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].ref)];
        const Cand& sat = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].sat)];
        Hwl(k, ref.l1_index) += 1.0 / ref.lambda1;
        Hwl(k, ref.l2_index) += -1.0 / ref.lambda2;
        Hwl(k, sat.l1_index) += -1.0 / sat.lambda1;
        Hwl(k, sat.l2_index) += 1.0 / sat.lambda2;
        const double wl_dd_float = wl_cycles(ref) - wl_cycles(sat);
        vwl(k) = static_cast<double>(wl_pairs[static_cast<size_t>(k)].wl_int) - wl_dd_float;
        Rwl(k, k) = kWlConstraintSigmaCyc * kWlConstraintSigmaCyc;
    }
    // Standard Kalman update written as P -= K (H P) to avoid the O(nx^3)
    // Joseph form on the large per-frequency state (nx ~ 300). The fix is
    // reverted to the float state after the solution is generated, so a single
    // epoch's update is never compounded.
    const MatrixXd HPwl = Hwl * filter_state_.covariance;  // nwl x nx
    MatrixXd Swl = HPwl * Hwl.transpose() + Rwl;
    const MatrixXd Swl_inv = Swl.ldlt().solve(MatrixXd::Identity(nwl, nwl));
    if (!Swl_inv.allFinite()) {
        return false;
    }
    const MatrixXd Kwl = HPwl.transpose() * Swl_inv;  // nx x nwl (P H^T = (H P)^T)
    filter_state_.state += Kwl * vwl;
    filter_state_.covariance -= Kwl * HPwl;
    filter_state_.covariance =
        0.5 * (filter_state_.covariance + filter_state_.covariance.transpose());

    // 4) Narrow-lane N1: LAMBDA on the L1 ambiguity double differences for the
    //    WL-fixed pairs, read from the (now WL-tightened) L1 states. Mirrors the
    //    oracle gen_sd_matrix_n1 (a = x[IB(s,0)]/lambda1 differenced).
    //    Partial AR: cap the LAMBDA dimension at the lowest-variance pairs so the
    //    integer search stays bounded (and only fix high-confidence ambiguities).
    constexpr int kMaxN1Dim = 12;
    constexpr double kMaxN1VarCyc2 = 0.25;  // include only pairs with N1 std < 0.5 cycle
    auto n1_var_of = [&](int k) {
        const Cand& rk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].ref)];
        const Cand& sk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].sat)];
        return filter_state_.covariance(rk.l1_index, rk.l1_index) / (rk.lambda1 * rk.lambda1) +
               filter_state_.covariance(sk.l1_index, sk.l1_index) / (sk.lambda1 * sk.lambda1);
    };
    std::vector<int> n1_sel;
    n1_sel.reserve(wl_pairs.size());
    for (int k = 0; k < nwl; ++k) {
        if (n1_var_of(k) < kMaxN1VarCyc2) {
            n1_sel.push_back(k);
        }
    }
    // Keep only the lowest-variance pairs so the integer search stays bounded.
    if (static_cast<int>(n1_sel.size()) > kMaxN1Dim) {
        std::sort(n1_sel.begin(), n1_sel.end(),
                  [&](int a, int b) { return n1_var_of(a) < n1_var_of(b); });
        n1_sel.resize(kMaxN1Dim);
    }
    if (static_cast<int>(n1_sel.size()) < ppp_config_.min_satellites_for_ar) {
        last_fixed_ambiguities_ = nwl;  // wide-lane already applied
        return true;
    }
    const int nb = static_cast<int>(n1_sel.size());
    VectorXd n1_float = VectorXd::Zero(nb);
    MatrixXd n1_cov = MatrixXd::Zero(nb, nb);
    for (int k = 0; k < nb; ++k) {
        const Cand& rk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].ref)];
        const Cand& sk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].sat)];
        n1_float(k) = filter_state_.state(rk.l1_index) / rk.lambda1 -
                      filter_state_.state(sk.l1_index) / sk.lambda1;
        for (int l = 0; l < nb; ++l) {
            const Cand& rl = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(l)])].ref)];
            const Cand& sl = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(l)])].sat)];
            n1_cov(k, l) =
                filter_state_.covariance(rk.l1_index, rl.l1_index) / (rk.lambda1 * rl.lambda1) -
                filter_state_.covariance(rk.l1_index, sl.l1_index) / (rk.lambda1 * sl.lambda1) -
                filter_state_.covariance(sk.l1_index, rl.l1_index) / (sk.lambda1 * rl.lambda1) +
                filter_state_.covariance(sk.l1_index, sl.l1_index) / (sk.lambda1 * sl.lambda1);
        }
    }
    if (env_overrides_.pfdump) {
        for (int k = 0; k < nb; ++k) {
            const Cand& rk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].ref)];
            const Cand& sk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].sat)];
            const double frac = n1_float(k) - std::round(n1_float(k));
            std::cerr << "[PFN1-DD] " << rk.sat.toString() << "-" << sk.sat.toString()
                      << " n1=" << n1_float(k) << " frac=" << frac
                      << " sigma=" << std::sqrt(std::max(0.0, n1_cov(k, k))) << "\n";
        }
    }
    VectorXd n1_fixed = VectorXd::Zero(nb);
    double ratio = 0.0;
    if (!lambdaSearch(n1_float, n1_cov, n1_fixed, ratio) || !std::isfinite(ratio)) {
        last_fixed_ambiguities_ = nwl;  // wide-lane already applied
        return true;
    }
    if (env_overrides_.pfdump) {
        std::cerr << "[PFN1] nb=" << nb << " ratio=" << ratio
                  << " threshold=" << ppp_config_.ar_ratio_threshold << "\n";
    }
    if (ratio < ppp_config_.ar_ratio_threshold) {
        last_ar_ratio_ = ratio;
        last_fixed_ambiguities_ = nwl;
        return true;
    }

    // 5) Apply the fixed N1 double differences as a tight Kalman pseudo-obs on
    //    the L1 ambiguity states (cycles).
    MatrixXd Hn = MatrixXd::Zero(nb, nx);
    VectorXd vn = VectorXd::Zero(nb);
    MatrixXd Rn = MatrixXd::Zero(nb, nb);
    constexpr double kN1ConstraintSigmaCyc = 0.03;
    for (int k = 0; k < nb; ++k) {
        const Cand& rk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].ref)];
        const Cand& sk = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])].sat)];
        Hn(k, rk.l1_index) += 1.0 / rk.lambda1;
        Hn(k, sk.l1_index) += -1.0 / sk.lambda1;
        vn(k) = n1_fixed(k) -
                (filter_state_.state(rk.l1_index) / rk.lambda1 -
                 filter_state_.state(sk.l1_index) / sk.lambda1);
        Rn(k, k) = kN1ConstraintSigmaCyc * kN1ConstraintSigmaCyc;
    }
    const MatrixXd HPn = Hn * filter_state_.covariance;  // nb x nx
    MatrixXd Sn = HPn * Hn.transpose() + Rn;
    const MatrixXd Sn_inv = Sn.ldlt().solve(MatrixXd::Identity(nb, nb));
    if (!Sn_inv.allFinite()) {
        last_fixed_ambiguities_ = nwl;
        return true;
    }
    const MatrixXd Kn = HPn.transpose() * Sn_inv;  // nx x nb
    filter_state_.state += Kn * vn;
    filter_state_.covariance -= Kn * HPn;
    filter_state_.covariance =
        0.5 * (filter_state_.covariance + filter_state_.covariance.transpose());

    for (const auto& p : wl_pairs) {
        ambiguity_states_[cands[static_cast<size_t>(p.ref)].sat].is_fixed = true;
        ambiguity_states_[cands[static_cast<size_t>(p.sat)].sat].is_fixed = true;
    }
    last_ar_ratio_ = ratio;
    last_fixed_ambiguities_ = nb;
    return true;
}

}  // namespace libgnss
