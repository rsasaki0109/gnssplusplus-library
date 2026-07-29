#include "ppp_internal.hpp"

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_correction_contract.hpp>
#include <libgnss++/algorithms/ppp_multifrequency.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

bool PPPProcessor::resolveAmbiguities(const ObservationData& obs, const NavigationData& nav) {
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_ar_wide_lane_only_ = false;

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
                const auto clock_policy = pppEnvOverrides().clas_base_clock_parity
                    ? SSRClockSelectionPolicy::ClaslibBaseHold
                    : SSRClockSelectionPolicy::MergedInterpolate;
                const bool allow_future_samples =
                    !pppEnvOverrides().clas_base_clock_parity;
                if (ssr_products_.interpolateCorrection(
                        real_satellite, obs.time, orbit_corr, clock_corr,
                        nullptr, nullptr, nullptr, nullptr,
                        nullptr, nullptr, nullptr,
                        0, nullptr, nullptr, nullptr,
                        allow_future_samples, nullptr, nullptr,
                        clock_policy)) {
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
    // Consume the prior epoch's provisional success at the start of every
    // attempt. Any early return below therefore breaks the required
    // consecutive-success sequence.
    const bool had_madoca_n1_confirmation =
        madoca_first_n1_confirmation_pending_;
    madoca_first_n1_confirmation_pending_ = false;

    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    ++ar_stage_telemetry_.per_frequency_attempts;
    ar_stage_telemetry_.last_stage = "collect_candidates";
    ar_stage_telemetry_.last_satellite_candidates = 0;
    ar_stage_telemetry_.last_wide_lane_pairs = 0;
    ar_stage_telemetry_.last_n1_candidates = 0;
    ar_stage_telemetry_.last_n1_ratio = 0.0;

    const auto dump_position_covariance = [&](const char* stage) {
        if (!env_overrides_.pfdump) {
            return;
        }
        double variance_sum = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            variance_sum += std::max(
                0.0,
                filter_state_.covariance(
                    filter_state_.pos_index + axis,
                    filter_state_.pos_index + axis));
        }
        std::cerr << "[PFCOV] week=" << obs.time.week
                  << " tow=" << obs.time.tow
                  << " stage=" << stage
                  << " pos_std_norm=" << std::sqrt(variance_sum)
                  << "\n";
    };
    dump_position_covariance("pre_ewl");

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
        std::array<int, 2> extra_indices{{-1, -1}};
        std::array<double, 2> extra_wavelengths{{0.0, 0.0}};
        double elevation = -M_PI_2;
        int wl_int = 0;
        bool wl_fixed = false;
    };
    const int min_lock = ppp_internal::perFrequencyArMinLockCount(
        require_coherent_ssr_ && ssr_products_loaded_,
        ssr_products_loaded_,
        ppp_config_.convergence_min_epochs);
    std::vector<Cand> cands;
    for (const auto& [sat, l1_index] : filter_state_.ambiguity_indices) {
        // MADOCALIB gen_sat_sd excludes GLONASS from PPP-AR even when it is
        // present in the float solution; its FDMA ambiguities must not create
        // a separate reference group here.
        if (sat.system == GNSSSystem::GLONASS) {
            continue;
        }
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
        for (int ordinal = 2; ordinal <= 3; ++ordinal) {
            const auto signals =
                algorithms::ppp_multifrequency::signalsForFrequencyOrdinal(
                    sat, ordinal);
            for (const SignalType signal : signals) {
                const auto state_it =
                    filter_state_.additional_ambiguity_indices.find({sat, signal});
                const auto lifecycle_it =
                    amb_it->second.frequency_lifecycle.find(signal);
                if (state_it == filter_state_.additional_ambiguity_indices.end() ||
                    lifecycle_it == amb_it->second.frequency_lifecycle.end() ||
                    state_it->second < 0 ||
                    state_it->second >= filter_state_.total_states ||
                    !(lifecycle_it->second.wavelength_m > 0.0)) {
                    continue;
                }
                const size_t slot = static_cast<size_t>(ordinal - 2);
                c.extra_indices[slot] = state_it->second;
                c.extra_wavelengths[slot] = lifecycle_it->second.wavelength_m;
                break;
            }
        }

        Vector3d satellite_position;
        Vector3d satellite_velocity;
        double satellite_clock = 0.0;
        double satellite_clock_drift = 0.0;
        if (!nav.calculateSatelliteState(
                sat, obs.time, satellite_position, satellite_velocity,
                satellite_clock, satellite_clock_drift)) {
            continue;
        }
        if (ssr_products_loaded_) {
            Vector3d orbit_correction;
            double clock_correction = 0.0;
            if (ssr_products_.interpolateCorrection(
                    sat, obs.time, orbit_correction, clock_correction)) {
                if (ssr_products_.orbitCorrectionsAreRac()) {
                    orbit_correction = ssrRacToEcef(
                        satellite_position, satellite_velocity, orbit_correction);
                }
                satellite_position += orbit_correction;
            }
        }
        const Vector3d receiver_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const Vector3d line_of_sight = satellite_position - receiver_position;
        if (line_of_sight.norm() <= 0.0 || receiver_position.norm() <= 0.0) {
            continue;
        }
        c.elevation = std::asin(
            line_of_sight.normalized().dot(receiver_position.normalized()));
        // Frozen MADOCALIB pppar profile: pos2-arelmask=15 degrees. The
        // measurement filter still uses the 10-degree observation mask, but
        // gen_sat_sd() excludes lower satellites from integer candidates.
        constexpr double kMadocalibArElevationMaskRad = 15.0 * M_PI / 180.0;
        if (ssr_products_loaded_ &&
            c.elevation < kMadocalibArElevationMaskRad) {
            continue;
        }
        cands.push_back(c);
    }
    ar_stage_telemetry_.last_satellite_candidates = static_cast<int>(cands.size());
    if (static_cast<int>(cands.size()) < ppp_config_.min_satellites_for_ar) {
        ++ar_stage_telemetry_.insufficient_satellite_epochs;
        ar_stage_telemetry_.last_stage = "insufficient_satellites";
        return false;
    }

    // 2) Wide-lane integer per satellite, double-differenced against the
    //    highest-elevation candidate of each system.
    //    WL_s = x[L1]/lambda1 - x[L2]/lambda2 (cycles).
    std::map<std::pair<GNSSSystem, int>, int> ref_of_group;
    for (size_t i = 0; i < cands.size(); ++i) {
        // MADOCALIB gen_sat_sd() never uses the BDS IGSO satellites as the
        // datum, although they remain eligible as non-reference candidates.
        if (cands[i].sat.system == GNSSSystem::BeiDou &&
            cands[i].sat.prn >= 38 && cands[i].sat.prn <= 40) {
            continue;
        }
        const auto group = ppp_ar::ambiguityDdGroup(cands[i].sat);
        const auto [it, inserted] =
            ref_of_group.emplace(group, static_cast<int>(i));
        if (!inserted &&
            cands[i].elevation > cands[static_cast<size_t>(it->second)].elevation) {
            it->second = static_cast<int>(i);
        }
    }
    struct DdPair {
        int ref = -1;
        int sat = -1;
        int wl_int = 0;
        double constraint_sigma_cycles = 0.01;
    };
    std::vector<DdPair> wl_pairs;
    const VectorXd& x = filter_state_.state;
    const MatrixXd& P = filter_state_.covariance;
    const int nx = filter_state_.total_states;

    if (env_overrides_.pfdump) {
        for (const auto& [sat, l1_index] : filter_state_.ambiguity_indices) {
            if (observed_now.count(sat) == 0) {
                continue;
            }
            const auto l2_it =
                filter_state_.ambiguity_l2_indices.find(sat);
            const auto ion_it =
                filter_state_.ionosphere_indices.find(sat);
            const auto amb_it = ambiguity_states_.find(sat);
            if (l2_it == filter_state_.ambiguity_l2_indices.end() ||
                ion_it == filter_state_.ionosphere_indices.end() ||
                amb_it == ambiguity_states_.end() ||
                l1_index < 0 || l1_index >= nx ||
                l2_it->second < 0 || l2_it->second >= nx ||
                ion_it->second < 0 || ion_it->second >= nx ||
                !(amb_it->second.wavelength_l1 > 0.0) ||
                !(amb_it->second.wavelength_l2 > 0.0)) {
                continue;
            }
            std::cerr << "[PFSTATE] " << sat.toString()
                      << " l1_m=" << x(l1_index)
                      << " l1_cyc="
                      << x(l1_index) / amb_it->second.wavelength_l1
                      << " l2_m=" << x(l2_it->second)
                      << " l2_cyc="
                      << x(l2_it->second) / amb_it->second.wavelength_l2
                      << " ion_m=" << x(ion_it->second) << "\n";
        }
        for (size_t i = 0; i < cands.size(); ++i) {
            const auto ref_it = ref_of_group.find(
                ppp_ar::ambiguityDdGroup(cands[i].sat));
            if (ref_it == ref_of_group.end()) {
                continue;
            }
            const int ref_idx = ref_it->second;
            if (ref_idx == static_cast<int>(i)) {
                continue;
            }
            const Cand& ref = cands[static_cast<size_t>(ref_idx)];
            const Cand& sat = cands[i];
            const double wl_dd =
                (x(ref.l1_index) / ref.lambda1 -
                 x(ref.l2_index) / ref.lambda2) -
                (x(sat.l1_index) / sat.lambda1 -
                 x(sat.l2_index) / sat.lambda2);
            std::cerr << "[PFWL-PRE-EWL] " << ref.sat.toString() << "-"
                      << sat.sat.toString() << " wl=" << wl_dd
                      << " frac=" << std::abs(std::round(wl_dd) - wl_dd)
                      << "\n";
        }
    }

    // MADOCALIB STEP_EWL: condition the extra-frequency states with accepted
    // F2-F3 and F2-F4 integer double differences before ordinary F1-F2 WL.
    struct EwlPair {
        int ref = -1;
        int sat = -1;
        int slot = 0;
        int integer = 0;
        double sigma_cycles = 0.10;
    };
    std::vector<EwlPair> ewl_pairs;
    constexpr double kMaxFracEwl = 0.20;
    constexpr double kMaxStdEwl = 1.0;
    for (size_t i = 0; i < cands.size(); ++i) {
        const auto ref_it = ref_of_group.find(
            ppp_ar::ambiguityDdGroup(cands[i].sat));
        if (ref_it == ref_of_group.end()) {
            continue;
        }
        const int ref_idx = ref_it->second;
        if (ref_idx == static_cast<int>(i)) {
            continue;
        }
        const Cand& ref = cands[static_cast<size_t>(ref_idx)];
        const Cand& sat = cands[i];
        for (int slot = 0; slot < 2; ++slot) {
            const int ref_extra = ref.extra_indices[static_cast<size_t>(slot)];
            const int sat_extra = sat.extra_indices[static_cast<size_t>(slot)];
            const double ref_lambda =
                ref.extra_wavelengths[static_cast<size_t>(slot)];
            const double sat_lambda =
                sat.extra_wavelengths[static_cast<size_t>(slot)];
            if (ref_extra < 0 || sat_extra < 0 || !(ref_lambda > 0.0) ||
                !(sat_lambda > 0.0)) {
                continue;
            }
            const double ewl_dd =
                algorithms::ppp_multifrequency::extraWideLaneDoubleDifferenceCycles(
                    x(ref.l2_index), ref.lambda2, x(ref_extra), ref_lambda,
                    x(sat.l2_index), sat.lambda2, x(sat_extra), sat_lambda);
            const double integer = std::round(ewl_dd);
            const double frac = std::abs(integer - ewl_dd);
            const auto single_sat_variance = [&](int l2_index,
                                                 double lambda2,
                                                 int extra_index,
                                                 double extra_lambda) {
                return P(l2_index, l2_index) / (lambda2 * lambda2) +
                       P(extra_index, extra_index) /
                           (extra_lambda * extra_lambda) -
                       2.0 * P(l2_index, extra_index) /
                           (lambda2 * extra_lambda);
            };
            // Match search_amb_ewl(): sum the two single-satellite variances.
            const double variance =
                single_sat_variance(
                    ref.l2_index, ref.lambda2, ref_extra, ref_lambda) +
                single_sat_variance(
                    sat.l2_index, sat.lambda2, sat_extra, sat_lambda);
            const double std_cycles = variance > 0.0
                ? std::sqrt(variance) : std::numeric_limits<double>::infinity();
            if (env_overrides_.pfdump) {
                std::cerr << "[PFEWL] " << ref.sat.toString() << "-"
                          << sat.sat.toString() << " F2-F" << (slot + 3)
                          << " value=" << ewl_dd << " frac=" << frac
                          << " std=" << std_cycles << "\n";
            }
            if (std_cycles > kMaxStdEwl || frac > kMaxFracEwl) {
                continue;
            }
            double constraint_sigma = 0.01;
            if (sat.sat.system == GNSSSystem::GPS ||
                sat.sat.system == GNSSSystem::BeiDou ||
                (sat.sat.system == GNSSSystem::Galileo && slot == 1)) {
                constraint_sigma = 0.10;
            }
            ewl_pairs.push_back({
                ref_idx, static_cast<int>(i), slot,
                static_cast<int>(integer), constraint_sigma});
        }
    }
    if (env_overrides_.pfdump) {
        std::cerr << "[PFEWL-SUM] fixed=" << ewl_pairs.size() << "\n";
    }
    if (!ewl_pairs.empty()) {
        const int ne = static_cast<int>(ewl_pairs.size());
        MatrixXd Hewl = MatrixXd::Zero(ne, nx);
        VectorXd vewl = VectorXd::Zero(ne);
        MatrixXd Rewl = MatrixXd::Zero(ne, ne);
        for (int k = 0; k < ne; ++k) {
            const EwlPair& pair = ewl_pairs[static_cast<size_t>(k)];
            const Cand& ref = cands[static_cast<size_t>(pair.ref)];
            const Cand& sat = cands[static_cast<size_t>(pair.sat)];
            const size_t slot = static_cast<size_t>(pair.slot);
            Hewl(k, ref.l2_index) += 1.0 / ref.lambda2;
            Hewl(k, ref.extra_indices[slot]) +=
                -1.0 / ref.extra_wavelengths[slot];
            Hewl(k, sat.l2_index) += -1.0 / sat.lambda2;
            Hewl(k, sat.extra_indices[slot]) +=
                1.0 / sat.extra_wavelengths[slot];
            const double float_value = (Hewl.row(k) * filter_state_.state)(0);
            vewl(k) = static_cast<double>(pair.integer) - float_value;
            Rewl(k, k) = pair.sigma_cycles * pair.sigma_cycles;
        }
        const MatrixXd HP = Hewl * filter_state_.covariance;
        const MatrixXd innovation_covariance =
            HP * Hewl.transpose() + Rewl;
        const MatrixXd inverse = innovation_covariance.ldlt().solve(
            MatrixXd::Identity(ne, ne));
        if (!inverse.allFinite()) {
            return false;
        }
        const MatrixXd gain = HP.transpose() * inverse;
        filter_state_.state += gain * vewl;
        filter_state_.covariance -= gain * HP;
        filter_state_.covariance = 0.5 *
            (filter_state_.covariance + filter_state_.covariance.transpose());
    }
    dump_position_covariance("post_ewl");

    auto wl_cycles = [&](const Cand& c) {
        return x(c.l1_index) / c.lambda1 - x(c.l2_index) / c.lambda2;
    };
    constexpr double kMaxFracWl = 0.20;
    constexpr double kMaxStdWl = 1.0;
    int wl_fix_count = 0;
    double wl_frac_sum = 0.0;
    for (size_t i = 0; i < cands.size(); ++i) {
        const auto ref_it = ref_of_group.find(
            ppp_ar::ambiguityDdGroup(cands[i].sat));
        if (ref_it == ref_of_group.end()) {
            continue;
        }
        const int ref_idx = ref_it->second;
        if (ref_idx == static_cast<int>(i)) {
            continue;
        }
        const Cand& ref = cands[static_cast<size_t>(ref_idx)];
        const Cand& sat = cands[i];
        const double wl_dd = wl_cycles(ref) - wl_cycles(sat);
        const double n = std::round(wl_dd);
        const double frac = std::abs(n - wl_dd);
        // Match search_amb_wl(): sum each satellite's internal WL variance.
        // The oracle intentionally omits cross-satellite covariance terms in
        // this integer-admission gate, even though the subsequent Kalman
        // constraint uses the complete covariance matrix.
        const int ir1 = ref.l1_index, ir2 = ref.l2_index;
        const int is1 = sat.l1_index, is2 = sat.l2_index;
        const auto wl_variance = [&](int l1_index,
                                     double lambda1,
                                     int l2_index,
                                     double lambda2) {
            return P(l1_index, l1_index) / (lambda1 * lambda1) +
                   P(l2_index, l2_index) / (lambda2 * lambda2) -
                   2.0 * P(l1_index, l2_index) / (lambda1 * lambda2);
        };
        const double var =
            wl_variance(ir1, ref.lambda1, ir2, ref.lambda2) +
            wl_variance(is1, sat.lambda1, is2, sat.lambda2);
        const double sd = var > 0.0 ? std::sqrt(var) : 9.9;
        if (env_overrides_.pfdump) {
            std::cerr << "[PFWL] " << ref.sat.toString() << "-" << sat.sat.toString()
                      << " wl=" << wl_dd << " frac=" << frac << " std=" << sd << "\n";
        }
        if (frac < kMaxFracWl && sd < kMaxStdWl) {
            const double constraint_sigma =
                sat.sat.system == GNSSSystem::BeiDou ? 0.05 : 0.01;
            wl_pairs.push_back({
                ref_idx, static_cast<int>(i), static_cast<int>(n),
                constraint_sigma});
            wl_fix_count++;
            wl_frac_sum += frac;
        }
    }
    if (env_overrides_.pfdump) {
        std::cerr << "[PFWL-SUM] cand=" << cands.size() << " wl_fixed=" << wl_fix_count
                  << " mean_frac=" << (wl_fix_count ? wl_frac_sum / wl_fix_count : 0.0) << "\n";
    }
    if (wl_pairs.empty()) {
        ++ar_stage_telemetry_.no_wide_lane_epochs;
        ar_stage_telemetry_.last_stage = "no_wide_lane";
        return false;
    }
    ar_stage_telemetry_.last_wide_lane_pairs = static_cast<int>(wl_pairs.size());

    // 3) Apply the fixed wide-lane integers as a batched Kalman pseudo-measurement
    //    (cycles). Tight but not rigid so a wrong WL does not lock the filter.
    const int nwl = static_cast<int>(wl_pairs.size());
    MatrixXd Hwl = MatrixXd::Zero(nwl, nx);
    VectorXd vwl = VectorXd::Zero(nwl);
    MatrixXd Rwl = MatrixXd::Zero(nwl, nwl);
    for (int k = 0; k < nwl; ++k) {
        const Cand& ref = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].ref)];
        const Cand& sat = cands[static_cast<size_t>(wl_pairs[static_cast<size_t>(k)].sat)];
        Hwl(k, ref.l1_index) += 1.0 / ref.lambda1;
        Hwl(k, ref.l2_index) += -1.0 / ref.lambda2;
        Hwl(k, sat.l1_index) += -1.0 / sat.lambda1;
        Hwl(k, sat.l2_index) += 1.0 / sat.lambda2;
        const double wl_dd_float = wl_cycles(ref) - wl_cycles(sat);
        vwl(k) = static_cast<double>(wl_pairs[static_cast<size_t>(k)].wl_int) - wl_dd_float;
        const double sigma =
            wl_pairs[static_cast<size_t>(k)].constraint_sigma_cycles;
        Rwl(k, k) = sigma * sigma;
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
    dump_position_covariance("post_wl");

    // MADOCALIB snapshots the WL-conditioned float state before N1 and
    // restores it when the final fixed-position covariance gate rejects.
    const VectorXd wide_lane_state = filter_state_.state;
    const MatrixXd wide_lane_covariance = filter_state_.covariance;

    // 4) Narrow-lane N1: LAMBDA on the L1 ambiguity double differences for the
    //    WL-fixed pairs, read from the (now WL-tightened) L1 states. Mirrors the
    //    oracle gen_sd_matrix_n1 (a = x[IB(s,0)]/lambda1 differenced).
    //    MADOCALIB gen_sd_matrix_n1 admits every pair that survived the WL
    //    search.  Candidate reduction is performed only by the subsequent
    //    ratio-driven partial-AR loop, not by a per-pair variance cutoff.
    //    Its NL search starts only when the norm of the three position-state
    //    standard deviations is below pos2-arthres1 (1.5 m in the pinned
    //    oracle config).  This replaces the unrelated solution-stability gate,
    //    while still protecting LAMBDA from an unbounded early covariance.
    double position_std_norm_sq = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double variance = filter_state_.covariance(
            filter_state_.pos_index + axis, filter_state_.pos_index + axis);
        position_std_norm_sq += std::max(0.0, variance);
    }
    constexpr double kMaxPositionStdNormM = 1.5;
    if (std::sqrt(position_std_norm_sq) >= kMaxPositionStdNormM) {
        last_fixed_ambiguities_ = nwl;
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        ar_stage_telemetry_.last_stage = "wide_lane_only_position_std_gate";
        last_ar_wide_lane_only_ = true;
        return false;
    }

    std::vector<int> n1_sel;
    n1_sel.reserve(wl_pairs.size());
    for (int k = 0; k < nwl; ++k) {
        n1_sel.push_back(k);
    }
    if (static_cast<int>(n1_sel.size()) < ppp_config_.min_satellites_for_ar) {
        last_fixed_ambiguities_ = nwl;  // wide-lane already applied
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        ar_stage_telemetry_.last_n1_candidates = static_cast<int>(n1_sel.size());
        ar_stage_telemetry_.last_stage = "wide_lane_only_insufficient_n1";
        last_ar_wide_lane_only_ = true;
        return false;
    }
    constexpr int kMinN1Ambiguities = 4;
    constexpr int kMaxPartialArIterations = 10;
    constexpr double kDimensionThresholdFactors[] = {5.0, 5.0, 3.0, 2.0, 1.5};
    VectorXd n1_fixed;
    VectorXd n1_second;
    double ratio = 0.0;
    double effective_threshold = ppp_config_.ar_ratio_threshold;
    bool n1_accepted = false;
    bool lambda_failed = false;

    // MADOCALIB partial AR: when the ratio test rejects the full set, remove
    // one non-reference satellite implicated by the disagreement between the
    // first and second LAMBDA candidates, then retry.  QZSS and BDS IGSO are
    // deliberately removed before the ordinary lowest-elevation candidate.
    for (int iteration = 0; iteration < kMaxPartialArIterations; ++iteration) {
        const int trial_nb = static_cast<int>(n1_sel.size());
        if (trial_nb < kMinN1Ambiguities) {
            break;
        }

        VectorXd n1_float = VectorXd::Zero(trial_nb);
        MatrixXd n1_cov = MatrixXd::Zero(trial_nb, trial_nb);
        for (int k = 0; k < trial_nb; ++k) {
            const DdPair& pair_k =
                wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])];
            const Cand& rk = cands[static_cast<size_t>(pair_k.ref)];
            const Cand& sk = cands[static_cast<size_t>(pair_k.sat)];
            n1_float(k) = filter_state_.state(rk.l1_index) / rk.lambda1 -
                          filter_state_.state(sk.l1_index) / sk.lambda1;
            for (int l = 0; l < trial_nb; ++l) {
                const DdPair& pair_l =
                    wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(l)])];
                const Cand& rl = cands[static_cast<size_t>(pair_l.ref)];
                const Cand& sl = cands[static_cast<size_t>(pair_l.sat)];
                n1_cov(k, l) =
                    filter_state_.covariance(rk.l1_index, rl.l1_index) /
                        (rk.lambda1 * rl.lambda1) -
                    filter_state_.covariance(rk.l1_index, sl.l1_index) /
                        (rk.lambda1 * sl.lambda1) -
                    filter_state_.covariance(sk.l1_index, rl.l1_index) /
                        (sk.lambda1 * rl.lambda1) +
                    filter_state_.covariance(sk.l1_index, sl.l1_index) /
                        (sk.lambda1 * sl.lambda1);
            }
        }
        if (env_overrides_.pfdump) {
            for (int k = 0; k < trial_nb; ++k) {
                const DdPair& pair =
                    wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])];
                const Cand& rk = cands[static_cast<size_t>(pair.ref)];
                const Cand& sk = cands[static_cast<size_t>(pair.sat)];
                const double frac = n1_float(k) - std::round(n1_float(k));
                std::cerr << "[PFN1-DD] " << rk.sat.toString() << "-"
                          << sk.sat.toString() << " n1=" << n1_float(k)
                          << " frac=" << frac << " sigma="
                          << std::sqrt(std::max(0.0, n1_cov(k, k))) << "\n";
            }
        }

        if (!lambdaSearchCandidates(
                n1_float, n1_cov, n1_fixed, n1_second, ratio) ||
            !std::isfinite(ratio)) {
            lambda_failed = true;
            break;
        }

        effective_threshold = ppp_config_.ar_ratio_threshold;
        const int dimension_offset = trial_nb - kMinN1Ambiguities;
        if (dimension_offset >= 0 && dimension_offset < 5) {
            effective_threshold *= kDimensionThresholdFactors[dimension_offset];
        }
        int matching_candidates = 0;
        for (int k = 0; k < trial_nb; ++k) {
            if (std::abs(n1_fixed(k) - n1_second(k)) < 0.001) {
                ++matching_candidates;
            }
        }
        const double match_rate =
            static_cast<double>(matching_candidates) / trial_nb;
        if (match_rate > 0.90) {
            effective_threshold *= 0.8;
        }
        if (match_rate < 0.10) {
            effective_threshold = 99.99;
        }
        ar_stage_telemetry_.last_n1_candidates = trial_nb;
        ar_stage_telemetry_.last_n1_ratio = ratio;
        if (env_overrides_.pfdump) {
            std::cerr << "[PFN1] iter=" << iteration << " nb=" << trial_nb
                      << " ratio=" << ratio
                      << " threshold=" << effective_threshold
                      << " match_rate=" << match_rate << "\n";
        }
        if (ratio >= effective_threshold) {
            n1_accepted = true;
            break;
        }

        int exclude_position = -1;
        double lowest_priority_elevation = std::numeric_limits<double>::infinity();
        for (int k = 0; k < trial_nb; ++k) {
            if (std::abs(n1_fixed(k) - n1_second(k)) < 0.001) {
                continue;
            }
            const DdPair& pair =
                wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])];
            const Cand& candidate = cands[static_cast<size_t>(pair.sat)];
            double priority_elevation = candidate.elevation;
            const bool prioritized = candidate.sat.system == GNSSSystem::QZSS ||
                (candidate.sat.system == GNSSSystem::BeiDou &&
                 candidate.sat.prn >= 38 && candidate.sat.prn <= 40);
            if (prioritized) {
                priority_elevation -= 0.5 * M_PI;
            }
            if (priority_elevation < lowest_priority_elevation) {
                lowest_priority_elevation = priority_elevation;
                exclude_position = k;
            }
        }
        if (exclude_position < 0) {
            break;
        }
        if (env_overrides_.pfdump) {
            const DdPair& excluded_pair = wl_pairs[static_cast<size_t>(
                n1_sel[static_cast<size_t>(exclude_position)])];
            std::cerr << "[PFN1-PAR] excluded="
                      << cands[static_cast<size_t>(excluded_pair.sat)].sat.toString()
                      << " elevation="
                      << cands[static_cast<size_t>(excluded_pair.sat)].elevation
                      << "\n";
        }
        n1_sel.erase(n1_sel.begin() + exclude_position);
    }

    const int nb = static_cast<int>(n1_sel.size());
    if (!n1_accepted) {
        last_ar_ratio_ = ratio;
        last_fixed_ambiguities_ = nwl;
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        if (lambda_failed) {
            ++ar_stage_telemetry_.n1_lambda_failure_epochs;
            ar_stage_telemetry_.last_stage = "wide_lane_only_lambda_failure";
        } else {
            ++ar_stage_telemetry_.n1_ratio_rejection_epochs;
            ar_stage_telemetry_.last_stage = "wide_lane_only_ratio_rejected";
        }
        last_ar_wide_lane_only_ = true;
        return false;
    }

    // Require one independent epoch of confirmation before the first
    // coherent-MADOCA N1 commit. This keeps the first ratio/PAR success
    // provisional (the already-applied EWL/WL state is retained) and prevents
    // a single sensitive LAMBDA candidate ordering from publishing an early
    // Fix. Subsequent accepted epochs use the normal commit path.
    if (algorithms::ppp_correction_contract::deferFirstMadocaN1Fix(
            require_coherent_ssr_,
            ar_stage_telemetry_.n1_fixed_epochs,
            had_madoca_n1_confirmation)) {
        madoca_first_n1_confirmation_pending_ = true;
        last_ar_ratio_ = ratio;
        last_fixed_ambiguities_ = nwl;
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        ar_stage_telemetry_.last_stage =
            "wide_lane_only_n1_confirmation_pending";
        last_ar_wide_lane_only_ = true;
        return false;
    }

    // 5) Apply the fixed N1 double differences as a tight Kalman pseudo-obs on
    //    the L1 ambiguity states (cycles).
    MatrixXd Hn = MatrixXd::Zero(nb, nx);
    VectorXd vn = VectorXd::Zero(nb);
    MatrixXd Rn = MatrixXd::Zero(nb, nb);
    for (int k = 0; k < nb; ++k) {
        const DdPair& pair =
            wl_pairs[static_cast<size_t>(n1_sel[static_cast<size_t>(k)])];
        const Cand& rk = cands[static_cast<size_t>(pair.ref)];
        const Cand& sk = cands[static_cast<size_t>(pair.sat)];
        Hn(k, rk.l1_index) += 1.0 / rk.lambda1;
        Hn(k, sk.l1_index) += -1.0 / sk.lambda1;
        vn(k) = n1_fixed(k) -
                (filter_state_.state(rk.l1_index) / rk.lambda1 -
                 filter_state_.state(sk.l1_index) / sk.lambda1);
        const double sigma = pair.constraint_sigma_cycles;
        Rn(k, k) = sigma * sigma;
    }
    const MatrixXd HPn = Hn * filter_state_.covariance;  // nb x nx
    MatrixXd Sn = HPn * Hn.transpose() + Rn;
    const MatrixXd Sn_inv = Sn.ldlt().solve(MatrixXd::Identity(nb, nb));
    if (!Sn_inv.allFinite()) {
        last_fixed_ambiguities_ = nwl;
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        ar_stage_telemetry_.last_stage = "wide_lane_only_n1_update_failure";
        last_ar_wide_lane_only_ = true;
        return false;
    }
    const MatrixXd Kn = HPn.transpose() * Sn_inv;  // nx x nb
    filter_state_.state += Kn * vn;
    filter_state_.covariance -= Kn * HPn;
    filter_state_.covariance =
        0.5 * (filter_state_.covariance + filter_state_.covariance.transpose());

    double fixed_position_std_norm_sq = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        fixed_position_std_norm_sq += std::max(
            0.0,
            filter_state_.covariance(
                filter_state_.pos_index + axis,
                filter_state_.pos_index + axis));
    }
    constexpr double kMaxFixedPositionStdNormM = 0.15;
    if (std::sqrt(fixed_position_std_norm_sq) >
        kMaxFixedPositionStdNormM) {
        filter_state_.state = wide_lane_state;
        filter_state_.covariance = wide_lane_covariance;
        last_fixed_ambiguities_ = nwl;
        ++ar_stage_telemetry_.wide_lane_only_epochs;
        ar_stage_telemetry_.last_stage =
            "wide_lane_only_fixed_position_std_gate";
        last_ar_wide_lane_only_ = true;
        return false;
    }

    for (const int selected : n1_sel) {
        const DdPair& pair = wl_pairs[static_cast<size_t>(selected)];
        auto& reference_state =
            ambiguity_states_[cands[static_cast<size_t>(pair.ref)].sat];
        auto& satellite_state =
            ambiguity_states_[cands[static_cast<size_t>(pair.sat)].sat];
        reference_state.is_fixed = true;
        reference_state.nl_is_fixed = true;
        satellite_state.is_fixed = true;
        satellite_state.nl_is_fixed = true;
    }
    last_ar_ratio_ = ratio;
    last_fixed_ambiguities_ = nb;
    ++ar_stage_telemetry_.n1_fixed_epochs;
    madoca_first_n1_confirmation_pending_ = false;
    ar_stage_telemetry_.last_stage = "n1_fixed";
    return true;
}

}  // namespace libgnss
