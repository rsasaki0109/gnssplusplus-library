// WL+NL ambiguity resolution and fixed-position solver methods for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <cmath>
#include <iostream>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

int minWlnlHoldAmbiguities(const PPPConfig& config) {
    return config.ar_method == PPPConfig::ARMethod::DD_WLNL
        ? 3
        : config.min_satellites_for_ar;
}

}  // namespace

// ---------- WL+NL Ambiguity Resolution ----------
// Step 1: Fix Wide-Lane integers using averaged Melbourne-Wübbena
// Step 2: Extract Narrow-Lane DD ambiguities from IFLC filter state + fixed WL
// Step 3: Fix NL integers using LAMBDA
// Step 4: Apply position correction

bool PPPProcessor::resolveAmbiguitiesWLNL(const ObservationData& obs, const NavigationData& nav) {
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;

    int held_fixed = 0;
    for (const auto& [satellite, ambiguity] : ambiguity_states_) {
        (void)satellite;
        if (!ambiguity.is_fixed || !ambiguity.nl_is_fixed || !ambiguity.wl_is_fixed ||
            ambiguity.needs_reinitialization ||
            ambiguity.lock_count < ppp_config_.wl_min_averaging_epochs) {
            continue;
        }
        ++held_fixed;
    }
    const bool holding_fixed = held_fixed >= minWlnlHoldAmbiguities(ppp_config_);
    const int retained_dd_constraints = static_cast<int>(wlnl_dd_constraints_.size());
    const bool holding_previous_constraints = retained_dd_constraints >= 3;

    const auto wlnl_preparation = ppp_ar::prepareWlnlCandidates(
        ppp_config_,
        filter_state_,
        ambiguity_states_,
        ssr_products_loaded_,
        pppDebugEnabled());
    const auto& eligible_ambiguities = wlnl_preparation.eligible_ambiguities;

    const int n = static_cast<int>(eligible_ambiguities.satellites.size());
    if (n < ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-WLNL] insufficient candidates: " << n
                      << " (total_amb=" << eligible_ambiguities.total_ambiguities
                      << " reinit=" << eligible_ambiguities.skipped_reinitialization
                      << " lock=" << eligible_ambiguities.skipped_lock
                      << " scale=" << eligible_ambiguities.skipped_scale
                      << " idx=" << eligible_ambiguities.skipped_index << ")\n";
        }
        return false;
    }

    const auto& wl_summary = wlnl_preparation.wl_summary;
    if (wl_summary.fixed_count < ppp_config_.min_satellites_for_ar) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-WLNL] insufficient WL fixes: " << wl_summary.fixed_count
                      << " n=" << n << " max_mw=" << wl_summary.max_mw_count << "\n";
        }
        return false;
    }

    const Vector3d receiver_position = filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clock_bias_m = filter_state_.state(filter_state_.clock_index);
    const double trop_zenith =
        ppp_config_.estimate_troposphere ? filter_state_.state(filter_state_.trop_index) : 2.3;
    const auto osr_by_sat = computeWlnlOsrCorrections(
        obs, nav, receiver_position, clock_bias_m, trop_zenith);

    const ppp_ar::WlnlFixAttempt attempt = ppp_ar::resolveWlnlFix(
        ppp_config_,
        filter_state_,
        ambiguity_states_,
        eligible_ambiguities,
        [&](const SatelliteId& sat, ppp_ar::WlnlNlInfo& info) {
            return buildWlnlNlInfoForSatellite(
                obs, nav, receiver_position, clock_bias_m, trop_zenith,
                osr_by_sat, sat, info);
        },
        pppDebugEnabled());
    if (!attempt.fixed) {
        if (holding_fixed || holding_previous_constraints) {
            last_ar_ratio_ = std::max(ppp_config_.ar_ratio_threshold, 1.0);
            last_fixed_ambiguities_ = std::max(held_fixed, retained_dd_constraints);
            if (pppDebugEnabled()) {
                std::cerr << "[PPP-WLNL] NL hold: nb=" << held_fixed << "\n";
            }
            return true;
        }
        has_wlnl_fixed_state_ = false;
        last_wlnl_fixed_state_dd_gap_ = 1e9;
        wlnl_dd_constraints_.clear();
        return false;
    }

    has_wlnl_fixed_state_ = false;
    last_ar_ratio_ = attempt.ratio;
    last_fixed_ambiguities_ = attempt.nb;
    if (attempt.has_fixed_state) {
        wlnl_fixed_state_ = attempt.fixed_state;
        has_wlnl_fixed_state_ = true;
        last_wlnl_fixed_state_dd_gap_ = attempt.fixed_state_dd_gap_cycles;
    } else {
        last_wlnl_fixed_state_dd_gap_ = 1e9;
    }
    wlnl_dd_constraints_ = attempt.dd_constraints;
    if (pppDebugEnabled()) {
        std::cerr << "[PPP-WLNL] NL fixed: nb=" << attempt.nb
                  << " ratio=" << attempt.ratio << "\n";
    }
    return true;
}

bool PPPProcessor::solveFixedPosition(const ObservationData& obs,
                                      const NavigationData& nav,
                                      Vector3d& fixed_position) {
    Vector3d position = filter_state_.state.segment(filter_state_.pos_index, 3);
    double clock_m = filter_state_.state(filter_state_.clock_index);
    const double trop_zenith =
        ppp_config_.estimate_troposphere ? filter_state_.state(filter_state_.trop_index) : 2.3;
    const auto osr_by_sat = computeWlnlOsrCorrections(obs, nav, position, clock_m, trop_zenith);

    const auto fixed_observations = ppp_ar::buildFixedNlObservations(
        ambiguity_states_,
        [&](const SatelliteId& satellite,
            const PPPAmbiguityInfo& amb,
            ppp_ar::FixedNlObservation& fixed_observation) {
            return buildFixedNlObservationForSatellite(
                obs, nav, osr_by_sat, satellite, amb, fixed_observation);
    });

    double position_shift_norm_m = 0.0;
    const bool solved = ppp_ar::solveFixedNlPosition(
        fixed_observations,
        position,
        clock_m,
        trop_zenith,
        obs.time,
        [&](const Vector3d& receiver_position, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_position, elevation, time);
        },
        fixed_position,
        &position_shift_norm_m);
    if (pppDebugEnabled() && solved) {
        std::cerr << "[PPP-WLNL] fixedWLS: sats=" << fixed_observations.size()
                  << " pos_shift=" << position_shift_norm_m << "m\n";
    }
    return solved;
}

bool PPPProcessor::solveFixedCarrierPhasePosition(
    const std::vector<IonosphereFreeObs>& observations,
    Vector3d& fixed_position) const {
    const auto fixed_observations = ppp_ar::buildFixedCarrierObservations(
        observations.size(),
        [&](size_t index, ppp_ar::FixedCarrierObservation& fixed_observation) {
            return buildFixedCarrierObservation(observations[index], fixed_observation);
    });

    return ppp_ar::solveFixedCarrierPosition(
        fixed_observations,
        filter_state_.state.segment(filter_state_.pos_index, 3),
        filter_state_.state(filter_state_.clock_index),
        ppp_config_.estimate_troposphere ? filter_state_.state(filter_state_.trop_index) : 2.3,
        ppp_config_.estimate_troposphere,
        fixed_position);
}

std::map<SatelliteId, OSRCorrection> PPPProcessor::computeWlnlOsrCorrections(
    const ObservationData& obs,
    const NavigationData& nav,
    const Vector3d& receiver_position,
    double clock_bias_m,
    double trop_zenith) const {
    std::map<SatelliteId, OSRCorrection> osr_by_sat;
    const bool use_clas_osr_wlnl =
        ssr_products_loaded_ && ppp_config_.estimate_ionosphere && !ppp_config_.use_ionosphere_free;
    if (!use_clas_osr_wlnl) {
        return osr_by_sat;
    }

    auto windup_cache = windup_cache_;
    auto dispersion_compensation = clas_dispersion_compensation_;
    auto sis_continuity = clas_sis_continuity_;
    auto phase_bias_repair = clas_phase_bias_repair_;
    const auto epoch_atmos = selectClasEpochAtmosTokens(
        ssr_products_, obs.getSatellites(), obs.time, receiver_position, ppp_config_);
    for (const auto& osr : computeOSR(obs, nav, ssr_products_, epoch_atmos,
                                      receiver_position, clock_bias_m, trop_zenith,
                                      ppp_config_,
                                      windup_cache, dispersion_compensation,
                                      sis_continuity, phase_bias_repair)) {
        if (osr.valid && osr.num_frequencies >= 2) {
            osr_by_sat[osr.satellite] = osr;
        }
    }
    return osr_by_sat;
}

bool PPPProcessor::buildWlnlNlInfoForSatellite(
    const ObservationData& obs,
    const NavigationData& nav,
    const Vector3d& receiver_position,
    double clock_bias_m,
    double trop_zenith,
    const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
    const SatelliteId& sat,
    ppp_ar::WlnlNlInfo& info) const {
    const Observation* l1_obs = nullptr;
    const Observation* l2_obs = nullptr;
    double sat_clk = 0.0;
    Vector3d sat_pos = Vector3d::Zero();
    double lambda_nl = 0.0;
    double lambda_wl = 0.0;
    double beta = 0.0;
    double alpha1 = 0.0;
    double alpha2 = 0.0;
    double l1_corr_m = 0.0;
    double l2_corr_m = 0.0;
    double nl_phase_m = 0.0;
    double predicted_m = 0.0;

    const auto osr_it = osr_by_sat.find(sat);
    if (osr_it != osr_by_sat.end()) {
        const auto& osr = osr_it->second;
        l1_obs = obs.getObservation(sat, osr.signals[0]);
        l2_obs = obs.getObservation(sat, osr.signals[1]);
        const double f1 = osr.frequencies[0];
        const double f2 = osr.frequencies[1];
        if (l1_obs == nullptr || l2_obs == nullptr ||
            !l1_obs->has_carrier_phase || !l2_obs->has_carrier_phase ||
            !std::isfinite(l1_obs->carrier_phase) || !std::isfinite(l2_obs->carrier_phase) ||
            f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            return false;
        }
        alpha1 = f1 / (f1 + f2);
        alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        beta = f1 * f2 / (f1 * f1 - f2 * f2);
        l1_corr_m =
            l1_obs->carrier_phase * osr.wavelengths[0] -
            (osr.CPC[0] - osr.phase_compensation_m[0]);
        l2_corr_m =
            l2_obs->carrier_phase * osr.wavelengths[1] -
            (osr.CPC[1] - osr.phase_compensation_m[1]);
        nl_phase_m = alpha1 * l1_corr_m + alpha2 * l2_corr_m;
        sat_pos = osr.satellite_position;
        sat_clk = osr.satellite_clock_bias_s;
        predicted_m = geodist(sat_pos, receiver_position) + clock_bias_m
                    - constants::SPEED_OF_LIGHT * sat_clk;
    } else {
        l1_obs = ppp_utils::findCarrierObservation(
            obs, sat, {SignalType::GPS_L1CA, SignalType::GPS_L1P,
                       SignalType::GAL_E1, SignalType::QZS_L1CA});
        l2_obs = ppp_utils::findCarrierObservation(
            obs, sat, {SignalType::GPS_L2C, SignalType::GPS_L2P, SignalType::GPS_L5,
                       SignalType::GAL_E5A, SignalType::QZS_L2C, SignalType::QZS_L5});
        if (l1_obs == nullptr || l2_obs == nullptr) {
            return false;
        }

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        const double f1 = signalFrequencyHz(l1_obs->signal, eph);
        const double f2 = signalFrequencyHz(l2_obs->signal, eph);
        const double lambda1 = signalWavelengthMeters(l1_obs->signal, eph);
        const double lambda2 = signalWavelengthMeters(l2_obs->signal, eph);
        if (lambda1 <= 0.0 || lambda2 <= 0.0 || f1 <= 0.0 || f2 <= 0.0 ||
            std::abs(f1 - f2) < 1.0) {
            return false;
        }

        alpha1 = f1 / (f1 + f2);
        alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        beta = f1 * f2 / (f1 * f1 - f2 * f2);
        l1_corr_m = l1_obs->carrier_phase * lambda1;
        l2_corr_m = l2_obs->carrier_phase * lambda2;
        nl_phase_m = alpha1 * l1_corr_m + alpha2 * l2_corr_m;

        Vector3d sat_vel;
        double sat_drift = 0.0;
        if (!nav.calculateSatelliteState(sat, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
            return false;
        }
        if (ssr_products_loaded_) {
            Vector3d orbit_corr;
            double clock_corr = 0.0;
            if (ssr_products_.interpolateCorrection(
                    sat, obs.time, orbit_corr, clock_corr, nullptr, nullptr, nullptr, nullptr)) {
                if (ssr_products_.orbitCorrectionsAreRac()) {
                    orbit_corr = ppp_utils::ssrRacToEcef(sat_pos, sat_vel, orbit_corr);
                }
                sat_pos += orbit_corr;
                sat_clk += clock_corr / constants::SPEED_OF_LIGHT;
            }
        }
        const double geo_range = geodist(sat_pos, receiver_position);
        const double elevation = std::asin(
            (sat_pos - receiver_position).normalized().dot(receiver_position.normalized()));
        const double trop_delay =
            calculateMappingFunction(receiver_position, elevation, obs.time) * trop_zenith;
        predicted_m = geo_range + clock_bias_m
                    - constants::SPEED_OF_LIGHT * sat_clk + trop_delay;
    }

    if (lambda_nl <= 0.0 || !std::isfinite(nl_phase_m) || !std::isfinite(predicted_m)) {
        return false;
    }

    const double nl_amb_cycles = (nl_phase_m - predicted_m) / lambda_nl;
    info = {
        nl_amb_cycles,
        lambda_nl,
        lambda_wl,
        beta,
        alpha1,
        alpha2,
        l1_corr_m,
        l2_corr_m,
        nl_phase_m,
        predicted_m,
        {sat.system,
         {static_cast<int>(l1_obs->signal), static_cast<int>(l2_obs->signal)}},
        true
    };
    return true;
}

bool PPPProcessor::buildFixedNlObservationForSatellite(
    const ObservationData& obs,
    const NavigationData& nav,
    const std::map<SatelliteId, OSRCorrection>& osr_by_sat,
    const SatelliteId& satellite,
    const PPPAmbiguityInfo& ambiguity,
    ppp_ar::FixedNlObservation& fixed_observation) const {
    const double fixed_nl = ambiguity.nl_fixed_cycles;
    double nl_phase_m = 0.0;
    double lambda_nl = 0.0;
    Vector3d sat_pos = Vector3d::Zero();
    double sat_clk = 0.0;
    bool use_trop_model = true;

    const auto osr_it = osr_by_sat.find(satellite);
    if (osr_it != osr_by_sat.end()) {
        const auto& osr = osr_it->second;
        const Observation* l1 = obs.getObservation(satellite, osr.signals[0]);
        const Observation* l2 = obs.getObservation(satellite, osr.signals[1]);
        const double f1 = osr.frequencies[0];
        const double f2 = osr.frequencies[1];
        if (l1 == nullptr || l2 == nullptr ||
            !l1->has_carrier_phase || !l2->has_carrier_phase ||
            !std::isfinite(l1->carrier_phase) || !std::isfinite(l2->carrier_phase) ||
            f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            return false;
        }
        const double alpha1 = f1 / (f1 + f2);
        const double alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        const double l1_corr_m =
            l1->carrier_phase * osr.wavelengths[0] -
            (osr.CPC[0] - osr.phase_compensation_m[0]);
        const double l2_corr_m =
            l2->carrier_phase * osr.wavelengths[1] -
            (osr.CPC[1] - osr.phase_compensation_m[1]);
        nl_phase_m = alpha1 * l1_corr_m + alpha2 * l2_corr_m;
        sat_pos = osr.satellite_position;
        sat_clk = osr.satellite_clock_bias_s;
        use_trop_model = false;
    } else {
        const Observation* l1 = ppp_utils::findCarrierObservation(
            obs, satellite, {SignalType::GPS_L1CA, SignalType::GAL_E1, SignalType::QZS_L1CA});
        const Observation* l2 = ppp_utils::findCarrierObservation(
            obs, satellite, {SignalType::GPS_L2C, SignalType::GPS_L2P,
                             SignalType::GAL_E5A, SignalType::QZS_L2C});
        if (!l1 || !l2) {
            return false;
        }

        const Ephemeris* eph = nav.getEphemeris(satellite, obs.time);
        const double f1 = signalFrequencyHz(l1->signal, eph);
        const double f2 = signalFrequencyHz(l2->signal, eph);
        const double lam1 = signalWavelengthMeters(l1->signal, eph);
        const double lam2 = signalWavelengthMeters(l2->signal, eph);
        if (lam1 <= 0.0 || lam2 <= 0.0 || f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            return false;
        }

        const double alpha1 = f1 / (f1 + f2);
        const double alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        nl_phase_m = alpha1 * l1->carrier_phase * lam1 + alpha2 * l2->carrier_phase * lam2;

        Vector3d sat_vel;
        double sat_drift = 0.0;
        if (!nav.calculateSatelliteState(
                satellite, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
            return false;
        }
        if (ssr_products_loaded_) {
            Vector3d orbit_corr;
            double clock_corr = 0.0;
            if (ssr_products_.interpolateCorrection(
                    satellite, obs.time, orbit_corr, clock_corr, nullptr, nullptr, nullptr, nullptr)) {
                if (ssr_products_.orbitCorrectionsAreRac()) {
                    orbit_corr = ppp_utils::ssrRacToEcef(sat_pos, sat_vel, orbit_corr);
                }
                sat_pos += orbit_corr;
                sat_clk += clock_corr / constants::SPEED_OF_LIGHT;
            }
        }
    }

    if (lambda_nl <= 0.0 || !std::isfinite(nl_phase_m)) {
        return false;
    }

    fixed_observation.nl_phase_m = nl_phase_m;
    fixed_observation.fixed_nl_cycles = fixed_nl;
    fixed_observation.lambda_nl_m = lambda_nl;
    fixed_observation.sat_pos = sat_pos;
    fixed_observation.sat_clk = sat_clk;
    fixed_observation.system_clock_offset_m =
        receiverClockBiasMeters(satellite) -
        filter_state_.state(filter_state_.clock_index);
    fixed_observation.use_trop_model = use_trop_model;
    return true;
}

bool PPPProcessor::buildFixedCarrierObservation(
    const IonosphereFreeObs& observation,
    ppp_ar::FixedCarrierObservation& fixed_observation) const {
    if (!observation.valid || !observation.has_carrier_phase) {
        return false;
    }
    const auto ambiguity_it = ambiguity_states_.find(observation.satellite);
    if (ambiguity_it == ambiguity_states_.end() || !ambiguity_it->second.is_fixed) {
        return false;
    }
    const int ambiguity_index = ambiguityStateIndex(observation.satellite);
    if (ambiguity_index < 0 || ambiguity_index >= filter_state_.total_states) {
        return false;
    }

    double ionosphere_m = 0.0;
    if (ppp_config_.estimate_ionosphere) {
        const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
        if (iono_it != filter_state_.ionosphere_indices.end()) {
            ionosphere_m = filter_state_.state(iono_it->second);
        }
    }

    fixed_observation.satellite_position = observation.satellite_position;
    fixed_observation.satellite_clock_bias_s = observation.satellite_clock_bias;
    fixed_observation.trop_mapping = observation.trop_mapping;
    fixed_observation.modeled_trop_delay_m = observation.modeled_trop_delay_m;
    fixed_observation.carrier_phase_if = observation.carrier_phase_if;
    fixed_observation.variance_cp = observation.variance_cp;
    fixed_observation.ambiguity_m = filter_state_.state(ambiguity_index);
    fixed_observation.system_clock_offset_m =
        receiverClockBiasMeters(observation.satellite) -
        filter_state_.state(filter_state_.clock_index);
    fixed_observation.ionosphere_m = ionosphere_m;
    return true;
}

}  // namespace libgnss
