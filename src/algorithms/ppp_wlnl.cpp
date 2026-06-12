// WL+NL ambiguity resolution and fixed-position solver methods for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <vector>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

double quietNaN() {
    return std::numeric_limits<double>::quiet_NaN();
}

std::ofstream* clasNlDebugStream() {
    const auto& path = pppEnvOverrides().clas_nl_debug_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,sat,signal1,signal2,"
                   << "lambda1_m,lambda2_m,lambda_nl_m,lambda_wl_m,"
                   << "alpha1,alpha2,beta,"
                   << "phase1_cycles,phase2_cycles,phase1_m,phase2_m,"
                   << "raw_nl_m,applied_cpc_minus_windup_comp_m,full_cpc_m,"
                   << "current_nl_phase_m,full_cpc_nl_phase_m,"
                   << "geo_m,receiver_clock_m,satellite_clock_m,trop_pred_m,nl_iono_pred_m,predicted_m,"
                   << "obs_total_cycles,obs_full_cpc_total_cycles,"
                   << "state_l1_m,state_l2_m,state_l1_cycles,state_l2_cycles,"
                   << "state_sum_cycles,state_l1_wl_cycles,state_l2_wl_cycles,state_beta_wl_cycles,"
                   << "delta_state_sum_cycles,delta_l1_wl_cycles,delta_l2_wl_cycles,delta_beta_wl_cycles,"
                   << "wl_fixed,wl_integer,mw_mean_cycles,mw_minus_wl_cycles,mw_count,lock_count,"
                   << "cpc1_m,cpc2_m,phase_bias1_m,phase_bias2_m,"
                   << "windup1_m,windup2_m,phase_comp1_m,phase_comp2_m,"
                   << "receiver_ant1_m,receiver_ant2_m,relativity_m,"
                   << "iono_l1_m,iono_cpc1_m,iono_cpc2_m,trop_correction_m,"
                   << "current_minus_full_cpc_cycles,phase_bias_nl_cycles,"
                   << "windup_nl_cycles,phase_comp_nl_cycles,receiver_ant_nl_cycles,"
                   << "relativity_nl_cycles,iono_cpc_nl_cycles,trop_correction_cycles\n";
        }
    }
    return stream ? &stream : nullptr;
}

std::ofstream* clasFixDebugStream() {
    const auto& path = pppEnvOverrides().clas_fix_debug_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,sat,ref_sat,nobs,"
                   << "row_index,row_type,weight,"
                   << "initial_x_m,initial_y_m,initial_z_m,"
                   << "fixed_x_m,fixed_y_m,fixed_z_m,position_shift_m,"
                   << "final_clock_m,residual_m,dd_residual_m,"
                   << "sat_x_m,sat_y_m,sat_z_m,satellite_clock_s,satellite_clock_m,"
                   << "geo_m,trop_applied_m,extra_prediction_m,"
                   << "fixed_nl_m,nl_phase_m,cpc_nl_m,"
                   << "osr_trop_correction_m,osr_nl_iono_m,receiver_ant_nl_m,"
                   << "lambda_nl_m,fixed_nl_cycles,use_trop_model,"
                   << "signal1,signal2,frequency1_hz,frequency2_hz,"
                   << "raw_phase1_m,raw_phase2_m,corrected_phase1_m,corrected_phase2_m,"
                   << "raw_code1_m,raw_code2_m,corrected_code1_m,corrected_code2_m,"
                   << "cpc1_m,cpc2_m,prc1_m,prc2_m,"
                   << "receiver_ant1_m,receiver_ant2_m,"
                   << "tide_x_m,tide_y_m,tide_z_m\n";
        }
    }
    return stream ? &stream : nullptr;
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
    last_clas_constrained_fixed_state_valid_ = false;

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
        pre_anchor_covariance_,
        ambiguity_states_,
        eligible_ambiguities,
        [&](const SatelliteId& sat, ppp_ar::WlnlNlInfo& info) {
            return buildWlnlNlInfoForSatellite(
                obs, nav, receiver_position, clock_bias_m, trop_zenith,
                osr_by_sat, sat, info);
        },
        pppDebugEnabled());
    if (!attempt.fixed) {
        return false;
    }

    last_ar_ratio_ = attempt.ratio;
    last_fixed_ambiguities_ = attempt.nb;
    if (attempt.has_constrained_state) {
        last_clas_constrained_fixed_state_ = attempt.constrained_state;
        last_clas_constrained_fixed_state_valid_ = true;
    }
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
    double fixed_clock_m = clock_m;
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
        &position_shift_norm_m,
        &fixed_clock_m);
    if (solved) {
        if (auto* debug = clasFixDebugStream(); debug != nullptr) {
            struct ResidualRecord {
                const ppp_ar::FixedNlObservation* observation = nullptr;
                double residual_m = 0.0;
                double geo_m = 0.0;
                double trop_applied_m = 0.0;
            };
            std::vector<ResidualRecord> residual_records;
            residual_records.reserve(fixed_observations.size());
            for (const auto& fixed_observation : fixed_observations) {
                const double geo = geodist(fixed_observation.sat_pos, fixed_position);
                const Vector3d los =
                    (fixed_observation.sat_pos - fixed_position).normalized();
                const double elevation = std::asin(los.dot(fixed_position.normalized()));
                const double trop_applied =
                    fixed_observation.use_trop_model
                        ? calculateMappingFunction(fixed_position, elevation, obs.time) *
                              trop_zenith
                        : 0.0;
                const double predicted =
                    geo + fixed_clock_m -
                    constants::SPEED_OF_LIGHT * fixed_observation.sat_clk +
                    trop_applied +
                    fixed_observation.extra_prediction_m +
                    fixed_observation.fixed_nl_cycles *
                        fixed_observation.lambda_nl_m;
                residual_records.push_back({
                    &fixed_observation,
                    fixed_observation.nl_phase_m - predicted,
                    geo,
                    trop_applied,
                });
            }

            std::map<GNSSSystem, size_t> reference_by_system;
            for (size_t index = 0; index < residual_records.size(); ++index) {
                const auto system = residual_records[index].observation->satellite.system;
                if (reference_by_system.find(system) == reference_by_system.end()) {
                    reference_by_system[system] = index;
                }
            }

            for (size_t row_index = 0; row_index < residual_records.size(); ++row_index) {
                const auto& record = residual_records[row_index];
                const auto& fixed_observation = *record.observation;
                const auto ref_it =
                    reference_by_system.find(fixed_observation.satellite.system);
                const ResidualRecord* ref_record =
                    ref_it != reference_by_system.end()
                        ? &residual_records[ref_it->second]
                        : &record;
                const double dd_residual =
                    ref_record->residual_m - record.residual_m;
                *debug << std::setprecision(17)
                       << "SAT,"
                       << obs.time.week << ','
                       << obs.time.tow << ','
                       << fixed_observation.satellite.toString() << ','
                       << ref_record->observation->satellite.toString() << ','
                       << fixed_observations.size() << ','
                       << row_index << ','
                       << "nl_phase,"
                       << 1.0 << ','
                       << position.x() << ','
                       << position.y() << ','
                       << position.z() << ','
                       << fixed_position.x() << ','
                       << fixed_position.y() << ','
                       << fixed_position.z() << ','
                       << position_shift_norm_m << ','
                       << fixed_clock_m << ','
                       << record.residual_m << ','
                       << dd_residual << ','
                       << fixed_observation.sat_pos.x() << ','
                       << fixed_observation.sat_pos.y() << ','
                       << fixed_observation.sat_pos.z() << ','
                       << fixed_observation.sat_clk << ','
                       << constants::SPEED_OF_LIGHT * fixed_observation.sat_clk << ','
                       << record.geo_m << ','
                       << record.trop_applied_m << ','
                       << fixed_observation.extra_prediction_m << ','
                       << fixed_observation.fixed_nl_cycles *
                              fixed_observation.lambda_nl_m << ','
                       << fixed_observation.nl_phase_m << ','
                       << fixed_observation.cpc_nl_m << ','
                       << fixed_observation.osr_trop_correction_m << ','
                       << fixed_observation.osr_nl_iono_m << ','
                       << fixed_observation.receiver_ant_nl_m << ','
                       << fixed_observation.lambda_nl_m << ','
                       << fixed_observation.fixed_nl_cycles << ','
                       << (fixed_observation.use_trop_model ? 1 : 0) << ','
                       << static_cast<int>(fixed_observation.signal1) << ','
                       << static_cast<int>(fixed_observation.signal2) << ','
                       << fixed_observation.frequency1_hz << ','
                       << fixed_observation.frequency2_hz << ','
                       << fixed_observation.raw_phase1_m << ','
                       << fixed_observation.raw_phase2_m << ','
                       << fixed_observation.corrected_phase1_m << ','
                       << fixed_observation.corrected_phase2_m << ','
                       << fixed_observation.raw_code1_m << ','
                       << fixed_observation.raw_code2_m << ','
                       << fixed_observation.corrected_code1_m << ','
                       << fixed_observation.corrected_code2_m << ','
                       << fixed_observation.cpc1_m << ','
                       << fixed_observation.cpc2_m << ','
                       << fixed_observation.prc1_m << ','
                       << fixed_observation.prc2_m << ','
                       << fixed_observation.receiver_ant1_m << ','
                       << fixed_observation.receiver_ant2_m << ','
                       << 0.0 << ','
                       << 0.0 << ','
                       << 0.0
                       << '\n';
            }
        }
    }
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
    double iono_state_scale = 0.0;
    double nl_phase_m = 0.0;
    double predicted_m = 0.0;
    double f1_hz = 0.0;
    double f2_hz = 0.0;
    double lambda1_m = 0.0;
    double lambda2_m = 0.0;
    double phase1_m = quietNaN();
    double phase2_m = quietNaN();
    double raw_nl_m = quietNaN();
    double applied_cpc_minus_windup_comp_m = quietNaN();
    double full_cpc_m = quietNaN();
    double full_cpc_nl_phase_m = quietNaN();
    double geo_m = quietNaN();
    double sat_clk_m = quietNaN();
    double trop_pred_m = quietNaN();
    double nl_iono_m = quietNaN();
    double cpc1_m = quietNaN();
    double cpc2_m = quietNaN();
    double phase_bias1_m = quietNaN();
    double phase_bias2_m = quietNaN();
    double windup1_m = quietNaN();
    double windup2_m = quietNaN();
    double phase_comp1_m = quietNaN();
    double phase_comp2_m = quietNaN();
    double receiver_ant1_m = quietNaN();
    double receiver_ant2_m = quietNaN();
    double relativity_m = quietNaN();
    double iono_l1_m = quietNaN();
    double iono_cpc1_m = quietNaN();
    double iono_cpc2_m = quietNaN();
    double trop_correction_m = quietNaN();

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
        f1_hz = f1;
        f2_hz = f2;
        lambda1_m = osr.wavelengths[0];
        lambda2_m = osr.wavelengths[1];
        alpha1 = f1 / (f1 + f2);
        alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        beta = f1 * f2 / (f1 * f1 - f2 * f2);
        phase1_m = l1_obs->carrier_phase * osr.wavelengths[0];
        phase2_m = l2_obs->carrier_phase * osr.wavelengths[1];
        cpc1_m = osr.CPC[0];
        cpc2_m = osr.CPC[1];
        phase_bias1_m = osr.phase_bias_m[0];
        phase_bias2_m = osr.phase_bias_m[1];
        windup1_m = osr.windup_m[0];
        windup2_m = osr.windup_m[1];
        phase_comp1_m = osr.phase_compensation_m[0];
        phase_comp2_m = osr.phase_compensation_m[1];
        receiver_ant1_m = osr.receiver_antenna_m[0];
        receiver_ant2_m = osr.receiver_antenna_m[1];
        relativity_m = osr.relativity_correction_m;
        iono_l1_m = osr.iono_l1_m;
        trop_correction_m = osr.trop_correction_m;
        const double iono_scale1 =
            (osr.frequencies[0] > 0.0 && osr.wavelengths[0] > 0.0)
                ? std::pow(osr.wavelengths[0] / osr.wavelengths[0], 2)
                : 1.0;
        const double iono_scale2 =
            (osr.frequencies[1] > 0.0 && osr.wavelengths[0] > 0.0)
                ? std::pow(osr.wavelengths[1] / osr.wavelengths[0], 2)
                : 1.0;
        iono_state_scale = alpha1 * iono_scale1 + alpha2 * iono_scale2;
        iono_cpc1_m = -iono_scale1 * osr.iono_l1_m;
        iono_cpc2_m = -iono_scale2 * osr.iono_l1_m;
        raw_nl_m = alpha1 * phase1_m + alpha2 * phase2_m;
        const bool use_full_cpc_for_nl =
            pppEnvOverrides().clas_nl_cpc_unified &&
            ppp_config_.use_clas_osr_filter;
        applied_cpc_minus_windup_comp_m =
            alpha1 * (osr.CPC[0] - osr.windup_m[0] - osr.phase_compensation_m[0]) +
            alpha2 * (osr.CPC[1] - osr.windup_m[1] - osr.phase_compensation_m[1]);
        full_cpc_m = alpha1 * osr.CPC[0] + alpha2 * osr.CPC[1];
        const double l1_nl_cpc_m = use_full_cpc_for_nl
            ? osr.CPC[0]
            : (osr.CPC[0] - osr.windup_m[0] - osr.phase_compensation_m[0]);
        const double l2_nl_cpc_m = use_full_cpc_for_nl
            ? osr.CPC[1]
            : (osr.CPC[1] - osr.windup_m[1] - osr.phase_compensation_m[1]);
        const double l1_corr_m = phase1_m - l1_nl_cpc_m;
        const double l2_corr_m = phase2_m - l2_nl_cpc_m;
        nl_phase_m = alpha1 * l1_corr_m + alpha2 * l2_corr_m;
        full_cpc_nl_phase_m = raw_nl_m - full_cpc_m;
        sat_pos = osr.satellite_position;
        sat_clk = osr.satellite_clock_bias_s;
        nl_iono_m = osr.has_iono ? osr.iono_l1_m * f1 / f2 : 0.0;
        geo_m = geodist(sat_pos, receiver_position);
        sat_clk_m = constants::SPEED_OF_LIGHT * sat_clk;
        trop_pred_m = osr.trop_correction_m;
        predicted_m = geo_m + clock_bias_m - sat_clk_m + trop_pred_m + nl_iono_m;
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

        f1_hz = f1;
        f2_hz = f2;
        lambda1_m = lambda1;
        lambda2_m = lambda2;
        alpha1 = f1 / (f1 + f2);
        alpha2 = f2 / (f1 + f2);
        lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
        lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        beta = f1 * f2 / (f1 * f1 - f2 * f2);
        iono_state_scale =
            alpha1 + alpha2 * std::pow(lambda2 / lambda1, 2);

        const double l1_m = l1_obs->carrier_phase * lambda1;
        const double l2_m = l2_obs->carrier_phase * lambda2;
        phase1_m = l1_m;
        phase2_m = l2_m;
        raw_nl_m = alpha1 * l1_m + alpha2 * l2_m;
        nl_phase_m = raw_nl_m;
        full_cpc_nl_phase_m = raw_nl_m;

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
        geo_m = geo_range;
        sat_clk_m = constants::SPEED_OF_LIGHT * sat_clk;
        trop_pred_m = trop_delay;
        nl_iono_m = 0.0;
        predicted_m = geo_range + clock_bias_m - sat_clk_m + trop_delay;
    }

    if (lambda_nl <= 0.0 || !std::isfinite(nl_phase_m) || !std::isfinite(predicted_m)) {
        return false;
    }

    const double nl_amb_cycles = (nl_phase_m - predicted_m) / lambda_nl;
    if (auto* debug = clasNlDebugStream(); debug != nullptr) {
        const SatelliteId l2_satellite(
            sat.system,
            static_cast<uint8_t>(std::min(255, static_cast<int>(sat.prn) + 100)));
        const auto l1_state_it = filter_state_.ambiguity_indices.find(sat);
        const auto l2_state_it = filter_state_.ambiguity_indices.find(l2_satellite);
        const auto l1_amb_it = ambiguity_states_.find(sat);
        const auto l2_amb_it = ambiguity_states_.find(l2_satellite);

        const bool has_l1_state =
            l1_state_it != filter_state_.ambiguity_indices.end() &&
            l1_state_it->second >= 0 &&
            l1_state_it->second < filter_state_.total_states &&
            lambda1_m > 0.0;
        const bool has_l2_state =
            l2_state_it != filter_state_.ambiguity_indices.end() &&
            l2_state_it->second >= 0 &&
            l2_state_it->second < filter_state_.total_states &&
            lambda2_m > 0.0;

        const double state_l1_m = has_l1_state
            ? filter_state_.state(l1_state_it->second)
            : quietNaN();
        const double state_l2_m = has_l2_state
            ? filter_state_.state(l2_state_it->second)
            : quietNaN();
        const double state_l1_cycles =
            has_l1_state ? state_l1_m / lambda1_m : quietNaN();
        const double state_l2_cycles =
            has_l2_state ? state_l2_m / lambda2_m : quietNaN();
        const bool wl_fixed =
            l1_amb_it != ambiguity_states_.end() && l1_amb_it->second.wl_is_fixed;
        const int wl_integer = wl_fixed ? l1_amb_it->second.wl_fixed_integer : 0;
        const double state_sum_cycles =
            has_l1_state && has_l2_state
                ? state_l1_cycles + state_l2_cycles
                : quietNaN();
        const double state_l1_wl_cycles =
            has_l1_state && wl_fixed
                ? 2.0 * state_l1_cycles - static_cast<double>(wl_integer)
                : quietNaN();
        const double state_l2_wl_cycles =
            has_l2_state && wl_fixed
                ? 2.0 * state_l2_cycles + static_cast<double>(wl_integer)
                : quietNaN();
        const double state_beta_wl_cycles =
            has_l1_state && wl_fixed
                ? (state_l1_m -
                   static_cast<double>(wl_integer) * beta * lambda_wl) / lambda_nl
                : quietNaN();
        const double obs_full_cpc_total_cycles =
            std::isfinite(full_cpc_nl_phase_m)
                ? (full_cpc_nl_phase_m - predicted_m) / lambda_nl
                : quietNaN();
        const double current_minus_full_cpc_cycles =
            std::isfinite(full_cpc_nl_phase_m)
                ? (nl_phase_m - full_cpc_nl_phase_m) / lambda_nl
                : quietNaN();
        const auto weighted_cycles = [&](double v1, double v2) {
            return (std::isfinite(v1) && std::isfinite(v2) && lambda_nl > 0.0)
                ? (alpha1 * v1 + alpha2 * v2) / lambda_nl
                : quietNaN();
        };
        const double phase_bias_nl_cycles =
            weighted_cycles(phase_bias1_m, phase_bias2_m);
        const double windup_nl_cycles =
            weighted_cycles(windup1_m, windup2_m);
        const double phase_comp_nl_cycles =
            weighted_cycles(phase_comp1_m, phase_comp2_m);
        const double receiver_ant_nl_cycles =
            weighted_cycles(receiver_ant1_m, receiver_ant2_m);
        const double relativity_nl_cycles =
            std::isfinite(relativity_m) && lambda_nl > 0.0
                ? relativity_m / lambda_nl
                : quietNaN();
        const double iono_cpc_nl_cycles =
            weighted_cycles(iono_cpc1_m, iono_cpc2_m);
        const double trop_correction_cycles =
            std::isfinite(trop_correction_m) && lambda_nl > 0.0
                ? trop_correction_m / lambda_nl
                : quietNaN();
        const double mw_mean =
            l1_amb_it != ambiguity_states_.end()
                ? l1_amb_it->second.mw_mean_cycles
                : quietNaN();
        const double mw_minus_wl =
            std::isfinite(mw_mean) && wl_fixed
                ? mw_mean - static_cast<double>(wl_integer)
                : quietNaN();
        const int mw_count =
            l1_amb_it != ambiguity_states_.end()
                ? l1_amb_it->second.mw_count
                : 0;
        const int lock_count =
            l1_amb_it != ambiguity_states_.end()
                ? l1_amb_it->second.lock_count
                : 0;

        *debug << std::setprecision(17)
               << "SAT,"
               << obs.time.week << ','
               << obs.time.tow << ','
               << sat.toString() << ','
               << static_cast<int>(l1_obs->signal) << ','
               << static_cast<int>(l2_obs->signal) << ','
               << lambda1_m << ','
               << lambda2_m << ','
               << lambda_nl << ','
               << lambda_wl << ','
               << alpha1 << ','
               << alpha2 << ','
               << beta << ','
               << l1_obs->carrier_phase << ','
               << l2_obs->carrier_phase << ','
               << phase1_m << ','
               << phase2_m << ','
               << raw_nl_m << ','
               << applied_cpc_minus_windup_comp_m << ','
               << full_cpc_m << ','
               << nl_phase_m << ','
               << full_cpc_nl_phase_m << ','
               << geo_m << ','
               << clock_bias_m << ','
               << sat_clk_m << ','
               << trop_pred_m << ','
               << nl_iono_m << ','
               << predicted_m << ','
               << nl_amb_cycles << ','
               << obs_full_cpc_total_cycles << ','
               << state_l1_m << ','
               << state_l2_m << ','
               << state_l1_cycles << ','
               << state_l2_cycles << ','
               << state_sum_cycles << ','
               << state_l1_wl_cycles << ','
               << state_l2_wl_cycles << ','
               << state_beta_wl_cycles << ','
               << (std::isfinite(state_sum_cycles)
                       ? nl_amb_cycles - state_sum_cycles
                       : quietNaN()) << ','
               << (std::isfinite(state_l1_wl_cycles)
                       ? nl_amb_cycles - state_l1_wl_cycles
                       : quietNaN()) << ','
               << (std::isfinite(state_l2_wl_cycles)
                       ? nl_amb_cycles - state_l2_wl_cycles
                       : quietNaN()) << ','
               << (std::isfinite(state_beta_wl_cycles)
                       ? nl_amb_cycles - state_beta_wl_cycles
                       : quietNaN()) << ','
               << (wl_fixed ? 1 : 0) << ','
               << wl_integer << ','
               << mw_mean << ','
               << mw_minus_wl << ','
               << mw_count << ','
               << lock_count << ','
               << cpc1_m << ','
               << cpc2_m << ','
               << phase_bias1_m << ','
               << phase_bias2_m << ','
               << windup1_m << ','
               << windup2_m << ','
               << phase_comp1_m << ','
               << phase_comp2_m << ','
               << receiver_ant1_m << ','
               << receiver_ant2_m << ','
               << relativity_m << ','
               << iono_l1_m << ','
               << iono_cpc1_m << ','
               << iono_cpc2_m << ','
               << trop_correction_m << ','
               << current_minus_full_cpc_cycles << ','
               << phase_bias_nl_cycles << ','
               << windup_nl_cycles << ','
               << phase_comp_nl_cycles << ','
               << receiver_ant_nl_cycles << ','
               << relativity_nl_cycles << ','
               << iono_cpc_nl_cycles << ','
               << trop_correction_cycles
               << '\n';
    }
    info = {
        nl_amb_cycles,
        lambda_nl,
        lambda_wl,
        beta,
        alpha1,
        alpha2,
        iono_state_scale,
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
        const double cpc_nl_m = alpha1 * osr.CPC[0] + alpha2 * osr.CPC[1];
        const double l1_corr_m = l1->carrier_phase * osr.wavelengths[0] - osr.CPC[0];
        const double l2_corr_m = l2->carrier_phase * osr.wavelengths[1] - osr.CPC[1];
        nl_phase_m = alpha1 * l1_corr_m + alpha2 * l2_corr_m;
        sat_pos = osr.satellite_position;
        sat_clk = osr.satellite_clock_bias_s;
        use_trop_model = false;
        fixed_observation.signal1 = l1->signal;
        fixed_observation.signal2 = l2->signal;
        fixed_observation.frequency1_hz = f1;
        fixed_observation.frequency2_hz = f2;
        fixed_observation.raw_phase1_m = l1->carrier_phase * osr.wavelengths[0];
        fixed_observation.raw_phase2_m = l2->carrier_phase * osr.wavelengths[1];
        fixed_observation.corrected_phase1_m = l1_corr_m;
        fixed_observation.corrected_phase2_m = l2_corr_m;
        fixed_observation.raw_code1_m =
            l1->has_pseudorange ? l1->pseudorange : quietNaN();
        fixed_observation.raw_code2_m =
            l2->has_pseudorange ? l2->pseudorange : quietNaN();
        fixed_observation.corrected_code1_m =
            l1->has_pseudorange ? l1->pseudorange - osr.PRC[0] : quietNaN();
        fixed_observation.corrected_code2_m =
            l2->has_pseudorange ? l2->pseudorange - osr.PRC[1] : quietNaN();
        fixed_observation.cpc1_m = osr.CPC[0];
        fixed_observation.cpc2_m = osr.CPC[1];
        fixed_observation.prc1_m = osr.PRC[0];
        fixed_observation.prc2_m = osr.PRC[1];
        fixed_observation.receiver_ant1_m = osr.receiver_antenna_m[0];
        fixed_observation.receiver_ant2_m = osr.receiver_antenna_m[1];
        fixed_observation.cpc_nl_m = cpc_nl_m;
        fixed_observation.osr_trop_correction_m = osr.trop_correction_m;
        fixed_observation.osr_nl_iono_m = osr.has_iono ? osr.iono_l1_m * f1 / f2 : 0.0;
        fixed_observation.extra_prediction_m =
            pppEnvOverrides().clas_vertical_fix && ppp_config_.use_clas_osr_filter
                ? fixed_observation.osr_trop_correction_m +
                      fixed_observation.osr_nl_iono_m
                : 0.0;
        fixed_observation.receiver_ant_nl_m =
            alpha1 * osr.receiver_antenna_m[0] + alpha2 * osr.receiver_antenna_m[1];
    } else {
        if (ppp_config_.use_clas_osr_filter && env_overrides_.clas_fix_require_osr) {
            return false;
        }
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
        fixed_observation.signal1 = l1->signal;
        fixed_observation.signal2 = l2->signal;
        fixed_observation.frequency1_hz = f1;
        fixed_observation.frequency2_hz = f2;
        fixed_observation.raw_phase1_m = l1->carrier_phase * lam1;
        fixed_observation.raw_phase2_m = l2->carrier_phase * lam2;
        fixed_observation.corrected_phase1_m = fixed_observation.raw_phase1_m;
        fixed_observation.corrected_phase2_m = fixed_observation.raw_phase2_m;
        fixed_observation.raw_code1_m =
            l1->has_pseudorange ? l1->pseudorange : quietNaN();
        fixed_observation.raw_code2_m =
            l2->has_pseudorange ? l2->pseudorange : quietNaN();
        fixed_observation.corrected_code1_m = fixed_observation.raw_code1_m;
        fixed_observation.corrected_code2_m = fixed_observation.raw_code2_m;

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

    fixed_observation.satellite = satellite;
    fixed_observation.nl_phase_m = nl_phase_m;
    fixed_observation.fixed_nl_cycles = fixed_nl;
    fixed_observation.lambda_nl_m = lambda_nl;
    fixed_observation.sat_pos = sat_pos;
    fixed_observation.sat_clk = sat_clk;
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
