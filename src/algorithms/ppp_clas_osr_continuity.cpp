// CLAS OSR continuity: SSR orbit/clock boundary tracking, SIS
// change-capture, atmosphere token/network selection and the
// computeOSR entry point. Split from the former monolithic
// ppp_osr.cpp; shared helpers are declared in ppp_osr_internal.hpp.

#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>


#include "ppp_osr_internal.hpp"

namespace libgnss {

bool isSsrOrbitBoundaryTowWithTolerance(double tow, double tolerance) {
    double remainder = std::fmod(tow, kSsrOrbitBoundaryIntervalSeconds);
    if (remainder < 0.0) {
        remainder += kSsrOrbitBoundaryIntervalSeconds;
    }
    return remainder < tolerance ||
        remainder > (kSsrOrbitBoundaryIntervalSeconds -
                      tolerance);
}

bool isSsrOrbitBoundaryTow(double tow) {
    return isSsrOrbitBoundaryTowWithTolerance(
        tow, kSsrOrbitBoundaryToleranceSeconds);
}

bool isSsrOrbitBoundaryOffset25TowWithTolerance(double tow,
                                                double tolerance) {
    double remainder = std::fmod(tow, kSsrOrbitBoundaryIntervalSeconds);
    if (remainder < 0.0) {
        remainder += kSsrOrbitBoundaryIntervalSeconds;
    }
    return std::abs(remainder - kSsrOrbitBoundaryOffset25Seconds) <
        tolerance;
}

bool isSsrOrbitBoundaryOffset25Tow(double tow) {
    return isSsrOrbitBoundaryOffset25TowWithTolerance(
        tow, kSsrOrbitBoundaryToleranceSeconds);
}

void captureClasSisBoundary(
    CLASSisContinuityInfo& info,
    const GNSSTime& epoch_time,
    double current_sis_m,
    double boundary_tolerance) {
    // CLASLIB pairs sis(offset-25 obs epoch) with sis(offset-0 obs epoch):
    // prevsis is captured at the mid-cycle clock step (tow % 30 == 25) using
    // the orbit/clock corrections in effect at that observation epoch; the
    // held compN delta is formed at the following 30s boundary (tow % 30 == 0)
    // as currsis - prevsis.  This matches ephemeris.c satpos_ssr_sis and
    // avoids mistiming from clock_reference_time steps that arrive a few
    // seconds before the observation epoch reaches the boundary grid.
    if (isSsrOrbitBoundaryOffset25TowWithTolerance(
            epoch_time.tow, boundary_tolerance)) {
        info.boundary_prev_time = epoch_time;
        info.boundary_prev_sis_m = current_sis_m;
        info.has_boundary_prev_sis = true;
        return;
    }
    if (!isSsrOrbitBoundaryTowWithTolerance(
            epoch_time.tow, boundary_tolerance) ||
        !info.has_boundary_prev_sis) {
        return;
    }
    const double dt_prev_to_boundary = epoch_time - info.boundary_prev_time;
    if (std::abs(dt_prev_to_boundary - kSsrOrbitBoundaryPrevToBoundarySeconds) >
        boundary_tolerance) {
        return;
    }
    info.boundary_time = epoch_time;
    info.boundary_delta_m = current_sis_m - info.boundary_prev_sis_m;
    info.has_boundary_delta = true;
}

void captureClasSisBoundary(
    CLASSisContinuityInfo& info,
    const GNSSTime& epoch_time,
    double current_sis_m) {
    captureClasSisBoundary(
        info, epoch_time, current_sis_m,
        kSsrOrbitBoundaryToleranceSeconds);
}

ClasSisApplyDecision computeClasSisApplyDecision(
    const CLASSisContinuityInfo& info,
    const GNSSTime& epoch_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& effective_phase_bias_reference_time,
    bool clock_time_valid,
    bool sis_boundary_gate_enabled,
    double boundary_tolerance) {
    ClasSisApplyDecision decision;
    if (sis_boundary_gate_enabled) {
        // GATED path: CLASLIB-style SSR-update-boundary semantics. The delta
        // captured at the most recent 30s orbit/clock boundary (see
        // captureClasSisBoundary()) is held and applied for the following
        // 15s of observation epochs (empirically pinned against the CLASLIB
        // oracle; see docs/clas_dd_filter_a5.md), then zeroed until the next
        // boundary. This replaces the (unreachable on real CLAS data) 30s
        // phase-bias-lag condition used below when the gate is off.
        //
        // Deliberately independent of `clock_time_valid`/`has_last_delta`
        // for *this* epoch: CLASLIB's held satcorr[] state outlives
        // transient per-epoch SSR gaps inside the hold window (observed on
        // real CLAS data), and captureClasSisBoundary()/updateSisContinuity()
        // already preserve `boundary_delta_m` across such gaps.
        if (!info.has_boundary_delta) {
            return decision;
        }
        const double dt_since_boundary = epoch_time - info.boundary_time;
        if (dt_since_boundary > -boundary_tolerance &&
            dt_since_boundary <
                (kSsrOrbitBoundaryApplyWindowSeconds -
                 boundary_tolerance)) {
            decision.applied = true;
            decision.delta_m = info.boundary_delta_m;
        }
        return decision;
    }
    if (!clock_time_valid || !gnsstimeIsSet(effective_phase_bias_reference_time) ||
        !info.has_last_delta) {
        return decision;
    }
    const double pbias_lag =
        clock_reference_time - effective_phase_bias_reference_time;
    if (std::abs(pbias_lag - kPhaseBiasLagSeconds) < 0.5) {
        decision.applied = true;
        decision.delta_m = info.last_delta_m;
    }
    return decision;
}

ClasSisApplyDecision computeClasSisApplyDecision(
    const CLASSisContinuityInfo& info,
    const GNSSTime& epoch_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& effective_phase_bias_reference_time,
    bool clock_time_valid,
    bool sis_boundary_gate_enabled) {
    return computeClasSisApplyDecision(
        info, epoch_time, clock_reference_time,
        effective_phase_bias_reference_time, clock_time_valid,
        sis_boundary_gate_enabled, kSsrOrbitBoundaryToleranceSeconds);
}

bool usesClasPhaseBiasRepair(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::REPAIR_ONLY;
}

bool usesClasClockBoundPhaseBias(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_PHASE_BIAS ||
           policy ==
               ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
}

bool usesClasClockBoundAtmos(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS;
}

bool usesClasExpandedPolynomialTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY;
}

bool usesClasExpandedResidualTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY;
}

int preferredClasNetworkId(const std::map<std::string, std::string>& atmos_tokens) {
    int network_id = 0;
    if (!ppp_atmosphere::parseAtmosTokenInt(atmos_tokens, "atmos_network_id", network_id)) {
        return 0;
    }
    return std::max(network_id, 0);
}

const Observation* findOsrFrequencyObservation(
    const ObservationData& obs,
    const OSRCorrection& osr,
    int freq_index) {
    return findOsrFrequencyObservationWithProvenance(obs, osr, freq_index)
        .observation;
}

OsrFrequencyObservationLookup findOsrFrequencyObservationWithProvenance(
    const ObservationData& obs,
    const OSRCorrection& osr,
    int freq_index) {
    OsrFrequencyObservationLookup result;
    const int available_frequency_count =
        std::max(osr.num_frequencies, osr.num_output_frequencies);
    if (freq_index < 0 || freq_index >= available_frequency_count ||
        freq_index >= OSR_MAX_FREQ) {
        return result;
    }
    const SignalType signal = osr.signals[freq_index];
    const Observation* fallback = obs.getObservation(osr.satellite, signal);
    result.observation = fallback;
    if (!osr.bias_exact_identity[freq_index]) {
        return result;
    }
    result.exact_identity_requested = true;

    const auto& pseudorange_code = osr.pseudorange_rinex_codes[freq_index];
    const auto& carrier_code = osr.carrier_rinex_codes[freq_index];
    if (pseudorange_code.size() >= 3 && carrier_code.size() >= 3 &&
        pseudorange_code.substr(1) == carrier_code.substr(1)) {
        if (const Observation* exact = obs.getRinexTrackingObservation(
                osr.satellite, pseudorange_code.substr(1));
            exact != nullptr && hasUsableCodeAndPhase(*exact) &&
            exact->pseudorange_observation_type == pseudorange_code &&
            exact->carrier_phase_observation_type == carrier_code) {
            result.observation = exact;
            result.exact_identity_matched = true;
            result.family_fallback = false;
            return result;
        }
    }
    for (const auto& candidate : obs.observations) {
        if (candidate.satellite != osr.satellite || candidate.signal != signal) {
            continue;
        }
        if (!hasUsableCodeAndPhase(candidate)) {
            continue;
        }
        const bool pseudorange_matches =
            pseudorange_code.empty() ||
            candidate.pseudorange_observation_type == pseudorange_code;
        const bool carrier_matches =
            carrier_code.empty() ||
            candidate.carrier_phase_observation_type == carrier_code;
        if (pseudorange_matches && carrier_matches) {
            result.observation = &candidate;
            result.exact_identity_matched = true;
            result.family_fallback = false;
            return result;
        }
    }
    result.family_fallback = fallback != nullptr;
    return result;
}

ClasOsrBiasMaterialization materializeClasOsrBiases(
    GNSSSystem system,
    SignalType signal,
    std::string_view pseudorange_observation_type,
    std::string_view carrier_phase_observation_type,
    const std::map<std::uint8_t, double>& code_biases_m,
    const std::map<std::uint8_t, double>& phase_biases_m,
    bool exact_bias_identity,
    bool enable_l2_class_fallback) {
    ClasOsrBiasMaterialization result;
    result.code_signal_id =
        algorithms::ppp_bias_identity::rtcmSsrSignalIdForObservation(
            system, signal, pseudorange_observation_type, exact_bias_identity);
    result.phase_signal_id =
        algorithms::ppp_bias_identity::rtcmSsrSignalIdForObservation(
            system, signal, carrier_phase_observation_type, exact_bias_identity);
    result.exact_identity = exact_bias_identity;

    const auto code_bias = clasOsrBiasLookup(
        code_biases_m,
        system,
        result.code_signal_id,
        enable_l2_class_fallback && !exact_bias_identity);
    result.code_bias_m = code_bias.value_m;
    result.code_source_signal_id = code_bias.source_signal_id;
    result.code_present = code_bias.present;
    result.code_fallback = code_bias.fallback;

    const auto phase_bias = phase_biases_m.find(result.phase_signal_id);
    if (phase_bias != phase_biases_m.end()) {
        result.phase_bias_m = phase_bias->second;
        result.phase_source_signal_id = result.phase_signal_id;
        result.phase_present = true;
    }
    return result;
}

double clasReceiverAntennaCorrectionMeters(
    const Vector3d& receiver_delta_enu,
    const Vector3d& antenna_offset_neu,
    double pcv_m,
    double azimuth_rad,
    double elevation_rad) {
    const Vector3d antenna_offset_enu(
        antenna_offset_neu.y(),
        antenna_offset_neu.x(),
        antenna_offset_neu.z());
    const Vector3d offset_enu = receiver_delta_enu + antenna_offset_enu;
    const double cosel = std::cos(elevation_rad);
    const Vector3d los_enu(
        std::sin(azimuth_rad) * cosel,
        std::cos(azimuth_rad) * cosel,
        std::sin(elevation_rad));
    return -offset_enu.dot(los_enu) + pcv_m;
}

void setClasOsrReceiverAntennaCorrection(
    OSRCorrection& osr,
    int freq_index,
    double receiver_antenna_m) {
    if (freq_index < 0 || freq_index >= OSR_MAX_FREQ ||
        freq_index >= osr.num_frequencies) {
        return;
    }
    const double delta_m = receiver_antenna_m - osr.receiver_antenna_m[freq_index];
    osr.receiver_antenna_m[freq_index] = receiver_antenna_m;
    osr.PRC[freq_index] += delta_m;
    osr.CPC[freq_index] += delta_m;
}

SignalType clasReceiverAntennaLookupSignal(
    const OSRCorrection& osr,
    int freq_index) {
    if (freq_index < 0 || freq_index >= OSR_MAX_FREQ ||
        freq_index >= osr.num_frequencies) {
        return SignalType::SIGNAL_TYPE_COUNT;
    }
    const SignalType signal = osr.signals[freq_index];
    if (osr.bias_exact_identity[freq_index] &&
        algorithms::ppp_bias_identity::isGpsL2wObservation(
            osr.satellite.system,
            signal,
            osr.pseudorange_rinex_codes[freq_index],
            osr.carrier_rinex_codes[freq_index])) {
        return SignalType::GPS_L1CA;
    }
    // CLASLIB's legacy ANTEX reader discards the constellation letter and
    // stores receiver antenna entries by frequency number only.  In the
    // canonical igs14_L5copy.atx receiver block, R01 is the final *01 entry
    // and C02 is the final *02 entry.  They therefore overwrite the two
    // RTKLIB antenna slots consumed by corrmeas(), including for QZSS.
    // Preserve that file-specific observable quirk only for explicitly
    // tagged QZSS parity rows.
    if (osr.claslib_qzss_receiver_antenna_slots &&
        osr.satellite.system == GNSSSystem::QZSS) {
        if (freq_index == 0) {
            return SignalType::GLO_L1CA;
        }
        if (freq_index == 1) {
            return SignalType::BDS_B1I;
        }
    }
    return signal;
}

std::map<std::string, std::string> selectClasEpochAtmosTokens(
    const SSRProducts& ssr_products,
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time,
    const Vector3d& receiver_position,
    const ppp_shared::PPPConfig& config) {
    constexpr double kAtmosSelectionGapSeconds = 120.0;

    ClasAtmosCandidate best;

    for (const auto& satellite : satellites) {
        const auto sat_it = ssr_products.orbit_clock_corrections.find(satellite);
        if (sat_it == ssr_products.orbit_clock_corrections.end()) {
            continue;
        }
        const GNSSTime window_start = time - kAtmosSelectionGapSeconds;
        const GNSSTime window_end = time + kAtmosSelectionGapSeconds;
        const auto first = std::lower_bound(
            sat_it->second.begin(), sat_it->second.end(), window_start,
            [](const SSROrbitClockCorrection& correction,
               const GNSSTime& epoch) {
                return correction.time < epoch;
            });
        const auto last = std::upper_bound(
            first, sat_it->second.end(), window_end,
            [](const GNSSTime& epoch,
               const SSROrbitClockCorrection& correction) {
                return epoch < correction.time;
            });
        for (auto correction_it = first; correction_it != last; ++correction_it) {
            const auto& correction = *correction_it;
            if (!correction.atmos_valid || correction.atmos_tokens.empty()) {
                continue;
            }
            if (pppEnvOverrides().clas_atmos_lifecycle &&
                correction.time > time + 1e-9) {
                continue;
            }
            const double time_gap = std::abs(correction.time - time);
            if (time_gap > kAtmosSelectionGapSeconds) {
                continue;
            }

            ppp_atmosphere::ClasGridReference grid_reference;
            const bool has_grid = ppp_atmosphere::resolveClasGridReference(
                correction.atmos_tokens, receiver_position, grid_reference);
            const double grid_distance_sq =
                has_grid
                    ? (pppEnvOverrides().clas_atmos_lifecycle
                           ? grid_reference.nearest_grid_distance_m *
                                 grid_reference.nearest_grid_distance_m
                           : grid_reference.dlat_deg * grid_reference.dlat_deg +
                                 grid_reference.dlon_deg * grid_reference.dlon_deg)
                    : std::numeric_limits<double>::infinity();
            const ClasAtmosCandidate candidate{
                correction.atmos_tokens,
                has_grid,
                grid_distance_sq,
                time_gap,
                static_cast<int>(correction.atmos_tokens.size()),
            };

            if (!isBetterClasAtmosCandidate(candidate, best, config)) {
                continue;
            }

            best = candidate;
        }
    }

    if (config.clas_atmos_selection_policy ==
        ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED &&
        !best.tokens.empty() &&
        best.time_gap > config.clas_atmos_stale_after_seconds) {
        if (pppDebugEnabled()) {
            std::cerr << "[PPP-ATMOS] rejected stale nearest-grid network="
                      << preferredClasNetworkId(best.tokens)
                      << " dt=" << best.time_gap
                      << " stale_after_s=" << config.clas_atmos_stale_after_seconds << "\n";
        }
        return {};
    }

    if (pppDebugEnabled() && !best.tokens.empty()) {
        std::cerr << "[PPP-ATMOS] selected network=" << preferredClasNetworkId(best.tokens)
                  << " tokens=" << best.tokens.size()
                  << " grid_selected=" << static_cast<int>(best.has_grid)
                  << " dt=" << best.time_gap
                  << " policy=" << clasAtmosSelectionPolicyName(config.clas_atmos_selection_policy)
                  << " stale_after_s=" << config.clas_atmos_stale_after_seconds << "\n";
    }

    return best.tokens;
}

CLASEpochContext prepareClasEpochContext(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair) {
    CLASEpochContext context;
    context.receiver_position = receiver_pos;
    context.receiver_clock_m = receiver_clk;
    context.trop_zenith_m = trop_zenith;
    context.epoch_atmos_tokens =
        selectClasEpochAtmosTokens(ssr, obs.getSatellites(), obs.time, receiver_pos, config);
    context.osr_corrections = computeOSR(
        obs,
        nav,
        ssr,
        context.epoch_atmos_tokens,
        receiver_pos,
        receiver_clk,
        trop_zenith,
        config,
        prev_windup,
        dispersion_compensation,
        sis_continuity,
        phase_bias_repair);
    return context;
}

std::vector<OSRCorrection> computeOSR(
    const ObservationData& obs,
    const NavigationData& nav,
    const SSRProducts& ssr,
    const std::map<std::string, std::string>& epoch_atmos_tokens,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair) {

    std::vector<OSRCorrection> corrections;
    const bool mrtklib_parity = clasMrtklibFloatParity(config);
    const bool clas_network_parity =
        mrtklib_parity || clasGpsL2wIdentityGateEnabled();
    int preferred_network_id = 0;
    ppp_atmosphere::parseAtmosTokenInt(
        epoch_atmos_tokens, "atmos_network_id", preferred_network_id);
    if (clas_network_parity) {
        ppp_atmosphere::ClasGridReference nearest_regional;
        if (ppp_atmosphere::resolveClasNearestRegionalGridReference(
                receiver_pos, nearest_regional) &&
            nearest_regional.network_id > 0) {
            preferred_network_id = nearest_regional.network_id;
        }
    }

    for (const auto& sat : obs.getSatellites()) {
        OSRCorrection osr;
        osr.satellite = sat;
        osr.claslib_qzss_receiver_antenna_slots =
            sat.system == GNSSSystem::QZSS &&
            pppEnvOverrides().clas_qzss_s_prn_fix;

        // --- 1. Find L1/L2 observations ---
        const auto findSignal = [&](const std::vector<SignalType>& candidates)
            -> const Observation* {
            for (auto sig : candidates) {
                const Observation* o = obs.getObservation(sat, sig);
                if (o && o->valid && o->has_carrier_phase && o->has_pseudorange) return o;
            }
            return nullptr;
        };

        std::vector<SignalType> l1_cands, l2_cands, l5_cands;
        switch (sat.system) {
            case GNSSSystem::GPS:
                l1_cands = {SignalType::GPS_L1CA, SignalType::GPS_L1P};
                l2_cands = {SignalType::GPS_L2C, SignalType::GPS_L2P};
                l5_cands = {SignalType::GPS_L5};
                break;
            case GNSSSystem::Galileo:
                l1_cands = {SignalType::GAL_E1};
                l2_cands = {SignalType::GAL_E5A, SignalType::GAL_E5B};
                break;
            case GNSSSystem::QZSS:
                l1_cands = {SignalType::QZS_L1CA};
                l2_cands = {SignalType::QZS_L2C};
                l5_cands = {SignalType::QZS_L5};
                break;
            default:
                continue;
        }

        const Observation* l1_obs = findSignal(l1_cands);
        const Observation* l2_obs = findSignal(l2_cands);
        const Observation* l5_obs = findSignal(l5_cands);
        if (sat.system == GNSSSystem::GPS &&
            (mrtklib_parity || clasGpsL2wIdentityGateEnabled())) {
            const Observation* exact_l2w = findExactGpsL2wObservation(obs, sat);
            // RTKLIB places the highest-priority usable L2 observation in
            // frequency slot 1. A4b selects C2W/L2W when present and C2X/L2X
            // otherwise; both consume the CLAS L2W correction cell. Do not
            // fall through to L2L: that creates a fictitious multi-metre
            // compensation on tokyo/run2 when the configured slot is absent.
            if (exact_l2w != nullptr) {
                l2_obs = exact_l2w;
            } else if (mrtklib_parity && !isGpsL2xObservation(l2_obs)) {
                l2_obs = nullptr;
            }
        }
        if (!l1_obs) {
            if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                          << " reason=no_l1_code_phase\n";
            }
            continue;
        }
        if (mrtklib_parity && l2_obs != nullptr &&
            l1_obs->has_carrier_phase && l2_obs->has_carrier_phase &&
            std::isfinite(l1_obs->carrier_phase) &&
            std::isfinite(l2_obs->carrier_phase)) {
            const double l1_wavelength = signalWavelengthMeters(*l1_obs);
            const double l2_wavelength = signalWavelengthMeters(*l2_obs);
            if (l1_wavelength > 0.0 && l2_wavelength > 0.0) {
                auto& warmup = dispersion_compensation[sat];
                while (!warmup.warmup_times.empty() &&
                       obs.time - warmup.warmup_times.front() > 120.0) {
                    warmup.warmup_times.erase(warmup.warmup_times.begin());
                    warmup.warmup_phase_m.erase(warmup.warmup_phase_m.begin());
                }
                if (warmup.warmup_times.empty() ||
                    warmup.warmup_times.back() != obs.time) {
                    warmup.warmup_times.push_back(obs.time);
                    warmup.warmup_phase_m.push_back({{
                        l1_obs->carrier_phase * l1_wavelength,
                        l2_obs->carrier_phase * l2_wavelength}});
                }
            }
        }

        // --- 2. Satellite position/clock from broadcast + SSR ---
        Vector3d sat_pos, sat_vel;
        double sat_clk = 0.0, sat_drift = 0.0;
        int clas_orbit_iode = -1;
        if (clas_network_parity) {
            const auto entries_it = ssr.orbit_clock_corrections.find(sat);
            if (entries_it != ssr.orbit_clock_corrections.end()) {
                for (auto entry = entries_it->second.rbegin();
                     entry != entries_it->second.rend(); ++entry) {
                    const double age = obs.time - entry->time;
                    if (age < -1e-9) continue;
                    if (age > 60.0) break;
                    if (entry->orbit_valid && entry->iode >= 0) {
                        clas_orbit_iode = entry->iode;
                        break;
                    }
                }
            }
        }
        if (!nav.calculateSatelliteState(
                sat, obs.time, sat_pos, sat_vel, sat_clk, sat_drift,
                clas_orbit_iode)) {
            if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                          << " reason=no_broadcast_state\n";
            }
            continue;
        }

        // Re-evaluate satellite state at signal transmission time. Without
        // this, CLAS per-frequency residuals stay at the 20-40 m level.
        if (l1_obs->pseudorange > 0.0) {
            const double travel_time = l1_obs->pseudorange / constants::SPEED_OF_LIGHT;
            GNSSTime emission_time;
            const Ephemeris* parity_eph = nullptr;
            if (mrtklib_parity) {
                const GNSSTime approximate_transmit_time = obs.time - travel_time;
                parity_eph = nav.getEphemeris(
                    sat, approximate_transmit_time, clas_orbit_iode);
                if (parity_eph == nullptr) continue;
                // RTKLIB satposs(): ephclk() iterates the broadcast clock
                // polynomial twice and deliberately excludes relativity for
                // the transmit-epoch correction.
                const double tc0 = approximate_transmit_time - parity_eph->toc;
                double tc = tc0;
                for (int iteration = 0; iteration < 2; ++iteration) {
                    const double polynomial = parity_eph->af0 +
                        parity_eph->af1 * tc + parity_eph->af2 * tc * tc;
                    tc = tc0 - polynomial;
                }
                const double polynomial = parity_eph->af0 +
                    parity_eph->af1 * tc + parity_eph->af2 * tc * tc;
                emission_time = approximate_transmit_time - polynomial;
            } else if (pppEnvOverrides().clas_tx_time_sign_fix) {
                const GNSSTime approximate_transmit_time = obs.time - travel_time;
                if (!nav.calculateSatelliteState(
                        sat,
                        approximate_transmit_time,
                        sat_pos,
                        sat_vel,
                        sat_clk,
                        sat_drift,
                        clas_orbit_iode)) {
                    if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                        std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                                  << " reason=no_tx_broadcast_state_approx\n";
                    }
                    continue;
                }
                emission_time = approximate_transmit_time - sat_clk;
            } else {
                emission_time = obs.time - travel_time + sat_clk;
            }
            if (!nav.calculateSatelliteState(
                    sat,
                    emission_time,
                    sat_pos,
                    sat_vel,
                    sat_clk,
                    sat_drift,
                    clas_orbit_iode)) {
                if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                    std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                              << " reason=no_tx_broadcast_state\n";
                }
                continue;
            }
            if (mrtklib_parity && parity_eph != nullptr) {
                // ephpos() forms velocity by a 1 ms forward difference;
                // satpos_ssr() then recomputes relativity as -2*r.v/c^2 and
                // uses the same velocity for its RAC basis. A central 1 s
                // derivative changes the clock by centimetres and the orbit
                // correction by millimetres, enough to flip the 2-sigma gate.
                Vector3d forward_pos;
                Vector3d ignored_velocity;
                double ignored_clock = 0.0;
                double ignored_drift = 0.0;
                constexpr double kRtklibVelocityStepSeconds = 1e-3;
                if (!parity_eph->calculateSatelliteState(
                        emission_time + kRtklibVelocityStepSeconds,
                        forward_pos,
                        ignored_velocity,
                        ignored_clock,
                        ignored_drift)) {
                    continue;
                }
                sat_vel = (forward_pos - sat_pos) /
                          kRtklibVelocityStepSeconds;
                const double tc = emission_time - parity_eph->toc;
                // calculateSatelliteState() already supplies the broadcast
                // relativity term from the eccentric anomaly. Retain that
                // value: recomputing -2*r.v/c^2 from a 1 ms numerical
                // derivative adds constellation-dependent centimetre noise.
                sat_drift = parity_eph->af1 + 2.0 * parity_eph->af2 * tc;
            }
            osr.signal_transmit_time = emission_time;
        }

        // Apply SSR orbit/clock corrections
        Vector3d orbit_corr = Vector3d::Zero();
        Vector3d orbit_corr_rac = Vector3d::Zero();
        double clock_corr = 0.0;
        double ura_sigma = 0.0;
        std::map<uint8_t, double> ssr_cbias, ssr_pbias;
        std::map<uint8_t, double> ssr_cbias_rtklib, ssr_pbias_rtklib;
        std::map<std::string, std::string> atmos_tokens;
        GNSSTime atmos_reference_time;
        GNSSTime phase_bias_reference_time;
        GNSSTime clock_reference_time;
        SSRCorrectionStatus ssr_status;
        double base_clock_corr = 0.0;
        bool base_clock_valid = false;
        const auto& env = pppEnvOverrides();
        const auto clock_policy = mrtklib_parity
            ? SSRClockSelectionPolicy::MrtklibLiteralBaseHold
            : (env.clas_base_clock_parity
                   ? SSRClockSelectionPolicy::ClaslibBaseHold
                   : SSRClockSelectionPolicy::MergedInterpolate);
        const bool allow_future_samples =
            !mrtklib_parity && !env.clas_base_clock_parity;
        if (ssr.interpolateCorrection(sat, obs.time, orbit_corr, clock_corr,
                                       &ura_sigma, &ssr_cbias, &ssr_pbias,
                                       &atmos_tokens,
                                       &atmos_reference_time,
                                       &phase_bias_reference_time,
                                       &clock_reference_time,
                                       preferred_network_id,
                                       nullptr,
                                       nullptr,
                                       &ssr_status,
                                       allow_future_samples,
                                       &base_clock_corr,
                                       &base_clock_valid,
                                       clock_policy,
                                       &ssr_cbias_rtklib,
                                       &ssr_pbias_rtklib)) {
            // CLASLIB's satpos_ssr() rejects QZSS observations when the
            // current compact orbit bank omits that satellite. Do not publish
            // a broadcast-only OSR row after the parity selector has marked
            // the held orbit as withdrawn.
            if (env.clas_qzss_s_prn_fix && sat.system == GNSSSystem::QZSS &&
                ssr_status.orbit_withdrawn) {
                continue;
            }
            orbit_corr_rac = orbit_corr;
            // CSV-expanded CLAS corrections carry orbit deltas in RAC, while
            // sampled RTCM SSR products are already stored in ECEF.
            // RAC frame follows RTCM-10403.1 / CLASLIB convention:
            //   Along-track  = normalize(velocity)
            //   Cross-track  = normalize(position × velocity)
            //   Radial       = along × cross  (outward from Earth center)
            // The correction is subtracted: rs -= er*dR + ea*dA + ec*dC
            if (ssr.orbitCorrectionsAreRac() &&
                orbit_corr.squaredNorm() > 0.0 &&
                sat_pos.squaredNorm() > 0.0 &&
                sat_vel.squaredNorm() > 0.0) {
                const Vector3d ea = sat_vel.normalized();  // Along-track
                Vector3d c_unit = sat_pos.cross(sat_vel);
                if (c_unit.squaredNorm() > 0.0) {
                    c_unit.normalize();  // Cross-track (normal to orbital plane)
                } else {
                    c_unit = Vector3d(0, 0, 1);
                }
                const Vector3d er = ea.cross(c_unit);  // Radial (outward)
                // orbit_corr = (dR, dA, dC) in RAC; applied as subtraction
                const Vector3d orbit_ecef = -(er * orbit_corr(0)
                                            + ea * orbit_corr(1)
                                            + c_unit * orbit_corr(2));
                orbit_corr = orbit_ecef;
            }
            if (pppDebugEnabled() && corrections.size() < 20) {
                std::cerr << "[OSR-SSR] " << sat.toString()
                          << " orbit_ecef=" << orbit_corr.transpose()
                          << " clk_m=" << clock_corr
                          << " brdc_clk_s=" << sat_clk
                          << " sat_clk_total_s=" << sat_clk + clock_corr / constants::SPEED_OF_LIGHT
                          << " cbias_n=" << ssr_cbias.size()
                          << " pbias_n=" << ssr_pbias.size() << "\n";
            }
            sat_pos += orbit_corr;
            sat_clk += clock_corr / constants::SPEED_OF_LIGHT;
            osr.has_code_bias = !ssr_cbias.empty();
            osr.has_phase_bias = !ssr_pbias.empty();
            osr.atmos_reference_time = atmos_reference_time;
            osr.phase_bias_reference_time = phase_bias_reference_time;
            osr.code_bias_reference_time = ssr_status.code_bias_reference_time;
            osr.clock_reference_time = clock_reference_time;
            osr.clock_correction_m = clock_corr;
            osr.base_clock_correction_m = base_clock_valid ? base_clock_corr : clock_corr;
            osr.base_clock_valid = base_clock_valid;
        } else {
            if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                          << " reason=no_ssr_correction\n";
            }
            continue;
        }
        if (pppEnvOverrides().clas_atmos_lifecycle &&
            !epoch_atmos_tokens.empty()) {
            atmos_tokens = epoch_atmos_tokens;
            int lifecycle_tow = 0;
            if (ppp_atmosphere::parseAtmosTokenInt(
                    atmos_tokens, "atmos_lifecycle_tow", lifecycle_tow)) {
                osr.atmos_reference_time = GNSSTime(obs.time.week, lifecycle_tow);
            } else {
                osr.atmos_reference_time = obs.time;
            }
        }

        if (clas_network_parity && preferred_network_id > 0 &&
            sat.system != GNSSSystem::QZSS) {
            std::map<uint8_t, int> repicked_discnt;
            GNSSTime repicked_ref;
            if (ssr.heldClasPhaseBiasForServiceNetwork(
                    sat, obs.time, preferred_network_id, &ssr_pbias,
                    &repicked_discnt, &repicked_ref, true,
                    kClasPhaseBiasBankPeriodSeconds)) {
                ssr_pbias_rtklib.clear();
                phase_bias_reference_time = repicked_ref;
                ssr_status.phase_bias_valid = !ssr_pbias.empty();
                ssr_status.phase_bias_reference_time = repicked_ref;
            } else {
                // clas_osr_zdres() requires a phase-bias cell for every
                // exported frequency.  Once the service-network bank is
                // withdrawn, retaining the interpolator's base/old value
                // creates rows absent from CLASLIB.
                ssr_pbias.clear();
                ssr_pbias_rtklib.clear();
                ssr_status.phase_bias_valid = false;
            }
        }

        const auto ssr_timing_policy = config.clas_ssr_timing_policy;
        const bool clock_ref_valid = gnsstimeIsSet(osr.clock_reference_time);
        const bool phase_bias_ref_valid = gnsstimeIsSet(osr.phase_bias_reference_time);
        const bool atmos_ref_valid = gnsstimeIsSet(osr.atmos_reference_time);
        if (usesClasClockBoundPhaseBias(ssr_timing_policy) &&
            clock_ref_valid &&
            phase_bias_ref_valid &&
            osr.phase_bias_reference_time != osr.clock_reference_time) {
            ssr_pbias.clear();
            osr.has_phase_bias = false;
        }
        if (usesClasClockBoundAtmos(ssr_timing_policy) &&
            clock_ref_valid &&
            atmos_ref_valid &&
            osr.atmos_reference_time != osr.clock_reference_time) {
            const bool lifecycle_atmos =
                pppEnvOverrides().clas_atmos_lifecycle &&
                atmos_tokens.find("atmos_lifecycle") != atmos_tokens.end();
            const double atmos_clock_gap =
                std::abs(osr.atmos_reference_time - osr.clock_reference_time);
            if (!lifecycle_atmos || atmos_clock_gap > 30.0) {
                atmos_tokens.clear();
                osr.atmos_reference_time = GNSSTime();
            }
        }

        osr.satellite_position = sat_pos;
        osr.satellite_velocity = sat_vel;
        osr.satellite_clock_bias_s = sat_clk;

        // --- 3. Geometry ---
        const double geo_range = geodist(sat_pos, receiver_pos);
        const Vector3d los = (sat_pos - receiver_pos) / geo_range;
        osr.orbit_projection_m = los.dot(orbit_corr);
        double lat = 0.0, lon = 0.0, h = 0.0;
        ecef2geodetic(receiver_pos, lat, lon, h);
        const Vector3d los_enu = ecef2enu(sat_pos - receiver_pos, lat, lon);
        const double elev = std::atan2(los_enu.z(), std::hypot(los_enu.x(), los_enu.y()));
        const double azim = std::atan2(los_enu.x(), los_enu.y());

        if (elev < kElevationMaskRad) continue;

        osr.elevation = elev;
        osr.azimuth = azim;

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        osr.signals[0] = l1_obs->signal;
        osr.pseudorange_rinex_codes[0] = l1_obs->pseudorange_observation_type;
        osr.carrier_rinex_codes[0] = l1_obs->carrier_phase_observation_type;
        osr.frequencies[0] = signalFrequencyHz(l1_obs->signal, eph);
        osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
        osr.num_frequencies = 1;
        osr.num_output_frequencies = 1;
        if (l2_obs) {
            osr.signals[1] = l2_obs->signal;
            osr.pseudorange_rinex_codes[1] = l2_obs->pseudorange_observation_type;
            osr.carrier_rinex_codes[1] = l2_obs->carrier_phase_observation_type;
            // CLASLIB exposes GPS frequency slot 1 as RTKLIB CODE_L2W even
            // when the receiver value occupying that slot was tracked as
            // C2X/L2X. Keep the raw Observation unchanged for measurement
            // provenance, but key the CLAS correction/bias slot as L2W.
            if (sat.system == GNSSSystem::GPS &&
                clasGpsL2wIdentityGateEnabled()) {
                osr.pseudorange_rinex_codes[1] = "C2W";
                osr.carrier_rinex_codes[1] = "L2W";
            }
            osr.frequencies[1] = signalFrequencyHz(l2_obs->signal, eph);
            osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
            osr.num_frequencies = 2;
            osr.num_output_frequencies = 2;
        }
        const bool claslib_output_l5 =
            sat.system == GNSSSystem::GPS
                ? clas_network_parity
                : (sat.system == GNSSSystem::QZSS &&
                   pppEnvOverrides().clas_qzss_s_prn_fix);
        if (claslib_output_l5 && l2_obs && l5_obs) {
            osr.signals[2] = l5_obs->signal;
            osr.pseudorange_rinex_codes[2] =
                l5_obs->pseudorange_observation_type;
            osr.carrier_rinex_codes[2] =
                l5_obs->carrier_phase_observation_type;
            osr.frequencies[2] = signalFrequencyHz(l5_obs->signal, eph);
            osr.wavelengths[2] =
                constants::SPEED_OF_LIGHT / osr.frequencies[2];
            // CLASLIB's nf=3 export contains the L5 signal identity and its
            // component fields, but PRC[2]/CPC[2] remain zero.  Keep it as an
            // output-only slot so it cannot collide with the estimator's L2
            // ambiguity key or create an extra measurement row.
            osr.num_output_frequencies = 3;
        }

        // --- 4. Troposphere ---
        // Use Saastamoinen as trop fallback when CLAS grid trop is unavailable.
        osr.trop_correction_m = models::tropDelaySaastamoinen(receiver_pos, elev);

        // CLAS troposphere grid correction (if available)
        if (!atmos_tokens.empty()) {
            if (pppDebugEnabled() && sat.system == GNSSSystem::GPS && sat.prn == 6) {
                const auto valueOr = [](const auto& tokens, const char* key) {
                    const auto it = tokens.find(key);
                    return it == tokens.end() ? std::string("-") : it->second;
                };
                std::cerr << "[CLAS-ATMOS-TOKENS] tow=" << obs.time.tow
                          << " sat_net=" << valueOr(atmos_tokens, "atmos_network_id")
                          << " sat_type=" << valueOr(atmos_tokens, "atmos_trop_type")
                          << " sat_t00=" << valueOr(atmos_tokens, "atmos_trop_t00_m")
                          << " sat_off=" << valueOr(atmos_tokens, "atmos_trop_offset_m")
                          << " sat_res=" << valueOr(atmos_tokens, "atmos_trop_residuals_m")
                          << " epoch_net=" << valueOr(epoch_atmos_tokens, "atmos_network_id")
                          << " epoch_type=" << valueOr(epoch_atmos_tokens, "atmos_trop_type")
                          << " epoch_t00=" << valueOr(epoch_atmos_tokens, "atmos_trop_t00_m")
                          << " epoch_off=" << valueOr(epoch_atmos_tokens, "atmos_trop_offset_m")
                          << " epoch_res=" << valueOr(epoch_atmos_tokens, "atmos_trop_residuals_m")
                          << "\n";
            }
            ppp_atmosphere::ClasGridReference grid_reference;
            if (ppp_atmosphere::resolveClasGridReference(
                    atmos_tokens,
                    receiver_pos,
                    grid_reference)) {
                osr.atmos_network_id = grid_reference.network_id;
                osr.atmos_grid_no = grid_reference.grid_no;
                osr.atmos_nearest_grid_distance_m =
                    grid_reference.nearest_grid_distance_m;
                if (grid_reference.interpolation_grid_count > 0) {
                    osr.atmos_interpolation_grid_count =
                        grid_reference.interpolation_grid_count;
                    for (int grid = 0;
                         grid < grid_reference.interpolation_grid_count &&
                         grid < static_cast<int>(osr.atmos_interpolation_grid_no.size());
                         ++grid) {
                        osr.atmos_interpolation_grid_no[grid] =
                            static_cast<int>(
                                grid_reference.interpolation_grid_indices[grid]) + 1;
                        osr.atmos_interpolation_weights[grid] =
                            grid_reference.interpolation_weights[grid];
                    }
                } else if (grid_reference.has_bilinear) {
                    osr.atmos_interpolation_grid_count = 4;
                    for (int grid = 0; grid < 4; ++grid) {
                        osr.atmos_interpolation_grid_no[grid] =
                            static_cast<int>(
                                grid_reference.bilinear_grid_indices[grid]) + 1;
                        osr.atmos_interpolation_weights[grid] =
                            grid_reference.bilinear_weights[grid];
                    }
                } else if (grid_reference.grid_no > 0) {
                    osr.atmos_interpolation_grid_count = 1;
                    osr.atmos_interpolation_grid_no[0] = grid_reference.grid_no;
                    osr.atmos_interpolation_weights[0] = 1.0;
                }
            }
            setAtmosLifecycleProvenance(osr, atmos_tokens, sat);
            std::map<std::string, std::string> trop_atmos_tokens = atmos_tokens;
            if (mrtklib_parity) {
                int minimum_trop_grid_count = 0;
                ppp_atmosphere::ClasGridReference nearest_regional;
                if (ppp_atmosphere::resolveClasNearestRegionalGridReference(
                        receiver_pos, nearest_regional) &&
                    nearest_regional.network_id > 0) {
                    osr.atmos_network_id = nearest_regional.network_id;
                    osr.atmos_grid_no = nearest_regional.grid_no;
                    osr.atmos_nearest_grid_distance_m =
                        nearest_regional.nearest_grid_distance_m;
                    for (int grid = 0;
                         grid < nearest_regional.interpolation_grid_count;
                         ++grid) {
                        minimum_trop_grid_count = std::max(
                            minimum_trop_grid_count,
                            static_cast<int>(
                                nearest_regional.interpolation_grid_indices[grid]) + 1);
                    }
                    if (nearest_regional.has_bilinear) {
                        for (int grid = 0; grid < 4; ++grid) {
                            minimum_trop_grid_count = std::max(
                                minimum_trop_grid_count,
                                static_cast<int>(
                                    nearest_regional.bilinear_grid_indices[grid]) + 1);
                        }
                    } else if (minimum_trop_grid_count == 0 &&
                               nearest_regional.grid_no > 0) {
                        minimum_trop_grid_count = nearest_regional.grid_no;
                    }
                    if (pppDebugEnabled() && sat.system == GNSSSystem::GPS &&
                        sat.prn == 6) {
                        std::cerr << "[CLAS-TROP-GRID] tow=" << obs.time.tow
                                  << " net=" << nearest_regional.network_id
                                  << " bilinear=" << nearest_regional.has_bilinear
                                  << " count="
                                  << nearest_regional.interpolation_grid_count;
                        for (int grid = 0; grid < 4; ++grid) {
                            std::cerr << " i" << grid << '='
                                      << nearest_regional.bilinear_grid_indices[grid]
                                      << " w" << grid << '='
                                      << nearest_regional.bilinear_weights[grid];
                            if (grid < nearest_regional.interpolation_grid_count) {
                                std::cerr << " mi" << grid << '='
                                          << nearest_regional.interpolation_grid_indices[grid]
                                          << " mw" << grid << '='
                                          << nearest_regional.interpolation_weights[grid];
                            }
                        }
                        std::cerr << "\n";
                    }
                }
                std::map<std::string, std::string> held_trop_tokens;
                bool have_held_trop = ssr.heldClasTropTokens(
                    obs.time, kClasQzssHeldStecAgeSeconds,
                    osr.atmos_network_id, minimum_trop_grid_count,
                    held_trop_tokens, nullptr);
                if (have_held_trop) {
                    // Compact subtype banks rotate independently.  Apply the
                    // newest payload for the receiver-selected network, as
                    // clas_bank_get_close()+clas_trop_grid_data do.
                    trop_atmos_tokens = std::move(held_trop_tokens);
                    if (pppDebugEnabled() && sat.system == GNSSSystem::GPS &&
                        sat.prn == 6) {
                        const auto get = [&](const char* key) {
                            const auto it = trop_atmos_tokens.find(key);
                            return it == trop_atmos_tokens.end()
                                ? std::string("-") : it->second;
                        };
                        std::cerr << "[CLAS-TROP-BANK] tow=" << obs.time.tow
                                  << " net=" << get("atmos_network_id")
                                  << " type=" << get("atmos_trop_type")
                                  << " t00=" << get("atmos_trop_t00_m")
                                  << " off=" << get("atmos_trop_offset_m")
                                  << " res=" << get("atmos_trop_residuals_m")
                                  << "\n";
                    }
                } else if (!epoch_atmos_tokens.empty() &&
                           ppp_atmosphere::hasParityTropGridTokens(
                               epoch_atmos_tokens)) {
                    // Legacy expanded CSV files do not preserve subtype-12
                    // bank ownership.  A network-agnostic lookup can return a
                    // payload from a different service area (tokyo/run2 chose
                    // network 11 for a network-7 rover).  Retain the selected
                    // epoch's internally consistent network/grid tokens when
                    // no owned compact bank is available.
                    trop_atmos_tokens = epoch_atmos_tokens;
                }
            } else if (pppEnvOverrides().clas_trop_grid_parity &&
                !epoch_atmos_tokens.empty() &&
                ppp_atmosphere::hasParityTropGridTokens(epoch_atmos_tokens)) {
                // CLASLIB trop_grid_data uses rover grid selection from get_grid_index
                // (grid.c:300-368), not per-satellite SSR atmos rows.
                trop_atmos_tokens = epoch_atmos_tokens;
            }
            const double clas_trop = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                trop_atmos_tokens,
                receiver_pos,
                obs.time,
                elev,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy,
                mrtklib_parity);
            if (std::isfinite(clas_trop) && std::abs(clas_trop) > 0.0) {
                // Sanity check: CLAS grid trop should be within 30% of Saastamoinen.
                // Distant networks produce unrealistic trop values.
                const double saastamoinen = osr.trop_correction_m;
                if (saastamoinen > 0.1 &&
                    std::abs(clas_trop - saastamoinen) / saastamoinen < 0.3) {
                    osr.trop_correction_m = clas_trop;
                }
            }
        }

        // --- 5. Relativity ---
        osr.relativity_correction_m = relativisticCorrection(sat_pos, sat_vel, receiver_pos);

        // --- 6. Ionosphere (STEC) ---
        if (!atmos_tokens.empty()) {
            std::map<std::string, std::string> stec_atmos_tokens = atmos_tokens;
            const std::map<std::string, std::string>* stec_tokens_view =
                &stec_atmos_tokens;
            GNSSTime stec_atmos_reference_time = osr.atmos_reference_time;
            int broadcast_stec_quality = 0;
            bool have_broadcast_stec_quality = false;
            if (env.clas_qzss_s_prn_fix && sat.system == GNSSSystem::QZSS) {
                have_broadcast_stec_quality =
                    parseQzssBroadcastStecQuality(atmos_tokens, sat, broadcast_stec_quality);
            }
            // CLASLIB stores STEC independently per service-network grid
            // (stec_grid_data) and evaluates every constellation from the
            // bank selected for the receiver position. The compact stream's
            // latest row may belong to a different network; using its global
            // coefficient tokens caused 0.3--0.6 m ionosphere jumps whenever
            // network messages rotated at 5 s boundaries. Repick the held
            // bank for the selected network for GPS/GAL/QZSS alike.
            int service_network_id =
                clas_network_parity ? osr.atmos_network_id : 0;
            if (clas_network_parity) {
                ppp_atmosphere::ClasGridReference nearest_regional;
                if (ppp_atmosphere::resolveClasNearestRegionalGridReference(
                        receiver_pos, nearest_regional) &&
                    nearest_regional.network_id > 0) {
                    service_network_id = nearest_regional.network_id;
                }
            }
            if (service_network_id > 0) {
                std::map<std::string, std::string> service_atmos_tokens;
                GNSSTime service_atmos_reference_time;
                const auto* clas_bank = clas_network_parity
                    ? ssr.heldClasAtmosBankTokens(
                          obs.time, kClasQzssHeldStecAgeSeconds,
                          service_network_id, &service_atmos_reference_time)
                    : nullptr;
                if (clas_bank != nullptr) {
                    stec_tokens_view = clas_bank;
                    stec_atmos_reference_time = service_atmos_reference_time;
                    osr.atmos_reference_time = service_atmos_reference_time;
                } else if (ssr.heldAtmosTokensForNetwork(
                        service_network_id,
                        obs.time,
                        kClasQzssHeldStecAgeSeconds,
                        service_atmos_tokens,
                        &service_atmos_reference_time)) {
                    stec_atmos_tokens = std::move(service_atmos_tokens);
                    stec_atmos_reference_time = service_atmos_reference_time;
                }
            }
            const double stec_tecu = [&]() {
                double value = ppp_atmosphere::atmosphericStecTecu(
                    *stec_tokens_view,
                    sat,
                    receiver_pos,
                    config.clas_expanded_value_construction_policy,
                    config.clas_subtype12_value_construction_policy,
                    config.clas_expanded_residual_sampling_policy,
                    clas_network_parity);
                if (!(env.clas_qzss_s_prn_fix && sat.system == GNSSSystem::QZSS)) {
                    return value;
                }
                auto& held_stec = qzssHeldStecBySatellite();
                if (have_broadcast_stec_quality && broadcast_stec_quality == 0) {
                    const auto held_it = held_stec.find(sat);
                    if (held_it != held_stec.end() &&
                        obs.time - held_it->second.first <=
                            kClasQzssHeldStecAgeSeconds + 1e-9) {
                        // CLASLIB get_cssr_latest_iono when ST9 dstec is invalid
                        // (cssr.c:1045-1046, cssr.c:947-961).
                        return held_it->second.second;
                    }
                }
                if (have_broadcast_stec_quality && broadcast_stec_quality != 0) {
                    held_stec[sat] = {obs.time, value};
                    return value;
                }
                const std::string stec_quality_key = "atmos_stec_quality:" + sat.toString();
                int stec_quality = 0;
                bool have_stec_quality =
                    ppp_atmosphere::parseAtmosTokenInt(
                        *stec_tokens_view, stec_quality_key, stec_quality);
                if (have_stec_quality && stec_quality != 0) {
                    held_stec[sat] = {obs.time, value};
                    return value;
                }
                const auto held_it = held_stec.find(sat);
                if (held_it != held_stec.end() &&
                    obs.time - held_it->second.first <= kClasQzssHeldStecAgeSeconds + 1e-9) {
                    return held_it->second.second;
                }
                return value;
            }();
            if (std::isfinite(stec_tecu) && std::abs(stec_tecu) > 0.001) {
                osr.stec_tecu = stec_tecu;
                // CLASLIB stores STEC internally with 1/(F1*F2) scaling, then
                // multiplies it by F2/F1 while forming PRC/CPC.  The net L1
                // delay is the conventional 1/F1^2 value represented here.
                osr.iono_l1_m = ppp_atmosphere::ionosphereDelayMetersFromTecu(
                    l1_obs->signal, eph, stec_tecu);
                osr.has_iono = true;
            }
        }
        if (!osr.has_iono) {
            continue;  // CLASLIB rejects satellites without STEC
        }

        if ((mrtklib_parity || env.clas_qzss_s_prn_fix) &&
            sat.system == GNSSSystem::QZSS) {
            int service_network_id = osr.atmos_network_id;
            ppp_atmosphere::ClasGridReference nearest_regional;
            if (ppp_atmosphere::resolveClasNearestRegionalGridReference(
                    receiver_pos, nearest_regional) &&
                nearest_regional.network_id > 0) {
                service_network_id = nearest_regional.network_id;
            }
            std::map<uint8_t, double> repicked_pbias;
            std::map<uint8_t, int> repicked_discnt;
            GNSSTime repicked_ref;
            const bool qzss_literal_bank =
                mrtklib_parity || env.clas_qzss_s_prn_fix;
            if (service_network_id > 0 &&
                ssr.heldQzssPhaseBiasForServiceNetwork(
                    sat, obs.time, service_network_id,
                    &repicked_pbias, &repicked_discnt, &repicked_ref,
                    qzss_literal_bank,
                    qzss_literal_bank ? kClasPhaseBiasBankPeriodSeconds : -1.0)) {
                ssr_pbias = std::move(repicked_pbias);
                phase_bias_reference_time = repicked_ref;
                osr.has_phase_bias = !ssr_pbias.empty();
                osr.phase_bias_reference_time = repicked_ref;
            } else if (qzss_literal_bank) {
                ssr_pbias.clear();
                ssr_pbias_rtklib.clear();
                osr.has_phase_bias = false;
            }
        }

        // --- 7. Code/Phase bias ---
        const Observation* freq_observations[OSR_MAX_FREQ] = {
            l1_obs, l2_obs, l5_obs};
        for (int f = 0; f < osr.num_output_frequencies; ++f) {
            const Observation* freq_obs = freq_observations[f];
            const bool claslib_gps_l2w_slot =
                f == 1 && sat.system == GNSSSystem::GPS &&
                clasGpsL2wIdentityGateEnabled();
            const bool exact_bias_identity =
                claslib_gps_l2w_slot ||
                gpsL2ExactBiasIdentityEnabled(freq_obs) ||
                (mrtklib_parity && freq_obs != nullptr &&
                 algorithms::ppp_bias_identity::isGpsL2wObservation(
                     freq_obs->satellite.system,
                     freq_obs->signal,
                     freq_obs->pseudorange_observation_type,
                     freq_obs->carrier_phase_observation_type));
            auto bias = materializeClasOsrBiases(
                sat.system,
                osr.signals[f],
                claslib_gps_l2w_slot ? "C2W" :
                    (freq_obs != nullptr ? freq_obs->pseudorange_observation_type : ""),
                claslib_gps_l2w_slot ? "L2W" :
                    (freq_obs != nullptr ? freq_obs->carrier_phase_observation_type : ""),
                ssr_cbias,
                ssr_pbias,
                exact_bias_identity,
                pppEnvOverrides().clas_code_row_bias_identity && !exact_bias_identity);
            // CLASLIB matches the exact RTKLIB CODE_* selected for the
            // observation.  RTCM SSR ids collapse several CSSR cells (notably
            // GPS L2P and L2W), so the parity path must honor the exact map,
            // including a NaN marker for an explicitly invalid cell.
            if (mrtklib_parity && freq_obs != nullptr) {
                const int code = algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                    freq_obs->pseudorange_observation_type);
                const auto code_it = ssr_cbias_rtklib.find(static_cast<uint8_t>(code));
                if (code > 0 && !ssr_cbias_rtklib.empty()) {
                    const bool valid = code_it != ssr_cbias_rtklib.end() &&
                                       std::isfinite(code_it->second);
                    bias.code_bias_m = valid ? code_it->second : 0.0;
                    bias.code_present = valid;
                    bias.code_source_signal_id = static_cast<uint8_t>(code);
                    bias.code_fallback = false;
                }
                const int phase_code = algorithms::ppp_bias_identity::rtklibCodeForObservationType(
                    freq_obs->carrier_phase_observation_type);
                const auto phase_it = ssr_pbias_rtklib.find(static_cast<uint8_t>(phase_code));
                if (phase_code > 0 && !ssr_pbias_rtklib.empty()) {
                    const bool valid = phase_it != ssr_pbias_rtklib.end() &&
                                       std::isfinite(phase_it->second);
                    bias.phase_bias_m = valid ? phase_it->second : 0.0;
                    bias.phase_present = valid;
                    bias.phase_source_signal_id = static_cast<uint8_t>(phase_code);
                    bias.phase_fallback = false;
                }
            }
            osr.code_bias_signal_ids[f] = bias.code_signal_id;
            osr.phase_bias_signal_ids[f] = bias.phase_signal_id;
            osr.bias_exact_identity[f] = bias.exact_identity;
            osr.code_bias_m[f] = bias.code_bias_m;
            osr.phase_bias_m[f] = bias.phase_bias_m;
            osr.code_bias_source_signal_ids[f] = bias.code_source_signal_id;
            osr.phase_bias_source_signal_ids[f] = bias.phase_source_signal_id;
            osr.code_bias_present[f] = bias.code_present;
            osr.phase_bias_present[f] = bias.phase_present;
            osr.code_bias_fallback[f] = bias.code_fallback;
            osr.phase_bias_fallback[f] = bias.phase_fallback;
        }

        if (osr.num_frequencies >= 2) {
            updateDispersionCompensation(
                osr, dispersion_compensation[sat], l1_obs, l2_obs, obs.time,
                mrtklib_parity);
        }

        // --- 8. Phase wind-up ---
        double& wu = prev_windup[sat];
        wu = clas_network_parity
            ? phaseWindupMrtklib(sat_pos, sat_vel, receiver_pos, wu)
            : phaseWindup(sat_pos, receiver_pos, wu);
        osr.windup_cycles = wu;
        for (int f = 0; f < osr.num_output_frequencies; ++f) {
            osr.windup_m[f] = wu * osr.wavelengths[f];
        }

        auto& phase_bias_repair_info = phase_bias_repair[sat];
        auto& sis_continuity_info = sis_continuity[sat];
        const auto phase_continuity_policy =
            config.clas_phase_continuity_policy;
        const bool clock_time_valid = gnsstimeIsSet(osr.clock_reference_time);
        const double sis_clock_m = osr.base_clock_valid ?
            osr.base_clock_correction_m : osr.clock_correction_m;
        const double current_sis_m = -sis_clock_m + osr.orbit_projection_m;
        updateSisContinuity(sis_continuity_info, osr, clock_time_valid);
        const bool sis_boundary_gate_enabled =
            pppEnvOverrides().clas_sis_boundary || clas_network_parity;
        if (sis_boundary_gate_enabled) {
            if (isSsrOrbitBoundaryOffset25Tow(obs.time.tow) &&
                clas_orbit_iode >= 0) {
                sis_continuity_info.boundary_prev_iode = clas_orbit_iode;
                sis_continuity_info.has_boundary_prev_iode = true;
            }
            captureClasSisBoundary(
                sis_continuity_info, obs.time, current_sis_m,
                mrtklib_parity ? kMrtklibOrbitBoundaryToleranceSeconds
                               : kSsrOrbitBoundaryToleranceSeconds);

            // The native CLAS pipeline can first publish a valid OSR several
            // seconds after the 30 s boundary. CLASLIB has already populated
            // satcorr[] while decoding those warm-up epochs, so reconstruct
            // its missed (boundary-5 s, boundary) SIS pair from SSR history.
            // The 30 s phase-bias lag is the literal adjust_cpc() trigger.
            const GNSSTime seed_effective_phase_bias_reference_time =
                selectClasPhaseBiasReferenceTime(
                    config.clas_phase_bias_reference_time_policy,
                    osr.phase_bias_reference_time,
                    osr.clock_reference_time,
                    obs.time);
            const GNSSTime seed_boundary_time =
                seed_effective_phase_bias_reference_time + kPhaseBiasLagSeconds;
            const double since_boundary = obs.time - seed_boundary_time;
            if (!sis_continuity_info.has_boundary_delta &&
                clock_time_valid &&
                gnsstimeIsSet(seed_effective_phase_bias_reference_time) &&
                since_boundary >= 0.0 &&
                since_boundary < kSsrOrbitBoundaryApplyWindowSeconds &&
                sat_vel.squaredNorm() > 0.0) {
                const GNSSTime previous_sample_time =
                    seed_boundary_time - kSsrOrbitBoundaryPrevToBoundarySeconds;
                const auto sample_sis = [&](const GNSSTime& sample_time,
                                            double& sampled_sis_m) {
                    Vector3d sampled_orbit_rac = Vector3d::Zero();
                    double sampled_clock_m = 0.0;
                    double sampled_ura = 0.0;
                    std::map<uint8_t, double> sampled_code_biases;
                    std::map<uint8_t, double> sampled_phase_biases;
                    std::map<std::string, std::string> sampled_atmos;
                    GNSSTime sampled_atmos_time;
                    GNSSTime sampled_phase_bias_time;
                    GNSSTime sampled_clock_time;
                    SSRCorrectionStatus sampled_status;
                    double sampled_base_clock_m = 0.0;
                    bool sampled_base_clock_valid = false;
                    if (!ssr.interpolateCorrection(
                            sat, sample_time,
                            sampled_orbit_rac, sampled_clock_m,
                            &sampled_ura, &sampled_code_biases,
                            &sampled_phase_biases, &sampled_atmos,
                            &sampled_atmos_time, &sampled_phase_bias_time,
                            &sampled_clock_time, preferred_network_id,
                            nullptr, nullptr, &sampled_status,
                            false, &sampled_base_clock_m,
                            &sampled_base_clock_valid, clock_policy)) {
                        return false;
                    }
                    Vector3d sampled_orbit_ecef = sampled_orbit_rac;
                    if (ssr.orbitCorrectionsAreRac()) {
                        const Vector3d broadcast_position = sat_pos - orbit_corr;
                        const Vector3d along = sat_vel.normalized();
                        Vector3d cross = broadcast_position.cross(sat_vel);
                        if (cross.squaredNorm() > 0.0) {
                            cross.normalize();
                            const Vector3d radial = along.cross(cross);
                            sampled_orbit_ecef =
                                -(radial * sampled_orbit_rac.x() +
                                  along * sampled_orbit_rac.y() +
                                  cross * sampled_orbit_rac.z());
                        }
                    }
                    const Vector3d sampled_position =
                        sat_pos - orbit_corr + sampled_orbit_ecef;
                    const Vector3d sampled_los =
                        (sampled_position - receiver_pos).normalized();
                    const double sampled_orbit_projection_m =
                        sampled_los.dot(sampled_orbit_ecef);
                    const double sampled_sis_clock_m =
                        sampled_base_clock_valid
                            ? sampled_base_clock_m
                            : sampled_clock_m;
                    sampled_sis_m =
                        -sampled_sis_clock_m + sampled_orbit_projection_m;
                    return true;
                };
                double previous_sis_m = 0.0;
                double boundary_sis_m = 0.0;
                if (sample_sis(previous_sample_time, previous_sis_m) &&
                    sample_sis(seed_boundary_time, boundary_sis_m)) {
                    sis_continuity_info.boundary_prev_time = previous_sample_time;
                    sis_continuity_info.boundary_prev_sis_m = previous_sis_m;
                    sis_continuity_info.has_boundary_prev_sis = true;
                    sis_continuity_info.boundary_time = seed_boundary_time;
                    sis_continuity_info.boundary_delta_m =
                        boundary_sis_m - previous_sis_m;
                    sis_continuity_info.has_boundary_delta = true;
                    if (pppDebugEnabled()) {
                        std::cerr << "[OSR-SIS-SEED] " << sat.toString()
                                  << " boundary_tow=" << seed_boundary_time.tow
                                  << " delta_m="
                                  << sis_continuity_info.boundary_delta_m
                                  << '\n';
                    }
                }
            }
        }
        const GNSSTime effective_phase_bias_reference_time =
            selectClasPhaseBiasReferenceTime(
                config.clas_phase_bias_reference_time_policy,
                osr.phase_bias_reference_time,
                osr.clock_reference_time,
                obs.time);
        const auto pbias_status = updatePhaseBiasRepairState(
            phase_bias_repair_info, effective_phase_bias_reference_time,
            phase_continuity_policy);
        const bool phase_bias_epoch_changed = pbias_status.epoch_changed;
        const double phase_bias_dt = pbias_status.dt;

        // --- 9. Aggregate PRC/CPC (CLASLIB L282-285) ---
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double fi = osr.frequencies[f] > 0.0 ? osr.wavelengths[f] / osr.wavelengths[0] : 1.0;
            const double iono_scaled = fi * fi * osr.iono_l1_m;
            const auto phase_bias_value_policy =
                config.clas_phase_bias_value_policy;
            const double phase_bias_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) &&
                        usesClasRawPhaseBiasValues(phase_bias_value_policy) ?
                    osr.phase_bias_m[f] :
                    0.0;
            const double phase_compensation_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) &&
                        usesClasPhaseCompensationValues(phase_bias_value_policy) ?
                    osr.phase_compensation_m[f] :
                    0.0;

            osr.PRC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] + iono_scaled + osr.code_bias_m[f];

            osr.CPC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] - iono_scaled
                       + phase_bias_term + osr.windup_m[f]
                       + phase_compensation_term;

            if (usesClasSisContinuity(phase_continuity_policy)) {
                const auto sis_decision = computeClasSisApplyDecision(
                    sis_continuity_info,
                    obs.time,
                    osr.clock_reference_time,
                    effective_phase_bias_reference_time,
                    clock_time_valid,
                    sis_boundary_gate_enabled,
                    mrtklib_parity ? kMrtklibOrbitBoundaryToleranceSeconds
                                   : kSsrOrbitBoundaryToleranceSeconds);
                if (sis_decision.applied) {
                    osr.CPC[f] -= sis_decision.delta_m;
                    osr.PRC[f] -= sis_decision.delta_m;
                    osr.network_compensation_m = sis_decision.delta_m;
                    if (f == 0 &&
                        sis_continuity_info.has_boundary_prev_iode &&
                        clas_orbit_iode >= 0 &&
                        clas_orbit_iode !=
                            sis_continuity_info.boundary_prev_iode &&
                        ssr.orbitCorrectionsAreRac()) {
                        double iode_compensation_m = 0.0;
                        if (computeClasIodeGeometryCompensation(
                                nav,
                                sat,
                                obs.time,
                                l1_obs->pseudorange,
                                receiver_pos,
                                sis_continuity_info.boundary_prev_iode,
                                clas_orbit_iode,
                                orbit_corr_rac,
                                clock_corr,
                                sat_pos,
                                sat_clk,
                                iode_compensation_m)) {
                            osr.iode_geometry_compensation_m =
                                iode_compensation_m;
                        }
                    }
                    osr.CPC[f] += osr.iode_geometry_compensation_m;
                    osr.PRC[f] += osr.iode_geometry_compensation_m;
                    if (pppDebugEnabled() && f == 0) {
                        std::cerr << (sis_boundary_gate_enabled ?
                                          "[OSR-SIS-BOUNDARY] " : "[OSR-SIS] ")
                                  << sat.toString()
                                  << " sis_delta_m=" << sis_decision.delta_m
                                  << " ref_policy="
                                  << clasPhaseBiasReferenceTimePolicyName(
                                         config.clas_phase_bias_reference_time_policy)
                                  << "\n";
                    }
                }
            }

            const double continuity_term =
                osr.orbit_projection_m - osr.clock_correction_m + osr.CPC[f];
            if (phase_bias_epoch_changed &&
                std::abs(phase_bias_dt) < kPhaseBiasRepairTimeoutSeconds &&
                phase_bias_repair_info.has_last[static_cast<size_t>(f)] &&
                osr.wavelengths[f] > 0.0 &&
                usesClasPhaseBiasRepair(phase_continuity_policy)) {
                const double dcpc =
                    continuity_term -
                    phase_bias_repair_info.last_continuity_m[static_cast<size_t>(f)];
                const double cycles = dcpc / osr.wavelengths[f];
                if (cycles >= kPhaseBiasJumpLowerCycles && cycles < kPhaseBiasJumpUpperCycles) {
                    phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)] -= kPhaseBiasJumpCorrectionCycles;
                    phase_bias_repair_info.pending_state_shift_cycles[static_cast<size_t>(f)] -= kPhaseBiasJumpCorrectionCycles;
                } else if (cycles <= -kPhaseBiasJumpLowerCycles && cycles > -kPhaseBiasJumpUpperCycles) {
                    phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)] += kPhaseBiasJumpCorrectionCycles;
                    phase_bias_repair_info.pending_state_shift_cycles[static_cast<size_t>(f)] += kPhaseBiasJumpCorrectionCycles;
                }
            }

            if (usesClasPhaseBiasRepair(phase_continuity_policy)) {
                osr.CPC[f] -=
                    phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)] *
                    osr.wavelengths[f];
                phase_bias_repair_info.last_continuity_m[static_cast<size_t>(f)] =
                    continuity_term;
                phase_bias_repair_info.has_last[static_cast<size_t>(f)] = true;
            }

            // Absorb phase bias changes across SSR epochs into the ambiguity
            // state.  When phase_bias_m changes between SSR updates, CPC jumps
            // by the delta.  Without compensation, the filter treats this as
            // an ambiguity change, resetting convergence.
        }

        osr.valid = true;
        if (pppDebugEnabled()) {
            std::cerr << "[OSR] " << sat.toString()
                      << " week=" << obs.time.week
                      << " tow=" << std::setprecision(15) << obs.time.tow
                      << " trop=" << osr.trop_correction_m
                      << " rel=" << osr.relativity_correction_m
                      << " iono_l1=" << osr.iono_l1_m
                      << " cbias0=" << osr.code_bias_m[0]
                      << " pbias0=" << osr.phase_bias_m[0]
                      << " windup=" << osr.windup_cycles
                      << " pbias_ref_tow=" << osr.phase_bias_reference_time.tow
                      << " eff_pbias_ref_tow=" << effective_phase_bias_reference_time.tow
                      << " orb_los=" << osr.orbit_projection_m
                      << " clk_corr=" << osr.clock_correction_m
                      << " base_clk=" << osr.base_clock_correction_m
                      << " base_clk_valid=" << osr.base_clock_valid
                      << " PRC0=" << osr.PRC[0]
                      << " CPC0=" << osr.CPC[0]
                      << "\n";
            std::cerr << std::setprecision(15)
                      << "[CLAS-OSR-COMP] sat=" << sat.toString()
                      << " week=" << obs.time.week
                      << " tow=" << obs.time.tow
                      << " sig=" << static_cast<int>(osr.signals[0]) << ';'
                      << static_cast<int>(osr.signals[1]) << ';'
                      << static_cast<int>(osr.signals[2])
                      << " cbid=" << static_cast<int>(osr.code_bias_signal_ids[0]) << ';'
                      << static_cast<int>(osr.code_bias_signal_ids[1]) << ';'
                      << static_cast<int>(osr.code_bias_signal_ids[2])
                      << " pbid=" << static_cast<int>(osr.phase_bias_signal_ids[0]) << ';'
                      << static_cast<int>(osr.phase_bias_signal_ids[1]) << ';'
                      << static_cast<int>(osr.phase_bias_signal_ids[2])
                      << " pb=" << osr.phase_bias_m[0] << ';'
                      << osr.phase_bias_m[1] << ';' << osr.phase_bias_m[2]
                      << " cb=" << osr.code_bias_m[0] << ';'
                      << osr.code_bias_m[1] << ';' << osr.code_bias_m[2]
                      << " trop=" << osr.trop_correction_m
                      << " iono=" << osr.iono_l1_m
                      << " rel=" << osr.relativity_correction_m
                      << " wind=" << osr.windup_m[0] << ';'
                      << osr.windup_m[1] << ';' << osr.windup_m[2]
                      << " comp=" << osr.phase_compensation_m[0] << ';'
                      << osr.phase_compensation_m[1] << ';'
                      << osr.phase_compensation_m[2]
                      << " sis=" << osr.network_compensation_m
                      << " cpc=" << osr.CPC[0] << ';' << osr.CPC[1] << ';'
                      << osr.CPC[2]
                      << " prc=" << osr.PRC[0] << ';' << osr.PRC[1] << ';'
                      << osr.PRC[2]
                      << " orb=" << osr.orbit_projection_m
                      << " clk=" << osr.clock_correction_m << '\n';
        }
        corrections.push_back(osr);
    }

    return corrections;
}

}  // namespace libgnss
