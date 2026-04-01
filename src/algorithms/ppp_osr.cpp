#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

// Local frequency helper (mirrors ppp.cpp's signalFrequencyHz)
static double signalFrequencyHz(libgnss::SignalType signal, const libgnss::Ephemeris* = nullptr) {
    using namespace libgnss;
    switch (signal) {
        case SignalType::GPS_L1CA: case SignalType::GPS_L1P:
        case SignalType::QZS_L1CA: case SignalType::GAL_E1:
            return constants::GPS_L1_FREQ;
        case SignalType::GPS_L2C: case SignalType::GPS_L2P:
        case SignalType::QZS_L2C:
            return constants::GPS_L2_FREQ;
        case SignalType::GPS_L5: case SignalType::QZS_L5:
        case SignalType::GAL_E5A:
            return 1176.45e6;
        case SignalType::GAL_E5B:
            return 1207.14e6;
        default: return 0.0;
    }
}

namespace libgnss {

namespace {

bool pppDebugEnabled() {
    return std::getenv("GNSS_PPP_DEBUG") != nullptr;
}

const char* clasAtmosSelectionPolicyName(
    ppp_shared::PPPConfig::ClasAtmosSelectionPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST:
            return "grid-first";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED:
            return "grid-guarded";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED:
            return "balanced";
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST:
            return "freshness-first";
    }
    return "grid-first";
}

void resetPhaseBiasRepairInfo(CLASPhaseBiasRepairInfo& info) {
    info.reference_time = GNSSTime();
    info.last_continuity_m = {0.0, 0.0, 0.0};
    info.offset_cycles = {0.0, 0.0, 0.0};
    info.pending_state_shift_cycles = {0.0, 0.0, 0.0};
    info.has_last = {false, false, false};
}

struct ClasAtmosCandidate {
    std::map<std::string, std::string> tokens;
    bool has_grid = false;
    double grid_distance_sq = std::numeric_limits<double>::infinity();
    double time_gap = std::numeric_limits<double>::infinity();
    int token_count = -1;
};

bool isBetterGridFirstCandidate(const ClasAtmosCandidate& candidate,
                                const ClasAtmosCandidate& best) {
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            candidate.token_count > best.token_count);
}

bool isBetterFreshnessFirstCandidate(const ClasAtmosCandidate& candidate,
                                     const ClasAtmosCandidate& best) {
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.token_count > best.token_count);
}

bool isBetterBalancedCandidate(const ClasAtmosCandidate& candidate,
                               const ClasAtmosCandidate& best,
                               double stale_after_seconds) {
    const bool candidate_stale = candidate.time_gap > stale_after_seconds;
    const bool best_stale = best.time_gap > stale_after_seconds;
    return (candidate.has_grid && !best.has_grid) ||
           (candidate.has_grid == best.has_grid && candidate_stale != best_stale &&
            !candidate_stale) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            !candidate_stale &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate_stale &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            !candidate_stale &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.time_gap + 1e-9 < best.time_gap) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            candidate_stale &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            candidate.grid_distance_sq + 1e-9 < best.grid_distance_sq) ||
           (candidate.has_grid == best.has_grid && candidate_stale == best_stale &&
            std::abs(candidate.time_gap - best.time_gap) <= 1e-9 &&
            std::abs(candidate.grid_distance_sq - best.grid_distance_sq) <= 1e-9 &&
            candidate.token_count > best.token_count);
}

bool isBetterClasAtmosCandidate(
    const ClasAtmosCandidate& candidate,
    const ClasAtmosCandidate& best,
    const ppp_shared::PPPConfig& config) {
    switch (config.clas_atmos_selection_policy) {
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_FIRST:
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::GRID_GUARDED:
            return isBetterGridFirstCandidate(candidate, best);
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::BALANCED:
            return isBetterBalancedCandidate(
                candidate, best, config.clas_atmos_stale_after_seconds);
        case ppp_shared::PPPConfig::ClasAtmosSelectionPolicy::FRESHNESS_FIRST:
            return isBetterFreshnessFirstCandidate(candidate, best);
    }
    return isBetterGridFirstCandidate(candidate, best);
}

/// Saastamoinen troposphere model with Niell mapping function
double troposphereDelay(const Vector3d& receiver_pos, double elevation, double trop_zenith) {
    if (elevation < 0.05) return trop_zenith * 10.0;  // near-horizon penalty
    // Simplified Niell dry mapping
    const double m = 1.0 / std::sin(std::max(elevation, 0.05));
    return trop_zenith * m;
}

/// Relativistic correction: only Shapiro delay.
/// The periodic relativity and gravitational redshift are already in the
/// broadcast clock polynomial, so we only need the signal propagation delay
/// due to the gravitational field (Shapiro effect).
double relativisticCorrection(const Vector3d& sat_pos, const Vector3d& /* sat_vel */,
                               const Vector3d& rcv_pos) {
    constexpr double GM = 3.986005e14;
    constexpr double c = constants::SPEED_OF_LIGHT;
    const double rs = sat_pos.norm();
    const double rr = rcv_pos.norm();
    const double rho = (sat_pos - rcv_pos).norm();
    const double arg = (rs + rr + rho) / std::max(rs + rr - rho, 1.0);
    // Shapiro delay in meters: 2*GM/c² * ln(...)
    return 2.0 * GM / (c * c) * std::log(std::max(arg, 1.0));
}

/// Phase wind-up (Wu et al. 1993) — full implementation
double phaseWindup(const Vector3d& sat_pos, const Vector3d& rcv_pos, double prev) {
    const Vector3d e_sr = (rcv_pos - sat_pos).normalized();
    const Vector3d e_z_sat = -sat_pos.normalized();
    Vector3d e_x_sat = e_sr.cross(e_z_sat);
    if (e_x_sat.norm() < 1e-10) return prev;
    e_x_sat.normalize();
    const Vector3d e_y_sat = e_z_sat.cross(e_x_sat);

    const Vector3d e_z_rcv = rcv_pos.normalized();
    Vector3d e_x_rcv(-rcv_pos.y(), rcv_pos.x(), 0.0);
    if (e_x_rcv.norm() < 1e-10) return prev;
    e_x_rcv.normalize();
    const Vector3d e_y_rcv = e_z_rcv.cross(e_x_rcv);

    const Vector3d d_sat = e_x_sat - e_sr * e_sr.dot(e_x_sat) + e_sr.cross(e_y_sat);
    const Vector3d d_rcv = e_x_rcv - e_sr * e_sr.dot(e_x_rcv) + e_sr.cross(e_y_rcv);

    const double cos_phi = d_sat.dot(d_rcv) / (d_sat.norm() * d_rcv.norm() + 1e-20);
    const double sign = e_sr.dot(d_sat.cross(d_rcv)) < 0.0 ? -1.0 : 1.0;
    double dphi = sign * std::acos(std::clamp(cos_phi, -1.0, 1.0)) / (2.0 * M_PI);

    double windup = prev + dphi;
    while (windup - prev > 0.5) windup -= 1.0;
    while (windup - prev < -0.5) windup += 1.0;
    return windup;
}

bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::abs(time.tow) > 0.0;
}

}  // anonymous namespace

const char* clasPhaseContinuityPolicyName(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR:
            return "full-repair";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY:
            return "sis-continuity-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::REPAIR_ONLY:
            return "repair-only";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::RAW_PHASE_BIAS:
            return "raw-phase-bias";
        case ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS:
            return "no-phase-bias";
    }
    return "full-repair";
}

const char* clasSsrTimingPolicyName(
    ppp_shared::PPPConfig::ClasSsrTimingPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::LAG_TOLERANT:
            return "lag-tolerant";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_PHASE_BIAS:
            return "clock-bound-phase-bias";
        case ppp_shared::PPPConfig::ClasSsrTimingPolicy::CLOCK_BOUND_ATMOS_AND_PHASE_BIAS:
            return "clock-bound-atmos-and-phase-bias";
    }
    return "lag-tolerant";
}

const char* clasExpandedValueConstructionPolicyName(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED:
            return "full-composed";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY:
            return "residual-only";
        case ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY:
            return "polynomial-only";
    }
    return "full-composed";
}

bool usesClasPhaseBiasTerms(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::NO_PHASE_BIAS;
}

bool usesClasSisContinuity(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY;
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
        for (const auto& correction : sat_it->second) {
            if (!correction.atmos_valid || correction.atmos_tokens.empty()) {
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
                    ? (grid_reference.dlat_deg * grid_reference.dlat_deg +
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
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_pos,
    double receiver_clk,
    double trop_zenith,
    const ppp_shared::PPPConfig& config,
    std::map<SatelliteId, double>& prev_windup,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASSisContinuityInfo>& sis_continuity,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair) {

    std::vector<OSRCorrection> corrections;
    int preferred_network_id = 0;
    ppp_atmosphere::parseAtmosTokenInt(
        atmos_tokens, "atmos_network_id", preferred_network_id);

    for (const auto& sat : obs.getSatellites()) {
        OSRCorrection osr;
        osr.satellite = sat;

        // --- 1. Find L1/L2 observations ---
        const auto findSignal = [&](const std::vector<SignalType>& candidates)
            -> const Observation* {
            for (auto sig : candidates) {
                const Observation* o = obs.getObservation(sat, sig);
                if (o && o->valid && o->has_carrier_phase && o->has_pseudorange) return o;
            }
            return nullptr;
        };

        std::vector<SignalType> l1_cands, l2_cands;
        switch (sat.system) {
            case GNSSSystem::GPS:
                l1_cands = {SignalType::GPS_L1CA, SignalType::GPS_L1P};
                l2_cands = {SignalType::GPS_L2C, SignalType::GPS_L2P, SignalType::GPS_L5};
                break;
            case GNSSSystem::Galileo:
                l1_cands = {SignalType::GAL_E1};
                l2_cands = {SignalType::GAL_E5A, SignalType::GAL_E5B};
                break;
            case GNSSSystem::QZSS:
                l1_cands = {SignalType::QZS_L1CA};
                l2_cands = {SignalType::QZS_L2C, SignalType::QZS_L5};
                break;
            default:
                continue;
        }

        const Observation* l1_obs = findSignal(l1_cands);
        const Observation* l2_obs = findSignal(l2_cands);
        if (!l1_obs) continue;

        // --- 2. Satellite position/clock from broadcast + SSR ---
        Vector3d sat_pos, sat_vel;
        double sat_clk = 0.0, sat_drift = 0.0;
        if (!nav.calculateSatelliteState(sat, obs.time, sat_pos, sat_vel, sat_clk, sat_drift)) {
            continue;
        }

        // Re-evaluate satellite state at signal transmission time. Without
        // this, CLAS per-frequency residuals stay at the 20-40 m level.
        if (l1_obs->pseudorange > 0.0) {
            const double travel_time = l1_obs->pseudorange / constants::SPEED_OF_LIGHT;
            const GNSSTime emission_time = obs.time - travel_time + sat_clk;
            if (!nav.calculateSatelliteState(
                    sat, emission_time, sat_pos, sat_vel, sat_clk, sat_drift)) {
                continue;
            }
        }

        // Apply SSR orbit/clock corrections
        Vector3d orbit_corr = Vector3d::Zero();
        double clock_corr = 0.0;
        double ura_sigma = 0.0;
        std::map<uint8_t, double> ssr_cbias, ssr_pbias;
        std::map<std::string, std::string> atmos_tokens;
        GNSSTime atmos_reference_time;
        GNSSTime phase_bias_reference_time;
        GNSSTime clock_reference_time;
        if (ssr.interpolateCorrection(sat, obs.time, orbit_corr, clock_corr,
                                       &ura_sigma, &ssr_cbias, &ssr_pbias,
                                       &atmos_tokens,
                                       &atmos_reference_time,
                                       &phase_bias_reference_time,
                                       &clock_reference_time,
                                       preferred_network_id)) {
            // CSV-expanded CLAS corrections carry orbit deltas in RAC, while
            // sampled RTCM SSR products are already stored in ECEF.
            if (ssr.orbitCorrectionsAreRac() &&
                orbit_corr.squaredNorm() > 0.0 &&
                sat_pos.squaredNorm() > 0.0) {
                const Vector3d r_unit = -sat_pos.normalized();  // Radial (toward Earth center)
                Vector3d c_unit = sat_pos.cross(sat_vel);
                if (c_unit.squaredNorm() > 0.0) {
                    c_unit.normalize();  // Cross-track (normal to orbital plane)
                } else {
                    c_unit = Vector3d(0, 0, 1);
                }
                const Vector3d a_unit = c_unit.cross(r_unit);  // Along-track
                // orbit_corr = (dR, dA, dC) in RAC
                const Vector3d orbit_ecef = r_unit * orbit_corr(0)
                                          + a_unit * orbit_corr(1)
                                          + c_unit * orbit_corr(2);
                orbit_corr = orbit_ecef;
            }
            if (std::getenv("GNSS_PPP_DEBUG") && corrections.size() < 20) {
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
            osr.clock_reference_time = clock_reference_time;
            osr.clock_correction_m = clock_corr;
        } else {
            continue;
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
            atmos_tokens.clear();
            osr.atmos_reference_time = GNSSTime();
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

        if (elev < 0.26) continue;  // 15 deg mask

        osr.elevation = elev;
        osr.azimuth = azim;

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);
        osr.signals[0] = l1_obs->signal;
        osr.frequencies[0] = signalFrequencyHz(l1_obs->signal, eph);
        osr.wavelengths[0] = constants::SPEED_OF_LIGHT / osr.frequencies[0];
        osr.num_frequencies = 1;
        if (l2_obs) {
            osr.signals[1] = l2_obs->signal;
            osr.frequencies[1] = signalFrequencyHz(l2_obs->signal, eph);
            osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
            osr.num_frequencies = 2;
        }

        // --- 4. Troposphere ---
        osr.trop_correction_m = troposphereDelay(receiver_pos, elev, trop_zenith);

        // CLAS troposphere grid correction (if available)
        if (!atmos_tokens.empty()) {
            const double clas_trop = ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                atmos_tokens,
                receiver_pos,
                obs.time,
                elev,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
            if (std::isfinite(clas_trop) && std::abs(clas_trop) > 0.0) {
                osr.trop_correction_m = clas_trop;  // Use CLAS trop instead of model
            }
        }

        // --- 5. Relativity ---
        osr.relativity_correction_m = relativisticCorrection(sat_pos, sat_vel, receiver_pos);

        // --- 6. Ionosphere (STEC) ---
        if (!atmos_tokens.empty()) {
            const double stec_tecu = ppp_atmosphere::atmosphericStecTecu(
                atmos_tokens,
                sat,
                receiver_pos,
                config.clas_expanded_value_construction_policy,
                config.clas_subtype12_value_construction_policy,
                config.clas_expanded_residual_sampling_policy);
            if (std::isfinite(stec_tecu) && std::abs(stec_tecu) > 0.001) {
                osr.iono_l1_m = ppp_atmosphere::ionosphereDelayMetersFromTecu(
                    l1_obs->signal, eph, stec_tecu);
                osr.has_iono = true;
            }
        }
        if (!osr.has_iono) {
            continue;  // CLASLIB rejects satellites without STEC
        }

        // --- 7. Code/Phase bias ---
        auto signalId = [](GNSSSystem sys, SignalType sig) -> uint8_t {
            // Simplified RTCM SSR signal ID mapping
            switch (sys) {
                case GNSSSystem::GPS:
                    if (sig == SignalType::GPS_L1CA) return 2;
                    if (sig == SignalType::GPS_L2C) return 8;
                    if (sig == SignalType::GPS_L2P) return 9;
                    if (sig == SignalType::GPS_L5) return 22;
                    break;
                case GNSSSystem::Galileo:
                    if (sig == SignalType::GAL_E1) return 2;
                    if (sig == SignalType::GAL_E5A) return 22;
                    break;
                case GNSSSystem::QZSS:
                    if (sig == SignalType::QZS_L1CA) return 2;
                    if (sig == SignalType::QZS_L2C) return 8;
                    break;
                default: break;
            }
            return 0;
        };

        for (int f = 0; f < osr.num_frequencies; ++f) {
            const uint8_t sid = signalId(sat.system, osr.signals[f]);
            auto cb_it = ssr_cbias.find(sid);
            osr.code_bias_m[f] = (cb_it != ssr_cbias.end()) ? cb_it->second : 0.0;
            auto pb_it = ssr_pbias.find(sid);
            osr.phase_bias_m[f] = (pb_it != ssr_pbias.end()) ? pb_it->second : 0.0;
        }

        if (osr.num_frequencies >= 2) {
            auto& compensation = dispersion_compensation[sat];
            const GNSSTime interval_reference =
                osr.atmos_reference_time.week != 0 ? osr.atmos_reference_time : obs.time;
            if (compensation.reference_time.week == 0 ||
                compensation.reference_time != interval_reference) {
                compensation.reference_time = interval_reference;
                compensation.base_phase_m = {0.0, 0.0};
                compensation.has_base = {false, false};
                compensation.slip = {false, false};
                const Observation* compensation_obs[2] = {l1_obs, l2_obs};
                for (int f = 0; f < 2; ++f) {
                    const Observation* raw = compensation_obs[f];
                    if (raw != nullptr && raw->valid && raw->has_carrier_phase &&
                        std::isfinite(raw->carrier_phase) && osr.wavelengths[f] > 0.0) {
                        compensation.base_phase_m[static_cast<size_t>(f)] =
                            raw->carrier_phase * osr.wavelengths[f];
                        compensation.has_base[static_cast<size_t>(f)] = true;
                    }
                }
            }

            if ((l1_obs == nullptr || !l1_obs->valid || !l1_obs->has_carrier_phase ||
                 !std::isfinite(l1_obs->carrier_phase) || l1_obs->loss_of_lock) &&
                compensation.reference_time.week != 0) {
                compensation.slip[0] = true;
            }
            if ((l2_obs == nullptr || !l2_obs->valid || !l2_obs->has_carrier_phase ||
                 !std::isfinite(l2_obs->carrier_phase) || l2_obs->loss_of_lock) &&
                compensation.reference_time.week != 0) {
                compensation.slip[1] = true;
            }

            if (!compensation.slip[0] && !compensation.slip[1] &&
                compensation.has_base[0] && compensation.has_base[1] &&
                l1_obs != nullptr && l2_obs != nullptr &&
                l1_obs->has_carrier_phase && l2_obs->has_carrier_phase &&
                std::isfinite(l1_obs->carrier_phase) &&
                std::isfinite(l2_obs->carrier_phase) &&
                osr.wavelengths[0] > 0.0 && osr.wavelengths[1] > 0.0) {
                const double l1_phase_m = l1_obs->carrier_phase * osr.wavelengths[0];
                const double l2_phase_m = l2_obs->carrier_phase * osr.wavelengths[1];
                const double fi = osr.wavelengths[1] / osr.wavelengths[0];
                const double denom = 1.0 - fi * fi;
                if (std::abs(denom) > 1e-9) {
                    const double dgf = l1_phase_m - l2_phase_m -
                        (compensation.base_phase_m[0] - compensation.base_phase_m[1]);
                    osr.phase_compensation_m[0] = dgf / denom;
                    osr.phase_compensation_m[1] = (fi * fi * dgf) / denom;
                }
            }
        }

        // --- 8. Phase wind-up ---
        double& wu = prev_windup[sat];
        wu = phaseWindup(sat_pos, receiver_pos, wu);
        osr.windup_cycles = wu;
        for (int f = 0; f < osr.num_frequencies; ++f) {
            osr.windup_m[f] = wu * osr.wavelengths[f];
        }

        auto& phase_bias_repair_info = phase_bias_repair[sat];
        auto& sis_continuity_info = sis_continuity[sat];
        const auto phase_continuity_policy =
            config.clas_phase_continuity_policy;
        const bool clock_time_valid =
            osr.clock_reference_time.week != 0 ||
            std::abs(osr.clock_reference_time.tow) > 0.0;
        const double current_sis_m = -osr.clock_correction_m + osr.orbit_projection_m;
        if (!clock_time_valid) {
            sis_continuity_info = CLASSisContinuityInfo{};
        } else if (!sis_continuity_info.has_current) {
            sis_continuity_info.current_time = osr.clock_reference_time;
            sis_continuity_info.current_sis_m = current_sis_m;
            sis_continuity_info.has_current = true;
        } else if (sis_continuity_info.current_time != osr.clock_reference_time) {
            const double dt_clock =
                osr.clock_reference_time - sis_continuity_info.current_time;
            sis_continuity_info.previous_time = sis_continuity_info.current_time;
            sis_continuity_info.previous_sis_m = sis_continuity_info.current_sis_m;
            sis_continuity_info.current_time = osr.clock_reference_time;
            sis_continuity_info.current_sis_m = current_sis_m;
            sis_continuity_info.has_previous = true;
            if (std::abs(dt_clock - 5.0) < 0.5) {
                sis_continuity_info.last_delta_m =
                    sis_continuity_info.current_sis_m -
                    sis_continuity_info.previous_sis_m;
                sis_continuity_info.has_last_delta = true;
            } else {
                sis_continuity_info.last_delta_m = 0.0;
                sis_continuity_info.has_last_delta = false;
            }
        }
        const bool phase_bias_time_valid =
            osr.phase_bias_reference_time.week != 0 ||
            std::abs(osr.phase_bias_reference_time.tow) > 0.0;
        bool phase_bias_epoch_changed = false;
        double phase_bias_dt = 0.0;
        if (!usesClasPhaseBiasRepair(phase_continuity_policy)) {
            resetPhaseBiasRepairInfo(phase_bias_repair_info);
        } else if (!phase_bias_time_valid) {
            resetPhaseBiasRepairInfo(phase_bias_repair_info);
        } else if (phase_bias_repair_info.reference_time.week == 0 &&
                   std::abs(phase_bias_repair_info.reference_time.tow) == 0.0) {
            phase_bias_repair_info.reference_time = osr.phase_bias_reference_time;
        } else if (phase_bias_repair_info.reference_time != osr.phase_bias_reference_time) {
            phase_bias_epoch_changed = true;
            phase_bias_dt = osr.phase_bias_reference_time - phase_bias_repair_info.reference_time;
            if (std::abs(phase_bias_dt) >= 120.0) {
                phase_bias_repair_info.offset_cycles = {0.0, 0.0, 0.0};
                phase_bias_repair_info.pending_state_shift_cycles = {0.0, 0.0, 0.0};
                phase_bias_repair_info.has_last = {false, false, false};
            }
            phase_bias_repair_info.reference_time = osr.phase_bias_reference_time;
        }

        // --- 9. Aggregate PRC/CPC (CLASLIB L282-285) ---
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const double fi = osr.frequencies[f] > 0.0 ? osr.wavelengths[f] / osr.wavelengths[0] : 1.0;
            const double iono_scaled = fi * fi * osr.iono_l1_m;
            const double phase_bias_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) ?
                    osr.phase_bias_m[f] :
                    0.0;
            const double phase_compensation_term =
                usesClasPhaseBiasTerms(phase_continuity_policy) ?
                    osr.phase_compensation_m[f] :
                    0.0;

            osr.PRC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] + iono_scaled + osr.code_bias_m[f];

            osr.CPC[f] = osr.trop_correction_m + osr.relativity_correction_m
                       + osr.receiver_antenna_m[f] - iono_scaled
                       + phase_bias_term + osr.windup_m[f]
                       + phase_compensation_term;

            if (clock_time_valid && phase_bias_time_valid &&
                sis_continuity_info.has_last_delta &&
                usesClasSisContinuity(phase_continuity_policy)) {
                const double pbias_lag =
                    osr.clock_reference_time - osr.phase_bias_reference_time;
                if (std::abs(pbias_lag - 30.0) < 0.5) {
                    osr.CPC[f] -= sis_continuity_info.last_delta_m;
                    osr.PRC[f] -= sis_continuity_info.last_delta_m;
                    if (std::getenv("GNSS_PPP_DEBUG") != nullptr && f == 0) {
                        std::cerr << "[OSR-SIS] " << sat.toString()
                                  << " lag_s=" << pbias_lag
                                  << " sis_delta_m=" << sis_continuity_info.last_delta_m
                                  << "\n";
                    }
                }
            }

            const double continuity_term =
                osr.orbit_projection_m - osr.clock_correction_m + osr.CPC[f];
            if (phase_bias_epoch_changed &&
                std::abs(phase_bias_dt) < 120.0 &&
                phase_bias_repair_info.has_last[static_cast<size_t>(f)] &&
                osr.wavelengths[f] > 0.0 &&
                usesClasPhaseBiasRepair(phase_continuity_policy)) {
                const double dcpc =
                    continuity_term -
                    phase_bias_repair_info.last_continuity_m[static_cast<size_t>(f)];
                const double cycles = dcpc / osr.wavelengths[f];
                if (cycles >= 95.0 && cycles < 105.0) {
                    phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)] -= 100.0;
                    phase_bias_repair_info.pending_state_shift_cycles[static_cast<size_t>(f)] -= 100.0;
                    if (std::getenv("GNSS_PPP_DEBUG") != nullptr) {
                        std::cerr << "[OSR-PBIAS] slip-100 " << sat.toString()
                                  << " f=" << f
                                  << " dcpc_cycles=" << cycles
                                  << " offset_cycles="
                                  << phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)]
                                  << "\n";
                    }
                } else if (cycles <= -95.0 && cycles > -105.0) {
                    phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)] += 100.0;
                    phase_bias_repair_info.pending_state_shift_cycles[static_cast<size_t>(f)] += 100.0;
                    if (std::getenv("GNSS_PPP_DEBUG") != nullptr) {
                        std::cerr << "[OSR-PBIAS] slip+100 " << sat.toString()
                                  << " f=" << f
                                  << " dcpc_cycles=" << cycles
                                  << " offset_cycles="
                                  << phase_bias_repair_info.offset_cycles[static_cast<size_t>(f)]
                                  << "\n";
                    }
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
        }

        osr.valid = true;
        if (std::getenv("GNSS_PPP_DEBUG") != nullptr && corrections.size() < 3) {
            std::cerr << "[OSR] " << sat.toString()
                      << " trop=" << osr.trop_correction_m
                      << " rel=" << osr.relativity_correction_m
                      << " iono_l1=" << osr.iono_l1_m
                      << " cbias0=" << osr.code_bias_m[0]
                      << " pbias0=" << osr.phase_bias_m[0]
                      << " windup=" << osr.windup_cycles
                      << " pbias_ref_tow=" << osr.phase_bias_reference_time.tow
                      << " orb_los=" << osr.orbit_projection_m
                      << " clk_corr=" << osr.clock_correction_m
                      << " PRC0=" << osr.PRC[0]
                      << " CPC0=" << osr.CPC[0]
                      << "\n";
        }
        corrections.push_back(osr);
    }

    return corrections;
}

}  // namespace libgnss
