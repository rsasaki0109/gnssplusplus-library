#pragma once

// Shared helpers from the former ppp_osr.cpp anonymous namespace
// (inline), plus declarations for cross-TU free functions.

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


namespace libgnss {
namespace ppp_osr_internal {


constexpr double kClasQzssHeldStecAgeSeconds = 3600.0;

inline std::map<SatelliteId, std::pair<GNSSTime, double>>& qzssHeldStecBySatellite() {
    static std::map<SatelliteId, std::pair<GNSSTime, double>> held;
    return held;
}

inline std::vector<std::string> qzssStecTokenSuffixes(const SatelliteId& sat) {
    std::vector<std::string> suffixes = {sat.toString()};
    if (sat.system == GNSSSystem::QZSS) {
        const std::map<std::string, std::string> compact_keys = {
            {"J01", "S120"},
            {"J02", "S121"},
            {"J03", "S122"},
        };
        const auto compact_it = compact_keys.find(sat.toString());
        if (compact_it != compact_keys.end()) {
            suffixes.push_back(compact_it->second);
        }
    }
    return suffixes;
}

inline bool parseQzssBroadcastStecQuality(const std::map<std::string, std::string>& atmos_tokens,
                                   const SatelliteId& sat,
                                   int& stec_quality) {
    for (const std::string& suffix : qzssStecTokenSuffixes(sat)) {
        if (ppp_atmosphere::parseAtmosTokenInt(
                atmos_tokens, "atmos_stec_quality:" + suffix, stec_quality)) {
            return true;
        }
    }
    return false;
}

inline bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

inline bool computeClasIodeGeometryCompensation(
    const NavigationData& nav,
    const SatelliteId& satellite,
    const GNSSTime& observation_time,
    double pseudorange_m,
    const Vector3d& receiver_position,
    int previous_iode,
    int current_iode,
    const Vector3d& current_orbit_rac_m,
    double current_clock_correction_m,
    const Vector3d& current_corrected_position,
    double current_corrected_clock_s,
    double& compensation_m) {
    compensation_m = 0.0;
    if (previous_iode < 0 || pseudorange_m <= 0.0) {
        return false;
    }

    const GNSSTime approximate_transmit_time =
        observation_time - pseudorange_m / constants::SPEED_OF_LIGHT;
    const Ephemeris* previous_eph =
        nav.getEphemeris(satellite, approximate_transmit_time, previous_iode);
    if (previous_eph == nullptr) {
        return false;
    }

    // RTKLIB eph2pos() and the native broadcast path use MU_GAL for Galileo.
    // Keep the explicit argument here so both sides of an IODE transition are
    // pinned to the same propagation constant even if a caller requests the
    // legacy GPS-mu behavior elsewhere.
    if (satellite.system == GNSSSystem::Galileo && current_iode >= 0) {
        const Ephemeris* current_eph =
            nav.getEphemeris(satellite, approximate_transmit_time, current_iode);
        if (current_eph == nullptr) {
            return false;
        }
        const auto corrected_model = [&](const Ephemeris& eph,
                                         double& model_m) -> bool {
            const double tc0 = approximate_transmit_time - eph.toc;
            double tc = tc0;
            for (int iteration = 0; iteration < 2; ++iteration) {
                const double polynomial =
                    eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
                tc = tc0 - polynomial;
            }
            const double polynomial =
                eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
            const GNSSTime transmit_time =
                approximate_transmit_time - polynomial;

            Vector3d position;
            Vector3d velocity;
            double clock_s = 0.0;
            double clock_drift = 0.0;
            if (!eph.calculateSatelliteState(
                    transmit_time, position, velocity, clock_s, clock_drift,
                    true)) {
                return false;
            }
            constexpr double kRtklibVelocityStepSeconds = 1e-3;
            Vector3d forward_position;
            Vector3d ignored_velocity;
            double ignored_clock = 0.0;
            double ignored_drift = 0.0;
            if (!eph.calculateSatelliteState(
                    transmit_time + kRtklibVelocityStepSeconds,
                    forward_position, ignored_velocity, ignored_clock,
                    ignored_drift, true)) {
                return false;
            }
            velocity = (forward_position - position) /
                kRtklibVelocityStepSeconds;
            if (position.squaredNorm() <= 0.0 || velocity.squaredNorm() <= 0.0) {
                return false;
            }
            const Vector3d along = velocity.normalized();
            Vector3d cross = position.cross(velocity);
            if (cross.squaredNorm() <= 0.0) {
                return false;
            }
            cross.normalize();
            const Vector3d radial = along.cross(cross);
            position -= radial * current_orbit_rac_m(0) +
                along * current_orbit_rac_m(1) +
                cross * current_orbit_rac_m(2);
            clock_s += current_clock_correction_m /
                constants::SPEED_OF_LIGHT;
            model_m = geodist(position, receiver_position) -
                constants::SPEED_OF_LIGHT * clock_s;
            return std::isfinite(model_m);
        };

        double previous_model_m = 0.0;
        double current_model_m = 0.0;
        if (!corrected_model(*previous_eph, previous_model_m) ||
            !corrected_model(*current_eph, current_model_m)) {
            return false;
        }
        compensation_m = previous_model_m - current_model_m;
        return std::isfinite(compensation_m);
    }

    // CLASLIB adjust_r_dts(): select the pre-boundary IODE, iterate the
    // broadcast clock polynomial at transmit time, then apply the *current*
    // SSR orbit/clock correction to that broadcast state.
    const double tc0 = approximate_transmit_time - previous_eph->toc;
    double tc = tc0;
    for (int iteration = 0; iteration < 2; ++iteration) {
        const double polynomial = previous_eph->af0 +
            previous_eph->af1 * tc + previous_eph->af2 * tc * tc;
        tc = tc0 - polynomial;
    }
    const double polynomial = previous_eph->af0 +
        previous_eph->af1 * tc + previous_eph->af2 * tc * tc;
    const GNSSTime transmit_time = approximate_transmit_time - polynomial;

    Vector3d previous_position;
    Vector3d ignored_velocity;
    double previous_clock_s = 0.0;
    double ignored_drift = 0.0;
    if (!previous_eph->calculateSatelliteState(
            transmit_time,
            previous_position,
            ignored_velocity,
            previous_clock_s,
            ignored_drift)) {
        return false;
    }

    constexpr double kRtklibVelocityStepSeconds = 1e-3;
    Vector3d forward_position;
    double ignored_clock = 0.0;
    if (!previous_eph->calculateSatelliteState(
            transmit_time + kRtklibVelocityStepSeconds,
            forward_position,
            ignored_velocity,
            ignored_clock,
            ignored_drift)) {
        return false;
    }
    const Vector3d previous_velocity =
        (forward_position - previous_position) / kRtklibVelocityStepSeconds;
    if (previous_position.squaredNorm() <= 0.0 ||
        previous_velocity.squaredNorm() <= 0.0) {
        return false;
    }

    const Vector3d along = previous_velocity.normalized();
    Vector3d cross = previous_position.cross(previous_velocity);
    if (cross.squaredNorm() <= 0.0) {
        return false;
    }
    cross.normalize();
    const Vector3d radial = along.cross(cross);
    const Vector3d previous_orbit_ecef =
        -(radial * current_orbit_rac_m(0) +
          along * current_orbit_rac_m(1) +
          cross * current_orbit_rac_m(2));
    previous_position += previous_orbit_ecef;
    previous_clock_s +=
        current_clock_correction_m / constants::SPEED_OF_LIGHT;

    const double previous_model_m =
        geodist(previous_position, receiver_position) -
        constants::SPEED_OF_LIGHT * previous_clock_s;
    const double current_model_m =
        geodist(current_corrected_position, receiver_position) -
        constants::SPEED_OF_LIGHT * current_corrected_clock_s;
    compensation_m = previous_model_m - current_model_m;
    return std::isfinite(compensation_m);
}

struct ClasOsrBiasLookup {
    double value_m = 0.0;
    std::uint8_t source_signal_id = 0;
    bool present = false;
    bool fallback = false;
};

inline ClasOsrBiasLookup clasOsrBiasLookup(const std::map<uint8_t, double>& bias_m,
                                    GNSSSystem system,
                                    uint8_t signal_id,
                                    bool enable_l2_class_fallback) {
    ClasOsrBiasLookup result;
    const auto it = bias_m.find(signal_id);
    if (it != bias_m.end()) {
        result.value_m = it->second;
        result.source_signal_id = signal_id;
        result.present = true;
        return result;
    }
    if (enable_l2_class_fallback && system == GNSSSystem::GPS &&
        (signal_id == 8U || signal_id == 9U)) {
        const auto sibling = bias_m.find(signal_id == 8U ? 9U : 8U);
        if (sibling != bias_m.end()) {
            result.value_m = sibling->second;
            result.source_signal_id = sibling->first;
            result.present = true;
            result.fallback = true;
            return result;
        }
    }
    return result;
}

inline bool gpsL2ExactBiasIdentityEnabled(const Observation* observation) {
    if (observation == nullptr || observation->satellite.system != GNSSSystem::GPS) {
        return false;
    }
    if (observation->signal != SignalType::GPS_L2C &&
        observation->signal != SignalType::GPS_L2P) {
        return false;
    }
    return pppEnvOverrides().clas_dd_filter &&
           pppEnvOverrides().clas_code_row_bias_identity;
}

inline bool clasGpsL2wIdentityGateEnabled() {
    return pppEnvOverrides().clas_dd_filter &&
           pppEnvOverrides().clas_code_row_bias_identity;
}

inline bool clasMrtklibFloatParity(const ppp_shared::PPPConfig& config) {
    return config.clas_mrtklib_float_parity && config.kinematic_mode &&
           !config.low_dynamics_mode && config.use_clas_osr_filter &&
           config.use_dynamics_model;
}

inline const Observation* findExactGpsL2wObservation(
    const ObservationData& obs,
    const SatelliteId& sat) {
    // The normal observation vector contains only the policy-selected signal
    // family (often 2X). RINEX slot assignment in RTKLIB is header-driven and
    // selects 2W when that tracking code is declared, so consult the preserved
    // exact tracking-code collection before the normal family observations.
    if (const Observation* exact =
            obs.getRinexTrackingObservation(sat, "2W");
        exact != nullptr && exact->valid && exact->has_pseudorange &&
        exact->has_carrier_phase && std::isfinite(exact->pseudorange) &&
        std::isfinite(exact->carrier_phase) &&
        algorithms::ppp_bias_identity::isGpsL2wObservation(
            exact->satellite.system,
            exact->signal,
            exact->pseudorange_observation_type,
            exact->carrier_phase_observation_type)) {
        return exact;
    }
    for (const auto& candidate : obs.observations) {
        if (!candidate.valid || candidate.satellite != sat) {
            continue;
        }
        if (!candidate.has_carrier_phase || !candidate.has_pseudorange) {
            continue;
        }
        if (algorithms::ppp_bias_identity::isGpsL2wObservation(
                candidate.satellite.system,
                candidate.signal,
                candidate.pseudorange_observation_type,
                candidate.carrier_phase_observation_type)) {
            return &candidate;
        }
    }
    return nullptr;
}

inline bool isGpsL2xObservation(const Observation* observation) {
    return observation != nullptr &&
           observation->satellite.system == GNSSSystem::GPS &&
           observation->pseudorange_observation_type == "C2X" &&
           observation->carrier_phase_observation_type == "L2X";
}

inline bool hasUsableCodeAndPhase(const Observation& observation) {
    return observation.valid &&
           observation.has_pseudorange &&
           observation.has_carrier_phase &&
           std::isfinite(observation.pseudorange) &&
           std::isfinite(observation.carrier_phase);
}

inline const char* clasAtmosSelectionPolicyName(
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

inline void resetPhaseBiasRepairInfo(CLASPhaseBiasRepairInfo& info) {
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

inline bool isBetterGridFirstCandidate(const ClasAtmosCandidate& candidate,
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

inline bool isBetterFreshnessFirstCandidate(const ClasAtmosCandidate& candidate,
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

inline bool isBetterBalancedCandidate(const ClasAtmosCandidate& candidate,
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

inline bool isBetterClasAtmosCandidate(
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
inline double troposphereDelay(const Vector3d& receiver_pos, double elevation, double trop_zenith) {
    if (elevation < 0.05) return trop_zenith * 10.0;  // near-horizon penalty
    // Simplified Niell dry mapping
    const double m = 1.0 / std::sin(std::max(elevation, 0.05));
    return trop_zenith * m;
}

/// Relativistic correction: only Shapiro delay.
/// The periodic relativity and gravitational redshift are already in the
/// broadcast clock polynomial, so we only need the signal propagation delay
/// due to the gravitational field (Shapiro effect).
inline double relativisticCorrection(const Vector3d& sat_pos, const Vector3d& /* sat_vel */,
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

inline double phaseWindup(const Vector3d& sat_pos,
                   const Vector3d& rcv_pos,
                   double previous) {
    const Vector3d e_sr = (rcv_pos - sat_pos).normalized();
    const Vector3d e_z_sat = -sat_pos.normalized();
    Vector3d e_x_sat = e_sr.cross(e_z_sat);
    if (e_x_sat.norm() < 1e-10) return previous;
    e_x_sat.normalize();
    const Vector3d e_y_sat = e_z_sat.cross(e_x_sat);

    const Vector3d e_z_rcv = rcv_pos.normalized();
    Vector3d e_x_rcv(-rcv_pos.y(), rcv_pos.x(), 0.0);
    if (e_x_rcv.norm() < 1e-10) return previous;
    e_x_rcv.normalize();
    const Vector3d e_y_rcv = e_z_rcv.cross(e_x_rcv);

    const Vector3d d_sat =
        e_x_sat - e_sr * e_sr.dot(e_x_sat) + e_sr.cross(e_y_sat);
    const Vector3d d_rcv =
        e_x_rcv - e_sr * e_sr.dot(e_x_rcv) + e_sr.cross(e_y_rcv);
    const double cos_phi = d_sat.dot(d_rcv) /
        (d_sat.norm() * d_rcv.norm() + 1e-20);
    const double sign =
        e_sr.dot(d_sat.cross(d_rcv)) < 0.0 ? -1.0 : 1.0;
    const double phase = sign *
        std::acos(std::clamp(cos_phi, -1.0, 1.0)) / (2.0 * M_PI);
    return phase + std::floor(previous - phase + 0.5);
}

inline double phaseWindupMrtklib(const Vector3d& sat_pos,
                          const Vector3d& sat_vel,
                          const Vector3d& rcv_pos,
                          double previous) {
    // Literal mrtk_clas_osr.c::windupcorr() frame construction.  The CLAS
    // path deliberately does not use the general PPP sat_yaw/model_phw model.
    constexpr double kEarthRotation = 7.2921151467e-5;
    const Vector3d ek_raw = rcv_pos - sat_pos;
    if (ek_raw.norm() < 1.0 || sat_pos.norm() < 1.0) return previous;
    const Vector3d ek = ek_raw.normalized();
    const Vector3d ezs = (-sat_pos).normalized();
    const Vector3d earth_cross_r(
        -kEarthRotation * sat_pos.y(),
         kEarthRotation * sat_pos.x(),
         0.0);
    const Vector3d ess_raw = sat_vel + earth_cross_r;
    if (ess_raw.norm() < 1e-12) return previous;
    const Vector3d ess = ess_raw.normalized();
    const Vector3d eys_raw = ezs.cross(ess);
    if (eys_raw.norm() < 1e-12) return previous;
    const Vector3d eys = eys_raw.normalized();
    const Vector3d exs = eys.cross(ezs);

    double lat = 0.0, lon = 0.0, height = 0.0;
    ecef2geodetic(rcv_pos, lat, lon, height);
    // xyz2enu() columns 0 and 1: east and north, respectively.
    const Vector3d exr(-std::sin(lon), std::cos(lon), 0.0);
    const Vector3d eyr(-std::sin(lat) * std::cos(lon),
                       -std::sin(lat) * std::sin(lon), std::cos(lat));
    const Vector3d ds = exs - ek * ek.dot(exs) - ek.cross(eys);
    const Vector3d dr = exr - ek * ek.dot(exr) + ek.cross(eyr);
    if (ds.norm() < 1e-12 || dr.norm() < 1e-12) return previous;
    double phase = std::acos(std::clamp(
        ds.dot(dr) / (ds.norm() * dr.norm()), -1.0, 1.0)) / (2.0 * M_PI);
    if (ek.dot(ds.cross(dr)) < 0.0) phase = -phase;
    return phase + std::floor(previous - phase + 0.5);
}

inline bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::abs(time.tow) > 0.0;
}

inline std::vector<std::string> splitSemicolonList(const std::string& text) {
    std::vector<std::string> values;
    std::istringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ';')) {
        if (!value.empty()) {
            values.push_back(value);
        }
    }
    return values;
}

inline int countSemicolonListValues(const std::map<std::string, std::string>& tokens,
                             const std::string& key) {
    const auto it = tokens.find(key);
    if (it == tokens.end()) {
        return 0;
    }
    return static_cast<int>(splitSemicolonList(it->second).size());
}

inline int countFiniteSemicolonListValues(const std::map<std::string, std::string>& tokens,
                                   const std::string& key) {
    const auto it = tokens.find(key);
    if (it == tokens.end()) {
        return 0;
    }
    int count = 0;
    for (const auto& value : splitSemicolonList(it->second)) {
        try {
            size_t parsed = 0;
            const double number = std::stod(value, &parsed);
            if (parsed == value.size() && std::isfinite(number)) {
                ++count;
            }
        } catch (...) {
        }
    }
    return count;
}

inline int countSelectedGridStecValues(const std::map<std::string, std::string>& tokens,
                                const std::string& key,
                                const OSRCorrection& osr) {
    int count = 0;
    for (int grid = 0;
         grid < osr.atmos_interpolation_grid_count &&
         grid < static_cast<int>(osr.atmos_interpolation_grid_no.size());
         ++grid) {
        const int grid_no = osr.atmos_interpolation_grid_no[grid];
        if (grid_no <= 0) {
            continue;
        }
        double value = 0.0;
        if (ppp_atmosphere::parseAtmosListValueAtIndex(
                tokens, key, static_cast<size_t>(grid_no - 1), value) &&
            std::isfinite(value)) {
            ++count;
        }
    }
    return count;
}

inline void setAtmosLifecycleProvenance(OSRCorrection& osr,
                                 const std::map<std::string, std::string>& tokens,
                                 const SatelliteId& satellite) {
    osr.atmos_lifecycle = tokens.find("atmos_lifecycle") != tokens.end();
    int lifecycle_tow = 0;
    osr.has_atmos_lifecycle_tow =
        ppp_atmosphere::parseAtmosTokenInt(tokens, "atmos_lifecycle_tow", lifecycle_tow);
    osr.atmos_lifecycle_tow =
        osr.has_atmos_lifecycle_tow ? static_cast<double>(lifecycle_tow) : 0.0;
    ppp_atmosphere::parseAtmosTokenInt(
        tokens, "atmos_selected_satellites", osr.atmos_selected_satellite_count);
    osr.atmos_valid_grid_count = countSemicolonListValues(tokens, "atmos_valid_grids");
    const std::string stec_grid_key =
        "atmos_stec_grid_tecu:" + satellite.toString();
    osr.atmos_stec_grid_value_count =
        countFiniteSemicolonListValues(tokens, stec_grid_key);
    osr.atmos_selected_grid_stec_value_count =
        countSelectedGridStecValues(tokens, stec_grid_key, osr);
    const auto grid_satellites = tokens.find("atmos_grid_satellites");
    osr.atmos_grid_satellite_membership_known =
        grid_satellites != tokens.end();
    if (osr.atmos_grid_satellite_membership_known) {
        const auto satellites = splitSemicolonList(grid_satellites->second);
        for (const auto& suffix : qzssStecTokenSuffixes(satellite)) {
            if (std::find(satellites.begin(), satellites.end(), suffix) !=
                satellites.end()) {
                osr.atmos_grid_has_satellite = true;
                break;
            }
        }
    }
    auto stec_satellites = tokens.find(
        "atmos_stec_satellites:" + std::to_string(osr.atmos_network_id));
    if (stec_satellites == tokens.end()) {
        stec_satellites = tokens.find("atmos_stec_satellites");
    }
    osr.atmos_stec_satellite_membership_known =
        stec_satellites != tokens.end();
    if (osr.atmos_stec_satellite_membership_known) {
        const auto satellites = splitSemicolonList(stec_satellites->second);
        for (const auto& suffix : qzssStecTokenSuffixes(satellite)) {
            if (std::find(satellites.begin(), satellites.end(), suffix) !=
                satellites.end()) {
                osr.atmos_stec_has_satellite = true;
                break;
            }
        }
    }
}

inline void updateDispersionCompensation(
    OSRCorrection& osr,
    CLASDispersionCompensationInfo& compensation,
    const Observation* l1_obs,
    const Observation* l2_obs,
    const GNSSTime& obs_time,
    bool mrtklib_parity) {
    const GNSSTime interval_reference =
        osr.atmos_reference_time.week != 0 ? osr.atmos_reference_time : obs_time;
    if (compensation.reference_time.week == 0 ||
        compensation.reference_time != interval_reference) {
        compensation.reference_time = interval_reference;
        compensation.base_phase_m = {0.0, 0.0};
        compensation.has_base = {false, false};
        compensation.slip = {false, false};
        if (mrtklib_parity) {
            for (size_t sample = 0;
                 sample < compensation.warmup_times.size() &&
                 sample < compensation.warmup_phase_m.size();
                 ++sample) {
                if (compensation.warmup_times[sample] < interval_reference ||
                    compensation.warmup_times[sample] > obs_time) {
                    continue;
                }
                compensation.base_phase_m =
                    compensation.warmup_phase_m[sample];
                compensation.has_base = {true, true};
                break;
            }
        }
        const Observation* freq_obs[2] = {l1_obs, l2_obs};
        for (int f = 0; f < 2; ++f) {
            if (compensation.has_base[static_cast<size_t>(f)]) {
                continue;
            }
            const Observation* raw = freq_obs[f];
            if (raw != nullptr && (!mrtklib_parity || raw->valid) &&
                raw->has_carrier_phase &&
                std::isfinite(raw->carrier_phase) && osr.wavelengths[f] > 0.0) {
                compensation.base_phase_m[static_cast<size_t>(f)] =
                    raw->carrier_phase * osr.wavelengths[f];
                compensation.has_base[static_cast<size_t>(f)] = true;
            }
        }
    }
    // A bank can become visible on an epoch where one carrier is temporarily
    // absent.  MRTKLIB initializes compensatedisp() when the satellite's STEC
    // entry is first processed with its usable observation; fill any missing
    // base as soon as that carrier becomes available within the same bank.
    const bool have_current_pair = l1_obs != nullptr && l2_obs != nullptr &&
        l1_obs->has_carrier_phase && l2_obs->has_carrier_phase &&
        std::isfinite(l1_obs->carrier_phase) &&
        std::isfinite(l2_obs->carrier_phase) &&
        osr.wavelengths[0] > 0.0 && osr.wavelengths[1] > 0.0;
    if (mrtklib_parity &&
        (!compensation.has_base[0] || !compensation.has_base[1]) &&
        have_current_pair) {
        compensation.base_phase_m[0] =
            l1_obs->carrier_phase * osr.wavelengths[0];
        compensation.base_phase_m[1] =
            l2_obs->carrier_phase * osr.wavelengths[1];
        compensation.has_base = {true, true};
    }

    auto checkSlip = [&](const Observation* obs_ptr, int freq_idx) {
        // MRTKLIB measurement compensation only latches ssat.slip; a
        // temporarily absent/invalid observation does not itself create a
        // cycle slip (compensatedisp(), meas branch).  Once latched, the
        // flag remains set until a newer STEC bank resets comp_slip while
        // recording its new carrier-phase datum.
        const bool slip = mrtklib_parity
            ? (obs_ptr != nullptr && obs_ptr->loss_of_lock)
            : (obs_ptr == nullptr || !obs_ptr->valid ||
               !obs_ptr->has_carrier_phase ||
               !std::isfinite(obs_ptr->carrier_phase) ||
               obs_ptr->loss_of_lock);
        if (slip && compensation.reference_time.week != 0) {
            compensation.slip[static_cast<size_t>(freq_idx)] = true;
        }
    };
    checkSlip(l1_obs, 0);
    checkSlip(l2_obs, 1);

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

// Minimum elevation angle for satellite inclusion (radians, ~15 degrees)
constexpr double kElevationMaskRad = 0.26;
constexpr double kClasPhaseBiasBankPeriodSeconds = 30.0;
// Maximum time gap for phase bias repair before resetting (seconds)
constexpr double kPhaseBiasRepairTimeoutSeconds = 120.0;
// Expected SSR clock interval for SIS continuity detection (seconds)
constexpr double kSsrClockIntervalSeconds = 5.0;
// Expected phase bias lag for SIS continuity correction (seconds)
constexpr double kPhaseBiasLagSeconds = 30.0;
// Phase bias 100-cycle jump detection window (cycles)
constexpr double kPhaseBiasJumpLowerCycles = 95.0;
constexpr double kPhaseBiasJumpUpperCycles = 105.0;
constexpr double kPhaseBiasJumpCorrectionCycles = 100.0;
// CLASLIB SSR update boundary cadence: orbit/clock corrections realign on a
// 30s grid (see ephemeris.c satpos_ssr_sis prevtow/currtow bookkeeping).
constexpr double kSsrOrbitBoundaryIntervalSeconds = 30.0;
// Tolerance (seconds) for detecting that a clock reference time sits on the
// 30s orbit boundary grid.
// Observation epochs are 5 Hz. A half-second window classified the adjacent
// 177025.2/.4 and 177030.2/.4 epochs as boundaries too, repeatedly replacing
// CLASLIB's exact offset-25/offset-0 SIS samples with moving-geometry values.
constexpr double kSsrOrbitBoundaryToleranceSeconds = 0.5;
constexpr double kMrtklibOrbitBoundaryToleranceSeconds = 1e-3;
// CLASLIB holds the boundary-captured SIS delta for the first half of the
// 30s orbit cycle (observed empirically: nonzero for tow%30 in [0,14],
// zero for tow%30 in [15,29]; see docs/clas_dd_filter_a5.md).
constexpr double kSsrOrbitBoundaryApplyWindowSeconds = 15.0;
// Mid-cycle offset where CLASLIB captures prevsis (timediff(clock,orbit)==25).
constexpr double kSsrOrbitBoundaryOffset25Seconds = 25.0;
// CLASLIB requires exactly one 5s clock step between prevsis capture and the
// following 30s boundary (offset-25 obs epoch to offset-0 obs epoch).
constexpr double kSsrOrbitBoundaryPrevToBoundarySeconds = 5.0;

/// Detect phase bias epoch change and update repair tracking state.
/// Returns {epoch_changed, phase_bias_dt} for use in PRC/CPC aggregation.
struct PhaseBiasEpochStatus {
    bool epoch_changed = false;
    double dt = 0.0;
};

inline PhaseBiasEpochStatus updatePhaseBiasRepairState(
    CLASPhaseBiasRepairInfo& repair_info,
    const GNSSTime& effective_reference_time,
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    PhaseBiasEpochStatus status;
    if (!usesClasPhaseBiasRepair(policy)) {
        resetPhaseBiasRepairInfo(repair_info);
    } else if (!gnsstimeIsSet(effective_reference_time)) {
        resetPhaseBiasRepairInfo(repair_info);
    } else if (repair_info.reference_time.week == 0 &&
               std::abs(repair_info.reference_time.tow) == 0.0) {
        repair_info.reference_time = effective_reference_time;
    } else if (repair_info.reference_time != effective_reference_time) {
        status.epoch_changed = true;
        status.dt = effective_reference_time - repair_info.reference_time;
        if (std::abs(status.dt) >= kPhaseBiasRepairTimeoutSeconds) {
            repair_info.offset_cycles = {0.0, 0.0, 0.0};
            repair_info.pending_state_shift_cycles = {0.0, 0.0, 0.0};
            repair_info.has_last = {false, false, false};
        }
        repair_info.reference_time = effective_reference_time;
    }
    return status;
}

}  // namespace ppp_osr_internal

using namespace ppp_osr_internal;
}  // namespace libgnss
