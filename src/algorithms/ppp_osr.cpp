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

namespace {

constexpr double kClasQzssHeldStecAgeSeconds = 3600.0;

std::map<SatelliteId, std::pair<GNSSTime, double>>& qzssHeldStecBySatellite() {
    static std::map<SatelliteId, std::pair<GNSSTime, double>> held;
    return held;
}

std::vector<std::string> qzssStecTokenSuffixes(const SatelliteId& sat) {
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

bool parseQzssBroadcastStecQuality(const std::map<std::string, std::string>& atmos_tokens,
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

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

struct ClasOsrBiasLookup {
    double value_m = 0.0;
    std::uint8_t source_signal_id = 0;
    bool present = false;
    bool fallback = false;
};

ClasOsrBiasLookup clasOsrBiasLookup(const std::map<uint8_t, double>& bias_m,
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

bool gpsL2ExactBiasIdentityEnabled(const Observation* observation) {
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

bool clasGpsL2wIdentityGateEnabled() {
    return pppEnvOverrides().clas_dd_filter &&
           pppEnvOverrides().clas_code_row_bias_identity;
}

bool clasMrtklibFloatParity(const ppp_shared::PPPConfig& config) {
    return config.clas_mrtklib_float_parity && config.kinematic_mode &&
           !config.low_dynamics_mode && config.use_clas_osr_filter &&
           config.use_dynamics_model;
}

const Observation* findExactGpsL2wObservation(
    const ObservationData& obs,
    const SatelliteId& sat) {
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

bool hasUsableCodeAndPhase(const Observation& observation) {
    return observation.valid &&
           observation.has_pseudorange &&
           observation.has_carrier_phase &&
           std::isfinite(observation.pseudorange) &&
           std::isfinite(observation.carrier_phase);
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

double phaseWindup(const Vector3d& sat_pos,
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

double phaseWindupMrtklib(const Vector3d& sat_pos,
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

bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::abs(time.tow) > 0.0;
}

std::vector<std::string> splitSemicolonList(const std::string& text) {
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

int countSemicolonListValues(const std::map<std::string, std::string>& tokens,
                             const std::string& key) {
    const auto it = tokens.find(key);
    if (it == tokens.end()) {
        return 0;
    }
    return static_cast<int>(splitSemicolonList(it->second).size());
}

int countFiniteSemicolonListValues(const std::map<std::string, std::string>& tokens,
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

int countSelectedGridStecValues(const std::map<std::string, std::string>& tokens,
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

void setAtmosLifecycleProvenance(OSRCorrection& osr,
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

void updateDispersionCompensation(
    OSRCorrection& osr,
    CLASDispersionCompensationInfo& compensation,
    const Observation* l1_obs,
    const Observation* l2_obs,
    const GNSSTime& obs_time,
    bool mrtklib_parity) {
    if (mrtklib_parity && compensation.mrtklib_qzss_suppressed) {
        return;
    }
    const GNSSTime interval_reference =
        osr.atmos_reference_time.week != 0 ? osr.atmos_reference_time : obs_time;
    if (compensation.reference_time.week == 0 ||
        compensation.reference_time != interval_reference) {
        compensation.reference_time = interval_reference;
        compensation.base_phase_m = {0.0, 0.0};
        compensation.has_base = {false, false};
        compensation.slip = {false, false};
        const Observation* freq_obs[2] = {l1_obs, l2_obs};
        for (int f = 0; f < 2; ++f) {
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

PhaseBiasEpochStatus updatePhaseBiasRepairState(
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

const char* clasPhaseBiasValuePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::FULL:
            return "full";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY:
            return "phase-bias-only";
        case ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY:
            return "compensation-only";
    }
    return "full";
}

const char* clasPhaseBiasReferenceTimePolicyName(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return "phase-bias-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return "clock-reference";
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return "observation-epoch";
    }
    return "phase-bias-reference";
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

bool usesClasRawPhaseBiasValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::COMPENSATION_ONLY;
}

bool usesClasPhaseCompensationValues(
    ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasPhaseBiasValuePolicy::PHASE_BIAS_ONLY;
}

GNSSTime selectClasPhaseBiasReferenceTime(
    ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy policy,
    const GNSSTime& phase_bias_reference_time,
    const GNSSTime& clock_reference_time,
    const GNSSTime& observation_time) {
    switch (policy) {
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::PHASE_BIAS_REFERENCE:
            return phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::CLOCK_REFERENCE:
            return gnsstimeIsSet(clock_reference_time) ? clock_reference_time
                                                       : phase_bias_reference_time;
        case ppp_shared::PPPConfig::ClasPhaseBiasReferenceTimePolicy::OBSERVATION_EPOCH:
            return observation_time;
    }
    return phase_bias_reference_time;
}

bool usesClasSisContinuity(
    ppp_shared::PPPConfig::ClasPhaseContinuityPolicy policy) {
    return policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::FULL_REPAIR ||
           policy ==
               ppp_shared::PPPConfig::ClasPhaseContinuityPolicy::SIS_CONTINUITY_ONLY;
}

void updateSisContinuity(
    CLASSisContinuityInfo& info,
    const OSRCorrection& osr,
    bool clock_time_valid) {
    const double current_sis_m = -osr.clock_correction_m + osr.orbit_projection_m;
    if (!clock_time_valid) {
        // Preserve the CLASLIB-style boundary-held delta (gated feature)
        // across transient clock-reference-time gaps: CLASLIB's satcorr[]
        // state (currtow/currsis) is keyed to message reception and outlives
        // a few epochs of missing per-epoch SSR data, so a brief gap inside
        // the 15s hold window (observed on real CLAS data) must not drop the
        // already-captured boundary delta. The rest of the continuity state
        // (current/previous SIS samples, last_delta_m) legitimately resets,
        // matching pre-existing (gate-OFF) behavior.
        const GNSSTime preserved_boundary_time = info.boundary_time;
        const double preserved_boundary_delta_m = info.boundary_delta_m;
        const bool preserved_has_boundary_delta = info.has_boundary_delta;
        const GNSSTime preserved_boundary_prev_time = info.boundary_prev_time;
        const double preserved_boundary_prev_sis_m = info.boundary_prev_sis_m;
        const bool preserved_has_boundary_prev_sis = info.has_boundary_prev_sis;
        info = CLASSisContinuityInfo{};
        info.boundary_time = preserved_boundary_time;
        info.boundary_delta_m = preserved_boundary_delta_m;
        info.has_boundary_delta = preserved_has_boundary_delta;
        info.boundary_prev_time = preserved_boundary_prev_time;
        info.boundary_prev_sis_m = preserved_boundary_prev_sis_m;
        info.has_boundary_prev_sis = preserved_has_boundary_prev_sis;
    } else if (!info.has_current) {
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_current = true;
    } else if (info.current_time != osr.clock_reference_time) {
        const double dt_clock = osr.clock_reference_time - info.current_time;
        info.previous_time = info.current_time;
        info.previous_sis_m = info.current_sis_m;
        info.current_time = osr.clock_reference_time;
        info.current_sis_m = current_sis_m;
        info.has_previous = true;
        if (std::abs(dt_clock - kSsrClockIntervalSeconds) < 0.5) {
            info.last_delta_m = info.current_sis_m - info.previous_sis_m;
            info.has_last_delta = true;
        } else {
            info.last_delta_m = 0.0;
            info.has_last_delta = false;
        }
    }
}

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
    if (freq_index < 0 || freq_index >= osr.num_frequencies ||
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
    int preferred_network_id = 0;
    ppp_atmosphere::parseAtmosTokenInt(
        epoch_atmos_tokens, "atmos_network_id", preferred_network_id);

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
        const bool mrtklib_parity = clasMrtklibFloatParity(config);
        if (sat.system == GNSSSystem::GPS &&
            (mrtklib_parity || clasGpsL2wIdentityGateEnabled())) {
            const Observation* exact_l2w = findExactGpsL2wObservation(obs, sat);
            // RTKLIB fixes frequency slot 1 to the header-selected L2W code.
            // On the literal path a missing L2W must remain absent for both
            // compensatedisp() and slip detection; falling through to the
            // simultaneously recorded L2L creates a fictitious multi-metre
            // phase compensation (G04 at tow 177068.8 in tokyo/run2).
            if (mrtklib_parity || exact_l2w != nullptr) {
                l2_obs = exact_l2w;
            }
        }
        if (!l1_obs) {
            if (pppDebugEnabled() && sat.system == GNSSSystem::QZSS) {
                std::cerr << "[OSR-QZSS-SKIP] " << sat.toString()
                          << " reason=no_l1_code_phase\n";
            }
            continue;
        }

        // --- 2. Satellite position/clock from broadcast + SSR ---
        Vector3d sat_pos, sat_vel;
        double sat_clk = 0.0, sat_drift = 0.0;
        int clas_orbit_iode = -1;
        if (mrtklib_parity) {
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
                const double polynomial = parity_eph->af0 +
                    parity_eph->af1 * tc + parity_eph->af2 * tc * tc;
                sat_clk = polynomial -
                    2.0 * sat_pos.dot(sat_vel) /
                        (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT);
                sat_drift = parity_eph->af1 + 2.0 * parity_eph->af2 * tc;
            }
            osr.signal_transmit_time = emission_time;
        }

        // Apply SSR orbit/clock corrections
        Vector3d orbit_corr = Vector3d::Zero();
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
        if (l2_obs) {
            osr.signals[1] = l2_obs->signal;
            osr.pseudorange_rinex_codes[1] = l2_obs->pseudorange_observation_type;
            osr.carrier_rinex_codes[1] = l2_obs->carrier_phase_observation_type;
            osr.frequencies[1] = signalFrequencyHz(l2_obs->signal, eph);
            osr.wavelengths[1] = constants::SPEED_OF_LIGHT / osr.frequencies[1];
            osr.num_frequencies = 2;
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
                mrtklib_parity ? osr.atmos_network_id : 0;
            if (mrtklib_parity) {
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
                const auto* clas_bank = mrtklib_parity
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
                    mrtklib_parity);
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
            if (service_network_id > 0 &&
                ssr.heldQzssPhaseBiasForServiceNetwork(
                    sat, obs.time, service_network_id,
                    &repicked_pbias, &repicked_discnt, &repicked_ref,
                    mrtklib_parity)) {
                ssr_pbias = std::move(repicked_pbias);
                phase_bias_reference_time = repicked_ref;
                osr.has_phase_bias = !ssr_pbias.empty();
                osr.phase_bias_reference_time = repicked_ref;
            }
        }

        // --- 7. Code/Phase bias ---
        const Observation* freq_observations[2] = {l1_obs, l2_obs};
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const Observation* freq_obs = freq_observations[f];
            const bool exact_bias_identity =
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
                freq_obs != nullptr ? freq_obs->pseudorange_observation_type : "",
                freq_obs != nullptr ? freq_obs->carrier_phase_observation_type : "",
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
        wu = mrtklib_parity
            ? phaseWindupMrtklib(sat_pos, sat_vel, receiver_pos, wu)
            : phaseWindup(sat_pos, receiver_pos, wu);
        osr.windup_cycles = wu;
        for (int f = 0; f < osr.num_frequencies; ++f) {
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
        if (pppEnvOverrides().clas_sis_boundary) {
            captureClasSisBoundary(
                sis_continuity_info, obs.time, current_sis_m,
                mrtklib_parity ? kMrtklibOrbitBoundaryToleranceSeconds
                               : kSsrOrbitBoundaryToleranceSeconds);
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
                const bool sis_boundary_gate_enabled =
                    pppEnvOverrides().clas_sis_boundary;
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
