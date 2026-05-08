#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

namespace libgnss {

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

std::string trimCopy(const std::string& text) {
    const auto first = std::find_if(
        text.begin(), text.end(), [](unsigned char ch) { return !std::isspace(ch); });
    if (first == text.end()) {
        return "";
    }
    const auto last = std::find_if(
        text.rbegin(), text.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base();
    return std::string(first, last);
}

std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return normalized;
}

SignalType antexSignalType(const std::string& code) {
    const std::string trimmed = trimCopy(code);
    if (trimmed == "G01") return SignalType::GPS_L1CA;
    if (trimmed == "G02") return SignalType::GPS_L2C;
    if (trimmed == "G05") return SignalType::GPS_L5;
    if (trimmed == "E01") return SignalType::GAL_E1;
    if (trimmed == "E05") return SignalType::GAL_E5A;
    if (trimmed == "E07") return SignalType::GAL_E5B;
    if (trimmed == "J01") return SignalType::QZS_L1CA;
    if (trimmed == "J02") return SignalType::QZS_L2C;
    if (trimmed == "J05") return SignalType::QZS_L5;
    return SignalType::SIGNAL_TYPE_COUNT;
}

SignalType canonicalAntexSignal(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1P: return SignalType::GPS_L1CA;
        case SignalType::GPS_L2P: return SignalType::GPS_L2C;
        default: return signal;
    }
}

std::string extractAntexFrequencyCode(const std::string& line) {
    for (size_t index = 0; index + 2 < std::min<size_t>(line.size(), 20U); ++index) {
        if (std::isalpha(static_cast<unsigned char>(line[index])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 1])) &&
            std::isdigit(static_cast<unsigned char>(line[index + 2]))) {
            return line.substr(index, 3);
        }
    }
    return "";
}

bool parseAntexNoaziValues(const std::string& line, std::array<double, 19>& pcv_m) {
    pcv_m.fill(0.0);
    const size_t pos = line.find("NOAZI");
    if (pos == std::string::npos) {
        return false;
    }
    std::istringstream iss(line.substr(pos + 5));
    double value_mm = 0.0;
    int count = 0;
    while (count < static_cast<int>(pcv_m.size()) && iss >> value_mm) {
        pcv_m[static_cast<size_t>(count)] = value_mm * 1e-3;
        ++count;
    }
    if (count == 0) {
        return false;
    }
    for (; count < static_cast<int>(pcv_m.size()); ++count) {
        pcv_m[static_cast<size_t>(count)] = pcv_m[static_cast<size_t>(count - 1)];
    }
    return true;
}

double interpolateNoaziPcv(const std::array<double, 19>& pcv_m, double zenith_angle_deg) {
    const double clamped = std::clamp(zenith_angle_deg, 0.0, 90.0);
    const double scaled = clamped / 5.0;
    const int index = static_cast<int>(std::floor(scaled));
    if (index <= 0) {
        return pcv_m.front();
    }
    if (index >= static_cast<int>(pcv_m.size()) - 1) {
        return pcv_m.back();
    }
    const double fraction = scaled - static_cast<double>(index);
    return pcv_m[static_cast<size_t>(index)] * (1.0 - fraction) +
           pcv_m[static_cast<size_t>(index + 1)] * fraction;
}

struct ReceiverAntexModel {
    std::map<SignalType, Vector3d> offsets_enu_m;
    std::map<SignalType, std::array<double, 19>> noazi_pcv_m;
};

ReceiverAntexModel loadReceiverAntexModel(const std::string& filename,
                                           const std::string& antenna_type) {
    ReceiverAntexModel model;
    if (filename.empty() || antenna_type.empty()) {
        return model;
    }
    std::ifstream input(filename);
    if (!input.is_open()) {
        return model;
    }

    const std::string target_type = normalizeAntennaType(antenna_type);
    bool in_antenna = false;
    bool target_entry = false;
    SignalType current_signal = SignalType::SIGNAL_TYPE_COUNT;
    std::string line;
    while (std::getline(input, line)) {
        const std::string label = line.size() >= 60 ? trimCopy(line.substr(60)) : "";
        if (label == "START OF ANTENNA") {
            in_antenna = true;
            target_entry = false;
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (!in_antenna) {
            continue;
        }
        if (label == "END OF ANTENNA") {
            if (target_entry) {
                return model;
            }
            in_antenna = false;
            continue;
        }
        if (label == "TYPE / SERIAL NO") {
            target_entry = normalizeAntennaType(line.substr(0, 20)) == target_type;
            continue;
        }
        if (!target_entry) {
            continue;
        }
        if (label == "START OF FREQUENCY") {
            current_signal = antexSignalType(extractAntexFrequencyCode(line));
            continue;
        }
        if (label == "END OF FREQUENCY") {
            current_signal = SignalType::SIGNAL_TYPE_COUNT;
            continue;
        }
        if (label == "NORTH / EAST / UP" &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::istringstream iss(line.substr(0, std::min<size_t>(line.size(), 30U)));
            double north_mm = 0.0;
            double east_mm = 0.0;
            double up_mm = 0.0;
            if (iss >> north_mm >> east_mm >> up_mm) {
                model.offsets_enu_m[current_signal] =
                    Vector3d(east_mm * 1e-3, north_mm * 1e-3, up_mm * 1e-3);
            }
            continue;
        }
        if (line.find("NOAZI") != std::string::npos &&
            current_signal != SignalType::SIGNAL_TYPE_COUNT) {
            std::array<double, 19> pcv_m{};
            if (parseAntexNoaziValues(line, pcv_m)) {
                model.noazi_pcv_m[current_signal] = pcv_m;
            }
        }
    }

    return model;
}

const ReceiverAntexModel& receiverAntexModel(const ppp_shared::PPPConfig& config) {
    static std::map<std::string, ReceiverAntexModel> cache;
    const std::string key =
        config.antex_file_path + "\n" + normalizeAntennaType(config.receiver_antenna_type);
    const auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }
    const auto inserted = cache.emplace(
        key, loadReceiverAntexModel(config.antex_file_path, config.receiver_antenna_type));
    return inserted.first->second;
}

double receiverAntennaCorrectionMeters(const ppp_shared::PPPConfig& config,
                                       const Vector3d& receiver_pos,
                                       const Vector3d& los_unit,
                                       double elevation_rad,
                                       SignalType signal) {
    Vector3d offset_enu = config.receiver_antenna_delta_enu;
    double pcv_m = 0.0;
    const auto& model = receiverAntexModel(config);
    const SignalType canonical = canonicalAntexSignal(signal);
    const auto offset_it = model.offsets_enu_m.find(canonical);
    if (offset_it != model.offsets_enu_m.end()) {
        offset_enu += offset_it->second;
    }
    const auto pcv_it = model.noazi_pcv_m.find(canonical);
    if (pcv_it != model.noazi_pcv_m.end() && std::isfinite(elevation_rad)) {
        pcv_m = interpolateNoaziPcv(pcv_it->second, 90.0 - elevation_rad * 180.0 / M_PI);
    }

    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_pos, lat, lon, h);
    const Vector3d offset_ecef = enu2ecef(offset_enu, lat, lon);
    return pcv_m - los_unit.dot(offset_ecef);
}

bool clasOsrSatelliteExcluded(const SatelliteId& sat) {
    const char* raw = std::getenv("GNSS_PPP_CLAS_OSR_EXCLUDE_SATS");
    if (raw == nullptr || *raw == '\0') {
        return false;
    }
    const std::string target = sat.toString();
    std::string token;
    const std::string text(raw);
    for (char ch : text) {
        if (ch == ',' || ch == ';' || std::isspace(static_cast<unsigned char>(ch))) {
            if (token == target) {
                return true;
            }
            token.clear();
        } else {
            token.push_back(ch);
        }
    }
    return token == target;
}

int parsePositiveIntEnv(const char* name) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || *raw == '\0') {
        return 0;
    }
    try {
        const int value = std::stoi(raw);
        return value > 0 ? value : 0;
    } catch (const std::exception&) {
        return 0;
    }
}

const char* clasOsrDumpPath() {
    const char* path = std::getenv("GNSS_PPP_CLAS_OSR_DUMP");
    return (path != nullptr && *path != '\0') ? path : nullptr;
}

void appendClasOsrDump(const GNSSTime& time,
                       const SatelliteId& sat,
                       int selected_atmos_network_id,
                       const std::map<std::string, std::string>& atmos_tokens,
                       const Vector3d& receiver_pos,
                       double stec_tecu,
                       bool has_iono) {
    const char* path = clasOsrDumpPath();
    if (path == nullptr) {
        return;
    }

    bool write_header = false;
    {
        std::ifstream existing(path);
        write_header = !existing.good() || existing.peek() == std::ifstream::traits_type::eof();
    }

    std::ofstream out(path, std::ios::app);
    if (!out) {
        return;
    }
    if (write_header) {
        out << "week,tow,sat,selected_atmos_network_id,token_network_id,token_count,"
            << "grid_count,have_grid_ref,grid_no,residual_index,has_bilinear,"
            << "b0_index,b1_index,b2_index,b3_index,residual_key_present,"
            << "nearest_valid,nearest_value,b0_valid,b0_value,b1_valid,b1_value,"
            << "b2_valid,b2_value,b3_valid,b3_value,stec_tecu,has_iono\n";
    }

    int token_network_id = 0;
    int grid_count = 0;
    ppp_atmosphere::parseAtmosTokenInt(atmos_tokens, "atmos_network_id", token_network_id);
    ppp_atmosphere::parseAtmosTokenInt(atmos_tokens, "atmos_grid_count", grid_count);

    ppp_atmosphere::ClasGridReference grid_reference;
    const bool have_grid_ref =
        ppp_atmosphere::resolveClasGridReference(atmos_tokens, receiver_pos, grid_reference);

    const std::string residual_key = "atmos_stec_residuals_tecu:" + sat.toString();
    const bool residual_key_present = atmos_tokens.find(residual_key) != atmos_tokens.end();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    double nearest_value = nan;
    const bool nearest_valid = have_grid_ref &&
        ppp_atmosphere::parseAtmosListValueAtIndex(
            atmos_tokens, residual_key, grid_reference.residual_index, nearest_value);
    double bilinear_values[4] = {nan, nan, nan, nan};
    bool bilinear_valid[4] = {false, false, false, false};
    for (int i = 0; i < 4; ++i) {
        if (have_grid_ref && grid_reference.has_bilinear) {
            bilinear_valid[i] = ppp_atmosphere::parseAtmosListValueAtIndex(
                atmos_tokens,
                residual_key,
                grid_reference.bilinear_grid_indices[i],
                bilinear_values[i]);
        }
    }

    out << time.week << ',' << std::fixed << std::setprecision(3) << time.tow << ','
        << sat.toString() << ',' << selected_atmos_network_id << ',' << token_network_id << ','
        << atmos_tokens.size() << ',' << grid_count << ',' << (have_grid_ref ? 1 : 0) << ','
        << (have_grid_ref ? grid_reference.grid_no : 0) << ','
        << (have_grid_ref ? grid_reference.residual_index : 0) << ','
        << (have_grid_ref && grid_reference.has_bilinear ? 1 : 0) << ','
        << (have_grid_ref ? grid_reference.bilinear_grid_indices[0] : 0) << ','
        << (have_grid_ref ? grid_reference.bilinear_grid_indices[1] : 0) << ','
        << (have_grid_ref ? grid_reference.bilinear_grid_indices[2] : 0) << ','
        << (have_grid_ref ? grid_reference.bilinear_grid_indices[3] : 0) << ','
        << (residual_key_present ? 1 : 0) << ','
        << (nearest_valid ? 1 : 0) << ',' << nearest_value << ','
        << (bilinear_valid[0] ? 1 : 0) << ',' << bilinear_values[0] << ','
        << (bilinear_valid[1] ? 1 : 0) << ',' << bilinear_values[1] << ','
        << (bilinear_valid[2] ? 1 : 0) << ',' << bilinear_values[2] << ','
        << (bilinear_valid[3] ? 1 : 0) << ',' << bilinear_values[3] << ','
        << stec_tecu << ',' << (has_iono ? 1 : 0) << '\n';
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

    // CLASLIB convention: ph is the absolute windup angle for this epoch.
    // Maintain integer-turn continuity with the previous value.
    return dphi + std::floor(prev - dphi + 0.5);
}

bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::abs(time.tow) > 0.0;
}

void updateDispersionCompensation(
    OSRCorrection& osr,
    CLASDispersionCompensationInfo& compensation,
    const Observation* l1_obs,
    const Observation* l2_obs,
    const GNSSTime& obs_time) {
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
            if (raw != nullptr && raw->valid && raw->has_carrier_phase &&
                std::isfinite(raw->carrier_phase) && osr.wavelengths[f] > 0.0) {
                compensation.base_phase_m[static_cast<size_t>(f)] =
                    raw->carrier_phase * osr.wavelengths[f];
                compensation.has_base[static_cast<size_t>(f)] = true;
            }
        }
    }

    auto checkSlip = [&](const Observation* obs_ptr, int freq_idx) {
        if ((obs_ptr == nullptr || !obs_ptr->valid || !obs_ptr->has_carrier_phase ||
             !std::isfinite(obs_ptr->carrier_phase) || obs_ptr->loss_of_lock) &&
            compensation.reference_time.week != 0) {
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

/// Update SIS (Signal-In-Space) continuity tracking for a satellite.
/// Detects clock epoch transitions and computes delta for SIS correction.
void updateSisContinuity(
    CLASSisContinuityInfo& info,
    const OSRCorrection& osr,
    bool clock_time_valid) {
    const double current_sis_m = -osr.clock_correction_m + osr.orbit_projection_m;
    if (!clock_time_valid) {
        info = CLASSisContinuityInfo{};
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
    const int forced_network_id =
        parsePositiveIntEnv("GNSS_PPP_CLAS_FORCE_ATMOS_NETWORK");

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
            int candidate_network_id = 0;
            ppp_atmosphere::parseAtmosTokenInt(
                correction.atmos_tokens, "atmos_network_id", candidate_network_id);
            if (forced_network_id > 0 && candidate_network_id != forced_network_id) {
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
    const int forced_network_id =
        parsePositiveIntEnv("GNSS_PPP_CLAS_FORCE_ATMOS_NETWORK");
    if (forced_network_id > 0) {
        preferred_network_id = forced_network_id;
    }

    for (const auto& sat : obs.getSatellites()) {
        if (clasOsrSatelliteExcluded(sat)) {
            continue;
        }
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
                l2_cands = {SignalType::GPS_L2P, SignalType::GPS_L2C, SignalType::GPS_L5};
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
        std::map<uint8_t, uint8_t> ssr_phase_disc;
        std::map<std::string, std::string> atmos_tokens;
        GNSSTime atmos_reference_time;
        GNSSTime phase_bias_reference_time;
        GNSSTime clock_reference_time;
        int selected_atmos_network_id = 0;
        int selected_code_bias_network_id = 0;
        int selected_phase_bias_network_id = 0;
        if (ssr.interpolateCorrection(sat, obs.time, orbit_corr, clock_corr,
                                       &ura_sigma, &ssr_cbias, &ssr_pbias,
                                       &atmos_tokens,
                                       &atmos_reference_time,
                                       &phase_bias_reference_time,
                                       &clock_reference_time,
                                       preferred_network_id,
                                       true,
                                       false,
                                       nullptr,
                                       &ssr_phase_disc,
                                       &selected_atmos_network_id,
                                       &selected_code_bias_network_id,
                                       &selected_phase_bias_network_id)) {
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
                          << " sat_clk_total_s="
                          << sat_clk + clock_corr / constants::SPEED_OF_LIGHT
                          << " cbias_n=" << ssr_cbias.size()
                          << " pbias_n=" << ssr_pbias.size()
                          << " atmos_net=" << selected_atmos_network_id
                          << " cbias_net=" << selected_code_bias_network_id
                          << " pbias_net=" << selected_phase_bias_network_id << "\n";
            }
            sat_pos += orbit_corr;
            sat_clk += clock_corr / constants::SPEED_OF_LIGHT;
            osr.has_code_bias = !ssr_cbias.empty();
            osr.has_phase_bias = !ssr_pbias.empty();
            osr.phase_discontinuity_indicators = ssr_phase_disc;
            osr.atmos_network_id = selected_atmos_network_id;
            osr.code_bias_network_id = selected_code_bias_network_id;
            osr.phase_bias_network_id = selected_phase_bias_network_id;
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
            ssr_phase_disc.clear();
            osr.phase_discontinuity_indicators.clear();
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

        if (elev < kElevationMaskRad) continue;

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
        for (int f = 0; f < osr.num_frequencies; ++f) {
            osr.receiver_antenna_m[f] =
                receiverAntennaCorrectionMeters(
                    config, receiver_pos, los, elev, osr.signals[f]);
        }

        // --- 4. Troposphere ---
        // Use Saastamoinen as trop fallback when CLAS grid trop is unavailable.
        osr.trop_correction_m = models::tropDelaySaastamoinen(receiver_pos, elev);

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
        double stec_tecu = 0.0;
        if (!atmos_tokens.empty()) {
            stec_tecu = ppp_atmosphere::atmosphericStecTecu(
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
        appendClasOsrDump(
            obs.time,
            sat,
            selected_atmos_network_id,
            atmos_tokens,
            receiver_pos,
            stec_tecu,
            osr.has_iono);
        if (!osr.has_iono) {
            continue;  // CLASLIB rejects satellites without STEC
        }

        // --- 7. Code/Phase bias ---
        for (int f = 0; f < osr.num_frequencies; ++f) {
            const uint8_t sid = rtcmSsrSignalId(sat.system, osr.signals[f]);
            auto cb_it = ssr_cbias.find(sid);
            osr.code_bias_m[f] = (cb_it != ssr_cbias.end()) ? cb_it->second : 0.0;
            auto pb_it = ssr_pbias.find(sid);
            osr.phase_bias_m[f] = (pb_it != ssr_pbias.end()) ? pb_it->second : 0.0;
        }

        if (osr.num_frequencies >= 2) {
            updateDispersionCompensation(
                osr, dispersion_compensation[sat], l1_obs, l2_obs, obs.time);
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
        const bool clock_time_valid = gnsstimeIsSet(osr.clock_reference_time);
        updateSisContinuity(sis_continuity_info, osr, clock_time_valid);
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

            if (clock_time_valid && gnsstimeIsSet(effective_phase_bias_reference_time) &&
                sis_continuity_info.has_last_delta &&
                usesClasSisContinuity(phase_continuity_policy)) {
                const double pbias_lag =
                    osr.clock_reference_time - effective_phase_bias_reference_time;
                if (std::abs(pbias_lag - kPhaseBiasLagSeconds) < 0.5) {
                    osr.CPC[f] -= sis_continuity_info.last_delta_m;
                    osr.PRC[f] -= sis_continuity_info.last_delta_m;
                    if (pppDebugEnabled() && f == 0) {
                        std::cerr << "[OSR-SIS] " << sat.toString()
                                  << " lag_s=" << pbias_lag
                                  << " sis_delta_m=" << sis_continuity_info.last_delta_m
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
        if (pppDebugEnabled() && corrections.size() < 3) {
            std::cerr << "[OSR] " << sat.toString()
                      << " trop=" << osr.trop_correction_m
                      << " rel=" << osr.relativity_correction_m
                      << " iono_l1=" << osr.iono_l1_m
                      << " cbias0=" << osr.code_bias_m[0]
                      << " pbias0=" << osr.phase_bias_m[0]
                      << " cbias1=" << (osr.num_frequencies > 1 ? osr.code_bias_m[1] : 0.0)
                      << " pbias1=" << (osr.num_frequencies > 1 ? osr.phase_bias_m[1] : 0.0)
                      << " atmos_net=" << osr.atmos_network_id
                      << " cbias_net=" << osr.code_bias_network_id
                      << " pbias_net=" << osr.phase_bias_network_id
                      << " windup=" << osr.windup_cycles
                      << " pbias_ref_tow=" << osr.phase_bias_reference_time.tow
                      << " eff_pbias_ref_tow=" << effective_phase_bias_reference_time.tow
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
