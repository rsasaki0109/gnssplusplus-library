#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <sstream>
#include <string>

namespace libgnss {
namespace ppp_atmosphere {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;

struct ClasGridPoint {
    int network_id;
    int grid_no;
    double latitude_deg;
    double longitude_deg;
    double height_m;
};

const std::array<ClasGridPoint, 212>& clasGridPoints() {
    static const std::array<ClasGridPoint, 212> kClasGridPoints = {{
#include "clas_grid_points.inc"
    }};
    return kClasGridPoints;
}

bool tryParseDouble(const std::string& text, double& value) {
    try {
        size_t parsed = 0;
        value = std::stod(text, &parsed);
        return parsed == text.size();
    } catch (const std::exception&) {
        return false;
    }
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

}  // namespace

bool parseAtmosTokenDouble(const std::map<std::string, std::string>& atmos_tokens,
                           const std::string& key,
                           double& value) {
    const auto it = atmos_tokens.find(key);
    if (it == atmos_tokens.end()) {
        return false;
    }
    return tryParseDouble(it->second, value);
}

bool parseAtmosTokenInt(const std::map<std::string, std::string>& atmos_tokens,
                        const std::string& key,
                        int& value) {
    const auto it = atmos_tokens.find(key);
    if (it == atmos_tokens.end()) {
        return false;
    }
    try {
        value = std::stoi(it->second);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseAtmosListMeanValue(const std::map<std::string, std::string>& atmos_tokens,
                             const std::string& key,
                             double& value) {
    const auto it = atmos_tokens.find(key);
    if (it == atmos_tokens.end() || it->second.empty()) {
        return false;
    }
    const std::string& encoded = it->second;
    double sum = 0.0;
    int count = 0;
    size_t start = 0;
    while (start <= encoded.size()) {
        const size_t delimiter = encoded.find(';', start);
        const std::string token =
            delimiter == std::string::npos ? encoded.substr(start) : encoded.substr(start, delimiter - start);
        double parsed = 0.0;
        if (!token.empty() && tryParseDouble(token, parsed) && std::isfinite(parsed)) {
            sum += parsed;
            ++count;
        }
        if (delimiter == std::string::npos) {
            break;
        }
        start = delimiter + 1;
    }
    if (count == 0) {
        return false;
    }
    value = sum / static_cast<double>(count);
    return true;
}

bool parseAtmosListValueAtIndex(const std::map<std::string, std::string>& atmos_tokens,
                                const std::string& key,
                                size_t index,
                                double& value) {
    const auto it = atmos_tokens.find(key);
    if (it == atmos_tokens.end() || it->second.empty()) {
        return false;
    }
    const std::string& encoded = it->second;
    size_t start = 0;
    size_t current_index = 0;
    while (start <= encoded.size()) {
        const size_t delimiter = encoded.find(';', start);
        const std::string token =
            delimiter == std::string::npos ? encoded.substr(start) : encoded.substr(start, delimiter - start);
        if (current_index == index) {
            return !token.empty() && tryParseDouble(token, value) && std::isfinite(value);
        }
        if (delimiter == std::string::npos) {
            break;
        }
        start = delimiter + 1;
        ++current_index;
    }
    return false;
}

bool resolveClasGridReference(const std::map<std::string, std::string>& atmos_tokens,
                              const Vector3d& receiver_position,
                              ClasGridReference& reference) {
    int network_id = 0;
    if (!parseAtmosTokenInt(atmos_tokens, "atmos_network_id", network_id) || network_id <= 0) {
        return false;
    }

    int grid_count = 0;
    parseAtmosTokenInt(atmos_tokens, "atmos_grid_count", grid_count);

    double receiver_lat_rad = 0.0;
    double receiver_lon_rad = 0.0;
    double receiver_height_m = 0.0;
    ecef2geodetic(receiver_position, receiver_lat_rad, receiver_lon_rad, receiver_height_m);
    if (!std::isfinite(receiver_lat_rad) || !std::isfinite(receiver_lon_rad)) {
        return false;
    }

    const double receiver_lat_deg = receiver_lat_rad / kDegreesToRadians;
    const double receiver_lon_deg = receiver_lon_rad / kDegreesToRadians;

    const ClasGridPoint* nearest = nullptr;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (const auto& point : clasGridPoints()) {
        if (point.network_id != network_id) {
            continue;
        }
        if (grid_count > 0 && point.grid_no > grid_count) {
            continue;
        }
        const double delta_lat_deg = receiver_lat_deg - point.latitude_deg;
        const double delta_lon_deg = receiver_lon_deg - point.longitude_deg;
        const double distance_sq = delta_lat_deg * delta_lat_deg + delta_lon_deg * delta_lon_deg;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            nearest = &point;
        }
    }

    if (nearest == nullptr) {
        return false;
    }

    reference.dlat_deg = receiver_lat_deg - nearest->latitude_deg;
    reference.dlon_deg = receiver_lon_deg - nearest->longitude_deg;
    reference.residual_index = static_cast<size_t>(std::max(nearest->grid_no - 1, 0));
    reference.network_id = nearest->network_id;
    reference.grid_no = nearest->grid_no;
    return true;
}

double atmosphericTroposphereCorrectionMeters(
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_position,
    const GNSSTime& time,
    double elevation) {
    double correction_m = 0.0;
    bool have_correction = false;
    ClasGridReference grid_reference;
    const bool have_grid_reference =
        resolveClasGridReference(atmos_tokens, receiver_position, grid_reference);
    int trop_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_trop_type", trop_type);

    double value = 0.0;
    if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t00_m", value) && std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    }
    if (have_grid_reference && trop_type > 0) {
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t01_m_per_deg", value) && std::isfinite(value)) {
            correction_m += value * grid_reference.dlat_deg;
            have_correction = true;
        }
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t10_m_per_deg", value) && std::isfinite(value)) {
            correction_m += value * grid_reference.dlon_deg;
            have_correction = true;
        }
    }
    if (have_grid_reference && trop_type > 1 &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t11_m_per_deg2", value) &&
        std::isfinite(value)) {
        correction_m += value * grid_reference.dlat_deg * grid_reference.dlon_deg;
        have_correction = true;
    }
    if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_offset_m", value) && std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    }
    if (have_grid_reference &&
        parseAtmosListValueAtIndex(atmos_tokens, "atmos_trop_residuals_m", grid_reference.residual_index, value) &&
        std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    } else if (parseAtmosListMeanValue(atmos_tokens, "atmos_trop_residuals_m", value) &&
               std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    }

    int trop_grid_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_trop_type", trop_grid_type);
    if (trop_grid_type == 1 && std::isfinite(elevation) && elevation > 0.0) {
        double latitude_rad = 0.0;
        double longitude_rad = 0.0;
        double height_m = 0.0;
        ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
        const double hydro_mapping =
            models::niellHydrostaticMapping(latitude_rad, height_m, elevation, dayOfYearFromTime(time));
        const double wet_mapping = models::niellWetMapping(latitude_rad, elevation);
        double hydro_delta_m = 0.0;
        double wet_delta_m = 0.0;
        bool have_grid_trop = false;
        if (have_grid_reference &&
            parseAtmosListValueAtIndex(atmos_tokens, "atmos_trop_hs_residuals_m", grid_reference.residual_index, value) &&
            std::isfinite(value)) {
            hydro_delta_m = value;
            have_grid_trop = true;
        } else if (parseAtmosListMeanValue(atmos_tokens, "atmos_trop_hs_residuals_m", value) &&
                   std::isfinite(value)) {
            hydro_delta_m = value;
            have_grid_trop = true;
        }
        if (have_grid_reference &&
            parseAtmosListValueAtIndex(atmos_tokens, "atmos_trop_wet_residuals_m", grid_reference.residual_index, value) &&
            std::isfinite(value)) {
            wet_delta_m = value;
            have_grid_trop = true;
        } else if (parseAtmosListMeanValue(atmos_tokens, "atmos_trop_wet_residuals_m", value) &&
                   std::isfinite(value)) {
            wet_delta_m = value;
            have_grid_trop = true;
        }
        if (have_grid_trop) {
            correction_m += hydro_mapping * (2.3 + hydro_delta_m) + wet_mapping * (0.252 + wet_delta_m);
            have_correction = true;
        }
    }

    return have_correction ? correction_m : 0.0;
}

double atmosphericStecTecu(const std::map<std::string, std::string>& atmos_tokens,
                           const SatelliteId& satellite,
                           const Vector3d& receiver_position) {
    const std::string suffix = ":" + satellite.toString();
    double stec_tecu = 0.0;
    bool have_correction = false;
    ClasGridReference grid_reference;
    const bool have_grid_reference =
        resolveClasGridReference(atmos_tokens, receiver_position, grid_reference);
    int stec_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_stec_type" + suffix, stec_type);

    double value = 0.0;
    if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c00_tecu" + suffix, value) &&
        std::isfinite(value)) {
        stec_tecu += value;
        have_correction = true;
    }
    if (have_grid_reference && stec_type > 0) {
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c01_tecu_per_deg" + suffix, value) &&
            std::isfinite(value)) {
            stec_tecu += value * grid_reference.dlat_deg;
            have_correction = true;
        }
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c10_tecu_per_deg" + suffix, value) &&
            std::isfinite(value)) {
            stec_tecu += value * grid_reference.dlon_deg;
            have_correction = true;
        }
    }
    if (have_grid_reference && stec_type > 1 &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c11_tecu_per_deg2" + suffix, value) &&
        std::isfinite(value)) {
        stec_tecu += value * grid_reference.dlat_deg * grid_reference.dlon_deg;
        have_correction = true;
    }
    if (have_grid_reference && stec_type > 2) {
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c02_tecu_per_deg2" + suffix, value) &&
            std::isfinite(value)) {
            stec_tecu += value * grid_reference.dlat_deg * grid_reference.dlat_deg;
            have_correction = true;
        }
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c20_tecu_per_deg2" + suffix, value) &&
            std::isfinite(value)) {
            stec_tecu += value * grid_reference.dlon_deg * grid_reference.dlon_deg;
            have_correction = true;
        }
    }
    if (have_grid_reference &&
        parseAtmosListValueAtIndex(atmos_tokens, "atmos_stec_residuals_tecu" + suffix, grid_reference.residual_index, value) &&
        std::isfinite(value)) {
        stec_tecu += value;
        have_correction = true;
    } else if (parseAtmosListMeanValue(atmos_tokens, "atmos_stec_residuals_tecu" + suffix, value) &&
               std::isfinite(value)) {
        stec_tecu += value;
        have_correction = true;
    }

    return have_correction ? stec_tecu : 0.0;
}

double ionosphereDelayMetersFromTecu(SignalType signal,
                                     const Ephemeris* eph,
                                     double stec_tecu) {
    double frequency_hz = 0.0;
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::QZS_L1CA:
        case SignalType::GAL_E1:
            frequency_hz = constants::GPS_L1_FREQ;
            break;
        case SignalType::GPS_L2C:
        case SignalType::QZS_L2C:
            frequency_hz = constants::GPS_L2_FREQ;
            break;
        case SignalType::GPS_L5:
        case SignalType::QZS_L5:
        case SignalType::GAL_E5A:
        case SignalType::BDS_B2A:
            frequency_hz = constants::GPS_L5_FREQ;
            break;
        case SignalType::GAL_E5B:
            frequency_hz = constants::GAL_E5B_FREQ;
            break;
        case SignalType::GAL_E6:
            frequency_hz = constants::GAL_E6_FREQ;
            break;
        case SignalType::BDS_B1I:
            frequency_hz = constants::BDS_B1I_FREQ;
            break;
        case SignalType::BDS_B1C:
            frequency_hz = constants::BDS_B1C_FREQ;
            break;
        case SignalType::BDS_B2I:
            frequency_hz = constants::BDS_B2I_FREQ;
            break;
        case SignalType::BDS_B3I:
            frequency_hz = constants::BDS_B3I_FREQ;
            break;
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            if (eph != nullptr && eph->satellite.system == GNSSSystem::GLONASS) {
                frequency_hz = constants::GLO_L1_BASE_FREQ +
                               eph->glonass_frequency_channel * constants::GLO_L1_STEP_FREQ;
            } else {
                frequency_hz = constants::GLO_L1_BASE_FREQ;
            }
            break;
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            if (eph != nullptr && eph->satellite.system == GNSSSystem::GLONASS) {
                frequency_hz = constants::GLO_L2_BASE_FREQ +
                               eph->glonass_frequency_channel * constants::GLO_L2_STEP_FREQ;
            } else {
                frequency_hz = constants::GLO_L2_BASE_FREQ;
            }
            break;
        default:
            break;
    }
    if (frequency_hz <= 0.0 || !std::isfinite(stec_tecu)) {
        return 0.0;
    }
    return 40.3e16 * stec_tecu / (frequency_hz * frequency_hz);
}

double observationIonosphereDelayMeters(const Ephemeris* eph,
                                        SignalType primary_signal,
                                        SignalType secondary_signal,
                                        bool use_ionosphere_free,
                                        double stec_tecu,
                                        double coeff_primary,
                                        double coeff_secondary) {
    const double primary_delay_m = ionosphereDelayMetersFromTecu(primary_signal, eph, stec_tecu);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_delay_m;
    }
    const double secondary_delay_m =
        ionosphereDelayMetersFromTecu(secondary_signal, eph, stec_tecu);
    return coeff_primary * primary_delay_m + coeff_secondary * secondary_delay_m;
}

}  // namespace ppp_atmosphere
}  // namespace libgnss
