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
constexpr double kClasTropHydrostaticRefM = 2.3;
constexpr double kClasTropWetRefM = 0.252;

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

bool usePolynomialTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::RESIDUAL_ONLY;
}

bool useResidualTerms(
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::POLYNOMIAL_ONLY;
}

bool useIndexedResidualSampling(
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy::MEAN_ONLY;
}

bool useMeanResidualSampling(
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy::INDEXED_ONLY;
}

bool isSubtype12TropRow(const std::map<std::string, std::string>& atmos_tokens) {
    const auto it = atmos_tokens.find("atmos_trop_source_subtype");
    return it != atmos_tokens.end() && it->second == "12";
}

bool isSubtype12StecRow(const std::map<std::string, std::string>& atmos_tokens,
                        const SatelliteId& satellite) {
    const auto it = atmos_tokens.find("atmos_stec_source_subtype:" + satellite.toString());
    return it != atmos_tokens.end() && it->second == "12";
}

bool useSubtype12LinearTerms(
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy policy) {
    return policy !=
           ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy::OFFSET_ONLY;
}

bool useSubtype12QuadraticTerms(
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy policy) {
    return policy ==
           ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy::FULL;
}

bool sampleAtmosResidualList(const std::map<std::string, std::string>& atmos_tokens,
                             const std::string& key,
                             bool have_grid_reference,
                             size_t residual_index,
                             ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy sampling_policy,
                             double& value) {
    if (useIndexedResidualSampling(sampling_policy) && have_grid_reference &&
        parseAtmosListValueAtIndex(atmos_tokens, key, residual_index, value) &&
        std::isfinite(value)) {
        return true;
    }
    if (useMeanResidualSampling(sampling_policy) &&
        parseAtmosListMeanValue(atmos_tokens, key, value) &&
        std::isfinite(value)) {
        return true;
    }
    return false;
}

double dot2(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

double norm2(double x, double y) {
    return std::sqrt(dot2(x, y, x, y));
}

bool invert4x4(const double input[16], double output[16]) {
    double augmented[4][8] = {};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            augmented[row][col] = input[row * 4 + col];
        }
        augmented[row][4 + row] = 1.0;
    }

    for (int pivot = 0; pivot < 4; ++pivot) {
        int best_row = pivot;
        double best_abs = std::abs(augmented[pivot][pivot]);
        for (int row = pivot + 1; row < 4; ++row) {
            const double candidate_abs = std::abs(augmented[row][pivot]);
            if (candidate_abs > best_abs) {
                best_abs = candidate_abs;
                best_row = row;
            }
        }
        if (best_abs < 1e-12) {
            return false;
        }
        if (best_row != pivot) {
            for (int col = 0; col < 8; ++col) {
                std::swap(augmented[pivot][col], augmented[best_row][col]);
            }
        }

        const double pivot_value = augmented[pivot][pivot];
        for (int col = 0; col < 8; ++col) {
            augmented[pivot][col] /= pivot_value;
        }
        for (int row = 0; row < 4; ++row) {
            if (row == pivot) continue;
            const double factor = augmented[row][pivot];
            for (int col = 0; col < 8; ++col) {
                augmented[row][col] -= factor * augmented[pivot][col];
            }
        }
    }

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            output[row * 4 + col] = augmented[row][4 + col];
        }
    }
    return true;
}

double applyGridModelInterpolation(const ClasGridReference& grid_reference,
                                   const double values[4]) {
    double transformed[4] = {};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            transformed[row] += grid_reference.model_gmat[row * 4 + col] * values[col];
        }
    }
    double result = 0.0;
    for (int row = 0; row < 4; ++row) {
        result += grid_reference.model_emat[row] * transformed[row];
    }
    return result;
}

bool sampleAtmosResidualGridModel(
    const std::map<std::string, std::string>& atmos_tokens,
    const std::string& key,
    const ClasGridReference& grid_reference,
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy sampling_policy,
    double& value) {
    if (grid_reference.has_model_interpolation) {
        double grid_values[4] = {};
        for (int g = 0; g < 4; ++g) {
            if (!sampleAtmosResidualList(
                    atmos_tokens,
                    key,
                    true,
                    grid_reference.model_grid_indices[g],
                    sampling_policy,
                    grid_values[g])) {
                break;
            }
            if (g == 3) {
                value = applyGridModelInterpolation(grid_reference, grid_values);
                return std::isfinite(value);
            }
        }
    }
    return sampleAtmosResidualList(
        atmos_tokens,
        key,
        true,
        grid_reference.residual_index,
        sampling_policy,
        value);
}

struct ClaslibVerticalTropDelays {
    double hydrostatic_m = 0.0;
    double wet_m = 0.0;
};

const ClasGridPoint* findClasGridPoint(int network_id, size_t residual_index) {
    const int grid_no = static_cast<int>(residual_index) + 1;
    for (const auto& point : clasGridPoints()) {
        if (point.network_id == network_id && point.grid_no == grid_no) {
            return &point;
        }
    }
    return nullptr;
}

double claslibInterpc(const std::array<double, 5>& values, double latitude_deg) {
    const int i = static_cast<int>(latitude_deg / 15.0);
    if (i < 1) return values[0];
    if (i > 4) return values[4];
    return values[static_cast<size_t>(i - 1)] * (1.0 - latitude_deg / 15.0 + i) +
           values[static_cast<size_t>(i)] * (latitude_deg / 15.0 - i);
}

double claslibEmbeddedGeoidHeight(double latitude_rad, double longitude_rad) {
    constexpr int lon_min_deg = 120;
    constexpr int lon_max_deg = 150;
    constexpr int lat_min_deg = 20;
    constexpr int lat_max_deg = 50;
    constexpr double geoid[lon_max_deg - lon_min_deg + 1][lat_max_deg - lat_min_deg + 1] = {
#include "clas_embedded_geoid_japan.inc"
    };

    double latitude_deg = latitude_rad / kDegreesToRadians;
    double longitude_deg = longitude_rad / kDegreesToRadians;
    if (longitude_deg < 0.0) longitude_deg += 360.0;
    if (latitude_deg < lat_min_deg || latitude_deg > lat_max_deg ||
        longitude_deg < lon_min_deg || longitude_deg > lon_max_deg) {
        return 0.0;
    }

    double lon_grid = longitude_deg - lon_min_deg;
    double lat_grid = latitude_deg - lat_min_deg;
    int lon0 = static_cast<int>(lon_grid);
    int lat0 = static_cast<int>(lat_grid);
    const double lon_frac = lon_grid - lon0;
    const double lat_frac = lat_grid - lat0;
    const int lon1 = std::min(lon0 + 1, lon_max_deg - lon_min_deg);
    const int lat1 = std::min(lat0 + 1, lat_max_deg - lat_min_deg);
    lon0 = std::clamp(lon0, 0, lon_max_deg - lon_min_deg);
    lat0 = std::clamp(lat0, 0, lat_max_deg - lat_min_deg);

    const double y00 = geoid[lon0][lat0];
    const double y10 = geoid[lon1][lat0];
    const double y01 = geoid[lon0][lat1];
    const double y11 = geoid[lon1][lat1];
    return y00 * (1.0 - lon_frac) * (1.0 - lat_frac) +
           y10 * lon_frac * (1.0 - lat_frac) +
           y01 * (1.0 - lon_frac) * lat_frac +
           y11 * lon_frac * lat_frac;
}

bool claslibStandardVerticalTropDelays(const GNSSTime& time,
                                       double latitude_rad,
                                       double ellipsoidal_height_m,
                                       double geoid_height_m,
                                       ClaslibVerticalTropDelays& out) {
    constexpr std::array<double, 5> mean_pressure = {
        1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    constexpr std::array<double, 5> mean_temperature = {
        299.65, 294.15, 283.15, 272.15, 263.65};
    constexpr std::array<double, 5> mean_water_pressure = {
        26.31, 21.79, 11.66, 6.78, 4.11};
    constexpr std::array<double, 5> mean_lapse_rate = {
        6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    constexpr std::array<double, 5> mean_water_vapor_rate = {
        2.77, 3.15, 2.57, 1.81, 1.55};
    constexpr std::array<double, 5> amp_pressure = {
        0.00, -3.75, -2.25, -1.75, -0.50};
    constexpr std::array<double, 5> amp_temperature = {
        0.00, 7.00, 11.00, 15.00, 14.50};
    constexpr std::array<double, 5> amp_water_pressure = {
        0.00, 8.85, 7.24, 5.36, 3.39};
    constexpr std::array<double, 5> amp_lapse_rate = {
        0.00e-3, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    constexpr std::array<double, 5> amp_water_vapor_rate = {
        0.00, 0.33, 0.46, 0.74, 0.30};
    constexpr double gravity = 9.80665;
    constexpr double gas_constant_dry = 287.0537625;

    const double latitude_deg = latitude_rad / kDegreesToRadians;
    if (latitude_deg < 0.0 || latitude_deg > 75.0) {
        return false;
    }

    const double doy = static_cast<double>(dayOfYearFromTime(time));
    const double seasonal = std::cos(2.0 * M_PI * (doy - 28.0) / 365.25);
    auto seasonalValues = [&](const std::array<double, 5>& mean_values,
                              const std::array<double, 5>& amp_values) {
        std::array<double, 5> values = {};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = mean_values[i] - amp_values[i] * seasonal;
        }
        return values;
    };

    const double pressure0 =
        claslibInterpc(seasonalValues(mean_pressure, amp_pressure), latitude_deg);
    const double temperature0 =
        claslibInterpc(seasonalValues(mean_temperature, amp_temperature), latitude_deg);
    const double water_pressure0 =
        claslibInterpc(seasonalValues(mean_water_pressure, amp_water_pressure), latitude_deg);
    const double lapse_rate =
        claslibInterpc(seasonalValues(mean_lapse_rate, amp_lapse_rate), latitude_deg);
    const double water_vapor_rate =
        claslibInterpc(seasonalValues(mean_water_vapor_rate, amp_water_vapor_rate), latitude_deg);

    const double orthometric_height_m = ellipsoidal_height_m - geoid_height_m;
    const double temperature = temperature0 - lapse_rate * orthometric_height_m;
    const double height_scale = 1.0 - lapse_rate * orthometric_height_m / temperature0;
    if (temperature <= 0.0 || height_scale <= 0.0 || lapse_rate <= 0.0) {
        return false;
    }
    const double pressure =
        pressure0 * std::pow(height_scale, gravity / (gas_constant_dry * lapse_rate));
    double water_pressure =
        water_pressure0 *
        std::pow(height_scale, (water_vapor_rate + 1.0) * gravity /
                                   (gas_constant_dry * lapse_rate));

    const double celsius = temperature - 273.15;
    const double saturation =
        6.11 * std::pow(temperature / 273.15, -5.3) *
        std::exp(25.2 * celsius / temperature);
    if (water_pressure > saturation) {
        water_pressure = saturation;
    }

    out.hydrostatic_m =
        2.2768 * pressure * 0.001 /
        (1.0 - 0.00266 * std::cos(2.0 * latitude_rad) -
         2.8e-7 * ellipsoidal_height_m);
    out.wet_m = 2.2768 * (1255.0 / temperature + 0.05) * water_pressure * 0.001;
    return std::isfinite(out.hydrostatic_m) && std::isfinite(out.wet_m) &&
           out.hydrostatic_m > 0.0 && out.wet_m > 0.0;
}

bool claslibStandardVerticalTropDelaysAtGrid(const GNSSTime& time,
                                             const ClasGridPoint& point,
                                             ClaslibVerticalTropDelays& out) {
    const double latitude_rad = point.latitude_deg * kDegreesToRadians;
    const double longitude_rad = point.longitude_deg * kDegreesToRadians;
    const double geoid_height_m = claslibEmbeddedGeoidHeight(latitude_rad, longitude_rad);
    return claslibStandardVerticalTropDelays(
        time, latitude_rad, 0.0, geoid_height_m, out);
}

bool claslibStandardVerticalTropDelaysAtReceiver(const GNSSTime& time,
                                                 const Vector3d& receiver_position,
                                                 ClaslibVerticalTropDelays& out,
                                                 double& latitude_rad,
                                                 double& height_m) {
    double longitude_rad = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const double geoid_height_m = claslibEmbeddedGeoidHeight(latitude_rad, longitude_rad);
    return claslibStandardVerticalTropDelays(
        time, latitude_rad, height_m, geoid_height_m, out);
}

double evaluateSubtype12TropDryCorrection(
    const std::map<std::string, std::string>& atmos_tokens,
    double dlat_deg,
    double dlon_deg,
    int trop_type,
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy) {
    double correction_m = 0.0;
    double value = 0.0;
    if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t00_m", value) &&
        std::isfinite(value)) {
        correction_m += value;
    }
    if (trop_type > 0 && useSubtype12LinearTerms(subtype12_value_policy)) {
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t01_m_per_deg", value) &&
            std::isfinite(value)) {
            correction_m += value * dlat_deg;
        }
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t10_m_per_deg", value) &&
            std::isfinite(value)) {
            correction_m += value * dlon_deg;
        }
    }
    if (trop_type > 1 && useSubtype12QuadraticTerms(subtype12_value_policy) &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t11_m_per_deg2", value) &&
        std::isfinite(value)) {
        correction_m += value * dlat_deg * dlon_deg;
    }
    return correction_m;
}

bool claslibTropGridPointNormalizedValues(
    const std::map<std::string, std::string>& atmos_tokens,
    const ClasGridReference& grid_reference,
    size_t residual_index,
    const GNSSTime& time,
    bool subtype12_row,
    int trop_type,
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy,
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy,
    double& normalized_hydrostatic,
    double& normalized_wet) {
    const ClasGridPoint* point =
        findClasGridPoint(grid_reference.network_id, residual_index);
    if (point == nullptr) {
        return false;
    }

    double dry_vertical_m = 0.0;
    double wet_vertical_m = 0.0;
    if (!subtype12_row) {
        double hydro_delta_m = 0.0;
        double wet_delta_m = 0.0;
        if (!sampleAtmosResidualList(
                atmos_tokens,
                "atmos_trop_hs_residuals_m",
                true,
                residual_index,
                residual_sampling_policy,
                hydro_delta_m) ||
            !sampleAtmosResidualList(
                atmos_tokens,
                "atmos_trop_wet_residuals_m",
                true,
                residual_index,
                residual_sampling_policy,
                wet_delta_m)) {
            return false;
        }
        dry_vertical_m = kClasTropHydrostaticRefM + hydro_delta_m;
        wet_vertical_m = kClasTropWetRefM + wet_delta_m;
    } else {
        double wet_offset_m = 0.0;
        double wet_residual_m = 0.0;
        if (!parseAtmosTokenDouble(atmos_tokens, "atmos_trop_offset_m", wet_offset_m) ||
            !std::isfinite(wet_offset_m) ||
            !sampleAtmosResidualList(
                atmos_tokens,
                "atmos_trop_residuals_m",
                true,
                residual_index,
                residual_sampling_policy,
                wet_residual_m)) {
            return false;
        }

        double dlat_deg = 0.0;
        double dlon_deg = 0.0;
        if (const ClasGridPoint* origin = findClasGridPoint(grid_reference.network_id, 0)) {
            dlat_deg = point->latitude_deg - origin->latitude_deg;
            dlon_deg = point->longitude_deg - origin->longitude_deg;
        }
        dry_vertical_m =
            kClasTropHydrostaticRefM +
            evaluateSubtype12TropDryCorrection(
                atmos_tokens, dlat_deg, dlon_deg, trop_type, subtype12_value_policy);
        wet_vertical_m = wet_offset_m + wet_residual_m;
    }

    ClaslibVerticalTropDelays grid_delays;
    if (!claslibStandardVerticalTropDelaysAtGrid(time, *point, grid_delays)) {
        return false;
    }
    normalized_hydrostatic = dry_vertical_m / grid_delays.hydrostatic_m;
    normalized_wet = wet_vertical_m / grid_delays.wet_m;
    return std::isfinite(normalized_hydrostatic) && std::isfinite(normalized_wet);
}

bool claslibTroposphereCorrectionMeters(
    const std::map<std::string, std::string>& atmos_tokens,
    const ClasGridReference& grid_reference,
    const Vector3d& receiver_position,
    const GNSSTime& time,
    double elevation,
    bool subtype12_row,
    int trop_type,
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy,
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy,
    double& correction_m) {
    if (!std::isfinite(elevation) || elevation <= 0.0) {
        return false;
    }

    double normalized_hydrostatic = 0.0;
    double normalized_wet = 0.0;
    if (grid_reference.has_model_interpolation) {
        double hydro_values[4] = {};
        double wet_values[4] = {};
        for (int g = 0; g < 4; ++g) {
            if (!claslibTropGridPointNormalizedValues(
                    atmos_tokens,
                    grid_reference,
                    grid_reference.model_grid_indices[g],
                    time,
                    subtype12_row,
                    trop_type,
                    subtype12_value_policy,
                    residual_sampling_policy,
                    hydro_values[g],
                    wet_values[g])) {
                return false;
            }
        }
        normalized_hydrostatic = applyGridModelInterpolation(grid_reference, hydro_values);
        normalized_wet = applyGridModelInterpolation(grid_reference, wet_values);
    } else if (!claslibTropGridPointNormalizedValues(
                   atmos_tokens,
                   grid_reference,
                   grid_reference.residual_index,
                   time,
                   subtype12_row,
                   trop_type,
                   subtype12_value_policy,
                   residual_sampling_policy,
                   normalized_hydrostatic,
                   normalized_wet)) {
        return false;
    }

    ClaslibVerticalTropDelays receiver_delays;
    double latitude_rad = 0.0;
    double height_m = 0.0;
    if (!claslibStandardVerticalTropDelaysAtReceiver(
            time, receiver_position, receiver_delays, latitude_rad, height_m)) {
        return false;
    }

    const double hydro_mapping =
        models::niellHydrostaticMapping(latitude_rad, height_m, elevation, dayOfYearFromTime(time));
    const double wet_mapping = models::niellWetMapping(latitude_rad, elevation);
    correction_m =
        hydro_mapping * receiver_delays.hydrostatic_m * normalized_hydrostatic +
        wet_mapping * receiver_delays.wet_m * normalized_wet;
    return std::isfinite(correction_m) && correction_m > 0.0;
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

    // Collect candidate grid points for this network
    struct GridCandidate {
        const ClasGridPoint* point = nullptr;
        double distance_sq = 0.0;
    };
    std::vector<GridCandidate> candidates;
    for (const auto& point : clasGridPoints()) {
        if (point.network_id != network_id) continue;
        if (grid_count > 0 && point.grid_no > grid_count) continue;
        const double dlat = receiver_lat_deg - point.latitude_deg;
        const double dlon = receiver_lon_deg - point.longitude_deg;
        candidates.push_back({&point, dlat * dlat + dlon * dlon});
    }
    if (candidates.empty()) return false;

    // Sort by distance
    std::sort(candidates.begin(), candidates.end(),
              [](const GridCandidate& a, const GridCandidate& b) {
                  return a.distance_sq < b.distance_sq;
              });

    const ClasGridPoint* nearest = candidates[0].point;

    reference.has_bilinear = false;
    reference.has_model_interpolation = false;

    const ClasGridPoint* model_points[4] = {nearest, nullptr, nullptr, nullptr};
    int model_count = 1;
    for (size_t i = 1; i < candidates.size(); ++i) {
        model_points[1] = candidates[i].point;
        model_count = 2;
        break;
    }
    if (model_count == 2) {
        const double dd12_lat =
            model_points[1]->latitude_deg - model_points[0]->latitude_deg;
        const double dd12_lon =
            model_points[1]->longitude_deg - model_points[0]->longitude_deg;
        for (size_t i = 1; i < candidates.size(); ++i) {
            const ClasGridPoint* candidate = candidates[i].point;
            if (candidate->grid_no == model_points[1]->grid_no) continue;
            const double dd13_lat =
                candidate->latitude_deg - model_points[0]->latitude_deg;
            const double dd13_lon =
                candidate->longitude_deg - model_points[0]->longitude_deg;
            const double denom = norm2(dd12_lat, dd12_lon) * norm2(dd13_lat, dd13_lon);
            if (denom <= 1e-12) continue;
            if (std::abs(dot2(dd12_lat, dd12_lon, dd13_lat, dd13_lon) / denom) < 1.0 - 1e-9) {
                model_points[2] = candidate;
                model_count = 3;
                break;
            }
        }
    }
    if (model_count == 3) {
        const double dd12_lat =
            model_points[1]->latitude_deg - model_points[0]->latitude_deg;
        const double dd12_lon =
            model_points[1]->longitude_deg - model_points[0]->longitude_deg;
        const double dd13_lat =
            model_points[2]->latitude_deg - model_points[0]->latitude_deg;
        const double dd13_lon =
            model_points[2]->longitude_deg - model_points[0]->longitude_deg;
        for (size_t i = 1; i < candidates.size(); ++i) {
            const ClasGridPoint* candidate = candidates[i].point;
            if (candidate->grid_no == model_points[1]->grid_no ||
                candidate->grid_no == model_points[2]->grid_no) {
                continue;
            }
            const double dd14_lat =
                candidate->latitude_deg - model_points[0]->latitude_deg;
            const double dd14_lon =
                candidate->longitude_deg - model_points[0]->longitude_deg;
            const double denom12 =
                norm2(dd12_lat, dd12_lon) * norm2(dd14_lat, dd14_lon);
            const double denom13 =
                norm2(dd13_lat, dd13_lon) * norm2(dd14_lat, dd14_lon);
            if (denom12 <= 1e-12 || denom13 <= 1e-12) continue;
            const bool not_collinear_12 =
                std::abs(dot2(dd12_lat, dd12_lon, dd14_lat, dd14_lon) / denom12) < 1.0 - 1e-9;
            const bool not_collinear_13 =
                std::abs(dot2(dd13_lat, dd13_lon, dd14_lat, dd14_lon) / denom13) < 1.0 - 1e-9;
            const bool forms_rectangle =
                (std::abs(model_points[1]->longitude_deg - candidate->longitude_deg) < 0.04 &&
                 std::abs(model_points[2]->latitude_deg - candidate->latitude_deg) < 0.04) ||
                (std::abs(model_points[1]->latitude_deg - candidate->latitude_deg) < 0.04 &&
                 std::abs(model_points[2]->longitude_deg - candidate->longitude_deg) < 0.04);
            if (not_collinear_12 && not_collinear_13 && forms_rectangle) {
                model_points[3] = candidate;
                model_count = 4;
                break;
            }
        }
    }

    if (model_count == 4) {
        const ClasGridPoint* network_origin = nullptr;
        for (const auto& point : clasGridPoints()) {
            if (point.network_id == nearest->network_id && point.grid_no == 1) {
                network_origin = &point;
                break;
            }
        }
        double umat[16] = {};
        for (int i = 0; i < 4; ++i) {
            const double local_dlat =
                model_points[i]->latitude_deg - model_points[0]->latitude_deg;
            const double local_dlon =
                model_points[i]->longitude_deg - model_points[0]->longitude_deg;
            umat[i * 4 + 0] = local_dlat;
            umat[i * 4 + 1] = local_dlon;
            umat[i * 4 + 2] = local_dlat * local_dlon;
            umat[i * 4 + 3] = 1.0;
            reference.model_grid_indices[i] =
                static_cast<size_t>(std::max(model_points[i]->grid_no - 1, 0));
            if (network_origin != nullptr) {
                reference.model_grid_dlat_deg[i] =
                    model_points[i]->latitude_deg - network_origin->latitude_deg;
                reference.model_grid_dlon_deg[i] =
                    model_points[i]->longitude_deg - network_origin->longitude_deg;
            } else {
                reference.model_grid_dlat_deg[i] = local_dlat;
                reference.model_grid_dlon_deg[i] = local_dlon;
            }
            reference.bilinear_grid_indices[i] = reference.model_grid_indices[i];
        }
        if (invert4x4(umat, reference.model_gmat)) {
            const double ref_dlat =
                receiver_lat_deg - model_points[0]->latitude_deg;
            const double ref_dlon =
                receiver_lon_deg - model_points[0]->longitude_deg;
            reference.model_emat[0] = ref_dlat;
            reference.model_emat[1] = ref_dlon;
            reference.model_emat[2] = ref_dlat * ref_dlon;
            reference.model_emat[3] = 1.0;
            reference.dlat_deg = ref_dlat;
            reference.dlon_deg = ref_dlon;
            reference.has_model_interpolation = true;
            reference.has_bilinear = true;
        }
    }

    if (!reference.has_model_interpolation) {
        reference.dlat_deg = receiver_lat_deg - nearest->latitude_deg;
        reference.dlon_deg = receiver_lon_deg - nearest->longitude_deg;
    }
    reference.residual_index = static_cast<size_t>(std::max(nearest->grid_no - 1, 0));
    reference.network_id = nearest->network_id;
    reference.grid_no = nearest->grid_no;
    return true;
}

double atmosphericTroposphereCorrectionMeters(
    const std::map<std::string, std::string>& atmos_tokens,
    const Vector3d& receiver_position,
    const GNSSTime& time,
    double elevation,
    ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy value_policy,
    ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy,
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy,
    bool use_claslib_grid_composition) {
    double correction_m = 0.0;
    bool have_correction = false;
    ClasGridReference grid_reference;
    const bool have_grid_reference =
        resolveClasGridReference(atmos_tokens, receiver_position, grid_reference);
    const bool subtype12_row = isSubtype12TropRow(atmos_tokens);
    int trop_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_trop_type", trop_type);

    if (use_claslib_grid_composition &&
        have_grid_reference &&
        value_policy == ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy::FULL_COMPOSED &&
        trop_type > 0) {
        const bool has_legacy_hs_wet_grid =
            atmos_tokens.find("atmos_trop_hs_residuals_m") != atmos_tokens.end() &&
            atmos_tokens.find("atmos_trop_wet_residuals_m") != atmos_tokens.end();
        const bool has_subtype12_total_wet_grid =
            subtype12_row &&
            atmos_tokens.find("atmos_trop_offset_m") != atmos_tokens.end() &&
            atmos_tokens.find("atmos_trop_residuals_m") != atmos_tokens.end();
        if (has_legacy_hs_wet_grid || has_subtype12_total_wet_grid) {
            double claslib_correction_m = 0.0;
            if (claslibTroposphereCorrectionMeters(
                    atmos_tokens,
                    grid_reference,
                    receiver_position,
                    time,
                    elevation,
                    subtype12_row,
                    trop_type,
                    subtype12_value_policy,
                    residual_sampling_policy,
                    claslib_correction_m)) {
                return claslib_correction_m;
            }
        }
    }

    double value = 0.0;
    if (usePolynomialTerms(value_policy) &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t00_m", value) && std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    }
    if (usePolynomialTerms(value_policy) && have_grid_reference && trop_type > 0 &&
        (!subtype12_row || useSubtype12LinearTerms(subtype12_value_policy))) {
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t01_m_per_deg", value) && std::isfinite(value)) {
            correction_m += value * grid_reference.dlat_deg;
            have_correction = true;
        }
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t10_m_per_deg", value) && std::isfinite(value)) {
            correction_m += value * grid_reference.dlon_deg;
            have_correction = true;
        }
    }
    if (usePolynomialTerms(value_policy) && have_grid_reference && trop_type > 1 &&
        (!subtype12_row || useSubtype12QuadraticTerms(subtype12_value_policy)) &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_trop_t11_m_per_deg2", value) &&
        std::isfinite(value)) {
        correction_m += value * grid_reference.dlat_deg * grid_reference.dlon_deg;
        have_correction = true;
    }
    if (useResidualTerms(value_policy) &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_trop_offset_m", value) && std::isfinite(value)) {
        correction_m += value;
        have_correction = true;
    }
    if (useResidualTerms(value_policy) && have_grid_reference &&
        sampleAtmosResidualGridModel(
            atmos_tokens,
            "atmos_trop_residuals_m",
            grid_reference,
            residual_sampling_policy,
            value)) {
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
        auto sampleTropResidual = [&](const std::string& key) -> double {
            if (!useResidualTerms(value_policy)) return 0.0;
            if (have_grid_reference && grid_reference.has_model_interpolation) {
                double grid_values[4] = {};
                bool ok = true;
                for (int g = 0; g < 4; ++g) {
                    if (!sampleAtmosResidualList(
                            atmos_tokens,
                            key,
                            true,
                            grid_reference.model_grid_indices[g],
                            residual_sampling_policy,
                            grid_values[g])) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    have_grid_trop = true;
                    return applyGridModelInterpolation(grid_reference, grid_values);
                }
            }
            double v = 0.0;
            if (sampleAtmosResidualList(atmos_tokens, key, have_grid_reference,
                    grid_reference.residual_index, residual_sampling_policy, v)) {
                have_grid_trop = true;
                return v;
            }
            return 0.0;
        };
        hydro_delta_m = sampleTropResidual("atmos_trop_hs_residuals_m");
        wet_delta_m = sampleTropResidual("atmos_trop_wet_residuals_m");
        if (have_grid_trop) {
            correction_m += hydro_mapping * (2.3 + hydro_delta_m) + wet_mapping * (0.252 + wet_delta_m);
            have_correction = true;
        }
    }

    return have_correction ? correction_m : 0.0;
}

double atmosphericStecTecu(const std::map<std::string, std::string>& atmos_tokens,
                           const SatelliteId& satellite,
                           const Vector3d& receiver_position,
                           ppp_shared::PPPConfig::ClasExpandedValueConstructionPolicy value_policy,
                           ppp_shared::PPPConfig::ClasSubtype12ValueConstructionPolicy subtype12_value_policy,
                           ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy) {
    const std::string suffix = ":" + satellite.toString();
    double stec_tecu = 0.0;
    bool have_correction = false;
    ClasGridReference grid_reference;
    const bool have_grid_reference =
        resolveClasGridReference(atmos_tokens, receiver_position, grid_reference);
    const bool subtype12_row = isSubtype12StecRow(atmos_tokens, satellite);
    int stec_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_stec_type" + suffix, stec_type);

    // STEC polynomial: c00 + c01*dlat + c10*dlon + c11*dlat*dlon [+ c02*dlat² + c20*dlon²]
    // When 4-grid bilinear is available, evaluate the polynomial at each grid
    // point's position and bilinear-interpolate the results (CLASLIB approach).
    auto evaluateStecPoly = [&](double dlat, double dlon) -> double {
        double val = 0.0;
        double term = 0.0;
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c00_tecu" + suffix, term) && std::isfinite(term))
            val += term;
        if (stec_type > 0 && (!subtype12_row || useSubtype12LinearTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c01_tecu_per_deg" + suffix, term) && std::isfinite(term))
                val += term * dlat;
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c10_tecu_per_deg" + suffix, term) && std::isfinite(term))
                val += term * dlon;
        }
        if (stec_type > 1 && (!subtype12_row || useSubtype12QuadraticTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c11_tecu_per_deg2" + suffix, term) && std::isfinite(term))
                val += term * dlat * dlon;
        }
        if (stec_type > 2 && (!subtype12_row || useSubtype12QuadraticTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c02_tecu_per_deg2" + suffix, term) && std::isfinite(term))
                val += term * dlat * dlat;
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c20_tecu_per_deg2" + suffix, term) && std::isfinite(term))
                val += term * dlon * dlon;
        }
        return val;
    };

    double value = 0.0;
    if (usePolynomialTerms(value_policy) &&
        parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c00_tecu" + suffix, value) &&
        std::isfinite(value)) {
        if (have_grid_reference && grid_reference.has_model_interpolation) {
            double grid_stecs[4] = {};
            for (int g = 0; g < 4; ++g) {
                grid_stecs[g] = evaluateStecPoly(
                    grid_reference.model_grid_dlat_deg[g],
                    grid_reference.model_grid_dlon_deg[g]);
                double grid_residual = 0.0;
                if (useResidualTerms(value_policy) &&
                    sampleAtmosResidualList(
                        atmos_tokens,
                        "atmos_stec_residuals_tecu" + suffix,
                        true,
                        grid_reference.model_grid_indices[g],
                        residual_sampling_policy,
                        grid_residual)) {
                    grid_stecs[g] += grid_residual;
                }
            }
            stec_tecu = applyGridModelInterpolation(grid_reference, grid_stecs);
        } else {
            stec_tecu = evaluateStecPoly(
                have_grid_reference ? grid_reference.dlat_deg : 0.0,
                have_grid_reference ? grid_reference.dlon_deg : 0.0);
        }
        have_correction = true;
    }
    // Residuals: for bilinear mode, residuals are already included in the
    // per-grid evaluation above.  For nearest-grid mode, add the residual.
    if (!grid_reference.has_bilinear && useResidualTerms(value_policy) &&
        sampleAtmosResidualList(
            atmos_tokens,
            "atmos_stec_residuals_tecu" + suffix,
            have_grid_reference,
            grid_reference.residual_index,
            residual_sampling_policy,
            value)) {
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
