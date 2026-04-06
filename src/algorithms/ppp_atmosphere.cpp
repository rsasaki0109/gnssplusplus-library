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

    // Try to find 4 surrounding grid points for bilinear interpolation
    // (CLASLIB approach: find grids that form a rectangle around the user)
    const ClasGridPoint* grid_sw = nullptr;  // south-west (lat<user, lon<user)
    const ClasGridPoint* grid_se = nullptr;  // south-east (lat<user, lon>user)
    const ClasGridPoint* grid_nw = nullptr;  // north-west (lat>user, lon<user)
    const ClasGridPoint* grid_ne = nullptr;  // north-east (lat>user, lon>user)
    for (const auto& cand : candidates) {
        const double dlat = cand.point->latitude_deg - receiver_lat_deg;
        const double dlon = cand.point->longitude_deg - receiver_lon_deg;
        if (dlat <= 0 && dlon <= 0 && !grid_sw) grid_sw = cand.point;
        if (dlat <= 0 && dlon > 0 && !grid_se) grid_se = cand.point;
        if (dlat > 0 && dlon <= 0 && !grid_nw) grid_nw = cand.point;
        if (dlat > 0 && dlon > 0 && !grid_ne) grid_ne = cand.point;
    }

    reference.has_bilinear = false;
    if (grid_sw && grid_se && grid_nw && grid_ne) {
        // Bilinear interpolation weights
        const double lat_range = grid_nw->latitude_deg - grid_sw->latitude_deg;
        const double lon_range = grid_se->longitude_deg - grid_sw->longitude_deg;
        // Only use bilinear if the surrounding rectangle is reasonable
        // (< 1 degree ≈ 100km per side).
        if (lat_range > 0.01 && lon_range > 0.01 &&
            lat_range < 0.7 && lon_range < 0.7) {
            const double t = (receiver_lat_deg - grid_sw->latitude_deg) / lat_range;
            const double u = (receiver_lon_deg - grid_sw->longitude_deg) / lon_range;
            reference.bilinear_weights[0] = (1 - t) * (1 - u);  // SW
            reference.bilinear_weights[1] = (1 - t) * u;        // SE
            reference.bilinear_weights[2] = t * (1 - u);        // NW
            reference.bilinear_weights[3] = t * u;              // NE
            reference.bilinear_grid_indices[0] =
                static_cast<size_t>(std::max(grid_sw->grid_no - 1, 0));
            reference.bilinear_grid_indices[1] =
                static_cast<size_t>(std::max(grid_se->grid_no - 1, 0));
            reference.bilinear_grid_indices[2] =
                static_cast<size_t>(std::max(grid_nw->grid_no - 1, 0));
            reference.bilinear_grid_indices[3] =
                static_cast<size_t>(std::max(grid_ne->grid_no - 1, 0));
            // dlat/dlon for polynomial: use SW grid as reference (CLASLIB convention)
            reference.dlat_deg = receiver_lat_deg - grid_sw->latitude_deg;
            reference.dlon_deg = receiver_lon_deg - grid_sw->longitude_deg;
            reference.has_bilinear = true;
        }
    }

    if (!reference.has_bilinear) {
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
    ppp_shared::PPPConfig::ClasExpandedResidualSamplingPolicy residual_sampling_policy) {
    double correction_m = 0.0;
    bool have_correction = false;
    ClasGridReference grid_reference;
    const bool have_grid_reference =
        resolveClasGridReference(atmos_tokens, receiver_position, grid_reference);
    const bool subtype12_row = isSubtype12TropRow(atmos_tokens);
    int trop_type = -1;
    parseAtmosTokenInt(atmos_tokens, "atmos_trop_type", trop_type);

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
    if (useResidualTerms(value_policy) &&
        sampleAtmosResidualList(
            atmos_tokens,
            "atmos_trop_residuals_m",
            have_grid_reference,
            grid_reference.residual_index,
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
            if (have_grid_reference && grid_reference.has_bilinear) {
                double bilinear_val = 0.0;
                bool ok = true;
                for (int g = 0; g < 4; ++g) {
                    double gv = 0.0;
                    if (sampleAtmosResidualList(atmos_tokens, key, true,
                            grid_reference.bilinear_grid_indices[g],
                            residual_sampling_policy, gv)) {
                        bilinear_val += grid_reference.bilinear_weights[g] * gv;
                    } else { ok = false; break; }
                }
                if (ok) { have_grid_trop = true; return bilinear_val; }
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
        if (have_grid_reference && grid_reference.has_bilinear) {
            // CLASLIB approach: evaluate polynomial at each of 4 grid points,
            // add per-grid residual, then bilinear interpolate.
            // Grid positions relative to nearest (SW) grid:
            // SW=(0,0), SE=(0,lon_range), NW=(lat_range,0), NE=(lat_range,lon_range)
            // But polynomial reference is nearest grid, so grid offsets are:
            const double lat_range = grid_reference.dlat_deg /
                (grid_reference.bilinear_weights[2] + grid_reference.bilinear_weights[3] > 0.01
                     ? grid_reference.dlat_deg / (grid_reference.bilinear_weights[2] + grid_reference.bilinear_weights[3])
                     : 1.0);
            // Simpler: use the actual grid positions stored in weights
            // t = dlat/lat_range, u = dlon/lon_range
            // lat_range = dlat / t, lon_range = dlon / u
            const double t = grid_reference.bilinear_weights[2] + grid_reference.bilinear_weights[3];
            const double u = grid_reference.bilinear_weights[1] + grid_reference.bilinear_weights[3];
            const double total_lat = (t > 0.01) ? grid_reference.dlat_deg / t : grid_reference.dlat_deg;
            const double total_lon = (u > 0.01) ? grid_reference.dlon_deg / u : grid_reference.dlon_deg;
            const double grid_dlat[4] = {0, 0, total_lat, total_lat};
            const double grid_dlon[4] = {0, total_lon, 0, total_lon};

            double bilinear_stec = 0.0;
            for (int g = 0; g < 4; ++g) {
                double grid_stec = evaluateStecPoly(grid_dlat[g], grid_dlon[g]);
                // Add per-grid residual if available
                double grid_residual = 0.0;
                if (useResidualTerms(value_policy) &&
                    sampleAtmosResidualList(atmos_tokens,
                        "atmos_stec_residuals_tecu" + suffix, true,
                        grid_reference.bilinear_grid_indices[g],
                        residual_sampling_policy, grid_residual)) {
                    grid_stec += grid_residual;
                }
                bilinear_stec += grid_reference.bilinear_weights[g] * grid_stec;
            }
            stec_tecu = bilinear_stec;
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
