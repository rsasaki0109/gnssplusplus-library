#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace libgnss {
namespace ppp_atmosphere {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kMaxClasGridDistanceM = 120000.0;
constexpr double kNearestGridThresholdM = 1000.0;

struct ClasGridPoint {
    int network_id;
    int grid_no;
    double latitude_deg;
    double longitude_deg;
    double height_m;
};

struct ClasGridCandidate {
    const ClasGridPoint* point = nullptr;
    double distance_m = std::numeric_limits<double>::infinity();
    size_t order = 0;
};

const std::array<ClasGridPoint, 212>& clasGridPoints() {
    static const std::array<ClasGridPoint, 212> kClasGridPoints = {{
#include "clas_grid_points.inc"
    }};
    return kClasGridPoints;
}

const ClasGridPoint* clasNetworkPolynomialOrigin(int network_id) {
    for (const auto& point : clasGridPoints()) {
        if (point.network_id == network_id && point.grid_no == 1) {
            return &point;
        }
    }
    return nullptr;
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

bool envFlagEnabled(const char* name) {
    const char* raw = std::getenv(name);
    return raw != nullptr && *raw != '\0' && std::string(raw) != "0";
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

bool requireClasIndexedStecResiduals() {
    return envFlagEnabled("GNSS_PPP_CLAS_REQUIRE_INDEXED_STEC_RESIDUAL");
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

bool hasFiniteIndexedResidual(const std::map<std::string, std::string>& atmos_tokens,
                              const std::string& key,
                              size_t residual_index) {
    double value = 0.0;
    return parseAtmosListValueAtIndex(atmos_tokens, key, residual_index, value) &&
           std::isfinite(value);
}

bool isNonCollinear(const ClasGridPoint& origin,
                    const ClasGridPoint& a,
                    const ClasGridPoint& b) {
    const double ax = (a.latitude_deg - origin.latitude_deg) * kDegreesToRadians;
    const double ay = (a.longitude_deg - origin.longitude_deg) * kDegreesToRadians;
    const double bx = (b.latitude_deg - origin.latitude_deg) * kDegreesToRadians;
    const double by = (b.longitude_deg - origin.longitude_deg) * kDegreesToRadians;
    const double an = std::hypot(ax, ay);
    const double bn = std::hypot(bx, by);
    if (an <= 0.0 || bn <= 0.0) {
        return false;
    }
    const double cos_angle = (ax * bx + ay * by) / (an * bn);
    return std::abs(cos_angle) < 1.0 - 1e-12;
}

std::vector<const ClasGridCandidate*> selectClasInterpolationGrids(
    const std::vector<ClasGridCandidate>& candidates) {
    std::vector<const ClasGridCandidate*> selected;
    if (candidates.empty()) {
        return selected;
    }

    selected.push_back(&candidates.front());
    if (candidates.size() < 3 ||
        candidates.front().point->network_id > 12 ||
        candidates.front().distance_m <= kNearestGridThresholdM) {
        return selected;
    }

    size_t first_index = 0;
    if (candidates.size() > 2 &&
        candidates[1].point->network_id == candidates[2].point->network_id &&
        candidates[0].point->network_id != candidates[1].point->network_id) {
        first_index = 1;
    }
    const ClasGridCandidate* first = &candidates[first_index];
    selected[0] = first;

    const ClasGridCandidate* second = nullptr;
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (i != first_index &&
            candidates[i].point->network_id == first->point->network_id) {
            second = &candidates[i];
            break;
        }
    }
    if (second == nullptr) {
        return selected;
    }
    selected.push_back(second);

    const ClasGridCandidate* third = nullptr;
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (i == first_index ||
            candidates[i].point->grid_no == second->point->grid_no ||
            candidates[i].point->network_id != first->point->network_id) {
            continue;
        }
        if (isNonCollinear(*first->point, *second->point, *candidates[i].point)) {
            third = &candidates[i];
            break;
        }
    }
    if (third == nullptr) {
        selected.resize(1);
        return selected;
    }
    selected.push_back(third);

    for (size_t i = 1; i < candidates.size(); ++i) {
        if (i == first_index ||
            candidates[i].point->grid_no == second->point->grid_no ||
            candidates[i].point->grid_no == third->point->grid_no ||
            candidates[i].point->network_id != first->point->network_id) {
            continue;
        }
        const ClasGridPoint& fourth = *candidates[i].point;
        if (!isNonCollinear(*first->point, *second->point, fourth) ||
            !isNonCollinear(*first->point, *third->point, fourth)) {
            continue;
        }
        const bool closes_rectangle =
            (std::abs(second->point->longitude_deg - fourth.longitude_deg) < 0.04 &&
             std::abs(third->point->latitude_deg - fourth.latitude_deg) < 0.04) ||
            (std::abs(second->point->latitude_deg - fourth.latitude_deg) < 0.04 &&
             std::abs(third->point->longitude_deg - fourth.longitude_deg) < 0.04);
        if (closes_rectangle) {
            selected.push_back(&candidates[i]);
            break;
        }
    }

    return selected;
}

void calculateInverseDistanceWeights(
    const std::vector<const ClasGridCandidate*>& selected,
    std::array<double, 4>& weights) {
    weights.fill(0.0);
    double inverse_distance_sum = 0.0;
    for (const auto* candidate : selected) {
        inverse_distance_sum += 1.0 / candidate->distance_m;
    }
    if (inverse_distance_sum <= 0.0) {
        if (!selected.empty()) {
            weights[0] = 1.0;
        }
        return;
    }
    for (size_t i = 0; i < selected.size() && i < weights.size(); ++i) {
        weights[i] = 1.0 / (selected[i]->distance_m * inverse_distance_sum);
    }
}

bool calculateThreePointWeights(const std::vector<const ClasGridCandidate*>& selected,
                                double user_dlat_deg,
                                double user_dlon_deg,
                                std::array<double, 4>& weights) {
    if (selected.size() != 3) {
        return false;
    }
    const double dlat1 =
        selected[1]->point->latitude_deg - selected[0]->point->latitude_deg;
    const double dlon1 =
        selected[1]->point->longitude_deg - selected[0]->point->longitude_deg;
    const double dlat2 =
        selected[2]->point->latitude_deg - selected[0]->point->latitude_deg;
    const double dlon2 =
        selected[2]->point->longitude_deg - selected[0]->point->longitude_deg;
    const double determinant = dlat1 * dlon2 - dlat2 * dlon1;
    if (std::abs(determinant) < 1e-12) {
        return false;
    }

    weights.fill(0.0);
    weights[1] = (user_dlat_deg * dlon2 - dlat2 * user_dlon_deg) / determinant;
    weights[2] = (dlat1 * user_dlon_deg - user_dlat_deg * dlon1) / determinant;
    weights[0] = 1.0 - weights[1] - weights[2];
    return true;
}

bool solveLinearSystem4(double a[4][5], std::array<double, 4>& x) {
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double pivot_abs = std::abs(a[col][col]);
        for (int row = col + 1; row < 4; ++row) {
            const double candidate_abs = std::abs(a[row][col]);
            if (candidate_abs > pivot_abs) {
                pivot_abs = candidate_abs;
                pivot = row;
            }
        }
        if (pivot_abs < 1e-12) {
            return false;
        }
        if (pivot != col) {
            for (int c = col; c < 5; ++c) {
                std::swap(a[col][c], a[pivot][c]);
            }
        }
        const double divisor = a[col][col];
        for (int c = col; c < 5; ++c) {
            a[col][c] /= divisor;
        }
        for (int row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            if (factor == 0.0) {
                continue;
            }
            for (int c = col; c < 5; ++c) {
                a[row][c] -= factor * a[col][c];
            }
        }
    }

    for (int i = 0; i < 4; ++i) {
        x[i] = a[i][4];
    }
    return true;
}

bool calculateFourPointWeights(const std::vector<const ClasGridCandidate*>& selected,
                               double user_dlat_deg,
                               double user_dlon_deg,
                               std::array<double, 4>& weights) {
    if (selected.size() != 4) {
        return false;
    }
    double system[4][5] = {};
    for (int grid = 0; grid < 4; ++grid) {
        const double dlat =
            selected[grid]->point->latitude_deg - selected[0]->point->latitude_deg;
        const double dlon =
            selected[grid]->point->longitude_deg - selected[0]->point->longitude_deg;
        system[0][grid] = dlat;
        system[1][grid] = dlon;
        system[2][grid] = dlat * dlon;
        system[3][grid] = 1.0;
    }
    system[0][4] = user_dlat_deg;
    system[1][4] = user_dlon_deg;
    system[2][4] = user_dlat_deg * user_dlon_deg;
    system[3][4] = 1.0;

    weights.fill(0.0);
    return solveLinearSystem4(system, weights);
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
    reference = ClasGridReference{};

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

    std::vector<ClasGridCandidate> candidates;
    size_t order = 0;
    for (const auto& point : clasGridPoints()) {
        if (point.network_id != network_id) continue;
        if (grid_count > 0 && point.grid_no > grid_count) continue;
        const double dlat_m =
            constants::WGS84_A * (receiver_lat_rad - point.latitude_deg * kDegreesToRadians);
        const double dlon_m =
            constants::WGS84_A * (receiver_lon_rad - point.longitude_deg * kDegreesToRadians) *
            std::cos(receiver_lat_rad);
        const double distance_m = std::max(std::hypot(dlat_m, dlon_m), 1.0);
        if (distance_m > kMaxClasGridDistanceM) {
            continue;
        }
        candidates.push_back({&point, distance_m, order});
        ++order;
    }
    if (candidates.empty()) return false;

    std::sort(candidates.begin(), candidates.end(),
              [](const ClasGridCandidate& a, const ClasGridCandidate& b) {
                  if (a.distance_m != b.distance_m) {
                      return a.distance_m < b.distance_m;
                  }
                  return a.order < b.order;
              });

    std::vector<const ClasGridCandidate*> selected =
        selectClasInterpolationGrids(candidates);
    if (selected.empty()) {
        return false;
    }

    const double user_dlat_deg =
        receiver_lat_deg - selected[0]->point->latitude_deg;
    const double user_dlon_deg =
        receiver_lon_deg - selected[0]->point->longitude_deg;
    std::array<double, 4> weights{};
    if (selected.size() == 4) {
        if (!calculateFourPointWeights(selected, user_dlat_deg, user_dlon_deg, weights)) {
            selected.resize(1);
            weights.fill(0.0);
            weights[0] = 1.0;
        }
    } else if (selected.size() == 3) {
        if (!calculateThreePointWeights(selected, user_dlat_deg, user_dlon_deg, weights)) {
            calculateInverseDistanceWeights(selected, weights);
        }
    } else {
        calculateInverseDistanceWeights(selected, weights);
    }

    reference.dlat_deg = user_dlat_deg;
    reference.dlon_deg = user_dlon_deg;
    reference.residual_index = static_cast<size_t>(
        std::max(selected[0]->point->grid_no - 1, 0));
    reference.network_id = selected[0]->point->network_id;
    reference.grid_no = selected[0]->point->grid_no;
    reference.interpolation_grid_count =
        static_cast<int>(std::min<size_t>(selected.size(), 4));
    reference.has_bilinear = reference.interpolation_grid_count == 4;
    const ClasGridPoint* polynomial_origin =
        clasNetworkPolynomialOrigin(reference.network_id);
    if (polynomial_origin == nullptr) {
        polynomial_origin = selected[0]->point;
    }

    for (int i = 0; i < reference.interpolation_grid_count; ++i) {
        reference.interpolation_weights[i] = weights[i];
        reference.interpolation_grid_indices[i] = static_cast<size_t>(
            std::max(selected[i]->point->grid_no - 1, 0));
        reference.interpolation_grid_dlat_deg[i] =
            selected[i]->point->latitude_deg - selected[0]->point->latitude_deg;
        reference.interpolation_grid_dlon_deg[i] =
            selected[i]->point->longitude_deg - selected[0]->point->longitude_deg;
        reference.interpolation_poly_dlat_deg[i] =
            selected[i]->point->latitude_deg - polynomial_origin->latitude_deg;
        reference.interpolation_poly_dlon_deg[i] =
            selected[i]->point->longitude_deg - polynomial_origin->longitude_deg;
        reference.bilinear_weights[i] = reference.interpolation_weights[i];
        reference.bilinear_grid_indices[i] = reference.interpolation_grid_indices[i];
    }

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
            if (have_grid_reference && grid_reference.interpolation_grid_count > 1) {
                double interpolated_val = 0.0;
                bool ok = true;
                for (int g = 0; g < grid_reference.interpolation_grid_count; ++g) {
                    double gv = 0.0;
                    if (sampleAtmosResidualList(atmos_tokens, key, true,
                            grid_reference.interpolation_grid_indices[g],
                            residual_sampling_policy, gv)) {
                        interpolated_val += grid_reference.interpolation_weights[g] * gv;
                    } else { ok = false; break; }
                }
                if (ok) { have_grid_trop = true; return interpolated_val; }
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
    int grid_count = 0;
    parseAtmosTokenInt(atmos_tokens, "atmos_grid_count", grid_count);
    const std::string residual_key = "atmos_stec_residuals_tecu" + suffix;
    if (requireClasIndexedStecResiduals() && useResidualTerms(value_policy) &&
        grid_count > 0) {
        if (!have_grid_reference) {
            return 0.0;
        }
        if (grid_reference.interpolation_grid_count > 1) {
            for (int g = 0; g < grid_reference.interpolation_grid_count; ++g) {
                if (!hasFiniteIndexedResidual(
                        atmos_tokens, residual_key, grid_reference.interpolation_grid_indices[g])) {
                    return 0.0;
                }
            }
        } else if (!hasFiniteIndexedResidual(
                       atmos_tokens, residual_key, grid_reference.residual_index)) {
            return 0.0;
        }
    }

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

    auto haveStecPolynomialTerms = [&]() -> bool {
        double term = 0.0;
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c00_tecu" + suffix, term) &&
            std::isfinite(term)) {
            return true;
        }
        if (stec_type > 0) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c01_tecu_per_deg" + suffix, term) &&
                std::isfinite(term)) {
                return true;
            }
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c10_tecu_per_deg" + suffix, term) &&
                std::isfinite(term)) {
                return true;
            }
        }
        if (stec_type > 1 &&
            parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c11_tecu_per_deg2" + suffix, term) &&
            std::isfinite(term)) {
            return true;
        }
        if (stec_type > 2) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c02_tecu_per_deg2" + suffix, term) &&
                std::isfinite(term)) {
                return true;
            }
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c20_tecu_per_deg2" + suffix, term) &&
                std::isfinite(term)) {
                return true;
            }
        }
        return false;
    };

    const bool have_polynomial_terms =
        usePolynomialTerms(value_policy) && haveStecPolynomialTerms();
    const bool want_residual_terms = useResidualTerms(value_policy);

    bool used_grid_interpolation = false;
    if (have_grid_reference && grid_reference.interpolation_grid_count > 1 &&
        (have_polynomial_terms || want_residual_terms)) {
        double interpolated_stec = 0.0;
        bool ok = true;
        bool any_value = false;
        for (int g = 0; g < grid_reference.interpolation_grid_count; ++g) {
            double grid_stec = 0.0;
            bool have_grid_value = false;
            if (have_polynomial_terms) {
                grid_stec += evaluateStecPoly(
                    grid_reference.interpolation_poly_dlat_deg[g],
                    grid_reference.interpolation_poly_dlon_deg[g]);
                have_grid_value = true;
            }
            double grid_residual = 0.0;
            if (want_residual_terms &&
                sampleAtmosResidualList(atmos_tokens,
                    residual_key, true,
                    grid_reference.interpolation_grid_indices[g],
                    residual_sampling_policy, grid_residual)) {
                grid_stec += grid_residual;
                have_grid_value = true;
            } else if (!have_polynomial_terms) {
                ok = false;
                break;
            }
            if (!have_grid_value) {
                ok = false;
                break;
            }
            interpolated_stec += grid_reference.interpolation_weights[g] * grid_stec;
            any_value = true;
        }
        if (ok && any_value) {
            stec_tecu = interpolated_stec;
            have_correction = true;
            used_grid_interpolation = true;
        }
    }

    if (!used_grid_interpolation) {
        if (have_polynomial_terms) {
            stec_tecu = evaluateStecPoly(
                have_grid_reference ? grid_reference.interpolation_poly_dlat_deg[0] : 0.0,
                have_grid_reference ? grid_reference.interpolation_poly_dlon_deg[0] : 0.0);
            have_correction = true;
        }

        double value = 0.0;
        if (want_residual_terms &&
            sampleAtmosResidualList(
                atmos_tokens,
                residual_key,
                have_grid_reference,
                grid_reference.residual_index,
                residual_sampling_policy,
                value)) {
            stec_tecu += value;
            have_correction = true;
        }
    }

    return have_correction ? stec_tecu : 0.0;
}

double ionosphereDelayMetersFromTecu(SignalType signal,
                                     const Ephemeris* eph,
                                     double stec_tecu) {
    double frequency_hz = 0.0;
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::QZS_L1CA:
        case SignalType::GAL_E1:
            frequency_hz = constants::GPS_L1_FREQ;
            break;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
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
