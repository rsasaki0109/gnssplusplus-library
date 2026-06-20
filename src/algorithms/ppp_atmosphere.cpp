#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace libgnss {
namespace ppp_atmosphere {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kClasMaxGridDistanceM = 120000.0;
constexpr double kClasNearestGridThresholdM = 1000.0;
constexpr int kClasMultiGridNetworkMax = 12;

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

std::set<int> parseAtmosValidGridNumbers(
    const std::map<std::string, std::string>& atmos_tokens) {
    std::set<int> valid_grids;
    const auto it = atmos_tokens.find("atmos_valid_grids");
    if (it == atmos_tokens.end() || it->second.empty()) {
        return valid_grids;
    }
    size_t start = 0;
    while (start <= it->second.size()) {
        const size_t delimiter = it->second.find(';', start);
        const std::string token =
            delimiter == std::string::npos ? it->second.substr(start)
                                           : it->second.substr(start, delimiter - start);
        try {
            const int grid_no = std::stoi(token);
            if (grid_no > 0) {
                valid_grids.insert(grid_no);
            }
        } catch (...) {
        }
        if (delimiter == std::string::npos) {
            break;
        }
        start = delimiter + 1;
    }
    return valid_grids;
}

struct GridCandidate {
    const ClasGridPoint* point = nullptr;
    double distance_m = 0.0;
};

double clasGridMetricDistanceMeters(double receiver_lat_rad,
                                    double receiver_lon_rad,
                                    const ClasGridPoint& point) {
    const double grid_lat_rad = point.latitude_deg * kDegreesToRadians;
    const double grid_lon_rad = point.longitude_deg * kDegreesToRadians;
    const double dlat_m = constants::WGS84_A * (receiver_lat_rad - grid_lat_rad);
    const double dlon_m =
        constants::WGS84_A * (receiver_lon_rad - grid_lon_rad) * std::cos(receiver_lat_rad);
    return std::max(std::hypot(dlat_m, dlon_m), 1.0);
}

bool nonCollinear2d(double ax, double ay, double bx, double by) {
    const double norm_product = std::hypot(ax, ay) * std::hypot(bx, by);
    if (norm_product <= 0.0) {
        return false;
    }
    return std::fabs((ax * bx + ay * by) / norm_product) < 1.0;
}

int selectClaslibSurroundingGrids(
    const std::vector<GridCandidate>& candidates,
    std::array<const GridCandidate*, 4>& selected) {
    const int n = static_cast<int>(candidates.size());
    if (n <= 0) {
        return 0;
    }
    const int first_index =
        (n > 2 &&
         candidates[1].point->network_id == candidates[2].point->network_id &&
         candidates[0].point->network_id != candidates[1].point->network_id)
            ? 1
            : 0;

    selected.fill(nullptr);
    selected[0] = &candidates[first_index];

    const GridCandidate* grid2 = nullptr;
    const GridCandidate* grid3 = nullptr;
    double dd12_lat = 0.0;
    double dd12_lon = 0.0;
    double dd13_lat = 0.0;
    double dd13_lon = 0.0;

    for (int i = 1; i < n; ++i) {
        if (i == first_index ||
            candidates[i].point->network_id != selected[0]->point->network_id) {
            continue;
        }
        selected[1] = &candidates[i];
        grid2 = &candidates[i];
        dd12_lat = candidates[i].point->latitude_deg -
                   selected[0]->point->latitude_deg;
        dd12_lon = candidates[i].point->longitude_deg -
                   selected[0]->point->longitude_deg;
        break;
    }
    if (grid2 == nullptr) {
        return 1;
    }

    for (int i = 1; i < n; ++i) {
        if (i == first_index || &candidates[i] == grid2 ||
            candidates[i].point->network_id != selected[0]->point->network_id) {
            continue;
        }
        const double dlat =
            candidates[i].point->latitude_deg - selected[0]->point->latitude_deg;
        const double dlon =
            candidates[i].point->longitude_deg - selected[0]->point->longitude_deg;
        if (nonCollinear2d(dd12_lat, dd12_lon, dlat, dlon)) {
            selected[2] = &candidates[i];
            grid3 = &candidates[i];
            dd13_lat = dlat;
            dd13_lon = dlon;
            break;
        }
    }
    if (grid3 == nullptr) {
        return 1;
    }

    for (int i = 1; i < n; ++i) {
        if (i == first_index || &candidates[i] == grid2 ||
            &candidates[i] == grid3 ||
            candidates[i].point->network_id != selected[0]->point->network_id) {
            continue;
        }
        const double dlat =
            candidates[i].point->latitude_deg - selected[0]->point->latitude_deg;
        const double dlon =
            candidates[i].point->longitude_deg - selected[0]->point->longitude_deg;
        const bool rectangular =
            (std::fabs(grid2->point->longitude_deg - candidates[i].point->longitude_deg) < 0.04 &&
             std::fabs(grid3->point->latitude_deg - candidates[i].point->latitude_deg) < 0.04) ||
            (std::fabs(grid2->point->latitude_deg - candidates[i].point->latitude_deg) < 0.04 &&
             std::fabs(grid3->point->longitude_deg - candidates[i].point->longitude_deg) < 0.04);
        if (nonCollinear2d(dd12_lat, dd12_lon, dlat, dlon) &&
            nonCollinear2d(dd13_lat, dd13_lon, dlat, dlon) &&
            rectangular) {
            selected[3] = &candidates[i];
            return 4;
        }
    }

    return 3;
}

bool solveLinear4x4(double matrix[4][5], double solution[4]) {
    for (int col = 0; col < 4; ++col) {
        int pivot = col;
        double pivot_abs = std::fabs(matrix[col][col]);
        for (int row = col + 1; row < 4; ++row) {
            const double candidate_abs = std::fabs(matrix[row][col]);
            if (candidate_abs > pivot_abs) {
                pivot = row;
                pivot_abs = candidate_abs;
            }
        }
        if (pivot_abs <= 1e-14) {
            return false;
        }
        if (pivot != col) {
            for (int c = col; c < 5; ++c) {
                std::swap(matrix[col][c], matrix[pivot][c]);
            }
        }
        const double scale = matrix[col][col];
        for (int c = col; c < 5; ++c) {
            matrix[col][c] /= scale;
        }
        for (int row = 0; row < 4; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = matrix[row][col];
            for (int c = col; c < 5; ++c) {
                matrix[row][c] -= factor * matrix[col][c];
            }
        }
    }
    for (int row = 0; row < 4; ++row) {
        solution[row] = matrix[row][4];
    }
    return true;
}

bool computeClaslibFourGridWeights(const std::array<const GridCandidate*, 4>& selected,
                                   double user_dlat_deg,
                                   double user_dlon_deg,
                                   double weights[4]) {
    double augmented[4][5] = {};
    for (int grid = 0; grid < 4; ++grid) {
        const double dlat =
            selected[grid]->point->latitude_deg - selected[0]->point->latitude_deg;
        const double dlon =
            selected[grid]->point->longitude_deg - selected[0]->point->longitude_deg;
        augmented[0][grid] = dlat;
        augmented[1][grid] = dlon;
        augmented[2][grid] = dlat * dlon;
        augmented[3][grid] = 1.0;
    }
    augmented[0][4] = user_dlat_deg;
    augmented[1][4] = user_dlon_deg;
    augmented[2][4] = user_dlat_deg * user_dlon_deg;
    augmented[3][4] = 1.0;
    return solveLinear4x4(augmented, weights);
}

bool computeThreeGridWeights(const std::array<const GridCandidate*, 4>& selected,
                             double user_dlat_deg,
                             double user_dlon_deg,
                             double weights[4]) {
    const double a00 =
        selected[1]->point->latitude_deg - selected[0]->point->latitude_deg;
    const double a01 =
        selected[2]->point->latitude_deg - selected[0]->point->latitude_deg;
    const double a10 =
        selected[1]->point->longitude_deg - selected[0]->point->longitude_deg;
    const double a11 =
        selected[2]->point->longitude_deg - selected[0]->point->longitude_deg;
    const double det = a00 * a11 - a01 * a10;
    if (std::fabs(det) <= 1e-14) {
        return false;
    }
    const double w1 = (a11 * user_dlat_deg - a01 * user_dlon_deg) / det;
    const double w2 = (-a10 * user_dlat_deg + a00 * user_dlon_deg) / det;
    weights[0] = 1.0 - w1 - w2;
    weights[1] = w1;
    weights[2] = w2;
    weights[3] = 0.0;
    return true;
}

void computeInverseDistanceWeights(const std::array<const GridCandidate*, 4>& selected,
                                   int count,
                                   double weights[4]) {
    double inverse_sum = 0.0;
    for (int i = 0; i < count; ++i) {
        inverse_sum += 1.0 / selected[i]->distance_m;
    }
    for (int i = 0; i < count; ++i) {
        weights[i] = (1.0 / selected[i]->distance_m) / inverse_sum;
    }
    for (int i = count; i < 4; ++i) {
        weights[i] = 0.0;
    }
}

void fillGridReference(const std::array<const GridCandidate*, 4>& selected,
                       int count,
                       const ClasGridPoint* network_origin,
                       const double weights[4],
                       ClasGridReference& reference) {
    const ClasGridPoint* origin =
        network_origin != nullptr ? network_origin : selected[0]->point;
    reference.dlat_deg = selected[0]->point->latitude_deg - origin->latitude_deg;
    reference.dlon_deg = selected[0]->point->longitude_deg - origin->longitude_deg;
    reference.residual_index =
        static_cast<size_t>(std::max(selected[0]->point->grid_no - 1, 0));
    reference.network_id = selected[0]->point->network_id;
    reference.grid_no = selected[0]->point->grid_no;
    reference.nearest_grid_distance_m = selected[0]->distance_m;
    reference.interpolation_grid_count = count;
    reference.has_bilinear = count == 4;
    for (int i = 0; i < 4; ++i) {
        reference.interpolation_weights[i] = 0.0;
        reference.interpolation_grid_indices[i] = 0;
        reference.interpolation_grid_dlat_deg[i] = 0.0;
        reference.interpolation_grid_dlon_deg[i] = 0.0;
        reference.bilinear_weights[i] = 0.0;
        reference.bilinear_grid_indices[i] = 0;
    }
    for (int i = 0; i < count; ++i) {
        const size_t residual_index =
            static_cast<size_t>(std::max(selected[i]->point->grid_no - 1, 0));
        const double dlat =
            selected[i]->point->latitude_deg - origin->latitude_deg;
        const double dlon =
            selected[i]->point->longitude_deg - origin->longitude_deg;
        reference.interpolation_weights[i] = weights[i];
        reference.interpolation_grid_indices[i] = residual_index;
        reference.interpolation_grid_dlat_deg[i] = dlat;
        reference.interpolation_grid_dlon_deg[i] = dlon;
        reference.bilinear_weights[i] = weights[i];
        reference.bilinear_grid_indices[i] = residual_index;
    }
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
    const bool lifecycle_enabled = pppEnvOverrides().clas_atmos_lifecycle;
    if (lifecycle_enabled && grid_count <= 0) {
        return false;
    }
    const std::set<int> valid_grid_numbers =
        lifecycle_enabled ? parseAtmosValidGridNumbers(atmos_tokens) : std::set<int>{};
    if (lifecycle_enabled && valid_grid_numbers.empty()) {
        return false;
    }
    const auto gridAllowed = [&](const ClasGridPoint& point) {
        return !lifecycle_enabled ||
               valid_grid_numbers.find(point.grid_no) != valid_grid_numbers.end();
    };

    double receiver_lat_rad = 0.0;
    double receiver_lon_rad = 0.0;
    double receiver_height_m = 0.0;
    ecef2geodetic(receiver_position, receiver_lat_rad, receiver_lon_rad, receiver_height_m);
    if (!std::isfinite(receiver_lat_rad) || !std::isfinite(receiver_lon_rad)) {
        return false;
    }

    const double receiver_lat_deg = receiver_lat_rad / kDegreesToRadians;
    const double receiver_lon_deg = receiver_lon_rad / kDegreesToRadians;

    const bool use_grid_matrix = pppEnvOverrides().clas_atmos_grid_matrix;
    if (!use_grid_matrix) {
        struct LegacyGridCandidate {
            const ClasGridPoint* point = nullptr;
            double distance_sq = 0.0;
        };
        std::vector<LegacyGridCandidate> candidates;
        for (const auto& point : clasGridPoints()) {
            if (point.network_id != network_id) continue;
            if (grid_count > 0 && point.grid_no > grid_count) continue;
            if (!gridAllowed(point)) continue;
            const double dlat = receiver_lat_deg - point.latitude_deg;
            const double dlon = receiver_lon_deg - point.longitude_deg;
            candidates.push_back({&point, dlat * dlat + dlon * dlon});
        }
        if (candidates.empty()) return false;

        std::sort(candidates.begin(), candidates.end(),
                  [](const LegacyGridCandidate& a, const LegacyGridCandidate& b) {
                      return a.distance_sq < b.distance_sq;
                  });

        const ClasGridPoint* nearest = candidates[0].point;

        const ClasGridPoint* grid_sw = nullptr;
        const ClasGridPoint* grid_se = nullptr;
        const ClasGridPoint* grid_nw = nullptr;
        const ClasGridPoint* grid_ne = nullptr;
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
            const double lat_range = grid_nw->latitude_deg - grid_sw->latitude_deg;
            const double lon_range = grid_se->longitude_deg - grid_sw->longitude_deg;
            if (lat_range > 0.01 && lon_range > 0.01 &&
                lat_range < 0.7 && lon_range < 0.7) {
                const double t = (receiver_lat_deg - grid_sw->latitude_deg) / lat_range;
                const double u = (receiver_lon_deg - grid_sw->longitude_deg) / lon_range;
                reference.bilinear_weights[0] = (1 - t) * (1 - u);
                reference.bilinear_weights[1] = (1 - t) * u;
                reference.bilinear_weights[2] = t * (1 - u);
                reference.bilinear_weights[3] = t * u;
                reference.bilinear_grid_indices[0] =
                    static_cast<size_t>(std::max(grid_sw->grid_no - 1, 0));
                reference.bilinear_grid_indices[1] =
                    static_cast<size_t>(std::max(grid_se->grid_no - 1, 0));
                reference.bilinear_grid_indices[2] =
                    static_cast<size_t>(std::max(grid_nw->grid_no - 1, 0));
                reference.bilinear_grid_indices[3] =
                    static_cast<size_t>(std::max(grid_ne->grid_no - 1, 0));
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
        reference.nearest_grid_distance_m =
            std::sqrt(candidates[0].distance_sq) * kDegreesToRadians * constants::WGS84_A;
        return true;
    }

    std::vector<GridCandidate> candidates;
    const ClasGridPoint* network_origin = nullptr;
    for (const auto& point : clasGridPoints()) {
        if (point.network_id != network_id) continue;
        if (point.grid_no == 1) {
            network_origin = &point;
        }
        if (grid_count > 0 && point.grid_no > grid_count) continue;
        if (std::fabs(point.height_m) > 0.0001) continue;
        if (!gridAllowed(point)) continue;
        const double distance_m =
            clasGridMetricDistanceMeters(receiver_lat_rad, receiver_lon_rad, point);
        if (distance_m > kClasMaxGridDistanceM) {
            continue;
        }
        candidates.push_back({&point, distance_m});
    }
    if (candidates.empty()) return false;

    std::sort(candidates.begin(), candidates.end(),
              [](const GridCandidate& a, const GridCandidate& b) {
                  return a.distance_m < b.distance_m;
              });

    std::array<const GridCandidate*, 4> selected{};
    selected.fill(nullptr);
    selected[0] = &candidates[0];
    int selected_count = 1;
    double weights[4] = {1.0, 0.0, 0.0, 0.0};

    if (candidates.size() >= 3 &&
        candidates[0].point->network_id <= kClasMultiGridNetworkMax &&
        candidates[0].distance_m > kClasNearestGridThresholdM) {
        selected_count = selectClaslibSurroundingGrids(candidates, selected);
        if (selected_count == 4) {
            const double user_dlat =
                receiver_lat_deg - selected[0]->point->latitude_deg;
            const double user_dlon =
                receiver_lon_deg - selected[0]->point->longitude_deg;
            if (!computeClaslibFourGridWeights(selected, user_dlat, user_dlon, weights)) {
                selected_count = 1;
                selected.fill(nullptr);
                selected[0] = &candidates[0];
                weights[0] = 1.0;
                weights[1] = weights[2] = weights[3] = 0.0;
            }
        } else if (selected_count == 3) {
            const double user_dlat =
                receiver_lat_deg - selected[0]->point->latitude_deg;
            const double user_dlon =
                receiver_lon_deg - selected[0]->point->longitude_deg;
            if (!computeThreeGridWeights(selected, user_dlat, user_dlon, weights)) {
                selected_count = 1;
                selected.fill(nullptr);
                selected[0] = &candidates[0];
                weights[0] = 1.0;
                weights[1] = weights[2] = weights[3] = 0.0;
            }
        } else if (selected_count > 1) {
            computeInverseDistanceWeights(selected, selected_count, weights);
        } else {
            selected_count = 1;
            selected.fill(nullptr);
            selected[0] = &candidates[0];
            weights[0] = 1.0;
            weights[1] = weights[2] = weights[3] = 0.0;
        }
    }

    fillGridReference(
        selected,
        selected_count,
        network_origin,
        weights,
        reference);
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
            if (have_grid_reference &&
                (grid_reference.has_bilinear ||
                 grid_reference.interpolation_grid_count > 1)) {
                const bool use_matrix_weights =
                    grid_reference.interpolation_grid_count > 1;
                const int grid_count =
                    use_matrix_weights ? grid_reference.interpolation_grid_count : 4;
                double interpolated_val = 0.0;
                bool ok = true;
                for (int g = 0; g < grid_count; ++g) {
                    const size_t residual_index =
                        use_matrix_weights
                            ? grid_reference.interpolation_grid_indices[g]
                            : grid_reference.bilinear_grid_indices[g];
                    const double weight =
                        use_matrix_weights
                            ? grid_reference.interpolation_weights[g]
                            : grid_reference.bilinear_weights[g];
                    double gv = 0.0;
                    if (sampleAtmosResidualList(atmos_tokens, key, true,
                            residual_index,
                            residual_sampling_policy, gv)) {
                        interpolated_val += weight * gv;
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
            double hydro_zenith_m = 2.3;
            double wet_zenith_m = 0.252;
            if (pppEnvOverrides().clas_trop_climatology) {
                const auto zenith =
                    models::estimateZenithTroposphereClimatology(
                        latitude_rad,
                        height_m,
                        dayOfYearFromTime(time));
                hydro_zenith_m = zenith.hydrostatic_delay_m;
                wet_zenith_m = zenith.wet_delay_m;
            }
            correction_m += hydro_mapping * (hydro_zenith_m + hydro_delta_m) +
                wet_mapping * (wet_zenith_m + wet_delta_m);
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

    const std::string materialized_grid_key =
        "atmos_stec_grid_tecu" + suffix;
    if (have_grid_reference &&
        atmos_tokens.find(materialized_grid_key) != atmos_tokens.end()) {
        auto sampleMaterializedGrid = [&](size_t grid_index, double& grid_value) {
            return parseAtmosListValueAtIndex(
                atmos_tokens, materialized_grid_key, grid_index, grid_value) &&
                   std::isfinite(grid_value);
        };
        double materialized_stec = 0.0;
        bool have_materialized = false;
        if (grid_reference.interpolation_grid_count > 1) {
            bool ok = true;
            for (int g = 0; g < grid_reference.interpolation_grid_count; ++g) {
                double grid_value = 0.0;
                if (!sampleMaterializedGrid(
                        grid_reference.interpolation_grid_indices[g],
                        grid_value)) {
                    ok = false;
                    break;
                }
                materialized_stec +=
                    grid_reference.interpolation_weights[g] * grid_value;
            }
            have_materialized = ok;
        } else if (grid_reference.has_bilinear) {
            bool ok = true;
            for (int g = 0; g < 4; ++g) {
                double grid_value = 0.0;
                if (!sampleMaterializedGrid(
                        grid_reference.bilinear_grid_indices[g],
                        grid_value)) {
                    ok = false;
                    break;
                }
                materialized_stec +=
                    grid_reference.bilinear_weights[g] * grid_value;
            }
            have_materialized = ok;
        } else {
            have_materialized =
                sampleMaterializedGrid(grid_reference.residual_index, materialized_stec);
        }
        if (have_materialized) {
            return materialized_stec;
        }
    }
    if (pppEnvOverrides().clas_atmos_lifecycle) {
        return 0.0;
    }

    // STEC polynomial: c00 + c01*dlat + c10*dlon + c11*dlat*dlon [+ c02*dlat² + c20*dlon²]
    // When 4-grid bilinear is available, evaluate the polynomial at each grid
    // point's position and bilinear-interpolate the results (CLASLIB approach).
    auto evaluateStecPoly = [&](double dlat, double dlon, bool* have_terms = nullptr) -> double {
        double val = 0.0;
        double term = 0.0;
        bool have_any_term = false;
        if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c00_tecu" + suffix, term) && std::isfinite(term)) {
            val += term;
            have_any_term = true;
        }
        if (stec_type > 0 && (!subtype12_row || useSubtype12LinearTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c01_tecu_per_deg" + suffix, term) && std::isfinite(term)) {
                val += term * dlat;
                have_any_term = true;
            }
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c10_tecu_per_deg" + suffix, term) && std::isfinite(term)) {
                val += term * dlon;
                have_any_term = true;
            }
        }
        if (stec_type > 1 && (!subtype12_row || useSubtype12QuadraticTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c11_tecu_per_deg2" + suffix, term) && std::isfinite(term)) {
                val += term * dlat * dlon;
                have_any_term = true;
            }
        }
        if (stec_type > 2 && (!subtype12_row || useSubtype12QuadraticTerms(subtype12_value_policy))) {
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c02_tecu_per_deg2" + suffix, term) && std::isfinite(term)) {
                val += term * dlat * dlat;
                have_any_term = true;
            }
            if (parseAtmosTokenDouble(atmos_tokens, "atmos_stec_c20_tecu_per_deg2" + suffix, term) && std::isfinite(term)) {
                val += term * dlon * dlon;
                have_any_term = true;
            }
        }
        if (have_terms != nullptr) {
            *have_terms = have_any_term;
        }
        return val;
    };

    double value = 0.0;
    if (usePolynomialTerms(value_policy) ||
        (useResidualTerms(value_policy) && have_grid_reference &&
         grid_reference.interpolation_grid_count > 1)) {
        bool have_polynomial_terms = false;
        if (have_grid_reference && grid_reference.interpolation_grid_count > 1) {
            double interpolated_stec = 0.0;
            for (int g = 0; g < grid_reference.interpolation_grid_count; ++g) {
                bool have_grid_terms = false;
                double grid_stec = 0.0;
                if (usePolynomialTerms(value_policy)) {
                    grid_stec = evaluateStecPoly(
                        grid_reference.interpolation_grid_dlat_deg[g],
                        grid_reference.interpolation_grid_dlon_deg[g],
                        &have_grid_terms);
                    have_polynomial_terms = have_polynomial_terms || have_grid_terms;
                }
                double grid_residual = 0.0;
                if (useResidualTerms(value_policy) &&
                    sampleAtmosResidualList(
                        atmos_tokens,
                        "atmos_stec_residuals_tecu" + suffix,
                        true,
                        grid_reference.interpolation_grid_indices[g],
                        residual_sampling_policy,
                        grid_residual)) {
                    grid_stec += grid_residual;
                    have_correction = true;
                }
                interpolated_stec +=
                    grid_reference.interpolation_weights[g] * grid_stec;
            }
            stec_tecu = interpolated_stec;
        } else if (have_grid_reference && grid_reference.has_bilinear) {
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
                bool have_grid_terms = false;
                double grid_stec = evaluateStecPoly(grid_dlat[g], grid_dlon[g], &have_grid_terms);
                have_polynomial_terms = have_polynomial_terms || have_grid_terms;
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
                have_grid_reference ? grid_reference.dlon_deg : 0.0,
                &have_polynomial_terms);
        }
        if (have_polynomial_terms) {
            have_correction = true;
        }
    }
    // Residuals: for bilinear mode, residuals are already included in the
    // per-grid evaluation above.  For nearest-grid mode, add the residual.
    if (!grid_reference.has_bilinear &&
        grid_reference.interpolation_grid_count <= 1 &&
        useResidualTerms(value_policy) &&
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
