#include "libgnss++/kml/utils.hpp"

#include <cstdio>
#include <cmath>
#include <algorithm>

namespace libgnss {
namespace kml {

namespace {

// WGS84 ellipsoid parameters
constexpr double WGS84_A = 6378137.0;            // Semi-major axis (m)
constexpr double WGS84_F = 1.0 / 298.257223563;  // Flattening
constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;  // First eccentricity squared
constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

}  // namespace

std::string col2hex(const Color& col, double alpha) {
    // KML uses AABBGGRR format (alpha, blue, green, red)
    auto clamp = [](double v) -> int {
        return std::max(0, std::min(255, static_cast<int>(std::round(v * 255.0))));
    };

    double r = std::isnan(col.r) ? 0.0 : col.r;
    double g = std::isnan(col.g) ? 0.0 : col.g;
    double b = std::isnan(col.b) ? 0.0 : col.b;

    int a_hex = clamp(alpha);
    int b_hex = clamp(b);
    int g_hex = clamp(g);
    int r_hex = clamp(r);

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", a_hex, b_hex, g_hex, r_hex);
    return std::string(buf);
}

double wrapTo720(double angle) {
    return std::fmod(angle + 360.0, 720.0) - 360.0;
}

std::vector<double> unwrap360(const std::vector<double>& angles) {
    if (angles.empty()) return {};

    std::vector<double> result = angles;
    double cumulative_shift = 0.0;

    for (size_t i = 1; i < angles.size(); ++i) {
        double diff = angles[i] - angles[i - 1];
        if (std::abs(diff) > 180.0) {
            cumulative_shift += (diff > 0 ? -360.0 : 360.0);
        }
        result[i] = angles[i] + cumulative_shift;
    }

    return result;
}

Location enu2llh(double east, double north, double up,
                 double ref_lat, double ref_lon, double ref_alt) {
    double lat0 = ref_lat * DEG2RAD;
    double sin_lat = std::sin(lat0);

    // Meridional radius of curvature
    double denom = 1.0 - WGS84_E2 * sin_lat * sin_lat;
    double M = WGS84_A * (1.0 - WGS84_E2) / std::pow(denom, 1.5);

    // Prime vertical radius of curvature
    double N = WGS84_A / std::sqrt(denom);

    double dlat = north / (M + ref_alt);
    double dlon = east / ((N + ref_alt) * std::cos(lat0));

    return Location(ref_lat + dlat * RAD2DEG,
                    ref_lon + dlon * RAD2DEG,
                    ref_alt + up);
}

CubeVertices calcCubeVertices(const Location& center, double side_length) {
    CubeVertices result;
    double d = side_length / 2.0;

    double c_lat = std::isnan(center.lat) ? 0.0 : center.lat;
    double c_lon = std::isnan(center.lon) ? 0.0 : center.lon;
    double c_alt = std::isnan(center.alt) ? 0.0 : center.alt;

    // 8 vertices in ENU coordinates (east, north, up)
    double enu[8][3] = {
        {-d, -d, -d}, { d, -d, -d}, { d,  d, -d}, {-d,  d, -d},  // bottom
        {-d, -d,  d}, { d, -d,  d}, { d,  d,  d}, {-d,  d,  d}   // top
    };

    for (int i = 0; i < 8; ++i) {
        result.vertices[i] = enu2llh(enu[i][0], enu[i][1], enu[i][2],
                                     c_lat, c_lon, c_alt);
    }

    // Face indices (0-based): top, front, right, back, left (skip bottom)
    result.faces = {{
        {{4, 5, 6, 7}},  // Top
        {{0, 1, 5, 4}},  // Front
        {{1, 2, 6, 5}},  // Right
        {{2, 3, 7, 6}},  // Back
        {{3, 0, 4, 7}}   // Left
    }};

    return result;
}

}  // namespace kml
}  // namespace libgnss
