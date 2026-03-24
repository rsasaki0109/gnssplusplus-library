#pragma once

/**
 * @file coordinates.hpp
 * @brief Coordinate conversion and geometric utility functions.
 *
 * Provides ECEF <-> geodetic <-> ENU conversions and the RTKLIB-compatible
 * geodist() function (geometric range + Sagnac correction).
 */

#include <libgnss++/core/constants.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace libgnss {

using Eigen::Vector3d;

// ---------------------------------------------------------------------------
// ECEF <-> Geodetic
// ---------------------------------------------------------------------------

/**
 * @brief Convert ECEF coordinates to geodetic (lat, lon, height).
 *
 * Iterative method matching RTKLIB's ecef2pos() output.
 *
 * @param ecef  ECEF position [x, y, z] in meters
 * @param lat   Output latitude in radians
 * @param lon   Output longitude in radians
 * @param h     Output ellipsoidal height in meters
 */
inline void ecef2geodetic(const Vector3d& ecef, double& lat, double& lon, double& h) {
    constexpr double a  = constants::WGS84_A;
    constexpr double e2 = 0.00669437999014;  // WGS84 first eccentricity squared

    double p = std::sqrt(ecef(0) * ecef(0) + ecef(1) * ecef(1));
    lon = std::atan2(ecef(1), ecef(0));

    double z = ecef(2), zk = 0.0, v = a;
    for (int k = 0; k < 10 && std::abs(z - zk) >= 1e-4; ++k) {
        zk = z;
        double sinp = z / std::sqrt(p * p + z * z);
        v = a / std::sqrt(1.0 - e2 * sinp * sinp);
        z = ecef(2) + v * e2 * sinp;
    }

    lat = (p > 1e-12) ? std::atan2(z, p) : (ecef(2) > 0 ? M_PI / 2.0 : -M_PI / 2.0);
    h = std::sqrt(p * p + z * z) - v;
}

/**
 * @brief Convert geodetic coordinates to ECEF.
 *
 * @param lat  Latitude (radians)
 * @param lon  Longitude (radians)
 * @param h    Ellipsoidal height (meters)
 * @return ECEF position vector (meters)
 */
inline Vector3d geodetic2ecef(double lat, double lon, double h) {
    constexpr double a  = constants::WGS84_A;
    constexpr double e2 = 0.00669437999014;

    double sinlat = std::sin(lat);
    double coslat = std::cos(lat);
    double N = a / std::sqrt(1.0 - e2 * sinlat * sinlat);

    return Vector3d(
        (N + h) * coslat * std::cos(lon),
        (N + h) * coslat * std::sin(lon),
        (N * (1.0 - e2) + h) * sinlat
    );
}

// ---------------------------------------------------------------------------
// ENU conversion
// ---------------------------------------------------------------------------

/**
 * @brief Convert ECEF difference vector to local ENU coordinates.
 *
 * @param ecef_diff  Difference vector in ECEF (target - origin) in meters
 * @param lat        Origin latitude (radians)
 * @param lon        Origin longitude (radians)
 * @return ENU vector (East, North, Up) in meters
 */
inline Vector3d ecef2enu(const Vector3d& ecef_diff, double lat, double lon) {
    double sinlat = std::sin(lat), coslat = std::cos(lat);
    double sinlon = std::sin(lon), coslon = std::cos(lon);

    return Vector3d(
        -sinlon * ecef_diff(0) + coslon * ecef_diff(1),
        -sinlat * coslon * ecef_diff(0) - sinlat * sinlon * ecef_diff(1) + coslat * ecef_diff(2),
         coslat * coslon * ecef_diff(0) +  coslat * sinlon * ecef_diff(1) + sinlat * ecef_diff(2)
    );
}

/**
 * @brief Convert local ENU vector to ECEF difference vector.
 *
 * @param enu  ENU vector (East, North, Up) in meters
 * @param lat  Origin latitude (radians)
 * @param lon  Origin longitude (radians)
 * @return ECEF difference vector in meters
 */
inline Vector3d enu2ecef(const Vector3d& enu, double lat, double lon) {
    double sinlat = std::sin(lat), coslat = std::cos(lat);
    double sinlon = std::sin(lon), coslon = std::cos(lon);

    return Vector3d(
        -sinlon * enu(0) - sinlat * coslon * enu(1) + coslat * coslon * enu(2),
         coslon * enu(0) - sinlat * sinlon * enu(1) + coslat * sinlon * enu(2),
                           coslat * enu(1)           + sinlat * enu(2)
    );
}

// ---------------------------------------------------------------------------
// Geometric range
// ---------------------------------------------------------------------------

/**
 * @brief RTKLIB-compatible geometric range with Sagnac correction.
 *
 * Computes the Euclidean distance between satellite and receiver plus the
 * analytical Sagnac correction term:
 *   range = |rs - rr| + (OMEGA_E / c) * (rs_x * rr_y - rs_y * rr_x)
 *
 * This is equivalent to RTKLIB's geodist() function.
 *
 * @param sat_pos  Satellite ECEF position (meters)
 * @param rec_pos  Receiver ECEF position (meters)
 * @return Geometric range including Sagnac correction (meters)
 */
inline double geodist(const Vector3d& sat_pos, const Vector3d& rec_pos) {
    double r = (sat_pos - rec_pos).norm();
    return r + constants::OMEGA_E *
        (sat_pos(0) * rec_pos(1) - sat_pos(1) * rec_pos(0)) / constants::SPEED_OF_LIGHT;
}

} // namespace libgnss
