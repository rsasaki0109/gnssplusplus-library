#pragma once

/**
 * @file troposphere.hpp
 * @brief Tropospheric delay models for GNSS processing.
 *
 * Extracted from RTKProcessor::tropModel() (rtk.cpp).
 * Provides the Saastamoinen model with 1/cos(z) mapping function.
 */

#include <libgnss++/core/coordinates.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace libgnss {
namespace models {

/**
 * @brief Saastamoinen troposphere model with 1/cos(z) mapping.
 *
 * Computes the total (hydrostatic + wet) tropospheric delay using the
 * Saastamoinen model with standard atmosphere assumptions.  The mapping
 * function is a simple 1/cos(z) which is adequate for short-baseline RTK
 * where double-differencing removes most of the tropospheric error.
 *
 * For short baselines the DD residual from NMF is < 1 mm and < 5 mm
 * at 3.3 km.
 *
 * @param pos_ecef  Receiver ECEF position (m)
 * @param elevation Satellite elevation angle (radians)
 * @return Tropospheric delay in meters (zero if elevation is too low)
 */
inline double tropDelaySaastamoinen(const Eigen::Vector3d& pos_ecef, double elevation) {
    if (elevation <= 0.05) return 0.0;

    double lat, lon, h;
    ecef2geodetic(pos_ecef, lat, lon, h);

    if (h < -100.0 || h > 10000.0) return 0.0;

    double hgt = std::max(h, 0.0);

    // Standard atmosphere
    double T = 15.0 - 6.5e-3 * hgt + 273.16;                          // Temperature (K)
    double P = 1013.25 * std::pow(1.0 - 2.2557e-5 * hgt, 5.2568);     // Pressure (mbar)
    double e = 6.108 * 0.7 * std::exp((17.15 * T - 4684.0) / (T - 38.45)); // Water vapour (mbar)

    // Zenith angle and 1/cos(z) mapping
    double z = M_PI / 2.0 - elevation;
    double cos_z = std::cos(z);
    if (cos_z < 0.067) cos_z = 0.067;

    // Hydrostatic (dry) component
    double trph = 0.0022768 * P /
        (1.0 - 0.00266 * std::cos(2.0 * lat) - 0.00028 * hgt * 1e-3) / cos_z;

    // Wet component
    double trpw = 0.002277 * (1255.0 / T + 0.05) * e / cos_z;

    return trph + trpw;
}

} // namespace models
} // namespace libgnss
