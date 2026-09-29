#pragma once

// Correction-product helpers shared by the SPP and PPP translation units
// (spp.cpp, ppp_corrections.cpp, ppp_atmosphere.cpp).

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/types.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <string>

namespace libgnss::correction_products_internal {

inline constexpr double kDegreesToRadians = M_PI / 180.0;

// RINEX 3 observation code used to look up observable-specific (OSB) biases.
inline std::string biasObservationCode(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "C1C";
        case SignalType::GPS_L1P: return "C1P";
        case SignalType::GPS_L2P: return "C2P";
        case SignalType::GPS_L2C: return "C2W";
        case SignalType::GPS_L5: return "C5Q";
        case SignalType::GLO_L1CA: return "C1C";
        case SignalType::GLO_L1P: return "C1P";
        case SignalType::GLO_L2CA: return "C2C";
        case SignalType::GLO_L2P: return "C2P";
        case SignalType::GAL_E1: return "C1C";
        case SignalType::GAL_E5A: return "C5Q";
        case SignalType::GAL_E5B: return "C7Q";
        case SignalType::GAL_E6: return "C6C";
        case SignalType::BDS_B1I: return "C2I";
        case SignalType::BDS_B2I: return "C7I";
        case SignalType::BDS_B3I: return "C6I";
        case SignalType::BDS_B1C: return "C1C";
        case SignalType::BDS_B2A: return "C5Q";
        case SignalType::QZS_L1CA: return "C1C";
        case SignalType::QZS_L2C: return "C2L";
        case SignalType::QZS_L5: return "C5Q";
        default: return "";
    }
}

// Single-layer ionosphere pierce point (degrees) and slant mapping factor
// for an IONEX shell.
inline bool ionexPiercePointAndMapping(const IONEXProducts& ionex_products,
                                       const Vector3d& receiver_position,
                                       double azimuth_rad,
                                       double elevation_rad,
                                       double& ipp_lat_deg,
                                       double& ipp_lon_deg,
                                       double& mapping_factor) {
    if (elevation_rad <= 0.0) {
        return false;
    }

    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);

    const double earth_radius_m =
        ionex_products.base_radius_km > 0.0
            ? ionex_products.base_radius_km * 1000.0
            : constants::WGS84_A;
    const double shell_height_m =
        !ionex_products.height_grid.empty()
            ? ionex_products.height_grid.front() * 1000.0
            : 450000.0;
    if (earth_radius_m <= 0.0 || shell_height_m < 0.0) {
        return false;
    }

    const double ratio = earth_radius_m / (earth_radius_m + shell_height_m);
    const double cos_elevation = std::cos(elevation_rad);
    const double argument = std::clamp(ratio * cos_elevation, -1.0, 1.0);
    const double psi = M_PI_2 - elevation_rad - std::asin(argument);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_psi = std::sin(psi);
    const double cos_psi = std::cos(psi);

    const double ipp_lat_rad = std::asin(
        std::clamp(sin_lat * cos_psi + cos_lat * sin_psi * std::cos(azimuth_rad),
                   -1.0,
                   1.0));
    const double ipp_lon_rad = longitude_rad + std::atan2(
        sin_psi * std::sin(azimuth_rad),
        cos_lat * cos_psi - sin_lat * sin_psi * std::cos(azimuth_rad));

    const double mapping_argument = ratio * cos_elevation;
    mapping_factor = 1.0 /
        std::sqrt(std::max(1e-12, 1.0 - mapping_argument * mapping_argument));
    ipp_lat_deg = ipp_lat_rad / kDegreesToRadians;
    ipp_lon_deg = ipp_lon_rad / kDegreesToRadians;
    while (ipp_lon_deg > 180.0) {
        ipp_lon_deg -= 360.0;
    }
    while (ipp_lon_deg < -180.0) {
        ipp_lon_deg += 360.0;
    }
    return std::isfinite(mapping_factor);
}

// Day of year (1-based) of the UTC calendar date of `time`.
inline int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
#if defined(_WIN32)
    gmtime_s(&utc_tm, &utc_seconds);
#else
    gmtime_r(&utc_seconds, &utc_tm);
#endif
    return utc_tm.tm_yday + 1;
}

}  // namespace libgnss::correction_products_internal
