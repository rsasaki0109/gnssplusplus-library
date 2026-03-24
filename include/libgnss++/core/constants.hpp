#pragma once

/**
 * @file constants.hpp
 * @brief Consolidated physical and geodetic constants for GNSS processing.
 *
 * All constants that were previously scattered across types.hpp, rtk.cpp,
 * spp.cpp, and coordinates.hpp are collected here. Other headers should
 * refer to libgnss::constants:: instead of defining local copies.
 */

namespace libgnss {
namespace constants {

// ---------------------------------------------------------------------------
// Fundamental physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s)
constexpr double SPEED_OF_LIGHT = 299792458.0;

/// WGS-84 Earth rotation rate (rad/s)
constexpr double OMEGA_E = 7.2921151467e-5;

// ---------------------------------------------------------------------------
// GPS signal frequencies and wavelengths
// ---------------------------------------------------------------------------

constexpr double GPS_L1_FREQ = 1575.42e6;     ///< GPS L1 frequency (Hz)
constexpr double GPS_L2_FREQ = 1227.60e6;     ///< GPS L2 frequency (Hz)
constexpr double GPS_L5_FREQ = 1176.45e6;     ///< GPS L5 frequency (Hz)

constexpr double GPS_L1_WAVELENGTH = SPEED_OF_LIGHT / GPS_L1_FREQ;  ///< ~0.1903 m
constexpr double GPS_L2_WAVELENGTH = SPEED_OF_LIGHT / GPS_L2_FREQ;  ///< ~0.2442 m
constexpr double GPS_L5_WAVELENGTH = SPEED_OF_LIGHT / GPS_L5_FREQ;  ///< ~0.2548 m

// ---------------------------------------------------------------------------
// WGS-84 ellipsoid parameters
// ---------------------------------------------------------------------------

constexpr double WGS84_A  = 6378137.0;                        ///< Semi-major axis (m)
constexpr double WGS84_F  = 1.0 / 298.257223563;              ///< Flattening
constexpr double WGS84_B  = WGS84_A * (1.0 - WGS84_F);       ///< Semi-minor axis (m)
constexpr double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; ///< First eccentricity squared

// ---------------------------------------------------------------------------
// Ionosphere-free linear combination coefficients (wavelength-based)
// ---------------------------------------------------------------------------

/// C1 = lam2^2 / (lam2^2 - lam1^2)
constexpr double IFLC_C1 = GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH /
    (GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH - GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH);

/// C2 = -lam1^2 / (lam2^2 - lam1^2)
constexpr double IFLC_C2 = -(GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH) /
    (GPS_L2_WAVELENGTH * GPS_L2_WAVELENGTH - GPS_L1_WAVELENGTH * GPS_L1_WAVELENGTH);

// ---------------------------------------------------------------------------
// Time constants
// ---------------------------------------------------------------------------

constexpr double SECONDS_PER_WEEK = 604800.0;  ///< Seconds in one GPS week

// ---------------------------------------------------------------------------
// Frequency-based ionosphere-free coefficients (for WL-NL AR)
// ---------------------------------------------------------------------------

constexpr double IFLC_F1 = GPS_L1_FREQ * GPS_L1_FREQ /
    (GPS_L1_FREQ * GPS_L1_FREQ - GPS_L2_FREQ * GPS_L2_FREQ);

constexpr double IFLC_F2 = -(GPS_L2_FREQ * GPS_L2_FREQ) /
    (GPS_L1_FREQ * GPS_L1_FREQ - GPS_L2_FREQ * GPS_L2_FREQ);

/// Wide-lane wavelength (m)
constexpr double GPS_WL_WAVELENGTH = SPEED_OF_LIGHT / (GPS_L1_FREQ - GPS_L2_FREQ);

/// Narrow-lane wavelength (m) = C1_IF * lam1 + C2_IF * lam2
constexpr double GPS_NL_WAVELENGTH = IFLC_F1 * GPS_L1_WAVELENGTH + IFLC_F2 * GPS_L2_WAVELENGTH;

} // namespace constants
} // namespace libgnss
