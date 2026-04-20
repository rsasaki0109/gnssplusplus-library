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
constexpr double GLO_L1_BASE_FREQ = 1602.0e6; ///< GLONASS L1 base frequency (Hz)
constexpr double GLO_L1_STEP_FREQ = 0.5625e6; ///< GLONASS L1 frequency channel step (Hz)
constexpr double GLO_L2_BASE_FREQ = 1246.0e6; ///< GLONASS L2 base frequency (Hz)
constexpr double GLO_L2_STEP_FREQ = 0.4375e6; ///< GLONASS L2 frequency channel step (Hz)
constexpr double GAL_E1_FREQ = 1575.42e6;     ///< Galileo E1 frequency (Hz)
constexpr double GAL_E5A_FREQ = 1176.45e6;    ///< Galileo E5a frequency (Hz)
constexpr double GAL_E5B_FREQ = 1207.14e6;    ///< Galileo E5b frequency (Hz)
constexpr double GAL_E6_FREQ = 1278.75e6;     ///< Galileo E6 frequency (Hz)
constexpr double BDS_B1I_FREQ = 1561.098e6;   ///< BeiDou B1I frequency (Hz)
constexpr double BDS_B2I_FREQ = 1207.14e6;    ///< BeiDou B2I frequency (Hz)
constexpr double BDS_B3I_FREQ = 1268.52e6;    ///< BeiDou B3I frequency (Hz)
constexpr double BDS_B1C_FREQ = 1575.42e6;    ///< BeiDou B1C frequency (Hz)
constexpr double BDS_B2A_FREQ = 1176.45e6;    ///< BeiDou B2a frequency (Hz)

constexpr double GPS_L1_WAVELENGTH = SPEED_OF_LIGHT / GPS_L1_FREQ;  ///< ~0.1903 m
constexpr double GPS_L2_WAVELENGTH = SPEED_OF_LIGHT / GPS_L2_FREQ;  ///< ~0.2442 m
constexpr double GPS_L5_WAVELENGTH = SPEED_OF_LIGHT / GPS_L5_FREQ;  ///< ~0.2548 m
constexpr double GAL_E1_WAVELENGTH = SPEED_OF_LIGHT / GAL_E1_FREQ;  ///< ~0.1903 m
constexpr double GAL_E5A_WAVELENGTH = SPEED_OF_LIGHT / GAL_E5A_FREQ;///< ~0.2548 m
constexpr double GAL_E5B_WAVELENGTH = SPEED_OF_LIGHT / GAL_E5B_FREQ;///< ~0.2483 m
constexpr double GAL_E6_WAVELENGTH = SPEED_OF_LIGHT / GAL_E6_FREQ;  ///< ~0.2344 m
constexpr double BDS_B1I_WAVELENGTH = SPEED_OF_LIGHT / BDS_B1I_FREQ;///< ~0.1920 m
constexpr double BDS_B2I_WAVELENGTH = SPEED_OF_LIGHT / BDS_B2I_FREQ;///< ~0.2483 m
constexpr double BDS_B3I_WAVELENGTH = SPEED_OF_LIGHT / BDS_B3I_FREQ;///< ~0.2363 m
constexpr double BDS_B1C_WAVELENGTH = SPEED_OF_LIGHT / BDS_B1C_FREQ;///< ~0.1903 m
constexpr double BDS_B2A_WAVELENGTH = SPEED_OF_LIGHT / BDS_B2A_FREQ;///< ~0.2548 m

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
