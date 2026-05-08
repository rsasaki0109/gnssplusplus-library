#pragma once

// libgnss++/iers/earth_rotation.hpp
//
// Thin C++ wrappers over the vendored IAU SOFA library (third_party/sofa/),
// surfacing IAU 2006/2000A earth-rotation primitives in libgnss-native
// types: MJD inputs (UTC), `Eigen::Matrix3d` rotation outputs.
//
// These primitives are required for cm-level PPP processing — earth
// rotation, polar motion, and ICRS<->ITRS transforms cannot be ignored
// at the centimeter level — and are not provided by the existing
// libgnss++ codebase.
//
// Leap-second handling: the underlying SOFA functions consult their
// internal leap-second table (current as of SOFA issue 2021-05-12).
// Callers operating on epochs after the last leap second known to SOFA
// will receive correct results until the next future leap second is
// inserted; in that case a future SOFA refresh is required.

#include <Eigen/Dense>

#include "libgnss++/core/types.hpp"

namespace libgnss::iers {

/// @brief Earth orientation parameters (EOP) at a given epoch.
///
/// Values come from the IERS Bulletin A / Final EOP series. For
/// consumers without access to a current EOP series, leaving all
/// fields at zero produces a result equivalent to the rigid-body
/// approximation: no UT1-UTC offset, no polar motion. The error
/// from doing so is ~0.5 m at the equator (UT1) plus ~10 m of
/// polar-motion-induced rotation, so EOP must be supplied for
/// cm-level work.
struct EarthOrientationParams {
    /// UT1 - UTC, seconds. Range typically ±0.9 s.
    double ut1_minus_utc_seconds = 0.0;

    /// Polar motion x component, arcseconds (IERS x_p).
    double xp_arcsec = 0.0;

    /// Polar motion y component, arcseconds (IERS y_p).
    double yp_arcsec = 0.0;
};

/// @brief Earth Rotation Angle (ERA), IAU 2000 model.
///
/// ERA is the angle between the Celestial Intermediate Origin (CIO)
/// and the Terrestrial Intermediate Origin (TIO), measured around
/// the Celestial Intermediate Pole. It replaces the older Greenwich
/// Apparent Sidereal Time (GAST) in the IAU 2006/2000A framework.
///
/// @param mjd_utc        UTC modified Julian date.
/// @param eop            Earth orientation parameters at this epoch.
/// @return Earth rotation angle in the range [0, 2π) radians.
double earthRotationAngle(double mjd_utc, const EarthOrientationParams& eop);

/// @brief ICRS (~ECI / J2000) to ITRS (~ECEF) rotation matrix.
///
/// Computes the full IAU 2006/2000A celestial-to-terrestrial rotation
/// matrix using the CIO-based formulation:
///
///     R(t) = R_pom(t) · R_era(t) · R_c2i(t)
///
/// where R_c2i is the precession-nutation-bias matrix at TT,
/// R_era is the earth rotation matrix at UT1, and R_pom is the
/// polar motion matrix at TT.
///
/// @param mjd_utc        UTC modified Julian date.
/// @param eop            Earth orientation parameters at this epoch.
/// @return 3x3 rotation matrix; if r_icrs is a vector in ICRS,
///         then `icrsToItrs(...) * r_icrs` is the same vector in ITRS.
Eigen::Matrix3d icrsToItrs(double mjd_utc, const EarthOrientationParams& eop);

/// @brief Inverse of icrsToItrs (ITRS to ICRS).
inline Eigen::Matrix3d itrsToIcrs(double mjd_utc, const EarthOrientationParams& eop) {
    return icrsToItrs(mjd_utc, eop).transpose();
}

/// @brief Convert a libgnss::GNSSTime (GPS week + TOW) to UTC MJD.
///
/// Uses the SOFA leap-second table internally (current as of SOFA
/// issue 2021-05-12). The GPS<->UTC offset is `leap_seconds - 19`
/// seconds where leap_seconds is TAI-UTC at the requested epoch.
///
/// @param gps_time       GPS time (week, time-of-week).
/// @return UTC modified Julian date.
double gnssTimeToMjdUtc(const GNSSTime& gps_time);

}  // namespace libgnss::iers
