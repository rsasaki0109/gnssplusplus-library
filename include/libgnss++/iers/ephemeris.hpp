#pragma once

// libgnss++/iers/ephemeris.hpp
//
// Geocentric Sun and Moon positions in the ICRS / GCRS, computed
// from the vendored IAU SOFA library (`iauEpv00` for the Earth-Sun
// heliocentric vector, `iauMoon98` for the geocentric Moon vector).
// These two positions are the lunisolar inputs required by the IERS
// Conventions 2010 solid-earth-tide model implemented in
// libgnss::iers::solidEarthTideDisplacement.
//
// Frame note: these functions return ICRS / GCRS positions. The IERS
// Conventions 2010 solid-tide routine (dehanttideinel) needs the Sun
// and Moon in the station's Earth-fixed frame, so rotate them with
// libgnss::iers::icrsToItrs() first; solidEarthTideDisplacementAt() in
// tides.hpp does this.
//
// Accuracy: SOFA's iauEpv00 has ~15 km positional error vs. JPL DE405
// (~1e-7 of the Sun-Earth distance); iauMoon98 has ~1 km positional
// error vs. high-precision lunar ephemerides. Both are far below the
// noise floor for tidal displacement calculations (which scale as
// 1/r^3) and adequate for any GNSS application that does not require
// JPL-grade planetary tables.

#include <Eigen/Dense>

namespace libgnss::iers {

/// @brief Sun geocentric position in ICRS / GCRS [m].
///
/// @param mjd_utc UTC modified Julian date.
/// @return Sun position vector in the ICRS frame [m].
Eigen::Vector3d sunPositionIcrs(double mjd_utc);

/// @brief Moon geocentric position in ICRS / GCRS [m].
///
/// @param mjd_utc UTC modified Julian date.
/// @return Moon position vector in the ICRS frame [m].
Eigen::Vector3d moonPositionIcrs(double mjd_utc);

}  // namespace libgnss::iers
