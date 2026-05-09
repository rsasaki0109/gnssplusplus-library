#pragma once

// libgnss++/iers/tides.hpp
//
// Thin C++ wrappers over the vendored ginan-iers2010 library
// (third_party/ginan-iers2010/), surfacing the IERS Conventions 2010
// solid-earth-tide displacement model in libgnss-native types.
//
// At the centimeter level, the solid-earth tide is the largest single
// non-trivial PPP correction (peak diurnal/semidiurnal amplitude is
// ~30 cm radial, ~10 cm horizontal). Ignoring it sets a hard floor
// on PPP positioning accuracy.

#include <Eigen/Dense>

#include "libgnss++/core/types.hpp"
#include "libgnss++/iers/earth_rotation.hpp"

namespace libgnss::iers {

/// @brief Solid-earth-tide station displacement.
///
/// Wraps `iers2010::dehanttideinel_impl`, which implements IERS
/// Conventions 2010 §7.1.1 (Dehant et al.) — the standard model for
/// solid-earth-tide displacement of a ground station due to lunisolar
/// gravitational forcing. The result is a 3-vector (m) to be ADDED
/// to the station's nominal ITRS coordinates to obtain the tidally
/// displaced instantaneous position.
///
/// FRAME NOTE: although the IERS Conventions 2010 routine
/// documentation describes all inputs as "ECEF", the published
/// reference test case actually uses ICRS positions for Sun and Moon
/// while keeping the station in ITRS. The libgnss::iers wrapper
/// follows the *test-case convention* (which defines the routine's
/// behavior in practice), so:
///
///   - `xsta_itrs` should be the station's ITRS / ECEF position.
///   - `xsun_icrs` and `xmon_icrs` should be Sun / Moon positions in
///     the ICRS / GCRS — *not* rotated to ITRS.
///
/// `libgnss::iers::sunPositionIcrs` and `moonPositionIcrs` (in
/// libgnss++/iers/ephemeris.hpp) return values in the correct frame
/// for direct use here.
///
/// @param mjd_utc        UTC modified Julian date of the epoch.
/// @param xsta_itrs      Station nominal ITRS / ECEF coordinates [m].
/// @param xsun_icrs      Sun ICRS / GCRS coordinates [m] at this epoch.
/// @param xmon_icrs      Moon ICRS / GCRS coordinates [m] at this epoch.
/// @return Displacement vector [m] in the station's ITRS frame; add
///         to `xsta_itrs` to obtain the instantaneously displaced
///         station position.
Eigen::Vector3d solidEarthTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const Eigen::Vector3d& xsun_icrs,
    const Eigen::Vector3d& xmon_icrs);

/// @brief Pole-tide station displacement (IERS Conventions 2010 §7.1.4).
///
/// The pole tide is the centrifugal effect on the deformable Earth
/// from the wandering of the rotation axis with respect to its mean
/// position (m1 = xp - x_mean, m2 = -(yp - y_mean)). Peak amplitude
/// is sub-cm in the horizontal and ~25 mm radial during large polar
/// motion excursions.
///
/// Mean-pole secular model: this implementation uses the IERS 2018
/// linear update (an outcome of the 2017 IERS conventions update —
/// the cubic 1976-2010 polynomial in the original IERS 2010 Technical
/// Note 36 was replaced because polar motion has actually drifted
/// linearly post-2010):
///
///   x_mean(t) =  55.000 +   1.677 · (t - 2000.0)   mas
///   y_mean(t) = 320.500 +   3.460 · (t - 2000.0)   mas
///
/// where t is the decimal year (Julian-year approximation suffices
/// for the secular term).
///
/// Displacement formula (Sr = -33 mm, Stt = -9 mm, Stl = 9 mm in the
/// IERS 2010 §7.1.4 conventions):
///
///   d_radial = -32 · sin(2θ) · (m1·cosλ + m2·sinλ)  mm
///   d_north  =  -9 · cos(2θ) · (m1·cosλ + m2·sinλ)  mm
///   d_east   =  +9 · cos(θ)  · (m1·sinλ - m2·cosλ)  mm
///
/// where θ is geocentric colatitude, λ is east longitude, both
/// derived from `xsta_itrs`.
///
/// Returned displacement is in the local ITRS frame (East, North, Up
/// rotated back into ECEF X/Y/Z) and should be ADDED to the station
/// nominal ITRS position to obtain the tidally displaced position.
///
/// @param mjd_utc    UTC modified Julian date of the epoch.
/// @param xsta_itrs  Station nominal ITRS / ECEF coordinates [m].
/// @param eop        Earth orientation parameters at this epoch.
/// @return Displacement vector [m] in ITRS / ECEF.
Eigen::Vector3d poleTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const EarthOrientationParams& eop);

}  // namespace libgnss::iers
