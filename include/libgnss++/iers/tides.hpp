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

#include <array>

#include <Eigen/Dense>

#include "libgnss++/core/types.hpp"
#include "libgnss++/iers/earth_rotation.hpp"

namespace libgnss::iers {

/// @brief BLQ "Scherneck" ocean-tide-loading coefficients for one site.
///
/// One station's BLQ block is six rows of 11 columns each: the
/// amplitude (m) and phase (deg) of each of the eleven IERS reference
/// tidal constituents (M2, S2, N2, K2, K1, O1, P1, Q1, Mf, Mm, Ssa)
/// projected onto each of the three displacement components (radial,
/// west, south). The order matches the BLQ format produced by the
/// Onsala / Bos & Scherneck ocean loading service.
struct OceanLoadingBlq {
    std::array<double, 11> radial_amplitudes_m{};
    std::array<double, 11> west_amplitudes_m{};
    std::array<double, 11> south_amplitudes_m{};
    std::array<double, 11> radial_phases_deg{};
    std::array<double, 11> west_phases_deg{};
    std::array<double, 11> south_phases_deg{};
};

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

/// @brief Atmospheric tidal loading coefficients for one site (S1+S2).
///
/// Per IERS Conventions 2010 §7.1.5: the atmospheric tide loading
/// signal is dominated by the diurnal S1 (24-h) and semi-diurnal S2
/// (12-h) constituents. Per-site amplitudes / phase lags can be
/// pre-computed from a global pressure model + site-specific Green's
/// function (e.g. by the TU Wien atmospheric loading service).
///
/// Convention follows the legacy ocean-loading file format: amplitudes
/// in metres, phases in degrees with the standard "lag" sign (a
/// positive phase means the response lags the forcing).
struct AtmosphericTidalLoadingCoefficients {
    /// S1 (diurnal) and S2 (semidiurnal) amplitudes / phases.
    /// Index 0 = S1, 1 = S2.
    std::array<double, 2> radial_amplitudes_m{};
    std::array<double, 2> west_amplitudes_m{};
    std::array<double, 2> south_amplitudes_m{};
    std::array<double, 2> radial_phases_deg{};
    std::array<double, 2> west_phases_deg{};
    std::array<double, 2> south_phases_deg{};
};

/// @brief Atmospheric tidal-loading station displacement (IERS §7.1.5).
///
/// Returns the station displacement due to the diurnal (S1) and
/// semi-diurnal (S2) atmospheric pressure tide. Peak amplitude at
/// mid- and low-latitudes is ~1 mm radial; horizontal is sub-mm.
///
/// Formula (per component): A_i · cos(2π · t / T_i − φ_i), summed
/// over i ∈ {S1, S2} where T_S1 = 86400 s and T_S2 = 43200 s. The
/// `t` argument is GPS-time-derived `seconds since Unix epoch`,
/// which matches the legacy ocean-loading dispatcher's reference
/// frame.
///
/// @param mjd_utc    UTC modified Julian date of the epoch.
/// @param xsta_itrs  Station nominal ITRS / ECEF coordinates [m].
/// @param coeffs     Per-site S1 + S2 amplitudes / phases.
/// @return Displacement vector [m] in ITRS / ECEF; ADD to xsta_itrs.
Eigen::Vector3d atmosphericTidalLoadingDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const AtmosphericTidalLoadingCoefficients& coeffs);

/// @brief HARDISP ocean-tide-loading station displacement at one epoch.
///
/// Wraps `iers2010::hisp::hardisp_impl` (IERS Conventions 2010 §7.1.2).
/// Given the station's BLQ "Scherneck" coefficients and the UTC epoch
/// of interest, returns the instantaneous local-tangent (radial / west
/// / south) ocean-loading displacement [m] using the spline-interpolated
/// admittance over 342 reference harmonics.
///
/// The returned vector is in the station's local *(radial, west,
/// south)* frame — the same convention as the BLQ amplitudes. Callers
/// are responsible for the local-tangent->ECEF rotation if they need
/// the displacement in ITRS / ECEF.
///
/// @param mjd_utc UTC modified Julian date of the epoch.
/// @param blq     Per-site BLQ amplitudes (m) and phases (deg).
/// @return Displacement [m] as `(radial_up, west, south)`.
Eigen::Vector3d oceanLoadingDisplacement(
    double mjd_utc,
    const OceanLoadingBlq& blq);

}  // namespace libgnss::iers
