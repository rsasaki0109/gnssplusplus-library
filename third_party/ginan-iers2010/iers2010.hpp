// IERS2010 wrapper header.
//
// Adapted from GeoscienceAustralia/ginan@535ef0a (release v4.1.1):
//     src/cpp/3rdparty/iers2010/iers2010.hpp
//
// Modifications from upstream (per Apache License 2.0 attribution
// requirements; see ./README.md for the full list):
//   - Removed `#include "common/gTime.hpp"` (replaced with direct
//     `double mjd_utc` parameters in the HARDISP API; SOFA UTC->TT
//     conversion replaces ginan's MjDateUtc/MjDateTT helpers inside
//     the vendored .cpp files).
//   - HARDISP namespace `hisp { ... }` declared with `double mjd_utc`
//     epoch arguments instead of ginan's `GTime` (ocean-loading
//     time-domain routines).
//   - Replaced `#include "common/eigenIncluder.hpp"` with a direct
//     `#include <Eigen/Dense>` so this public header does not depend on
//     the libgnss++ internal `shim/` (which is PRIVATE to the vendored
//     .cpp files only).
//
// Original ginan code is licensed under Apache License 2.0; see
// ./LICENSE in this directory.

#pragma once

#include <vector>

#include <Eigen/Dense>

namespace iers2010
{
/// @brief Equatorial radius of the Earth [m].
/// @see Table 1.1: IERS numerical standards, IERS 2010
constexpr const double Re = 6'378'136.6e0;

/// Compute the global total FCULa mapping function.
double fcul_a(double, double, double, double) noexcept;
/*
/// Computes the global total FCULb mapping function.
double fcul_b(double, double, double, double) noexcept;
*/
// Determine the total zenith delay following Mendes and Pavlis, 2004.
int fcul_zd_hpa(double, double, double, double, double, double &, double &, double &) noexcept;

/// @brief Compute tidal corrections of station displacements caused by lunar
/// and solar gravitational attraction. This is just the implementation, use the
/// generic template function instead.
int dehanttideinel_impl(
    double julian_centuries_tt, double fhr_ut,
    const Eigen::Matrix<double, 3, 1> &xsun,
    const Eigen::Matrix<double, 3, 1> &xmon,
    const std::vector<Eigen::Matrix<double, 3, 1>> &xsta_vec,
    std::vector<Eigen::Matrix<double, 3, 1>> &xcor_vec) noexcept;

namespace hisp {

/// Maximum length of an internal output buffer used by the time-domain
/// HARDISP recursion. The original IERS code computes one full chunk
/// (`nl` samples) per call; the libgnss++ wrapper only ever asks for one
/// or a few samples but still re-uses the upstream buffer pattern.
constexpr int nl{600};

/// Number of harmonics in the spline-interpolated tidal admittance.
/// Setting this to 342 reproduces the IERS Conventions 2010 reference
/// implementation precision (~0.1%).
constexpr int nt{342};

/// Number of input harmonics in the BLQ "Scherneck" coefficient block
/// (M2, S2, N2, K2, K1, O1, P1, Q1, Mf, Mm, Ssa).
constexpr int ntin{11};

int recurs(double *, int, const double *, int, const double *, double *);
int shells(double *, int *, int) noexcept;
int spline(int, const double *, const double *, double *, double *);
double eval(double, int, const double *, const double *, const double *);
int read_hardisp_args(double tamp[3][ntin], double tph[3][ntin],
                      const char *filename = nullptr);

/// Compute the frequency (cycles/day) and phase (deg) of one tidal
/// constituent identified by its 6-element Doodson number, evaluated at
/// the given UTC epoch expressed as a Modified Julian Date.
int tdfrph(const int *idood, double mjd_utc, double &freq, double &phase);

/// Spline-interpolate the BLQ "Scherneck" admittance for a single
/// component (radial / west / south) over the 342 reference harmonics
/// of `nt`, evaluated at `mjd_utc`. Outputs the per-harmonic amplitude,
/// frequency and phase. `ntout` returns the number of harmonics
/// actually populated.
int admint(const double *ampin, const double *phin, double mjd_utc,
           double *amp, double *f, double *p, int nin, int &ntout);

/// Time-domain HARDISP main entry: produce `irnt` ocean-tide-loading
/// displacement samples at `samp` second cadence starting at the UTC
/// MJD epoch `mjd_utc`. Output arrays `odu` (radial, +up), `ods`
/// (south) and `odw` (west) must be sized at least `irnt`.
int hardisp_impl(int irnt, double samp, double tamp[3][ntin],
                 double tph[3][ntin], double mjd_utc,
                 double *odu, double *ods, double *odw);

}  // namespace hisp

};
