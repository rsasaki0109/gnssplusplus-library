// libgnss++ shim: replace ginan's GTime epoch with a `double mjd_utc`
// argument and use the vendored IAU SOFA implementations of iauUtctai +
// iauTaitt to derive TT centuries from J2000. This keeps the rest of the
// upstream code (Delaunay arguments, Doodson variable expansion, IERS
// expressions) identical to GeoscienceAustralia/ginan@535ef0a.
#include "iers2010.hpp"
#include <cmath>

extern "C" {
int iauUtctai(double utc1, double utc2, double *tai1, double *tai2);
int iauTaitt(double tai1, double tai2, double *tt1, double *tt2);
}

namespace {

// Convert UTC MJD to TT centuries from J2000.0 using SOFA. The original
// ginan code performed the same chain via dso::datetime / MjDateTT.
double tt_centuries_from_j2000(double mjd_utc) {
    constexpr double kJulianDateMjdReference = 2400000.5;
    constexpr double kJ2000MjdTt = 51544.5;
    double tai1 = 0.0;
    double tai2 = 0.0;
    double tt1 = 0.0;
    double tt2 = 0.0;
    iauUtctai(kJulianDateMjdReference, mjd_utc, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);
    const double mjd_tt = (tt1 - kJulianDateMjdReference) + tt2;
    return (mjd_tt - kJ2000MjdTt) / 36525.0;
}

}  // namespace

/// @details This subroutine returns the frequency and phase of a tidal
///          constituent when its Doodson number is given as input.
///
/// @param[in]  idood Doodson number of a tidal constituent (6-element integer
/// array)
/// @param[in]  itm   Date as integer array in UTC. The format should be:
///                   [year, day_of_year, hours, minutes, seconds].
/// @param[out] freq  Frequency of a tidal constituent
/// @param[out] phase Phase of a tidal constituent (Note 1)
/// @return           Always 0.
///
/// @note
///     -# The phases must be decreased by 90 degrees if the sum of the order
///        and the species number is odd (as for the 2nd degree diurnals, and
///        3rd degree low frequency and semidiurnals).
///        These phases may need further adjustment to allow for the spherical
///        harmonic normalization used; e.g. for that used for the potential
///        by Cartwright and Tayler, 180 degrees must be added for (species,
///        order) = (1,2), (1,3), or (3,3).
///     -# Status:  Class 1 model
///
/// @version 21.05.2015
///
/// @cite iers2010
///
int iers2010::hisp::tdfrph(const int idood[6],
                           double mjd_utc, double &freq,
                           double &phase) {
  // libgnss++ shim: cache the last UTC MJD we evaluated. The original
  // ginan code stored a GTime; here we use a sentinel that is
  // guaranteed to differ from any real MJD argument.
  static double last_mjd_utc = -1.0e300;
  static double d[6];
  static double dd[6];

  //  Test to see if time has changed; if so, set the phases and frequencies
  //  for each of the Doodson arguments
  if (mjd_utc != last_mjd_utc) {

    double mjd = mjd_utc;
    int mjdInt = static_cast<int>(mjd);
    double dayfr = mjd - mjdInt;
    const double t = tt_centuries_from_j2000(mjd_utc);

    // IERS expressions for the Delaunay arguments, in degrees
    const double f1 = 134.9634025100e0 +
                      t * (477198.8675605000e0 +
                           t * (0.0088553333e0 +
                                t * (0.0000143431e0 + t * (-0.0000000680e0))));

    const double f2 = 357.5291091806e0 +
                      t * (35999.0502911389e0 +
                           t * (-0.0001536667e0 +
                                t * (0.0000000378e0 + t * (-0.0000000032e0))));

    const double f3 = 93.2720906200e0 +
                      t * (483202.0174577222e0 +
                           t * (-0.0035420000e0 +
                                t * (-0.0000002881e0 + t * (0.0000000012e0))));

    const double f4 = 297.8501954694e0 +
                      t * (445267.1114469445e0 +
                           t * (-0.0017696111e0 +
                                t * (0.0000018314e0 + t * (-0.0000000088e0))));

    const double f5 = 125.0445550100e0 +
                      t * (-1934.1362619722e0 +
                           t * (0.0020756111e0 +
                                t * (0.0000021394e0 + t * (-0.0000000165e0))));

    // Convert to Doodson (Darwin) variables
    d[0] = 360e0 * dayfr - f4;
    d[1] = f3 + f5;
    d[2] = d[1] - f4;
    d[3] = d[1] - f1;
    d[4] = -f5;
    d[5] = d[2] - f2;

    //  Find frequencies of Delauney variables (in cycles/day), and from
    //+ these the same for the Doodson arguments
    const double fd1 = 0.0362916471e0 + 0.0000000013e0 * t;
    const double fd2 = 0.0027377786e0;
    const double fd3 = 0.0367481951e0 - 0.0000000005e0 * t;
    const double fd4 = 0.0338631920e0 - 0.0000000003e0 * t;
    const double fd5 = -0.0001470938e0 + 0.0000000003e0 * t;
    dd[0] = 1e0 - fd4;
    dd[1] = fd3 + fd5;
    dd[2] = dd[1] - fd4;
    dd[3] = dd[1] - fd1;
    dd[4] = -fd5;
    dd[5] = dd[2] - fd2;

    // copy the just used date to the static one for next use of function
    last_mjd_utc = mjd_utc;

  } // End of intialization (likely to be called only once)

  //  Compute phase and frequency of the given tidal constituent
  freq = 0e0;
  phase = 0e0;
  for (int i = 0; i < 6; i++) {
    freq += static_cast<double>(idood[i]) * dd[i];
    phase += static_cast<double>(idood[i]) * d[i];
  }

  // Adjust phases so that they fall in the positive range 0 to 360
  phase = std::fmod(phase, 360e0);
  if (phase < 0e0)
    phase += 360e0;

  // Finished
  return 0;
}
