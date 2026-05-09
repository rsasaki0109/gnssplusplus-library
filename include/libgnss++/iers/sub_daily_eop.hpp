#pragma once

// libgnss++/iers/sub_daily_eop.hpp
//
// IERS Conventions 2010 sub-daily Earth Orientation Parameter (EOP)
// corrections. Adds the high-frequency contributions to xp / yp / UT1
// / LOD that are NOT captured by daily-sampled C04 or Bulletin A
// series:
//
//   - §5.5.1.1 + Tables 5.1a/5.1b: libration corrections from
//     lunar tidal libration of the Celestial Intermediate Pole and
//     UT1 at sub-diurnal periods (10 + 11 = 21 terms).
//
//   - §8.2 + Table 8.2: ocean-tide corrections (Eanes-Ray model;
//     71 diurnal + semidiurnal terms).
//
// Peak amplitudes: ~0.5 mas in xp/yp, ~30 µs in UT1. The corrections
// are sub-mm at the receiver position for typical PPP, but flipping
// them on tightens the modeled CIP location and is part of an
// IERS-conformant cm-level pipeline.

namespace libgnss::iers {

/// @brief Sub-daily corrections to be added to a daily EOP record.
struct SubDailyEopDelta {
    /// Polar motion corrections, arcseconds.
    double dxp_arcsec{0.0};
    double dyp_arcsec{0.0};
    /// UT1-UTC correction, seconds.
    double dut1_seconds{0.0};
    /// LOD correction, seconds.
    double dlod_seconds{0.0};
};

/// @brief Compute the IERS Conventions 2010 sub-daily EOP correction.
///
/// @param mjd_utc            UTC modified Julian date.
/// @param ut1_utc_seconds    UT1-UTC at this epoch from a daily series
///                           (C04 or Bulletin A). Used to derive UT1
///                           for GMST.
/// @return Additive deltas (arcsec / seconds) for xp, yp, UT1, LOD.
///
/// Computation: (i) Delaunay arguments l, l', F, D, Ω at TT via SOFA
/// `iauFal03` etc; (ii) GMST at UT1 via SOFA `iauGmst06` (with the
/// IERS table convention `+π` offset); (iii) sum the harmonic series
/// in Tables 5.1a / 5.1b / 8.2 with the Doodson-style integer
/// multipliers as the dot product against (gmst, l, l', F, D, Ω).
SubDailyEopDelta subDailyEopCorrection(double mjd_utc,
                                       double ut1_utc_seconds);

}  // namespace libgnss::iers
