#pragma once

// Shared file-local helpers for the CLAS epoch pipeline TU; extracted
// from the former monolithic ppp_clas_epoch.cpp anonymous namespace.

// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/ephemeris.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

extern "C" {
#include "sofa.h"
}


namespace libgnss {
namespace ppp_clas_epoch_internal {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;


inline bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

constexpr double kClasNlDatumJumpThresholdCycles = 0.5;
// Good WL-NL / SD-MAR fixes stay below ~0.7 m; parity-path blunders were 38–595 m.
constexpr double kClasBaseClockParitySdMarMaxPositionShiftM = 2.0;
// Kinematic CLAS: reject fixed positions that jump beyond float prediction.
// RTK PPC gates (PR #177/#178/#179) use 20 m min / 25 m/s adaptive trusted jump;
// at 5 Hz urban (~15 m/s) we tighten to max(1 m, 15 m/s·dt) capped at 8 m, and
// max(1 m, 3σ_float) when covariance is available (ppp.cpp uses 25 m kinematic).
constexpr double kClasKinematicFixedJumpMinM = 1.0;
constexpr double kClasKinematicFixedJumpRateMps = 10.0;
constexpr double kClasKinematicFixedJumpMaxM = 2.0;
constexpr double kClasKinematicFixedJumpSigmaScale = 3.0;
constexpr double kClasKinematicWlnlMaxPositionShiftM = 2.0;
constexpr double kClasKinematicMinFixRatio = 3.0;
constexpr int kClasKinematicMinFixCount = 1;
// MRTKLIB clas.toml rejection.hold_chi_square / fix_chi_square (mrtk_ppp_rtk.c:2312-2315)
constexpr double kMrtklibHoldChiSquareGate = 0.5;
constexpr double kMrtklibFixChiSquareGate = 5.0;
// Measurement-only override for the post-fix DD phase chi-square gate
// (kMrtklibFixChiSquareGate above, applied at its use site further down this
// file). Unset (the default): behavior is exactly kMrtklibFixChiSquareGate,
// bit-identical to before this override existed. Set to a finite float: that
// value replaces the gate threshold for this process, so
// GNSS_PPP_CLAS_KIN_CHISQ_MAX=1e9 effectively disables the gate (chisq is
// never >= 1e9) while e.g. =20 loosens it. Used to measure how much of the
// kinematic AR attrition on nagoya_run3 (292 AR-accepted epochs vs. 250
// scored FIX) this single gate accounts for -- see [CLAS-KIN-CHISQ] reject
// logging at the use site.
inline double clasKinematicChiSquareGateM() {
    static const double gate = [] {
        const char* env = std::getenv("GNSS_PPP_CLAS_KIN_CHISQ_MAX");
        if (env == nullptr) return kMrtklibFixChiSquareGate;
        char* end = nullptr;
        const double parsed = std::strtod(env, &end);
        if (end == env || !std::isfinite(parsed)) {
            return kMrtklibFixChiSquareGate;
        }
        return parsed;
    }();
    return gate;
}
// MRTKLIB clas.toml rejection.l1_l2_residual (pos2-rejionno1) sigma gate used
// by residual_test() to drop individual outlier carrier residuals.
constexpr double kMrtklibPhaseResidualSigmaGate = 2.0;
// MRTKLIB clas.toml rejection.pseudorange_diff / position_error_count
// (pos2-rejdiffpse -> opt.maxdiffp, pos2-poserrcnt; mrtk_ppp_rtk.c:2333-2352)
constexpr double kMrtklibMaxSppDivergenceM = 10.0;
constexpr int kMrtklibMaxSppDivergenceEpochs = 5;
// Maxdiff-only WLNL hold-continuation carve-out: let AR continue validating
// an already-active WLNL hold on an epoch whose masked-SPP seed trips only
// the maxdiff watchdog (see clas_maxdiff_hold_continuation_this_epoch,
// computed below). Two designs were measured and superseded before this one:
//
// v1 (no extra gate beyond maxdiff-only + wlnlHoldStillValid()): diagnosed
// on nagoya_run2 tow 556406-556427 letting the carve-out keep re-engaging
// across an already-multi-epoch-long divergence run, building a hold right
// up to the reset watchdog's teardown threshold (kMrtklibMaxSppDivergence-
// Epochs above); the watchdog then tore the position out from under that
// hold -- either discarding otherwise-good work or, on the post-reset
// re-fix attempt into weak geometry, producing an outright wrong integer.
//
// v2 (require the reset watchdog's own entry divergence-epoch count == 0):
// falsified by measurement. n2 v2 reproduced the identical 323 lost epochs
// (the full destructive 317-epoch block included) -- that counter resets to
// 0 the instant divergence dips <=10 m even mid-episode, so it is ~0 at
// *both* productive and destructive activations and separates nothing.
//
// v3 (this one): gate on the hold's own track record instead of any
// divergence counter. clas_wlnl_hold_.consecutive_fix_count (the entry,
// pre-update-carried-over value -- see its capture at the carve-out site
// below) counts consecutive prior epochs the hold has already survived the
// post-fix chi-square gate. CLAS-HOLDCONT-DBG instrumentation over n2's
// destructive episode and t2's 365 gained-FIX epochs showed a clean split:
// n2's destructive continuation never exceeded consecutive_fix_count=44
// (climbing from a freshly-rebuilt hold, ages 1-2 then 21-44, before the
// reset hit); every t2-productive activation with a consecutive-maxdiff
// streak beyond 5 epochs had consecutive_fix_count >= 71 entering it (the
// four long gained-FIX clusters, ~80% of t2's gains, start their carve-out
// episodes at track-record ages 76/126/145 -- comfortably clear). A raw
// streak-length cap was also tried and rejected: t2 has productive
// activations at streak up to 75, so any cap tight enough to stop n2 (<=5)
// would also gut t2's long clusters, and a streak<=5 allowance alone is a
// superset of what v2 already let through at the start of n2's destructive
// episode (v2's entry==0 gate fired there too, at streak 1-5, and still
// lost the block) -- so streak cannot be the gating axis by itself either.
// Track record (not streak, not the reset watchdog's own counter) is the
// only measured axis that cleanly separates the two cases.
constexpr int kClasHoldContinuationMinTrackRecordFixes = 60;
// Kill switch only (A/B use during this investigation): unset/any value
// other than "-1" runs the age-gated carve-out above; exactly "-1" disables
// the maxdiff-only hold-continuation carve-out entirely (both touch points
// always false, reproducing baseline develop semantics on the parity path
// -- verified bit-identical: raw PPP fixed solutions 2225/float 4046 on
// nagoya_run2 match unmodified develop's own log exactly). The default
// (unset) path does not depend on this env var's *value*, only on its
// absence, so normal runs never need it set.
inline bool clasMaxdiffHoldContinuationDisabledByEnv() {
    static const bool disabled = [] {
        const char* env =
            std::getenv("GNSS_PPP_CLAS_HOLD_CONT_MAX_DIVCNT");
        return env != nullptr && std::atoi(env) == -1;
    }();
    return disabled;
}
// Measurement-only override for kClasHoldContinuationMinTrackRecordFixes
// above (GNSS_PPP_CLAS_HOLD_CONT_MIN_TRACK). PPPEnvOverrides carries the raw
// env int with an unset sentinel of -1; this wrapper substitutes the
// built-in default whenever the override is unset (or explicitly < 0), so
// the default path is bit-identical to before this override existed. This
// remains an experiment-only probe: min_track=10 recovers t2 fixes but is
// rejected for production because n2 loses its 322-fix productive block and
// raises >3 m fixes from 19 to 42.
inline int clasHoldContinuationMinTrackRecordFixes() {
    static const int value = [] {
        const int env_value = pppEnvOverrides().clas_hold_cont_min_track;
        return env_value >= 0 ? env_value : kClasHoldContinuationMinTrackRecordFixes;
    }();
    return value;
}
// A candidate that failed valsol-equivalent validation is weaker evidence than
// an accepted SPP seed. Use it only to recover a catastrophically stale FLOAT
// state, not to turn ordinary urban tens-of-metres disagreement into a reset.
constexpr double kClasRejectedMaxdiffRecoveryM = 250.0;
// Keep a bounded recovery marker after raw maxdiff. It no longer suppresses
// all AR attempts; recovery candidates receive stricter row-count and ratio
// validation below.
constexpr int kClasSeedArQuarantineEpochs = 30;
// Immediate (uncounted) sanity ceiling for the reset seed vs. the filter's
// own tracked position -- see the long comment at its use site. Well above
// any plausible ground-vehicle displacement per epoch, and far below the
// hundreds-of-meters to multi-km blunders this guard targets.
constexpr double kClasSeedImplausibleSpeedMps = 100.0;
constexpr double kClasSeedImplausibleJumpFloorM = 300.0;
constexpr double kClasRejectedOutputSpeedMps = 100.0;
constexpr double kClasRejectedOutputJumpFloorM = 100.0;
constexpr double kClasRejectedOutputTrustedLimitM = 150.0;

// MRTKLIB mrtk_spp.c valsol(): chi-square(alpha=0.001) table indexed by
// degrees of freedom (nv-nx), used to validate every pntpos() solution
// whenever raim_fde (clas.toml pos1-posopt5) is enabled -- which the real
// clas.toml benchmark config does. The parity path first evaluates this gate
// on the baseline seed, then permits leave-one-out FDE only for a rejected or
// already maxdiff-inconsistent seed. Left completely
// unchecked, a multi-GNSS SPP epoch with only 1-2 satellites per non-
// reference system consumes all its redundancy on per-system clock/ISB
// unknowns (dof collapses to 0) and reports a numerically "perfect"
// (near-zero residual) but arbitrarily wrong fix -- observed to reach
// 17 km on the tokyo_run1 benchmark. Re-apply valsol()'s literal chi-square
// gate here (using the seed's own already-computed spp_chi_square /
// spp_degrees_of_freedom) and additionally require dof >= 1: valsol()
// itself only runs the chi-square test when nv>nx, so a dof<=0 fit is
// exactly the gap real MRTKLIB's own gate cannot see either, and is the
// gap this guard closes. Failing either check marks the seed invalid,
// which reuses the filter's existing seed-unavailable handling (clock
// coasting instead of the every-epoch hard clock reseed). The finite rejected
// candidate remains available only to the counted MRTKLIB maxdiffp recovery
// below; it is never an ordinary filter seed.
constexpr std::array<double, 30> kMrtklibChiSquare001Table = {
    10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
    31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
    46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
};

inline bool clasSeedLacksRedundancy(const PositionSolution& seed) {
    return seed.spp_degrees_of_freedom < 1;
}

inline bool clasSeedFailsChiSquareGate(const PositionSolution& seed) {
    const int dof = seed.spp_degrees_of_freedom;
    if (dof < 1) return false;
    const auto table_index = static_cast<std::size_t>(std::min(
        dof, static_cast<int>(kMrtklibChiSquare001Table.size()))) - 1;
    return std::isfinite(seed.spp_chi_square) &&
           seed.spp_chi_square > kMrtklibChiSquare001Table[table_index];
}

inline bool clasSeedFailsRedundancyGate(const PositionSolution& seed) {
    return clasSeedLacksRedundancy(seed) ||
           clasSeedFailsChiSquareGate(seed);
}

using ClasBlqRows = std::array<std::array<double, 11>, 6>;

// Official clas_grid.blq records 7-17, 7-18, 7-20 and 7-21 selected by
// the CLAS OSR interpolation metadata for tokyo/run2 (network 7).
// Rows are radial/west/south amplitudes followed by their phases.
constexpr std::array<ClasBlqRows, 4> kTokyoClasBlq{{
    {{{.00799,.00419,.00123,.00124,.01091,.00848,.00361,.00168,.00034,.00010,.00004},
      {.00259,.00132,.00034,.00037,.00219,.00179,.00073,.00035,.00010,.00003,.00001},
      {.00227,.00078,.00047,.00018,.00194,.00147,.00064,.00029,.00004,.00004,.00004},
      {52.6,72.7,58.9,67.1,-136.5,-154.9,-136.4,-162.6,-13.8,-32.3,-15.2},
      {-14.0,24.6,-26.2,25.2,-170.7,169.5,-170.4,161.7,-34.5,-49.3,-29.8},
      {-79.8,-63.6,-90.5,-68.0,86.0,67.5,86.6,56.0,46.9,33.6,4.8}}},
    {{{.00716,.00386,.00107,.00116,.01031,.00802,.00341,.00158,.00035,.00011,.00005},
      {.00255,.00132,.00033,.00037,.00228,.00187,.00076,.00036,.00011,.00003,.00001},
      {.00201,.00067,.00042,.00015,.00181,.00137,.00060,.00027,.00004,.00004,.00004},
      {49.4,70.0,56.4,64.7,-137.1,-155.5,-137.1,-163.3,-11.4,-26.7,-10.7},
      {-11.0,27.7,-22.6,28.3,-169.6,170.7,-169.4,162.9,-34.4,-48.0,-27.3},
      {-77.6,-62.3,-88.7,-65.7,87.9,69.4,88.5,57.6,44.5,32.8,4.7}}},
    {{{.00904,.00476,.00134,.00142,.01203,.00939,.00398,.00186,.00037,.00011,.00004},
      {.00279,.00140,.00037,.00039,.00230,.00189,.00077,.00036,.00011,.00003,.00001},
      {.00206,.00066,.00043,.00014,.00183,.00137,.00060,.00027,.00004,.00004,.00004},
      {45.2,68.1,50.3,63.2,-138.3,-156.7,-138.2,-164.4,-14.7,-32.4,-14.9},
      {-14.1,24.7,-26.9,25.4,-170.5,169.9,-170.3,162.2,-33.1,-47.7,-27.7},
      {-77.3,-62.1,-89.7,-65.8,89.8,71.7,90.4,60.2,41.9,31.8,4.6}}},
    {{{.00902,.00478,.00129,.00143,.01203,.00940,.00398,.00186,.00040,.00013,.00006},
      {.00280,.00142,.00037,.00040,.00243,.00199,.00081,.00039,.00011,.00003,.00001},
      {.00189,.00062,.00040,.00013,.00178,.00134,.00059,.00026,.00004,.00004,.00004},
      {40.8,65.2,44.8,60.8,-139.3,-157.7,-139.2,-165.4,-12.9,-27.3,-10.7},
      {-11.0,27.8,-23.2,28.4,-169.4,171.3,-169.2,163.6,-32.9,-46.1,-24.9},
      {-78.6,-64.3,-90.3,-68.2,89.2,71.0,89.9,59.3,43.8,32.5,4.7}}}
}};

// igu00p01.erp interpolated by CLASLIB at the tokyo/run2 epoch.  The
// reference passes UTC to geterp()'s GPST parameter, so this includes its
// historical extra GPS-to-UTC conversion.
constexpr double kClasParityUt1MinusUtcSeconds = -0.1530769443333333;
constexpr double kClasParityXpArcsec = 0.21629163958333328;
constexpr double kClasParityYpArcsec = 0.35860480499999975;

inline std::array<double, 5> mrtklibAstronomicalArguments(double centuries) {
    constexpr std::array<std::array<double, 5>, 5> coefficients{{
        {{134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470}},
        {{357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149}},
        {{93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417}},
        {{297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169}},
        {{125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939}},
    }};
    std::array<double, 5> arguments{};
    for (size_t argument = 0; argument < arguments.size(); ++argument) {
        double power = centuries;
        double arcseconds = coefficients[argument][0] * 3600.0;
        for (size_t term = 1; term < coefficients[argument].size(); ++term) {
            arcseconds += coefficients[argument][term] * power;
            power *= centuries;
        }
        arguments[argument] = std::fmod(
            arcseconds * M_PI / (180.0 * 3600.0), 2.0 * M_PI);
    }
    return arguments;
}

inline void mrtklibClasSunMoonItrs(double mjd_utc,
                            Vector3d& sun_itrs,
                            Vector3d& moon_itrs,
                            double& gmst) {
    constexpr double kAuM = 149597870691.0;
    const double mjd_ut1 = mjd_utc + kClasParityUt1MinusUtcSeconds / 86400.0;
    const double centuries_ut1 = (mjd_ut1 - 51544.5) / 36525.0;
    const auto arguments = mrtklibAstronomicalArguments(centuries_ut1);
    const double obliquity =
        (23.439291 - 0.0130042 * centuries_ut1) * M_PI / 180.0;
    const double sin_obliquity = std::sin(obliquity);
    const double cos_obliquity = std::cos(obliquity);

    const double sun_mean_anomaly =
        (357.5277233 + 35999.05034 * centuries_ut1) * M_PI / 180.0;
    const double sun_longitude =
        (280.460 + 36000.770 * centuries_ut1 +
         1.914666471 * std::sin(sun_mean_anomaly) +
         0.019994643 * std::sin(2.0 * sun_mean_anomaly)) * M_PI / 180.0;
    const double sun_range = kAuM *
        (1.000140612 - 0.016708617 * std::cos(sun_mean_anomaly) -
         0.000139589 * std::cos(2.0 * sun_mean_anomaly));
    const Vector3d sun_eci(
        sun_range * std::cos(sun_longitude),
        sun_range * cos_obliquity * std::sin(sun_longitude),
        sun_range * sin_obliquity * std::sin(sun_longitude));

    const double moon_longitude =
        (218.32 + 481267.883 * centuries_ut1 +
         6.29 * std::sin(arguments[0]) -
         1.27 * std::sin(arguments[0] - 2.0 * arguments[3]) +
         0.66 * std::sin(2.0 * arguments[3]) +
         0.21 * std::sin(2.0 * arguments[0]) -
         0.19 * std::sin(arguments[1]) -
         0.11 * std::sin(2.0 * arguments[2])) * M_PI / 180.0;
    const double moon_latitude =
        (5.13 * std::sin(arguments[2]) +
         0.28 * std::sin(arguments[0] + arguments[2]) -
         0.28 * std::sin(arguments[2] - arguments[0]) -
         0.17 * std::sin(arguments[2] - 2.0 * arguments[3])) * M_PI / 180.0;
    const double moon_range = constants::WGS84_A / std::sin(
        (0.9508 + 0.0518 * std::cos(arguments[0]) +
         0.0095 * std::cos(arguments[0] - 2.0 * arguments[3]) +
         0.0078 * std::cos(2.0 * arguments[3]) +
         0.0028 * std::cos(2.0 * arguments[0])) * M_PI / 180.0);
    const double cos_moon_latitude = std::cos(moon_latitude);
    const double sin_moon_latitude = std::sin(moon_latitude);
    const double sin_moon_longitude = std::sin(moon_longitude);
    const Vector3d moon_eci(
        moon_range * cos_moon_latitude * std::cos(moon_longitude),
        moon_range * (cos_obliquity * cos_moon_latitude * sin_moon_longitude -
                      sin_obliquity * sin_moon_latitude),
        moon_range * (sin_obliquity * cos_moon_latitude * sin_moon_longitude +
                      cos_obliquity * sin_moon_latitude));

    // RTKLIB uses TT for IAU 1976/1980 precession-nutation and UT1 for
    // sidereal rotation.  SOFA's legacy primitives implement the same model.
    constexpr double kTaiMinusUtcSeconds = 37.0;
    const double mjd_tt =
        mjd_utc + (kTaiMinusUtcSeconds + 32.184) / 86400.0;
    double pnm80[3][3]{};
    iauPnm80(2400000.5, mjd_tt, pnm80);
    Matrix3d celestial_to_true;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            celestial_to_true(row, col) = pnm80[row][col];
        }
    }
    gmst = iauGmst82(2400000.5, mjd_ut1);
    const double gast = gmst + iauEqeq94(2400000.5, mjd_tt);
    Matrix3d sidereal;
    sidereal << std::cos(gast), std::sin(gast), 0.0,
               -std::sin(gast), std::cos(gast), 0.0,
                0.0, 0.0, 1.0;
    const double xp = kClasParityXpArcsec * M_PI / (180.0 * 3600.0);
    const double yp = kClasParityYpArcsec * M_PI / (180.0 * 3600.0);
    Matrix3d polar_y;
    polar_y << std::cos(-xp), 0.0, -std::sin(-xp),
               0.0, 1.0, 0.0,
               std::sin(-xp), 0.0, std::cos(-xp);
    Matrix3d polar_x;
    polar_x << 1.0, 0.0, 0.0,
               0.0, std::cos(-yp), std::sin(-yp),
               0.0, -std::sin(-yp), std::cos(-yp);
    const Matrix3d eci_to_ecef =
        polar_y * polar_x * sidereal * celestial_to_true;
    sun_itrs = eci_to_ecef * sun_eci;
    moon_itrs = eci_to_ecef * moon_eci;
}

inline Vector3d mrtklibClasBodyTide(const Vector3d& receiver_position,
                            const Vector3d& body_position,
                            double body_gm) {
    constexpr double kEarthGm = 3.986004415e14;
    constexpr double kSunMoonDegree3H = 0.292;
    constexpr double kSunMoonDegree3L = 0.015;
    const double body_range = body_position.norm();
    if (!(body_range > 0.0)) {
        return Vector3d::Zero();
    }

    const Vector3d body_unit = body_position / body_range;
    const Vector3d receiver_up = receiver_position.normalized();
    const double latitude = std::asin(receiver_up.z());
    const double body_latitude = std::asin(body_unit.z());
    const double body_longitude = std::atan2(body_unit.y(), body_unit.x());
    const double receiver_longitude =
        std::atan2(receiver_position.y(), receiver_position.x());
    const double sin_latitude = std::sin(latitude);
    const double cos_latitude = std::cos(latitude);
    const double p = (3.0 * sin_latitude * sin_latitude - 1.0) / 2.0;
    const double h2 = 0.6078 - 0.0006 * p;
    const double l2 = 0.0847 + 0.0002 * p;
    const double earth_radius = constants::WGS84_A;
    const double k2 = body_gm / kEarthGm *
        std::pow(earth_radius, 4) / std::pow(body_range, 3);
    const double k3 = k2 * earth_radius / body_range;
    const double projection = body_unit.dot(receiver_up);

    double tangential = k2 * 3.0 * l2 * projection;
    double radial = k2 *
        (h2 * (1.5 * projection * projection - 0.5) -
         3.0 * l2 * projection * projection);
    tangential += k3 * kSunMoonDegree3L *
        (7.5 * projection * projection - 1.5);
    radial += k3 *
        (kSunMoonDegree3H *
             (2.5 * projection * projection * projection -
              1.5 * projection) -
         kSunMoonDegree3L *
             (7.5 * projection * projection - 1.5) * projection);
    radial += 0.75 * 0.0025 * k2 * std::sin(2.0 * body_latitude) *
        std::sin(2.0 * latitude) *
        std::sin(receiver_longitude - body_longitude);
    radial += 0.75 * 0.0022 * k2 *
        std::pow(std::cos(body_latitude), 2) *
        cos_latitude * cos_latitude *
        std::sin(2.0 * (receiver_longitude - body_longitude));
    return tangential * body_unit + radial * receiver_up;
}

inline Vector3d mrtklibClasSolidTide(const Vector3d& receiver_position,
                             double mjd_utc) {
    constexpr double kSunGm = 1.327124e20;
    constexpr double kMoonGm = 4.902801e12;
    Vector3d sun_itrs = Vector3d::Zero();
    Vector3d moon_itrs = Vector3d::Zero();
    double gmst = 0.0;
    mrtklibClasSunMoonItrs(mjd_utc, sun_itrs, moon_itrs, gmst);
    Vector3d displacement =
        mrtklibClasBodyTide(receiver_position, sun_itrs, kSunGm) +
        mrtklibClasBodyTide(receiver_position, moon_itrs, kMoonGm);

    const double latitude = std::asin(
        receiver_position.z() / receiver_position.norm());
    const double longitude =
        std::atan2(receiver_position.y(), receiver_position.x());
    const double radial_k1 =
        -0.012 * std::sin(2.0 * latitude) * std::sin(gmst + longitude);
    displacement += radial_k1 * receiver_position.normalized();
    // CLASLIB's PPP tide path passes flag=1 and therefore applies the legacy
    // permanent-deformation elimination term (its flag differs from the
    // upstream RTKLIB option-bit API).
    const double permanent_up =
        0.1196 * (1.5 * std::pow(std::sin(latitude), 2) - 0.5);
    const double permanent_north = 0.0247 * std::sin(2.0 * latitude);
    displacement += enu2ecef(
        Vector3d(0.0, permanent_north, permanent_up), latitude, longitude);
    return displacement;
}

inline Vector3d mrtklibClasOceanTide(const Vector3d& receiver_position,
                             double mjd_utc,
                             const ClasBlqRows& blq) {
    constexpr std::array<std::array<double, 5>, 11> kArguments{{
        {{1.40519e-4,  2.0, -2.0,  0.0,  0.00}},
        {{1.45444e-4,  0.0,  0.0,  0.0,  0.00}},
        {{1.37880e-4,  2.0, -3.0,  1.0,  0.00}},
        {{1.45842e-4,  2.0,  0.0,  0.0,  0.00}},
        {{0.72921e-4,  1.0,  0.0,  0.0,  0.25}},
        {{0.67598e-4,  1.0, -2.0,  0.0, -0.25}},
        {{0.72523e-4, -1.0,  0.0,  0.0, -0.25}},
        {{0.64959e-4,  1.0, -3.0,  1.0, -0.25}},
        {{0.53234e-5,  0.0,  2.0,  0.0,  0.00}},
        {{0.26392e-5,  0.0,  1.0, -1.0,  0.00}},
        {{0.03982e-5,  2.0,  0.0,  0.0,  0.00}},
    }};
    constexpr double kMjd1975Jan1 = 42413.0;
    const double utc_seconds_of_day =
        (mjd_utc - std::floor(mjd_utc)) * 86400.0;
    const double days = std::floor(mjd_utc) - kMjd1975Jan1 + 1.0;
    const double t = (27392.500528 + 1.000000035 * days) / 36525.0;
    const double t2 = t * t;
    const double t3 = t2 * t;
    const std::array<double, 5> astronomical{
        utc_seconds_of_day,
        (279.69668 + 36000.768930485 * t + 3.03e-4 * t2) * M_PI / 180.0,
        (270.434358 + 481267.88314137 * t - 0.001133 * t2 + 1.9e-6 * t3) * M_PI / 180.0,
        (334.329653 + 4069.0340329577 * t - 0.010325 * t2 - 1.2e-5 * t3) * M_PI / 180.0,
        2.0 * M_PI};
    Vector3d radial_west_south = Vector3d::Zero();
    for (size_t constituent = 0; constituent < kArguments.size(); ++constituent) {
        double angle = 0.0;
        for (size_t term = 0; term < astronomical.size(); ++term) {
            angle += astronomical[term] * kArguments[constituent][term];
        }
        for (size_t component = 0; component < 3; ++component) {
            radial_west_south[static_cast<Eigen::Index>(component)] +=
                blq[component][constituent] *
                std::cos(angle - blq[component + 3][constituent] * M_PI / 180.0);
        }
    }
    const double latitude = std::asin(
        receiver_position.z() / receiver_position.norm());
    const double longitude =
        std::atan2(receiver_position.y(), receiver_position.x());
    return enu2ecef(
        Vector3d(-radial_west_south.y(),
                 -radial_west_south.z(),
                 radial_west_south.x()),
        latitude,
        longitude);
}

inline Vector3d mrtklibClasPoleTide(const Vector3d& receiver_position,
                            double mjd_utc) {
    const double mjd_ut1 =
        mjd_utc + kClasParityUt1MinusUtcSeconds / 86400.0;
    const double years_since_2000 = (mjd_ut1 - 51544.0) / 365.25;
    const double xp_mean_mas = 23.513 + 7.6141 * years_since_2000;
    const double yp_mean_mas = 358.891 - 0.6287 * years_since_2000;
    const double m1 = kClasParityXpArcsec - xp_mean_mas * 1.0e-3;
    const double m2 = -kClasParityYpArcsec + yp_mean_mas * 1.0e-3;
    const double latitude = std::asin(
        receiver_position.z() / receiver_position.norm());
    const double longitude =
        std::atan2(receiver_position.y(), receiver_position.x());
    const Vector3d enu(
        9.0e-3 * std::sin(latitude) *
            (m1 * std::sin(longitude) - m2 * std::cos(longitude)),
        -9.0e-3 * std::cos(2.0 * latitude) *
            (m1 * std::cos(longitude) + m2 * std::sin(longitude)),
        -33.0e-3 * std::sin(2.0 * latitude) *
            (m1 * std::cos(longitude) + m2 * std::sin(longitude)));
    return enu2ecef(enu, latitude, longitude);
}

inline Vector3d mrtklibTokyoClasTideDisplacement(const Vector3d& receiver_position,
                                          const GNSSTime& time,
                                          int network_id,
                                          const std::array<double, 4>& grid_weights) {
    if (network_id != 7) {
        return Vector3d::Zero();
    }
    const auto* blq = &kTokyoClasBlq;
    if (blq == nullptr || !receiver_position.allFinite() ||
        receiver_position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const bool use_oracle = external::madocalib_oracle::tideAvailable();
    const double rr[3]{receiver_position.x(), receiver_position.y(),
                       receiver_position.z()};
    Vector3d displacement = Vector3d::Zero();
    if (use_oracle) {
        double solid_pole[3]{};
        external::madocalib_oracle::tideDisplacement(
            time.week, time.tow, rr, 1 | 4, nullptr, solid_pole);
        displacement = Vector3d(solid_pole[0], solid_pole[1], solid_pole[2]);
    } else {
        const double mjd_utc = iers::gnssTimeToMjdUtc(time);
        const Vector3d solid = mrtklibClasSolidTide(receiver_position, mjd_utc);
        const Vector3d pole = mrtklibClasPoleTide(receiver_position, mjd_utc);
        displacement = solid + pole;
        if (pppDebugEnabled()) {
            std::cerr << std::setprecision(15)
                      << "[CLAS-TIDE-COMP] tow=" << time.tow
                      << " solid=" << solid.transpose()
                      << " pole=" << pole.transpose() << '\n';
        }
    }

    // CLASLIB applies Emat*Gmat interpolation weights from the selected
    // atmospheric grid, not a fresh bilinear interpolation at the receiver
    // coordinates. Reuse the typed weights carried by the accepted OSR row.
    std::array<double, 4> weights = grid_weights;
    double weight_sum = 0.0;
    for (double weight : weights) weight_sum += weight;
    if (!(weight_sum > 0.0)) {
        return Vector3d::Zero();
    }
    for (double& weight : weights) weight /= weight_sum;

    const Vector3d solid_pole_displacement = displacement;
    for (size_t grid = 0; grid < blq->size(); ++grid) {
        if (!use_oracle) {
            displacement += weights[grid] * mrtklibClasOceanTide(
                receiver_position,
                iers::gnssTimeToMjdUtc(time),
                (*blq)[grid]);
            continue;
        }
        std::array<double, 66> record{};
        for (size_t constituent = 0; constituent < 11; ++constituent) {
            for (size_t row = 0; row < 6; ++row) {
                record[row + constituent * 6] =
                    (*blq)[grid][row][constituent];
            }
        }
        double ocean[3]{};
        external::madocalib_oracle::tideDisplacement(
            time.week, time.tow, rr, 2, record.data(), ocean);
        displacement += weights[grid] * Vector3d(ocean[0], ocean[1], ocean[2]);
    }
    if (pppDebugEnabled()) {
        std::cerr << std::setprecision(15)
                  << "[CLAS-TIDE-COMP] tow=" << time.tow
                  << " ocean="
                  << (displacement - solid_pole_displacement).transpose()
                  << '\n';
    }
    return displacement;
}

inline double clasKinematicHorizontalPositionSigmaM(const PPPState& filter_state) {
    if (filter_state.covariance.rows() < 3 ||
        filter_state.covariance.cols() < 3 ||
        filter_state.pos_index < 0 ||
        filter_state.pos_index + 2 >= filter_state.covariance.rows()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const int base = filter_state.pos_index;
    const double var_xy =
        filter_state.covariance(base, base) + filter_state.covariance(base + 1, base + 1);
    if (!std::isfinite(var_xy) || var_xy <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::sqrt(var_xy);
}

inline double clasKinematicMaxFixedFloatJumpM(
    double dt_seconds,
    double horizontal_position_sigma_m) {
    const double adaptive_limit = rtk_validation::adaptiveJumpLimit(
        dt_seconds,
        kClasKinematicFixedJumpMinM,
        kClasKinematicFixedJumpRateMps);
    double limit = adaptive_limit;
    if (std::isfinite(horizontal_position_sigma_m) && horizontal_position_sigma_m > 0.0) {
        limit = std::max(
            limit,
            kClasKinematicFixedJumpMinM +
                kClasKinematicFixedJumpSigmaScale * horizontal_position_sigma_m);
    }
    return std::min(kClasKinematicFixedJumpMaxM, limit);
}

inline std::ofstream* clasFloatDumpStream() {
    const auto& path = pppEnvOverrides().clas_float_dump_path;
    if (path.empty()) {
        return nullptr;
    }

    static std::ofstream stream;
    static bool initialized = false;
    if (!initialized) {
        initialized = true;
        stream.open(path, std::ios::out | std::ios::trunc);
        if (stream) {
            stream << "record,week,tow,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,"
                   << "ax_mps2,ay_mps2,az_mps2,px_m2,py_m2,pz_m2,"
                   << "pvx_m2ps2,pvy_m2ps2,pvz_m2ps2,pax_m2ps4,pay_m2ps4,"
                   << "paz_m2ps4,clock_m,trop_z_m,num_osr,num_rows,"
                   << "dx_m,dx_y_m,dx_z_m\n";
        }
    }
    return stream ? &stream : nullptr;
}

inline double clasNlPhaseBiasDatumCycles(const OSRCorrection& osr) {
    if (osr.num_frequencies < 2 ||
        osr.frequencies[0] <= 0.0 ||
        osr.frequencies[1] <= 0.0 ||
        std::abs(osr.frequencies[0] - osr.frequencies[1]) < 1.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double f1 = osr.frequencies[0];
    const double f2 = osr.frequencies[1];
    const double lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
    const double alpha1 = f1 / (f1 + f2);
    const double alpha2 = f2 / (f1 + f2);
    return (alpha1 * osr.phase_bias_m[0] + alpha2 * osr.phase_bias_m[1]) /
           lambda_nl;
}

inline void clearClasWlnlFixedDatumState(PPPAmbiguityInfo& ambiguity) {
    ambiguity.is_fixed = false;
    ambiguity.fixed_value = 0.0;
    ambiguity.wl_is_fixed = false;
    ambiguity.wl_fixed_integer = 0;
    ambiguity.nl_is_fixed = false;
    ambiguity.nl_fixed_cycles = 0.0;
    ambiguity.mw_sum_cycles = 0.0;
    ambiguity.mw_count = 0;
    ambiguity.mw_mean_cycles = 0.0;
}

inline bool applyClasNlDatumReset(
    const GNSSTime& time,
    const std::vector<OSRCorrection>& osr_corrections,
    std::map<SatelliteId, PPPAmbiguityInfo>& ambiguity_states,
    bool debug_enabled) {
    bool any_reset = false;
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.num_frequencies < 2) {
            continue;
        }
        const double datum_cycles = clasNlPhaseBiasDatumCycles(osr);
        if (!std::isfinite(datum_cycles)) {
            continue;
        }

        auto& ambiguity = ambiguity_states[osr.satellite];
        if (ambiguity.has_clas_nl_phase_bias_datum) {
            const double step_cycles =
                datum_cycles - ambiguity.clas_nl_phase_bias_datum_cycles;
            if (std::abs(step_cycles) > kClasNlDatumJumpThresholdCycles) {
                any_reset = true;
                clearClasWlnlFixedDatumState(ambiguity);
                const SatelliteId l2_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100)));
                const auto l2_it = ambiguity_states.find(l2_satellite);
                if (l2_it != ambiguity_states.end()) {
                    clearClasWlnlFixedDatumState(l2_it->second);
                }
                if (debug_enabled) {
                    std::cerr << "[CLAS-NL-DATUM] reset "
                              << osr.satellite.toString()
                              << " tow=" << time.tow
                              << " step_cycles=" << step_cycles
                              << " datum_cycles=" << datum_cycles
                              << "\n";
                }
            }
        }
        ambiguity.clas_nl_phase_bias_datum_cycles = datum_cycles;
        ambiguity.has_clas_nl_phase_bias_datum = true;
    }
    return any_reset;
}

inline void dumpClasFloatPosition(
    const GNSSTime& time,
    const PPPState& filter_state,
    const ppp_clas::EpochUpdateResult& epoch_update,
    size_t num_osr) {
    auto* dump = clasFloatDumpStream();
    if (dump == nullptr || !epoch_update.update_stats.updated ||
        filter_state.state.size() < filter_state.total_states ||
        filter_state.total_states <= filter_state.trop_index) {
        return;
    }
    const Vector3d position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const Vector3d velocity =
        filter_state.state.segment(filter_state.vel_index, 3);
    const bool have_acceleration =
        filter_state.accel_index >= 0 &&
        filter_state.accel_index + 2 < filter_state.state.size();
    Vector3d acceleration = Vector3d::Zero();
    if (have_acceleration) {
        acceleration = filter_state.state.segment(filter_state.accel_index, 3);
    }
    Vector3d dx = Vector3d::Zero();
    if (epoch_update.update_stats.dx.size() >= filter_state.pos_index + 3) {
        dx = epoch_update.update_stats.dx.segment(filter_state.pos_index, 3);
    }
    *dump << std::setprecision(17)
          << "FLOAT,"
          << time.week << ','
          << time.tow << ','
          << position.x() << ','
          << position.y() << ','
          << position.z() << ','
          << velocity.x() << ','
          << velocity.y() << ','
          << velocity.z() << ','
          << acceleration.x() << ','
          << acceleration.y() << ','
          << acceleration.z() << ','
          << filter_state.covariance(filter_state.pos_index,
                                     filter_state.pos_index) << ','
          << filter_state.covariance(filter_state.pos_index + 1,
                                     filter_state.pos_index + 1) << ','
          << filter_state.covariance(filter_state.pos_index + 2,
                                     filter_state.pos_index + 2) << ','
          << filter_state.covariance(filter_state.vel_index,
                                     filter_state.vel_index) << ','
          << filter_state.covariance(filter_state.vel_index + 1,
                                     filter_state.vel_index + 1) << ','
          << filter_state.covariance(filter_state.vel_index + 2,
                                     filter_state.vel_index + 2) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index,
                                            filter_state.accel_index)
                  : 0.0) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index + 1,
                                            filter_state.accel_index + 1)
                  : 0.0) << ','
          << (have_acceleration
                  ? filter_state.covariance(filter_state.accel_index + 2,
                                            filter_state.accel_index + 2)
                  : 0.0) << ','
          << filter_state.state(filter_state.clock_index) << ','
          << filter_state.state(filter_state.trop_index) << ','
          << num_osr << ','
          << epoch_update.update_stats.nobs << ','
          << dx.x() << ','
          << dx.y() << ','
          << dx.z() << '\n';
}

inline bool readClasAtmosNetworkId(
    const std::map<std::string, std::string>& epoch_atmos,
    int& network_id) {
    const auto network_it = epoch_atmos.find("atmos_network_id");
    if (network_it == epoch_atmos.end()) {
        return false;
    }
    network_id = std::atoi(network_it->second.c_str());
    return true;
}

inline void resetClasIonosphereStateValues(PPPState& filter_state) {
    for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
        if (state_index >= 0 && state_index < filter_state.total_states) {
            filter_state.state(state_index) = 0.0;
        }
    }
}

inline void applyOptionalSolutionEpochMetadata(
    PositionSolution& solution,
    const GNSSTime& time,
    const PPPConfig& config) {
    if (!config.emit_solution_epoch_time) {
        return;
    }
    solution.time = time;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
}

inline void recordClasDispersionWarmupPhasePairs(
    const ObservationData& obs,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& compensation_by_sat) {
    for (const SatelliteId& satellite : obs.getSatellites()) {
        const Observation* l1 = nullptr;
        const Observation* l2 = nullptr;
        for (const auto& candidate : obs.observations) {
            if (candidate.satellite != satellite || !candidate.valid ||
                !candidate.has_carrier_phase ||
                !std::isfinite(candidate.carrier_phase)) {
                continue;
            }
            const bool is_l1 =
                candidate.signal == SignalType::GPS_L1CA ||
                candidate.signal == SignalType::GPS_L1P ||
                candidate.signal == SignalType::GAL_E1 ||
                candidate.signal == SignalType::QZS_L1CA;
            if (is_l1 && l1 == nullptr) {
                l1 = &candidate;
            }
            if (satellite.system == GNSSSystem::GPS) {
                const bool exact_l2w =
                    candidate.carrier_phase_observation_type == "L2W";
                if (exact_l2w) {
                    l2 = &candidate;
                }
            } else if (satellite.system == GNSSSystem::Galileo &&
                       (candidate.signal == SignalType::GAL_E5A ||
                        (l2 == nullptr && candidate.signal == SignalType::GAL_E5B))) {
                l2 = &candidate;
            } else if (satellite.system == GNSSSystem::QZSS &&
                       candidate.signal == SignalType::QZS_L2C) {
                l2 = &candidate;
            }
        }
        if (l1 == nullptr || l2 == nullptr) {
            continue;
        }
        const double l1_wavelength = signalWavelengthMeters(*l1);
        const double l2_wavelength = signalWavelengthMeters(*l2);
        if (!(l1_wavelength > 0.0) || !(l2_wavelength > 0.0)) {
            continue;
        }
        auto& compensation = compensation_by_sat[satellite];
        while (!compensation.warmup_times.empty() &&
               obs.time - compensation.warmup_times.front() > 120.0) {
            compensation.warmup_times.erase(compensation.warmup_times.begin());
            compensation.warmup_phase_m.erase(
                compensation.warmup_phase_m.begin());
        }
        if (!compensation.warmup_times.empty() &&
            compensation.warmup_times.back() == obs.time) {
            continue;
        }
        compensation.warmup_times.push_back(obs.time);
        compensation.warmup_phase_m.push_back({{
            l1->carrier_phase * l1_wavelength,
            l2->carrier_phase * l2_wavelength}});
    }
}

}  // namespace ppp_clas_epoch_internal
}  // namespace libgnss
