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

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
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
// MRTKLIB clas.toml rejection.l1_l2_residual (pos2-rejionno1) sigma gate used
// by residual_test() to drop individual outlier carrier residuals.
constexpr double kMrtklibPhaseResidualSigmaGate = 2.0;
// MRTKLIB clas.toml rejection.pseudorange_diff / position_error_count
// (pos2-rejdiffpse -> opt.maxdiffp, pos2-poserrcnt; mrtk_ppp_rtk.c:2333-2352)
constexpr double kMrtklibMaxSppDivergenceM = 10.0;
constexpr int kMrtklibMaxSppDivergenceEpochs = 5;
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
// coasting instead of the every-epoch hard clock reseed, and skipping the
// maxdiffp position/filter reset below) instead of introducing new state
// machinery.
constexpr std::array<double, 30> kMrtklibChiSquare001Table = {
    10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
    31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
    46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
};

bool clasSeedLacksRedundancy(const PositionSolution& seed) {
    return seed.spp_degrees_of_freedom < 1;
}

bool clasSeedFailsChiSquareGate(const PositionSolution& seed) {
    const int dof = seed.spp_degrees_of_freedom;
    if (dof < 1) return false;
    const auto table_index = static_cast<std::size_t>(std::min(
        dof, static_cast<int>(kMrtklibChiSquare001Table.size()))) - 1;
    return std::isfinite(seed.spp_chi_square) &&
           seed.spp_chi_square > kMrtklibChiSquare001Table[table_index];
}

bool clasSeedFailsRedundancyGate(const PositionSolution& seed) {
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

std::array<double, 5> mrtklibAstronomicalArguments(double centuries) {
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

void mrtklibClasSunMoonItrs(double mjd_utc,
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

Vector3d mrtklibClasBodyTide(const Vector3d& receiver_position,
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

Vector3d mrtklibClasSolidTide(const Vector3d& receiver_position,
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

Vector3d mrtklibClasOceanTide(const Vector3d& receiver_position,
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

Vector3d mrtklibClasPoleTide(const Vector3d& receiver_position,
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

Vector3d mrtklibTokyoClasTideDisplacement(const Vector3d& receiver_position,
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

double clasKinematicHorizontalPositionSigmaM(const PPPState& filter_state) {
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

double clasKinematicMaxFixedFloatJumpM(
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

std::ofstream* clasFloatDumpStream() {
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

double clasNlPhaseBiasDatumCycles(const OSRCorrection& osr) {
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

void clearClasWlnlFixedDatumState(PPPAmbiguityInfo& ambiguity) {
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

bool applyClasNlDatumReset(
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

void dumpClasFloatPosition(
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

bool readClasAtmosNetworkId(
    const std::map<std::string, std::string>& epoch_atmos,
    int& network_id) {
    const auto network_it = epoch_atmos.find("atmos_network_id");
    if (network_it == epoch_atmos.end()) {
        return false;
    }
    network_id = std::atoi(network_it->second.c_str());
    return true;
}

void resetClasIonosphereStateValues(PPPState& filter_state) {
    for (const auto& [_, state_index] : filter_state.ionosphere_indices) {
        if (state_index >= 0 && state_index < filter_state.total_states) {
            filter_state.state(state_index) = 0.0;
        }
    }
}

void applyOptionalSolutionEpochMetadata(
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

void recordClasDispersionWarmupPhasePairs(
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

}  // namespace

PositionSolution PPPProcessor::processEpochCLAS(const ObservationData& obs,
                                                 const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = false;
    last_clas_hybrid_fallback_reason_.clear();
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_clas_constrained_fixed_state_valid_ = false;
    if (ppp_config_.clas_mrtklib_float_parity &&
        ppp_config_.kinematic_mode &&
        !ppp_config_.low_dynamics_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.use_dynamics_model) {
        recordClasDispersionWarmupPhasePairs(
            obs, clas_dispersion_compensation_);
    }
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution && !ppp_config_.use_ionosphere_free) {
        ppp_config_.ar_method = PPPConfig::ARMethod::DD_WLNL;
        // CLAS corrections stabilize MW rapidly; fewer averaging epochs needed.
        if (ppp_config_.wl_min_averaging_epochs > 5) {
            ppp_config_.wl_min_averaging_epochs = 5;
        }
    }

    struct ClasFallbackSnapshot {
        PPPState filter_state;
        bool filter_initialized = false;
        GNSSTime convergence_start_time;
        Vector3d static_anchor_position = Vector3d::Zero();
        bool has_static_anchor_position = false;
        std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
        std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
        std::map<SatelliteId, double> windup_cache;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
        bool has_last_processed_time = false;
        GNSSTime last_processed_time;
        int last_clas_atmos_network_id = -1;
        bool has_last_clas_atmos_network_id = false;
        PositionSolution last_valid_spp_seed;
        bool has_last_valid_spp_seed = false;
        Vector3d last_rejected_output_position = Vector3d::Zero();
        GNSSTime last_rejected_output_time;
        bool has_last_rejected_output = false;
    };
    const ClasFallbackSnapshot fallback_snapshot{
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        windup_cache_,
        clas_phase_bias_repair_,
        has_last_processed_time_,
        last_processed_time_,
        last_clas_atmos_network_id_,
        has_last_clas_atmos_network_id_,
        clas_last_valid_spp_seed_,
        has_clas_last_valid_spp_seed_,
        clas_last_rejected_output_position_ecef_,
        clas_last_rejected_output_time_,
        has_clas_last_rejected_output_,
    };
    const auto restore_clas_snapshot = [&]() {
        filter_state_ = fallback_snapshot.filter_state;
        filter_initialized_ = fallback_snapshot.filter_initialized;
        convergence_start_time_ = fallback_snapshot.convergence_start_time;
        static_anchor_position_ = fallback_snapshot.static_anchor_position;
        has_static_anchor_position_ = fallback_snapshot.has_static_anchor_position;
        ambiguity_states_ = fallback_snapshot.ambiguity_states;
        clas_dispersion_compensation_ = fallback_snapshot.dispersion_compensation;
        clas_sis_continuity_ = fallback_snapshot.sis_continuity;
        windup_cache_ = fallback_snapshot.windup_cache;
        clas_phase_bias_repair_ = fallback_snapshot.phase_bias_repair;
        has_last_processed_time_ = fallback_snapshot.has_last_processed_time;
        last_processed_time_ = fallback_snapshot.last_processed_time;
        last_clas_atmos_network_id_ =
            fallback_snapshot.last_clas_atmos_network_id;
        has_last_clas_atmos_network_id_ =
            fallback_snapshot.has_last_clas_atmos_network_id;
        clas_last_valid_spp_seed_ = fallback_snapshot.last_valid_spp_seed;
        has_clas_last_valid_spp_seed_ =
            fallback_snapshot.has_last_valid_spp_seed;
        clas_last_rejected_output_position_ecef_ =
            fallback_snapshot.last_rejected_output_position;
        clas_last_rejected_output_time_ =
            fallback_snapshot.last_rejected_output_time;
        has_clas_last_rejected_output_ =
            fallback_snapshot.has_last_rejected_output;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        return processEpochStandard(obs, nav, reason);
    };

    const bool clas_mrtklib_parity =
        ppp_config_.clas_mrtklib_float_parity &&
        ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode &&
        ppp_config_.use_clas_osr_filter && ppp_config_.use_dynamics_model;
    PositionSolution seed;
    // Set below (parity path only) once the redundancy/jump guards have run;
    // see the long comment at its assignment for what this gates.
    bool clas_seed_untrusted_this_epoch = false;
    bool clas_baseline_seed_maxdiff_this_epoch = false;
    bool clas_seed_failed_before_continuity_fallback = false;
    bool clas_seed_ar_recovery_this_epoch = false;
    PositionSolution clas_continuity_output;
    bool has_clas_continuity_output = false;
    bool clas_rejected_seed_output_prepared = false;
    if (clas_mrtklib_parity) {
        const auto original_spp_config = spp_processor_.getSPPConfig();
        auto clas_spp_config = original_spp_config;
        // MRTKLIB CLAS pntpos only admits constellations represented by the
        // current CLAS SSR mask.  The benchmark stream contains GPS, Galileo,
        // and QZSS; feeding unrelated broadcast-only BDS/GLO observations to
        // the reset seed changes the kinematic float trajectory.
        clas_spp_config.enable_beidou = false;
        clas_spp_config.enable_glonass = false;
        // pntpos() switches a multi-frequency configuration to code IFLC.
        // Its CLAS err[] model is approximately one metre plus elevation,
        // with IFLC's 3x sigma multiplier; native's SNR/atmosphere variance
        // and MAD clipping are not part of that solve.
        clas_spp_config.use_ionosphere_free_combination = true;
        clas_spp_config.mrtklib_iflc_code_bias = true;
        clas_spp_config.mrtklib_clas_snr_mask = true;
        // rtkpos() forces EPHOPT_BRDC for the PPP-RTK pntpos() seed even
        // though the subsequent CLAS filter uses SSR APC products.
        clas_spp_config.use_ssr_corrections = false;
        clas_spp_config.pseudorange_sigma = 1.0;
        clas_spp_config.use_variance_model = false;
        clas_spp_config.enable_outlier_detection = false;
        clas_spp_config.elevation_mask_override_deg = 15.0;
        // First solve the baseline exactly once. Applying native's
        // leave-one-out selection to every valid epoch changes otherwise-good
        // reset seeds and AR cadence. Retry with FDE below only when the seed
        // has already failed the parity validity/maxdiff tests.
        clas_spp_config.enable_raim_fde = false;
        spp_processor_.setSPPConfig(clas_spp_config);
        seed = spp_processor_.processEpoch(obs, nav);
        const bool clas_seed_redundancy_failed =
            seed.isValid() && clasSeedFailsRedundancyGate(seed);
        double baseline_filter_spp_distance_m =
            std::numeric_limits<double>::quiet_NaN();
        if (seed.isValid() && filter_initialized_ &&
            filter_state_.pos_index >= 0 &&
            filter_state_.pos_index + 2 < filter_state_.state.size()) {
            baseline_filter_spp_distance_m =
                (filter_state_.state.segment<3>(filter_state_.pos_index) -
                 seed.position_ecef).norm();
        }
        const bool clas_seed_needs_fde =
            ppp_shared::shouldRetryClasSeedWithFde(
                seed.isValid(), clas_seed_redundancy_failed,
                filter_initialized_, baseline_filter_spp_distance_m,
                kMrtklibMaxSppDivergenceM);
        clas_baseline_seed_maxdiff_this_epoch =
            seed.isValid() && filter_initialized_ &&
            std::isfinite(baseline_filter_spp_distance_m) &&
            baseline_filter_spp_distance_m > kMrtklibMaxSppDivergenceM;
        if (clas_seed_needs_fde) {
            clas_spp_config.enable_raim_fde = true;
            spp_processor_.setSPPConfig(clas_spp_config);
            const PositionSolution fde_seed =
                spp_processor_.processEpoch(obs, nav);
            if (fde_seed.isValid()) {
                seed = fde_seed;
            }
        }
        // pntpos() failure includes valsol() rejecting an otherwise finite LS
        // solution (chi-square/redundancy), not just failure to form the LS.
        // Capture the final FDE result using that same definition; the two
        // failure classes have different sol.rr semantics below.
        const bool clas_seed_chi_square_failed =
            seed.isValid() && clasSeedFailsChiSquareGate(seed);
        const bool clas_seed_dof_failed =
            seed.isValid() && clasSeedLacksRedundancy(seed);
        const PositionSolution clas_validation_rejected_candidate = seed;
        // MRTKLIB's estpos() writes the converged current-epoch sol.rr and
        // sol.time before valsol() checks chi-square.  If valsol() then
        // returns false, dynamics mode still enters ppp_rtk_pos() with that
        // current SPP position; it is not the previous epoch's stale sol.
        // The full six-run gate showed that admitting ordinary chi-square
        // failures to the filter helps Tokyo 2 but regresses Nagoya 2 all-
        // solution RMS and FIX rate. Keep both validation failures out of
        // filter/maxdiff state; a bounded output-only path below can still
        // publish their current SPP coordinates without changing lifecycle.
        const bool clas_masked_spp_admission_failed =
            !seed.isValid() || clas_seed_chi_square_failed ||
            clas_seed_dof_failed;
        spp_processor_.setSPPConfig(original_spp_config);
        if (clas_seed_chi_square_failed || clas_seed_dof_failed) {
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-SEED-CHI2] reject tow=" << obs.time.tow
                          << " dof=" << seed.spp_degrees_of_freedom
                          << " chi2=" << seed.spp_chi_square
                          << " ns=" << seed.num_satellites << "\n";
            }
            seed.status = SolutionStatus::NONE;
        }
        // Second, complementary guard: a sparse multi-GNSS SPP epoch can
        // still pass the chi-square/dof check above (enough redundancy for
        // a numerically fine fit) while one of its few satellites carries an
        // undetected bias (bad ephemeris/SSR lookup, multipath) that a
        // single-satellite RAIM/FDE exclusion would normally catch. Observed
        // on nagoya_run2 tow 557038.4: dof=1, chi2=2.27 (well under the
        // dof=1 table entry 10.8), yet the fix was ~4.2 km from the last
        // published position. That epoch is also mid-reset (filter_
        // initialized_ is false: this urban segment cycles through repeated
        // cold reinitialization, never holding a fix long enough to
        // accumulate 5 maxdiffp-counted epochs below), so comparing against
        // filter_state_ would miss it -- there is no filter_state_ yet.
        // last_published_solution_position_ecef_ is the right reference: it
        // is updated from any isValid() publication regardless of filter_
        // initialized_/resets, so it still holds the last trustworthy fix
        // across a cold reinit. Reject a seed that implies covering more
        // than kClasSeedImplausibleSpeedMps in dt: no ground vehicle does,
        // regardless of how well the seed fits its own (possibly biased)
        // measurements.
        if (seed.isValid() && has_last_published_solution_position_) {
            const double dt = has_last_processed_time_
                ? std::max(obs.time - last_processed_time_, 0.001)
                : 1.0;
            const double seed_vs_last_published_m =
                (seed.position_ecef - last_published_solution_position_ecef_)
                    .norm();
            const double implausible_jump_m = std::max(
                kClasSeedImplausibleJumpFloorM,
                kClasSeedImplausibleSpeedMps * dt);
            if (seed_vs_last_published_m > implausible_jump_m) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-SEED-JUMP] reject tow=" << obs.time.tow
                              << " jump=" << seed_vs_last_published_m
                              << " limit=" << implausible_jump_m
                              << " dt=" << dt << "\n";
                }
                seed.status = SolutionStatus::NONE;
            }
        }
        if (seed.isValid()) {
            clas_last_valid_spp_seed_ = seed;
            has_clas_last_valid_spp_seed_ = true;
            clas_last_rejected_output_position_ecef_ = seed.position_ecef;
            clas_last_rejected_output_time_ = obs.time;
            has_clas_last_rejected_output_ = true;
        }
        // Captured before the output-only coast splice below: this is the
        // guards' true verdict on the raw SPP this epoch
        // (redundancy/chi-square gate, jump gate, or a plain
        // pre-existing SPP failure -- e.g. <4 usable satellites -- all
        // collapse to !seed.isValid() here). Debug evidence on
        // nagoya_run2 (tow 556698-556733, a 176-epoch/35 s continuous
        // chi-square-reject stretch) showed the coast/tt-freeze mechanism
        // above stops position blind-extrapolation but does nothing to
        // stop WLNL AR: ambiguity eligibility is governed purely by
        // lock_count (phase continuity), which is completely decoupled
        // from seed quality, so AR kept resolving and PUBLISHING a stable
        // wrong fix (~12.2 m error, all 176/176 epochs guard-rejected)
        // every single epoch of the coast. Suppress the AR *attempt*
        // itself on any epoch whose own seed is untrusted -- mirroring
        // MRTKLIB's pntpos-failure semantics one step further: a stale/
        // coasted state should not be allowed to originate a new
        // publishable fix, only to keep the float filter alive until a
        // trustworthy seed returns.
        clas_seed_ar_recovery_this_epoch =
            ppp_shared::updateClasSeedArQuarantine(
                clas_baseline_seed_maxdiff_this_epoch,
                kClasSeedArQuarantineEpochs,
                clas_seed_ar_quarantine_epochs_);
        clas_seed_failed_before_continuity_fallback = !seed.isValid();
        clas_seed_untrusted_this_epoch =
            clas_seed_failed_before_continuity_fallback ||
            clas_seed_chi_square_failed ||
            clas_baseline_seed_maxdiff_this_epoch;
        // Suppressing this epoch's AR *attempt* alone is not enough: lock
        // counts are untouched by that gate, so a short (2-3 epoch)
        // rejection window can still leave every ambiguity's lock_count
        // comfortably above the min_lock_count=1 eligibility floor
        // (ppp_ar.cpp:210) the instant a seed is accepted again -- no
        // cooldown. Observed on tokyo_run1 tow 188097.0-188097.4 (~8.45 m,
        // 3 epochs): the seed was accepted there (no CHI2/JUMP reject in
        // the debug log), but a rejection/coast stretch had ended only
        // ~2.2 s (11 epochs) earlier at tow 188092.8, and the filter fixed
        // a wrong integer almost immediately on reconvergence. Reset every
        // tracked ambiguity's lock_count to -minlock on a rejected epoch,
        // mirroring udbias_ppp()'s existing outage-reset semantics
        // (mrtk_ppp_rtk.c ~865-875; the identical -kMrtklibMinLock/
        // outage_count=0 pattern already used for the floatcnt and
        // outage_gap resets elsewhere in this function): this reuses the
        // existing lock_count>=min_lock_count gate to enforce a natural
        // kMrtklibMinLock+1-epoch (6 accepted-phase-epoch) cooldown after
        // the rejected seed becomes trustworthy again.
        // A raw maxdiff sample does not invalidate MRTKLIB's ambiguity locks:
        // AR runs before cntdiffp is updated, and state reset only happens
        // after poserrcnt consecutive excesses. Keep the ambiguity cooldown
        // for an actually failed SPP seed, but let a maxdiff-only recovery
        // epoch reuse the still-valid lock history.
        if (clas_seed_failed_before_continuity_fallback ||
            clas_seed_chi_square_failed) {
            constexpr int kMrtklibMinLock = 5;
            for (auto& [_, ambiguity] : ambiguity_states_) {
                ambiguity.lock_count = -kMrtklibMinLock;
                ambiguity.outage_count = 0;
            }
        }
        // Validation protects the filter from inconsistent or
        // underdetermined solves, but freezing its last good SPP as the
        // public SINGLE position can accumulate hundreds of metres as the
        // receiver keeps moving. Keep the filter rejection unchanged and
        // use the finite candidate only as output when it is continuous with
        // the preceding SPP/output candidate and remains within 150 m of the
        // last trustworthy publication. The first condition rejects abrupt
        // 4.2 km/17 km rank-collapse jumps; the second prevents a sequence of
        // individually small steps from drifting unboundedly (observed on
        // Nagoya 2).
        const bool clas_seed_validation_failed =
            clas_seed_chi_square_failed || clas_seed_dof_failed;
        if (clas_seed_validation_failed &&
            has_clas_last_rejected_output_) {
            const double rejected_dt = std::max(
                obs.time - clas_last_rejected_output_time_, 0.0);
            const double rejected_jump =
                (clas_validation_rejected_candidate.position_ecef -
                 clas_last_rejected_output_position_ecef_).norm();
            const double trusted_distance = has_last_published_solution_position_
                ? (clas_validation_rejected_candidate.position_ecef -
                   last_published_solution_position_ecef_).norm()
                : std::numeric_limits<double>::infinity();
            if (ppp_shared::clasRejectedSeedOutputIsContinuous(
                    clas_validation_rejected_candidate.isValid(),
                    clas_seed_validation_failed, true, rejected_jump,
                    rejected_dt, kClasRejectedOutputJumpFloorM,
                    kClasRejectedOutputSpeedMps, trusted_distance,
                    kClasRejectedOutputTrustedLimitM)) {
                clas_continuity_output = clas_validation_rejected_candidate;
                clas_continuity_output.status = SolutionStatus::SPP;
                clas_continuity_output.velocity_ecef = Vector3d::Zero();
                has_clas_continuity_output = true;
                clas_rejected_seed_output_prepared = true;
                clas_last_rejected_output_position_ecef_ =
                    clas_validation_rejected_candidate.position_ecef;
                clas_last_rejected_output_time_ = obs.time;
            }
        }
        // Coverage fallback for a seed that has no usable current position.
        // mrtk_rtkpos.c:2417-2425: when pntpos() fails, MRTKLIB's caller
        // only skips the epoch outright in the static branch
        // (!rtk->opt.dynamics -> return 0); in dynamics mode (our path) it
        // falls through and keeps running ppp_rtk_pos(). A failure before
        // estpos() converges leaves the prior sol.rr/time in place, whereas
        // the chi-square failure handled above has already written the
        // current position/time. Native has no usable current seed in this
        // branch, so mirror the former case with a continuity row while
        // leaving the invalid seed and filter lifecycle unchanged.
        // A plain masked-admission failure commonly reports fewer than four
        // satellites in native's invalid result. MRTKLIB still coasts in that
        // exact case because an early pntpos() failure leaves the prior
        // sol.rr untouched. Keep the >=4 guard only for native dof/jump
        // rejections, where it prevents manufacturing an epoch without a
        // minimally viable current solve.
        const bool coast_from_last_spp =
            clas_masked_spp_admission_failed &&
            has_clas_last_valid_spp_seed_;
        const bool coast_from_last_published =
            !clas_masked_spp_admission_failed &&
            has_last_published_solution_position_ &&
            ppp_shared::shouldCoastClasSeed(
                false, filter_initialized_,
                static_cast<int>(seed.satellites_used.size()));
        if (!seed.isValid() && !has_clas_continuity_output &&
            (coast_from_last_spp || coast_from_last_published)) {
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-SEED-COAST] tow=" << obs.time.tow
                          << " coasting to last published position\n";
            }
            PositionSolution coast_seed = seed;
            if (coast_from_last_spp) {
                coast_seed = clas_last_valid_spp_seed_;
                coast_seed.time = obs.time;
            } else {
                coast_seed.position_ecef =
                    last_published_solution_position_ecef_;
                coast_seed.num_satellites =
                    static_cast<int>(coast_seed.satellites_used.size());
            }
            coast_seed.velocity_ecef = Vector3d::Zero();
            coast_seed.status = SolutionStatus::SPP;
            // Preserve MRTKLIB's stale-sol continuity row without feeding it
            // into native's filter. The original invalid seed must continue
            // through the unchanged clear-path lifecycle; only an otherwise
            // invalid return value is spliced below. Injecting the stale code
            // position into either cold initialization or a running filter
            // can originate delayed wrong fixes.
            clas_continuity_output = coast_seed;
            has_clas_continuity_output = coast_seed.isValid();
        }
    } else {
        seed = spp_processor_.processEpoch(obs, nav);
    }
    if (clas_mrtklib_parity && pppDebugEnabled() &&
        (std::abs(std::fmod(obs.time.tow, 3.0)) < 1e-6)) {
        std::cerr << "[CLAS-SPP-SEED] tow=" << obs.time.tow
                  << " rr=" << seed.position_ecef.transpose()
                  << " ns=" << seed.satellites_used.size()
                  << " sats=";
        for (const auto& sat : seed.satellites_used) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << " rejected=";
        for (const auto& sat : seed.spp_rejected_satellites) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << '\n';
    }
    clas_mrtklib_ar_rejected_ambiguities_.clear();
    // MRTKLIB mrtk_rtkpos.c ~2395-2416: an early pntpos() failure leaves
    // rtk->sol.time unchanged and therefore freezes tt. Although valsol()
    // chi-square failure retains the current MRTKLIB sol.time/position,
    // admitting that candidate into native's filter regressed Nagoya 2 in
    // the six-run gate. Keep native's prior freeze lifecycle for every seed
    // rejection; only publication uses the bounded current candidate above.
    // Capture "was the filter already running before this epoch" here,
    // before the floatcnt/measurement-update resets below can change
    // filter_initialized_, so the freeze below only ever applies to a
    // genuine mid-stream coast (never a cold/re-init epoch, which already
    // has its own seed-availability handling).
    const bool clas_seed_guard_rejected_mid_stream =
        clas_mrtklib_parity && filter_initialized_ &&
        clas_seed_failed_before_continuity_fallback;
    // Advances clas_last_accepted_seed_time_ alongside every
    // last_processed_time_ update below, except on an epoch the guards
    // above just rejected -- i.e. exactly the pntpos-success gating
    // described above. Called at each of the existing bookkeeping sites
    // regardless of which later stage (OSR availability, measurement
    // update) the epoch ultimately exits through, since MRTKLIB's tt
    // freeze is governed solely by pntpos, not by ppp_rtk_pos()'s own
    // later success/failure.
    const auto clas_update_seed_anchor = [&]() {
        if (clas_mrtklib_parity && !clas_seed_guard_rejected_mid_stream) {
            clas_last_accepted_seed_time_ = obs.time;
            has_clas_last_accepted_seed_time_ = true;
        }
    };
    // The canonical raw-L6 CLASLIB oracle enters ppp_rtk_pos() at the first
    // observation and advances its float/iono/ambiguity states immediately.
    // Its first five epochs are FLOAT and it then fixes; it does not skip a
    // 15-second filter warm-up. Keep only the stream-origin bookkeeping used
    // by diagnostics. Publication quality comes from the filter result.
    if (clas_mrtklib_parity) {
        if (!has_clas_mrtklib_stream_start_time_) {
            clas_mrtklib_stream_start_time_ = obs.time;
            has_clas_mrtklib_stream_start_time_ = true;
        }
    }
    // MRTKLIB v0.5.1 mrtk_ppp_rtk.c:1989-2002, clas.toml float_count=15.
    // On the kinematic path every state is zeroed before udstate_ppp(), which
    // then initializes position from the current SPP solution and recreates
    // ionosphere/ambiguity states. Persistent OSR continuity contexts are not
    // part of x and intentionally survive this reset.
    constexpr int kMrtklibFloatResetEpochs = 15;
    bool clas_mrtklib_floatcnt_reset_this_epoch = false;
    if (clas_mrtklib_parity &&
        clas_mrtklib_float_count_ >= kMrtklibFloatResetEpochs) {
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-FLOATCNT] reset after "
                      << clas_mrtklib_float_count_ << " FLOAT epochs tow="
                      << obs.time.tow
                      << " spp_rr=" << seed.position_ecef.transpose()
                      << "\n";
        }
        filter_state_ = PPPState{};
        filter_initialized_ = false;
        ambiguity_states_.clear();
        est_stec_outage_.clear();
        clas_dd_accumulator_ = {};
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        last_clas_constrained_fixed_state_valid_ = false;
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        clas_mrtklib_float_count_ = 0;
        clas_mrtklib_floatcnt_reset_this_epoch = true;
    }
    // The parity path runs detectClasCycleSlips() below on OSR phase-bias-
    // corrected GF/MW. Running the generic detector first stores raw GF/MW
    // after an LLI reset, so the CLAS detector compares corrected against raw
    // combinations and falsely resets nearly every ambiguity every epoch.
    if (!clas_mrtklib_parity) {
        detectCycleSlips(obs, nav);
    }
    // MRTKLIB literal-port track: clas.toml [kalman_filter.initial_std]
    // bias = 100 cycles (mrtk_ppp_rtk.c:792). The scalar is passed through
    // the shared reset machinery here; the parity path converts freshly
    // reset entries to meter^2 once the per-frequency OSR wavelengths exist.
    // Dynamics-model kinematic CLAS
    // only; other paths keep the historical 3600 / 1e6 values.
    const double clas_ambiguity_initial_variance =
        clas_mrtklib_parity
            ? 1e4
            : (precise_products_loaded_ ? 1e6
                                        : ppp_config_.initial_ambiguity_variance);
    // Feed prepareEpochState()/predictFilterState() the tt-style anchor
    // described above instead of the plain last_processed_time_ bookkeeping
    // whenever this is the parity path: clas_last_accepted_seed_time_ only
    // moves forward on epochs the seed guard accepted (see the six update
    // sites below), so a guard-rejected mid-stream epoch here synthesizes a
    // near-zero dt (freezing position/velocity/clock/iono/ambiguity
    // propagation for exactly this epoch, mirroring tt==0), while the
    // return-to-accepted epoch after one or more rejections naturally sees
    // the full accumulated real gap in one predict step -- exactly as
    // MRTKLIB's own tt does. Non-parity paths (static CLAS anchors,
    // white-noise mode) are untouched: they keep using last_processed_time_
    // directly, unaffected by this guard.
    bool clas_prepare_has_last_processed_time = has_last_processed_time_;
    GNSSTime clas_prepare_last_processed_time = last_processed_time_;
    if (clas_mrtklib_parity) {
        if (clas_seed_guard_rejected_mid_stream) {
            clas_prepare_has_last_processed_time = true;
            clas_prepare_last_processed_time = obs.time - 0.001;
        } else {
            clas_prepare_has_last_processed_time = has_clas_last_accepted_seed_time_;
            clas_prepare_last_processed_time = clas_last_accepted_seed_time_;
        }
    }
    const auto epoch_preparation = ppp_clas::prepareEpochState(
        obs,
        seed,
        ssr_products_,
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ppp_config_,
        modeledZenithTroposphereDelayMeters(seed.position_ecef, obs.time),
        clas_prepare_has_last_processed_time,
        clas_prepare_last_processed_time,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_phase_bias_repair_,
        clas_ambiguity_initial_variance);
    if (!epoch_preparation.ready) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("prepare_epoch_state");
        }
        if (has_clas_continuity_output) {
            solution = clas_continuity_output;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            // Output-only splice: preserve the exact pre-existing filter
            // lifecycle. Advancing time/counters here changes the dt seen by
            // the next genuinely accepted seed and suppresses later FIX
            // recovery, even though no filter epoch was processed now.
            return solution;
        }
        return solution;
    }

    auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr_products_,
        filter_state_.state.segment(0, 3),
        filter_state_.state(filter_state_.clock_index),
        filter_state_.state(filter_state_.trop_index),
        ppp_config_,
        windup_cache_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        clas_phase_bias_repair_);
    if (clas_mrtklib_parity) {
        int tide_network_id = -1;
        std::array<double, 4> tide_grid_weights{{0.0, 0.0, 0.0, 0.0}};
        bool have_tide_network = readClasAtmosNetworkId(
            epoch_context.epoch_atmos_tokens, tide_network_id);
        bool have_tide_weights = false;
        {
            // The lifecycle matrix can materialize per-satellite atmosphere
            // without retaining a single epoch-level network token. CLASLIB
            // still applies the receiver tide for the selected service area;
            // recover that typed selection from the accepted OSR rows.
            for (const auto& osr : epoch_context.osr_corrections) {
                if (osr.valid && osr.atmos_network_id > 0) {
                    if (!have_tide_network) {
                        tide_network_id = osr.atmos_network_id;
                        have_tide_network = true;
                    }
                    if (osr.atmos_network_id == tide_network_id) {
                        double sum = 0.0;
                        for (double weight : osr.atmos_interpolation_weights) {
                            sum += weight;
                        }
                        if (sum > 0.0) {
                            tide_grid_weights = osr.atmos_interpolation_weights;
                            have_tide_weights = true;
                            break;
                        }
                    }
                }
            }
        }
        if (have_tide_network && have_tide_weights) {
            const Vector3d tide = mrtklibTokyoClasTideDisplacement(
                epoch_context.receiver_position, obs.time, tide_network_id,
                tide_grid_weights);
            epoch_context.receiver_tide_displacement = tide;
            if (pppDebugEnabled() && tide.squaredNorm() > 0.0) {
                std::cerr << "[CLAS-TIDE] tow=" << obs.time.tow
                          << " net=" << tide_network_id
                          << " d=" << tide.transpose() << "\n";
            }
        }
    }
    materializeClasReceiverAntennaCorrections(epoch_context.osr_corrections);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    auto& osr_corrections = epoch_context.osr_corrections;

    // Same tt-anchor substitution as the prepareEpochState() call above,
    // applied here so detectClasCycleSlips()'s outage_gap (dt_seconds > 2s)
    // ambiguity-reset path sees the real accumulated gap on the
    // return-to-accepted epoch, exactly like the predict step does --
    // otherwise a multi-epoch coast would leave every ambiguity's lock
    // count untouched even though the filter froze underneath it.
    const double clas_dt_seconds =
        clas_prepare_has_last_processed_time
            ? std::max(obs.time - clas_prepare_last_processed_time, 0.001)
            : 1.0;
    if (ppp_config_.kinematic_mode && ppp_config_.enable_cycle_slip_detection) {
        const auto slip_stats = ppp_clas::detectClasCycleSlips(
            obs,
            osr_corrections,
            ppp_config_,
            clas_dt_seconds,
            filter_state_,
            ambiguity_states_,
            clas_dispersion_compensation_,
            clas_phase_bias_repair_,
            [&](const SatelliteId& satellite, SignalType signal) {
                resetAmbiguity(satellite, signal);
            },
            clas_ambiguity_initial_variance,
            pppDebugEnabled());
        if (slip_stats.total_resets > 0) {
            clas_dd_accumulator_ = {};
        }
        if (clas_mrtklib_parity) {
            // MRTKLIB runs udbias_ppp() before corrmeas().  Recreating a
            // zeroed ambiguity sets ssat.pbreset for that frequency, and
            // compensatedisp() then returns compL=0 for the complete L1/L2
            // pair on this epoch.  Native prepares the OSR corrections
            // before running its slip/outage detector, so discard the
            // already-computed compensation for every satellite whose bias
            // was reset here.  The persistent carrier datum is deliberately
            // retained: pbreset suppresses only the current epoch and normal
            // compensation resumes on the next one.
            for (auto& osr : osr_corrections) {
                if (slip_stats.reset_satellites.count(osr.satellite) != 0) {
                    for (int frequency = 0;
                         frequency < osr.num_frequencies &&
                         frequency < OSR_MAX_FREQ;
                         ++frequency) {
                        // CPC was aggregated while preparing the OSR, before
                        // the later udbias-equivalent reset became known.
                        osr.CPC[frequency] -=
                            osr.phase_compensation_m[frequency];
                    }
                    std::fill(std::begin(osr.phase_compensation_m),
                              std::end(osr.phase_compensation_m), 0.0);
                }
            }
        }
    }

    if (pppEnvOverrides().clas_stec_constraint &&
        ppp_config_.estimate_ionosphere &&
        ppp_config_.use_clas_osr_filter) {
        int network_id = -1;
        if (readClasAtmosNetworkId(epoch_atmos, network_id)) {
            if (has_last_clas_atmos_network_id_ &&
                network_id != last_clas_atmos_network_id_) {
                resetClasIonosphereStateValues(filter_state_);
            }
            last_clas_atmos_network_id_ = network_id;
            has_last_clas_atmos_network_id_ = true;
        }
    }

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        // Dynamics mode: prepareEpochState already ran predictFilterState,
        // which propagated pos += vel*dt and inflated the covariance for
        // this interval, so the processed-time bookkeeping MUST advance
        // before this early return. Otherwise the next epoch's dt spans
        // this interval AGAIN and the same coast is re-applied every epoch:
        // across a low-satellite stretch dt grows unboundedly (observed
        // 2.2 s -> 15+ s at 5 Hz approaching the tokyo_run2 bridge outage)
        // and the repeated vel*dt over-propagation drags the float
        // kilometers away (the 4.5 km post-bridge blunder). White-noise
        // mode keeps historical behavior (its per-epoch SPP re-anchor of
        // position and clock bounds the damage of a stale dt).
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
        }
        solution = has_clas_continuity_output
            ? clas_continuity_output
            : seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(
        filter_state_, osr_corrections, clas_ambiguity_initial_variance);
    if (clas_mrtklib_parity) {
        constexpr double kMrtklibBiasVarianceCycles2 = 100.0 * 100.0;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            for (int frequency = 0; frequency < osr.num_frequencies && frequency < 2;
                 ++frequency) {
                const double wavelength = osr.wavelengths[frequency];
                if (!(wavelength > 0.0)) continue;
                const SatelliteId ambiguity_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(
                        255, static_cast<int>(osr.satellite.prn) +
                                 (frequency == 0 ? 0 : 100))));
                const int ambiguity_index = ambiguityStateIndex(ambiguity_satellite);
                if (ambiguity_index < 0) continue;
                double& variance =
                    filter_state_.covariance(ambiguity_index, ambiguity_index);
                // A reset can precede predictFilterState(), which adds the
                // tiny bias random walk and makes the sentinel slightly
                // larger than exactly 10000.
                if (variance > 0.9 * kMrtklibBiasVarianceCycles2) {
                    variance = kMrtklibBiasVarianceCycles2 *
                               wavelength * wavelength;
                }
            }
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // udbias_ppp() recreates every zeroed phase-bias state with
        // lock=-minlock. The accepted observation at the end of this epoch
        // increments it once, so the published lock is -4 for minlock=5.
        // This happens on every whole-filter initialization, not only at a
        // 30-second CSSR phase-bias boundary: udbias_ppp() keys the reset to
        // x[IB]==0 after udpos/udstate, independently of correction timing.
        constexpr int kMrtklibMinLock = 5;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            ambiguity_states_[osr.satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[osr.satellite].outage_count = 0;
            const SatelliteId l2_satellite(
                osr.satellite.system,
                static_cast<uint8_t>(std::min(
                    255, static_cast<int>(osr.satellite.prn) + 100)));
            ambiguity_states_[l2_satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[l2_satellite].outage_count = 0;
        }
    }
    if (pppEnvOverrides().clas_nl_datum_reset &&
        ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        ppp_config_.use_clas_osr_filter &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.estimate_ionosphere) {
        if (applyClasNlDatumReset(
                obs.time, osr_corrections, ambiguity_states_, pppDebugEnabled())) {
            clas_dd_accumulator_ = {};
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // Fresh udbias_ppp() states must be seeded from raw L*lambda-P.
        // A continuity shift queued for the discarded filter would make a
        // zero state non-zero before seeding and falsely mark it initialized.
        for (auto& [_, repair] : clas_phase_bias_repair_) {
            repair.pending_state_shift_cycles = {0.0, 0.0, 0.0};
        }
    }
    ppp_clas::applyPendingPhaseBiasStateShifts(
        filter_state_, osr_corrections, clas_phase_bias_repair_, pppDebugEnabled());

    const auto epoch_update = ppp_clas::runEpochMeasurementUpdate(
        obs,
        epoch_context,
        filter_state_,
        ppp_config_,
        seed,
        ambiguity_states_,
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        },
        [&](const SatelliteId& satellite, SignalType signal) {
            resetAmbiguity(satellite, signal);
        },
        [&](const SatelliteId& satellite) {
            return ambiguityStateIndex(satellite);
        },
        pppDebugEnabled());
    if (!epoch_update.updated) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("measurement_update");
        }
        if (clas_mrtklib_parity &&
            (epoch_update.insufficient_valid_satellites ||
             epoch_preparation.initialized_this_epoch)) {
            // mrtk_ppp_rtk.c resets every kinematic state when stat remains
            // SOLQ_NONE after the ssat.vsat L1 count check. float_count is
            // neither incremented nor cleared on this SINGLE epoch.
            filter_state_ = PPPState{};
            filter_initialized_ = false;
            ambiguity_states_.clear();
            est_stec_outage_.clear();
            clas_dd_accumulator_ = {};
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            last_clas_constrained_fixed_state_valid_ = false;
            solution = has_clas_continuity_output
                ? clas_continuity_output
                : seed;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
            ++total_epochs_processed_;
            return solution;
        }
        // MRTKLIB keeps publishing the predicted FLOAT state when all
        // post-fit DD reference trials are rejected; the epoch is not
        // reclassified as SINGLE merely because filter2_ supplied no new
        // posterior. Keeping the float status also advances float_count and
        // preserves the literal 15-epoch reset cadence.
        if (clas_mrtklib_parity && filter_initialized_) {
            solution = ppp_clas::finalizeEpochSolution(
                filter_state_, obs.time, false, 0.0, 0,
                static_cast<int>(epoch_context.osr_corrections.size()));
            ++clas_mrtklib_float_count_;
            had_fixed_last_epoch_ = false;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
            ++total_epochs_processed_;
            if (solution.isValid()) {
                last_published_solution_position_ecef_ = solution.position_ecef;
                has_last_published_solution_position_ = true;
            }
            return solution;
        }
        // Same dt-bookkeeping requirement as the insufficient_osr early
        // return above: the predict for this interval already happened.
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
        }
        solution = has_clas_continuity_output
            ? clas_continuity_output
            : seed;
        return solution;
    }
    dumpClasFloatPosition(obs.time, filter_state_, epoch_update, osr_corrections.size());
    const auto& update_stats = epoch_update.update_stats;
    if (clas_mrtklib_parity) {
        clas_mrtklib_ar_rejected_ambiguities_ =
            update_stats.rejected_phase_ambiguities;
    }
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    if (ppp_config_.use_clas_dd_filter) {
        if (!clas_dd_filter_) {
            clas_dd_filter_ = std::make_unique<ppp_clas_dd::DdFilterScaffold>();
        }
        const PositionSolution native_float_solution = ppp_clas::finalizeEpochSolution(
            filter_state_,
            obs.time,
            false,
            0.0,
            0,
            static_cast<int>(osr_corrections.size()));
        solution = clas_dd_filter_->processFloatUpdate(
            obs,
            epoch_context,
            filter_state_,
            native_float_solution,
            ppp_config_,
            [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
                return calculateMappingFunction(receiver_pos, elevation, time);
            });
        applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
        has_last_processed_time_ = true;
        last_processed_time_ = obs.time;
        clas_update_seed_anchor();
        ++total_epochs_processed_;
        return solution;
    }

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = findOsrFrequencyObservation(obs, osr, 0);
            const Observation* l2_raw = findOsrFrequencyObservation(obs, osr, 1);
            if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) continue;
            if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) continue;
            if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) continue;
            if (ppp_config_.kinematic_mode &&
                ppp_config_.use_clas_osr_filter &&
                ((l1_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(0, osr.elevation, l1_raw->snr)) ||
                 (l2_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(1, osr.elevation, l2_raw->snr)))) {
                continue;
            }
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) continue;
            const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0]
                              - osr.phase_bias_m[0];
            const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1]
                              - osr.phase_bias_m[1];
            const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
            const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
            const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2)
                              - (f1 * p1 + f2 * p2) / (f1 + f2);
            constexpr double lambda_wl_gps = constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl_gps;
            auto& amb = ambiguity_states_[osr.satellite];
            const bool mw_slip = amb.needs_reinitialization;
            if (!mw_slip && amb.mw_count > 0) {
                amb.mw_sum_cycles += mw_cycles;
                amb.mw_count += 1;
                amb.mw_mean_cycles = amb.mw_sum_cycles / amb.mw_count;
            } else {
                amb.mw_sum_cycles = mw_cycles;
                amb.mw_count = 1;
                amb.mw_mean_cycles = mw_cycles;
                amb.wl_is_fixed = false;
            }
        }
    }

    const auto trop_mapping_for_validation =
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        };
    const auto ambiguity_index_for_validation = [&](const SatelliteId& satellite) {
        return ambiguityStateIndex(satellite);
    };

    const Vector3d clas_float_position_ecef =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clas_float_horizontal_sigma_m =
        clasKinematicHorizontalPositionSigmaM(filter_state_);
    const PPPState clas_float_filter_state = filter_state_;
    const auto clas_float_ambiguity_states = ambiguity_states_;

    const auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                // clas_seed_untrusted_this_epoch (parity path only; always
                // false otherwise) -- do not let AR originate a new
                // publishable fix from a coasted/stale state; see the long
                // comment at its assignment above.
                return ppp_config_.enable_ambiguity_resolution &&
                       !clas_seed_untrusted_this_epoch &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            pppDebugEnabled());

    // MRTKLIB mrtk_ppp_rtk.c:2296-2330 parity: validate the fixed solution with
    // the post-fix DD phase chi-square. Publish FIX only when chisq < thres_fix
    // (5.0) and hold (constrain the float filter toward the fixed DD
    // ambiguities, holdamb()) only when chisq < thres_hold (0.5). AR-failed
    // epochs publish FLOAT and reset the nfix counter; there is no hold-driven
    // FIX publication.
    const bool kinematic_clas_wlnl_hold_path =
        ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL;
    bool clas_kinematic_chisq_rejected = false;
    if (kinematic_clas_wlnl_hold_path &&
        ppp_config_.enable_ambiguity_resolution) {
        if (!ppp_ar::wlnlHoldStillValid(clas_wlnl_hold_, ambiguity_states_)) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }

        if (ambiguity_resolution.accepted &&
            !ppp_shared::clasRecoveryFixIsSupported(
                clas_seed_ar_recovery_this_epoch, last_fixed_ambiguities_,
                last_ar_ratio_, kClasKinematicMinFixRatio)) {
            // Native's minimum-row fixes immediately after a maxdiff event
            // are the remaining wrong-integer mode (Tokyo run2: 24 bad FIX,
            // all nb=6). MRTKLIB's desired recovery begins at nb=8 and the
            // native equivalent at nb=7 with ratio above the normal
            // kinematic publication floor, so keep AR running but reject
            // only the under-supported recovery candidate.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-RECOVERY] reject nb="
                          << last_fixed_ambiguities_ << " ratio="
                          << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted &&
            !last_clas_constrained_fixed_state_valid_) {
            // MRTKLIB publishes FIX only from the constrained xa solution
            // (sol.rr = xa). Without a validated state-DD LAMBDA fix the epoch
            // stays FLOAT instead of labelling the float state as fixed.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] no constrained state, demote"
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted) {
            const PPPState& fixed_state_for_validation =
                last_clas_constrained_fixed_state_;
            ppp_clas::FixValidationOptions fix_validation_options;
            fix_validation_options.outlier_sigma_gate =
                kMrtklibPhaseResidualSigmaGate;
            fix_validation_options.mrtklib_chisq_fallback = true;
            // MRTKLIB parity (dynamics path only): the post-fix residual
            // gate/chi-square normalize by the innovation covariance
            // H'*P*H + R formed from the FLOAT posterior covariance
            // (mrtk_ppp_rtk.c:2296-2313 -> filter2_ -> residual_test), not
            // by the measurement variance alone. Under the tight Stage-2
            // varerr an R-only basis rejects every centimeter-level DD
            // residual and drives fix% to zero.
            if (ppp_config_.clas_mrtklib_float_parity &&
                ppp_config_.use_dynamics_model) {
                fix_validation_options.innovation_covariance =
                    &clas_float_filter_state.covariance;
            }
            const auto fix_validation = ppp_clas::validateFixedSolution(
                obs,
                osr_corrections,
                fixed_state_for_validation,
                ppp_config_,
                trop_mapping_for_validation,
                ambiguity_index_for_validation,
                pppDebugEnabled(),
                fix_validation_options);
            const double phase_chisq = fix_validation.phase_chisq;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] phase_chisq=" << phase_chisq
                          << " rows=" << fix_validation.phase_rows
                          << " outliers=" << fix_validation.phase_outlier_rows
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
            if (!std::isfinite(phase_chisq) ||
                phase_chisq >= kMrtklibFixChiSquareGate) {
                clas_kinematic_chisq_rejected = true;
            } else if (phase_chisq < kMrtklibHoldChiSquareGate) {
                std::map<SatelliteId, double> clas_satellite_elevations_rad;
                for (const auto& osr : osr_corrections) {
                    if (osr.valid) {
                        clas_satellite_elevations_rad[osr.satellite] =
                            osr.elevation;
                    }
                }
                ++clas_wlnl_hold_.consecutive_fix_count;
                std::vector<ppp_ar::WlnlHoldConstraint> hold_constraints;
                if (!clas_wlnl_candidate_hold_constraints_.empty()) {
                    hold_constraints = clas_wlnl_candidate_hold_constraints_;
                } else {
                    ppp_ar::buildWlnlHoldConstraints(
                        last_clas_constrained_fixed_state_,
                        ambiguity_states_,
                        clas_satellite_elevations_rad,
                        hold_constraints,
                        ppp_config_.use_dynamics_model &&
                            !ppp_config_.low_dynamics_mode);
                }
                if (clas_wlnl_hold_.consecutive_fix_count >=
                        ppp_ar::kMrtklibMinFixCount &&
                    !hold_constraints.empty()) {
                    clas_wlnl_hold_.constraints = std::move(hold_constraints);
                    clas_wlnl_hold_.active = true;
                    const bool hold_applied = ppp_ar::applyWlnlHoldAmbiguity(
                        filter_state_, clas_wlnl_hold_.constraints);
                    if (pppDebugEnabled()) {
                        std::cerr << "[CLAS-WLNL-HOLD] applied=" << hold_applied
                                  << " constraints="
                                  << clas_wlnl_hold_.constraints.size()
                                  << " nfix="
                                  << clas_wlnl_hold_.consecutive_fix_count
                                  << " chisq=" << phase_chisq << "\n";
                    }
                } else if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-WLNL-HOLD] skipped nfix="
                              << clas_wlnl_hold_.consecutive_fix_count
                              << " chisq=" << phase_chisq << "\n";
                }
            }
        } else {
            // MRTKLIB resets rtk->nfix on every non-FIX epoch
            // (mrtk_ppp_rtk.c:2371).
            clas_wlnl_hold_.consecutive_fix_count = 0;
        }
    }

    bool ambiguity_fixed_epoch =
        ambiguity_resolution.accepted && !clas_kinematic_chisq_rejected;
    if (clas_kinematic_chisq_rejected) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        ambiguity_states_ = clas_float_ambiguity_states;
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-KIN-CHISQ] reject fix (chisq >= "
                      << kMrtklibFixChiSquareGate << ")\n";
        }
    }
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        if (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }

    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        !kinematic_clas_wlnl_hold_path) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (pppDebugEnabled() && wlnl_fixed_position_ok) {
            const double shift = (wlnl_fixed_position -
                filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
        }
    }

    // MRTKLIB publishes the fixed solution from xa/Pa while the float filter
    // x/P is only nudged by holdamb() (mrtk_ppp_rtk.c:2355-2361).
    const bool use_constrained_fixed_state =
        ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        last_clas_constrained_fixed_state_valid_ &&
        (env_overrides_.clas_resamb ||
         (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter));
    const PPPState& solution_filter_state =
        use_constrained_fixed_state ? last_clas_constrained_fixed_state_ : filter_state_;

    solution = ppp_clas::finalizeEpochSolution(
        solution_filter_state,
        obs.time,
        ambiguity_fixed_epoch,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok &&
        !ppp_config_.kinematic_mode &&
        !env_overrides_.clas_fixed_state_output &&
        !use_constrained_fixed_state) {
        solution.position_ecef = wlnl_fixed_position;
    }

    // Multi-epoch SD AR: accumulate DD float ambiguities over epochs,
    // then fix with LAMBDA when variance is small enough.
    {
        const auto sd_ar_result = ppp_clas_sd::solveMultiEpochSdAr(
            clas_dd_accumulator_,
            obs,
            osr_corrections,
            solution.position_ecef,
            3.0,   // AR ratio threshold
            20,    // Min accumulation epochs before attempting LAMBDA
            pppDebugEnabled());
        const bool sd_ar_fixed =
            sd_ar_result.valid && sd_ar_result.ar_ratio >= 3.0;
        const bool apply_sd_mar_shift_gate =
            ppp_config_.kinematic_mode ||
            env_overrides_.clas_base_clock_parity;
        const bool sd_ar_position_ok =
            !apply_sd_mar_shift_gate ||
            sd_ar_result.position_shift_m <=
                kClasBaseClockParitySdMarMaxPositionShiftM;
        if (sd_ar_fixed && sd_ar_position_ok && !ppp_config_.kinematic_mode) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        } else if (sd_ar_fixed && pppDebugEnabled()) {
            std::cerr << "[CLAS-SD-MAR] reject"
                      << (ppp_config_.kinematic_mode ? " kinematic" : " parity")
                      << " pos_shift="
                      << sd_ar_result.position_shift_m
                      << " ratio=" << sd_ar_result.ar_ratio << "\n";
        }
    }

    // On the kinematic CLAS WLNL path the MRTKLIB-parity post-fix chi-square
    // gate (plus the maxdiffp guard below) already validates every fixed
    // publication, so the custom float-jump/continuity gates are skipped: the
    // float filter itself carries meter-level error, and a correct fix
    // legitimately jumps away from the previously published float position.
    bool clas_kinematic_fix_rejected = false;
    if (ppp_config_.kinematic_mode &&
        solution.status == SolutionStatus::PPP_FIXED &&
        !(kinematic_clas_wlnl_hold_path && ambiguity_fixed_epoch)) {
        const double dt_seconds =
            has_last_processed_time_ ? obs.time - last_processed_time_ : 0.2;
        const double max_fixed_float_jump_m = clasKinematicMaxFixedFloatJumpM(
            dt_seconds,
            clas_float_horizontal_sigma_m);
        const Vector3d post_ar_filter_position =
            solution_filter_state.state.segment(solution_filter_state.pos_index, 3);
        double fixed_float_jump_m =
            (solution.position_ecef - clas_float_position_ecef).norm();
        if (wlnl_fixed_position_ok) {
            fixed_float_jump_m = std::max(
                fixed_float_jump_m,
                (wlnl_fixed_position - post_ar_filter_position).norm());
        }
        const bool ratio_ok =
            solution.ratio <= 0.0 ||
            solution.ratio >= (kinematic_clas_wlnl_hold_path
                 ? ppp_config_.ar_ratio_threshold
                 : std::max(
                       kClasKinematicMinFixRatio,
                       ppp_config_.ar_ratio_threshold + 0.5));
        const bool wlnl_shift_ok =
            !wlnl_fixed_position_ok ||
            (wlnl_fixed_position - post_ar_filter_position).norm() <=
                kClasKinematicWlnlMaxPositionShiftM;
        double continuity_jump_m = 0.0;
        if (has_last_published_solution_position_) {
            continuity_jump_m = (solution.position_ecef -
                last_published_solution_position_ecef_).norm();
        }
        const bool continuity_ok =
            !has_last_published_solution_position_ ||
            continuity_jump_m <= max_fixed_float_jump_m;
        const bool float_jump_ok =
            std::isfinite(fixed_float_jump_m) &&
            fixed_float_jump_m <= max_fixed_float_jump_m;
        const bool jump_ok =
            wlnl_shift_ok &&
            continuity_ok &&
            float_jump_ok &&
            ratio_ok;
        if (jump_ok) {
            ++clas_kinematic_fix_candidate_streak_;
        } else {
            clas_kinematic_fix_candidate_streak_ = 0;
        }
        const bool minfix_ok =
            clas_kinematic_fix_candidate_streak_ >= kClasKinematicMinFixCount;
        if (!jump_ok || !minfix_ok) {
            clas_kinematic_fix_rejected = true;
            solution.position_ecef = clas_float_position_ecef;
            solution.status = SolutionStatus::PPP_FLOAT;
            solution.ratio = 0.0;
            solution.num_fixed_ambiguities = 0;
            filter_state_ = clas_float_filter_state;
            ambiguity_states_ = clas_float_ambiguity_states;
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            if (!jump_ok) {
                clas_kinematic_fix_candidate_streak_ = 0;
            }
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-FIX] reject"
                          << (!wlnl_shift_ok ? " wlnl_shift" :
                              !continuity_ok ? " continuity" :
                              !float_jump_ok ? " float_jump" :
                              !ratio_ok ? " ratio" :
                              " minfix")
                          << " jump=" << fixed_float_jump_m
                          << " continuity=" << continuity_jump_m
                          << " limit=" << max_fixed_float_jump_m
                          << " ratio=" << solution.ratio
                          << " streak=" << clas_kinematic_fix_candidate_streak_
                          << "\n";
            }
        }
    } else if (ppp_config_.kinematic_mode) {
        clas_kinematic_fix_candidate_streak_ = 0;
    }

    // MRTKLIB maxdiffp guard (mrtk_ppp_rtk.c:2333-2352). MRTKLIB applies it
    // with dynamics on (benchmark clas.toml: dynamics=true + pseudorange_diff);
    // without it the dynamics filter can enter a rejection spiral and coast
    // kilometers away on the velocity states.
    if (ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        seed.isValid()) {
        const Vector3d float_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const double spp_divergence_m =
            (float_position - seed.position_ecef).norm();
        if (spp_divergence_m > kMrtklibMaxSppDivergenceM) {
            ++clas_kinematic_spp_divergence_count_;
            if (clas_kinematic_spp_divergence_count_ >
                    kMrtklibMaxSppDivergenceEpochs) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-KIN-MAXDIFFP] reset to SPP dist="
                              << spp_divergence_m << "\n";
                }
                filter_state_.state.segment(filter_state_.pos_index, 3) =
                    seed.position_ecef;
                for (int i = 0; i < 3; ++i) {
                    const int idx = filter_state_.pos_index + i;
                    filter_state_.covariance.row(idx).setZero();
                    filter_state_.covariance.col(idx).setZero();
                    const double spp_variance =
                        seed.position_covariance(i, i) > 0.0
                            ? seed.position_covariance(i, i)
                            : 100.0;
                    filter_state_.covariance(idx, idx) = spp_variance;
                }
                if (ppp_config_.use_dynamics_model &&
                    filter_state_.vel_index >= 0 &&
                    filter_state_.vel_index + 2 < filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.vel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) =
                            clas_mrtklib_parity
                                ? 1.0
                                : ppp_config_.initial_velocity_variance;
                    }
                }
                if (clas_mrtklib_parity &&
                    filter_state_.accel_index >= 0 &&
                    filter_state_.accel_index + 2 <
                        filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.accel_index, 3)
                        .setConstant(1e-6);
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.accel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) = 1.0;
                    }
                }
                solution.position_ecef = seed.position_ecef;
                solution.status = SolutionStatus::SPP;
                solution.ratio = 0.0;
                solution.num_fixed_ambiguities = 0;
                clas_kinematic_fix_rejected = false;
                clas_kinematic_fix_candidate_streak_ = 0;
                clas_kinematic_spp_divergence_count_ = 0;
                ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
                if (clas_mrtklib_parity && ppp_config_.use_dynamics_model) {
                    // Drop the stale ambiguity/ionosphere state attached to
                    // the pre-reset position. AR stays quarantined through
                    // the recovery window above while the filter rebuilds.
                    filter_state_ = PPPState{};
                    filter_initialized_ = false;
                    ambiguity_states_.clear();
                    est_stec_outage_.clear();
                    clas_dd_accumulator_ = {};
                    last_clas_constrained_fixed_state_valid_ = false;
                    last_ar_ratio_ = 0.0;
                    last_fixed_ambiguities_ = 0;
                }
            }
        } else {
            clas_kinematic_spp_divergence_count_ = 0;
        }
    }

    had_fixed_last_epoch_ =
        solution.status == SolutionStatus::PPP_FIXED && !clas_kinematic_fix_rejected;

    if (clas_mrtklib_parity) {
        if (solution.status == SolutionStatus::PPP_FIXED) {
            clas_mrtklib_float_count_ = 0;
        } else if (solution.status == SolutionStatus::PPP_FLOAT) {
            ++clas_mrtklib_float_count_;
        }
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;
    clas_update_seed_anchor();
    ++total_epochs_processed_;

    applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
    const bool publishing_rejected_seed_output =
        clas_rejected_seed_output_prepared &&
        solution.status == SolutionStatus::SPP &&
        (solution.position_ecef - clas_continuity_output.position_ecef).norm() <
            1e-4;
    if (ppp_config_.kinematic_mode && solution.isValid() &&
        !publishing_rejected_seed_output) {
        last_published_solution_position_ecef_ = solution.position_ecef;
        has_last_published_solution_position_ = true;
    }
    return solution;
}

}  // namespace libgnss
