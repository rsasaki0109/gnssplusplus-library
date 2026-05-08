#include "libgnss++/iers/tides.hpp"

#include <cmath>
#include <vector>

#include "iers2010.hpp"

extern "C" {
#include "sofa.h"
}

namespace libgnss::iers {

namespace {
constexpr double kMjdJ2000Epoch  = 2400000.5;     // MJD <-> JD offset
constexpr double kJdJ2000        = 2451545.0;     // J2000.0 in JD (TT)
constexpr double kJulianCentury  = 36525.0;       // days per Julian century
}  // namespace

Eigen::Vector3d solidEarthTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const Eigen::Vector3d& xsun_icrs,
    const Eigen::Vector3d& xmon_icrs) {
    // dehanttideinel_impl wants:
    //   julian_centuries_tt — TT in Julian centuries from J2000
    //   fhr_ut              — fractional hour UT of the day
    //
    // For TT we go through SOFA's leap-second-aware UTC -> TAI -> TT
    // chain so the result is correct regardless of what leap-second
    // table the caller has in mind.
    double tai1 = 0.0;
    double tai2 = 0.0;
    double tt1  = 0.0;
    double tt2  = 0.0;
    iauUtctai(kMjdJ2000Epoch, mjd_utc, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);

    const double centuries_tt =
        ((tt1 - kJdJ2000) + tt2) / kJulianCentury;

    // The Dehant model uses fhr_ut at the ~10 ms level only (it
    // affects the diurnal Step-2 corrections), so taking UT ≈ UTC
    // here introduces a sub-millimeter displacement error — well
    // below the model's intrinsic accuracy.
    const double fhr_ut = (mjd_utc - std::floor(mjd_utc)) * 24.0;

    std::vector<Eigen::Vector3d> xsta_vec = { xsta_itrs };
    std::vector<Eigen::Vector3d> xcor_vec;
    xcor_vec.reserve(1);

    iers2010::dehanttideinel_impl(
        centuries_tt, fhr_ut, xsun_icrs, xmon_icrs, xsta_vec, xcor_vec);

    if (xcor_vec.empty()) {
        return Eigen::Vector3d::Zero();
    }
    return xcor_vec.front();
}

namespace {

// IERS 2018 Update of the Mean Pole Model (linear secular drift).
// Replaces the cubic 1976–2010 polynomial that appeared in the
// original IERS 2010 Technical Note 36. Outputs are in milli-arcseconds.
//
// References:
//   - IERS Conventions 2010 §7.1.4 (post-2018 linear update)
//   - https://hpiers.obspm.fr/iers/conv/conv2010/conv2010_c7.html
void meanPoleMas(double decimal_year, double& xp_mean_mas, double& yp_mean_mas) {
    const double dt = decimal_year - 2000.0;
    xp_mean_mas =  55.0 + 1.677 * dt;
    yp_mean_mas = 320.5 + 3.460 * dt;
}

double mjdUtcToDecimalYear(double mjd_utc) {
    // J2000 = 2000-01-01.5 UTC ≈ MJD 51544.5; one Julian year is
    // 365.25 days. Sub-day accuracy is unnecessary for the secular
    // mean-pole term (its slope is ~mas/yr).
    constexpr double kMjdJ2000Utc = 51544.5;
    constexpr double kDaysPerYear = 365.25;
    return 2000.0 + (mjd_utc - kMjdJ2000Utc) / kDaysPerYear;
}

}  // namespace

Eigen::Vector3d poleTideDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const EarthOrientationParams& eop) {
    // Mean-pole reference position at this epoch (mas).
    double xp_mean_mas = 0.0;
    double yp_mean_mas = 0.0;
    meanPoleMas(mjdUtcToDecimalYear(mjd_utc), xp_mean_mas, yp_mean_mas);

    // m1, m2 in arcseconds — instantaneous polar motion offset from
    // the mean pole (IERS 2010 §7.1.4 eq. 7.24).
    constexpr double kMasPerArcsec = 1000.0;
    const double m1 = eop.xp_arcsec - xp_mean_mas / kMasPerArcsec;
    const double m2 = -(eop.yp_arcsec - yp_mean_mas / kMasPerArcsec);

    // Geocentric latitude / longitude (no flattening — pole tide
    // formulation is geocentric).
    const double x = xsta_itrs.x();
    const double y = xsta_itrs.y();
    const double z = xsta_itrs.z();
    const double r_xy = std::sqrt(x * x + y * y);
    const double phi = std::atan2(z, r_xy);     // geocentric latitude
    const double theta = M_PI / 2.0 - phi;      // colatitude
    const double lambda = std::atan2(y, x);     // east longitude

    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double sin_2theta = 2.0 * sin_theta * cos_theta;
    const double cos_2theta = cos_theta * cos_theta - sin_theta * sin_theta;
    const double sin_lambda = std::sin(lambda);
    const double cos_lambda = std::cos(lambda);

    // IERS 2010 eq. 7.25 (with conventions S_r = -33, S_θ = -9, S_λ = +9
    // mm). All m1/m2 inputs in arcseconds.
    const double m1_cosL_plus_m2_sinL = m1 * cos_lambda + m2 * sin_lambda;
    const double m1_sinL_minus_m2_cosL = m1 * sin_lambda - m2 * cos_lambda;

    const double d_radial_mm = -33.0 * sin_2theta * m1_cosL_plus_m2_sinL;
    const double d_north_mm  =  -9.0 * cos_2theta * m1_cosL_plus_m2_sinL;
    const double d_east_mm   =   9.0 * cos_theta  * m1_sinL_minus_m2_cosL;

    constexpr double kMmToM = 1.0e-3;
    const double dE = d_east_mm  * kMmToM;
    const double dN = d_north_mm * kMmToM;
    const double dU = d_radial_mm * kMmToM;

    // Local ENU → ECEF rotation. Uses geocentric φ to match the
    // formulation above (the IERS conventions explicitly compute the
    // displacement in a geocentric local frame).
    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    Eigen::Vector3d d_ecef;
    d_ecef.x() = -sin_lambda * dE - sin_phi * cos_lambda * dN + cos_phi * cos_lambda * dU;
    d_ecef.y() =  cos_lambda * dE - sin_phi * sin_lambda * dN + cos_phi * sin_lambda * dU;
    d_ecef.z() =                     cos_phi * dN              + sin_phi * dU;
    return d_ecef;
}

namespace {
// MJD reference for converting UTC MJD → seconds since Unix epoch.
// 1970-01-01 00:00:00 UTC = MJD 40587.
constexpr double kMjdUnixEpoch        = 40587.0;
constexpr double kSecondsPerDay       = 86400.0;
constexpr double kAtmDegreesToRadians = M_PI / 180.0;
}  // namespace

Eigen::Vector3d atmosphericTidalLoadingDisplacement(
    double mjd_utc,
    const Eigen::Vector3d& xsta_itrs,
    const AtmosphericTidalLoadingCoefficients& coeffs) {
    // Time argument: seconds since Unix epoch (UTC). Mirrors the legacy
    // ocean-loading dispatcher in PPPProcessor so the two paths share
    // a single time reference for paired-comparison benches.
    const double seconds_since_unix =
        (mjd_utc - kMjdUnixEpoch) * kSecondsPerDay;

    // S1 = 24 h, S2 = 12 h.
    constexpr std::array<double, 2> kPeriodsSeconds = {
        kSecondsPerDay,
        kSecondsPerDay / 2.0,
    };

    double up_m = 0.0;
    double west_m = 0.0;
    double south_m = 0.0;
    for (std::size_t i = 0; i < kPeriodsSeconds.size(); ++i) {
        const double angle =
            2.0 * M_PI * seconds_since_unix / kPeriodsSeconds[i];
        up_m    += coeffs.radial_amplitudes_m[i] *
                   std::cos(angle - coeffs.radial_phases_deg[i] *
                                     kAtmDegreesToRadians);
        west_m  += coeffs.west_amplitudes_m[i]   *
                   std::cos(angle - coeffs.west_phases_deg[i] *
                                     kAtmDegreesToRadians);
        south_m += coeffs.south_amplitudes_m[i]  *
                   std::cos(angle - coeffs.south_phases_deg[i] *
                                     kAtmDegreesToRadians);
    }

    // Local (E, N, U) → ECEF; geocentric φ is accurate at sub-mm.
    const double x = xsta_itrs.x();
    const double y = xsta_itrs.y();
    const double z = xsta_itrs.z();
    const double r_xy = std::sqrt(x * x + y * y);
    const double phi_g = std::atan2(z, r_xy);
    const double lambda = std::atan2(y, x);
    const double sin_phi    = std::sin(phi_g);
    const double cos_phi    = std::cos(phi_g);
    const double sin_lambda = std::sin(lambda);
    const double cos_lambda = std::cos(lambda);
    const double dE =  -west_m;
    const double dN =  -south_m;
    const double dU =   up_m;

    Eigen::Vector3d d_ecef;
    d_ecef.x() = -sin_lambda * dE - sin_phi * cos_lambda * dN +
                  cos_phi    * cos_lambda * dU;
    d_ecef.y() =  cos_lambda * dE - sin_phi * sin_lambda * dN +
                  cos_phi    * sin_lambda * dU;
    d_ecef.z() =                     cos_phi    * dN +
                  sin_phi    * dU;
    return d_ecef;
}

Eigen::Vector3d oceanLoadingDisplacement(
    double mjd_utc,
    const OceanLoadingBlq& blq) {
    // hardisp_impl wants amplitude/phase tables in [3][ntin] form,
    // BLQ row order: vertical (radial-up), west, south.
    constexpr int ntin = iers2010::hisp::ntin;
    static_assert(ntin == 11, "BLQ format expects 11 reference harmonics");

    double tamp[3][ntin];
    double tph[3][ntin];
    for (int k = 0; k < ntin; ++k) {
        tamp[0][k] = blq.radial_amplitudes_m[static_cast<std::size_t>(k)];
        tamp[1][k] = blq.west_amplitudes_m[static_cast<std::size_t>(k)];
        tamp[2][k] = blq.south_amplitudes_m[static_cast<std::size_t>(k)];
        // HARDISP convention: phase is "lag" (negative). Upstream
        // read_hardisp_args negates the phases on read; we replicate
        // that here so callers can supply Onsala-style positive
        // phases as published.
        tph[0][k] = -blq.radial_phases_deg[static_cast<std::size_t>(k)];
        tph[1][k] = -blq.west_phases_deg[static_cast<std::size_t>(k)];
        tph[2][k] = -blq.south_phases_deg[static_cast<std::size_t>(k)];
    }

    double odu = 0.0;
    double ods = 0.0;
    double odw = 0.0;
    iers2010::hisp::hardisp_impl(
        /*irnt=*/1, /*samp=*/1.0, tamp, tph, mjd_utc, &odu, &ods, &odw);

    // Returned in (radial_up, west, south) — same as BLQ convention.
    return Eigen::Vector3d(odu, odw, ods);
}

}  // namespace libgnss::iers
