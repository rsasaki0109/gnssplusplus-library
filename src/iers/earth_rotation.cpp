#include "libgnss++/iers/earth_rotation.hpp"

#include <cmath>

extern "C" {
#include "sofa.h"
#include "sofam.h"
}

namespace libgnss::iers {

namespace {
constexpr double kMjdJ2000Epoch = 2400000.5;  // MJD <-> JD offset
constexpr double kMjd0Gps       = 44244.0;    // MJD UTC of 1980-01-06 00:00:00
constexpr double kArcsecToRad   = M_PI / (180.0 * 3600.0);
}  // namespace

double earthRotationAngle(double mjd_utc, const EarthOrientationParams& eop) {
    const double utc1 = kMjdJ2000Epoch;
    const double utc2 = mjd_utc;

    double ut11 = 0.0;
    double ut12 = 0.0;
    iauUtcut1(utc1, utc2, eop.ut1_minus_utc_seconds, &ut11, &ut12);

    return iauEra00(ut11, ut12);
}

Eigen::Matrix3d icrsToItrs(double mjd_utc, const EarthOrientationParams& eop) {
    const double utc1 = kMjdJ2000Epoch;
    const double utc2 = mjd_utc;

    // UTC -> UT1 (for the earth rotation angle)
    double ut11 = 0.0;
    double ut12 = 0.0;
    iauUtcut1(utc1, utc2, eop.ut1_minus_utc_seconds, &ut11, &ut12);

    // UTC -> TAI -> TT (for precession-nutation and the TIO locator s')
    double tai1 = 0.0;
    double tai2 = 0.0;
    double tt1  = 0.0;
    double tt2  = 0.0;
    iauUtctai(utc1, utc2, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);

    // Celestial-to-intermediate matrix (CIO based, IAU 2006/2000A)
    double rc2i[3][3];
    iauC2i06a(tt1, tt2, rc2i);

    // Earth rotation angle, evaluated at UT1
    const double era = iauEra00(ut11, ut12);

    // Polar motion: convert arcseconds to radians and pick up the TIO
    // locator s' from the IAU 2006 model.
    const double sp = iauSp00(tt1, tt2);
    const double xp = eop.xp_arcsec * kArcsecToRad;
    const double yp = eop.yp_arcsec * kArcsecToRad;

    double rpom[3][3];
    iauPom00(xp, yp, sp, rpom);

    // Combine into the full celestial-to-terrestrial rotation
    double rc2t[3][3];
    iauC2tcio(rc2i, era, rpom, rc2t);

    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R(i, j) = rc2t[i][j];
        }
    }
    return R;
}

double gnssTimeToMjdUtc(const GNSSTime& gps_time) {
    // First-pass MJD UTC, treating GPS time as if it were UTC (off by
    // ~30 s due to elapsed leap seconds since the GPS epoch).
    const double total_seconds_gps =
        static_cast<double>(gps_time.week) * 604800.0 + gps_time.tow;
    const double mjd_approx = kMjd0Gps + total_seconds_gps / 86400.0;

    // Look up TAI-UTC at this approximate UTC date so we can correct
    // the offset. SOFA's iauDat returns ΔAT in seconds.
    int iy = 0;
    int im = 0;
    int id = 0;
    double fd = 0.0;
    iauJd2cal(kMjdJ2000Epoch, mjd_approx, &iy, &im, &id, &fd);

    double dat = 0.0;
    iauDat(iy, im, id, fd, &dat);

    // GPS time = TAI - 19 s = UTC + (ΔAT - 19).  Therefore
    //   UTC seconds elapsed = GPS seconds elapsed - (ΔAT - 19).
    // (At the GPS epoch ΔAT was 19, so this is exact at t = 0.)
    const double total_seconds_utc = total_seconds_gps - (dat - 19.0);
    return kMjd0Gps + total_seconds_utc / 86400.0;
}

}  // namespace libgnss::iers
