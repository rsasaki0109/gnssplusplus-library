#include "libgnss++/iers/ephemeris.hpp"

extern "C" {
#include "sofa.h"
#include "sofam.h"
}

namespace libgnss::iers {

namespace {

constexpr double kMjdJ2000Epoch = 2400000.5;  // MJD <-> JD offset

/// Convert MJD UTC to TT 2-part Julian Date via SOFA's leap-second-aware
/// chain. Output: (tt1, tt2) such that tt1 + tt2 == JD(TT).
inline void mjdUtcToTt2(double mjd_utc, double& tt1, double& tt2) {
    double tai1 = 0.0;
    double tai2 = 0.0;
    iauUtctai(kMjdJ2000Epoch, mjd_utc, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);
}

}  // namespace

Eigen::Vector3d sunPositionIcrs(double mjd_utc) {
    double tt1 = 0.0;
    double tt2 = 0.0;
    mjdUtcToTt2(mjd_utc, tt1, tt2);

    // iauEpv00 returns Earth's position-velocity in two reference
    // frames at the requested TT epoch:
    //   pvh: heliocentric ([0] = position, [1] = velocity), AU
    //   pvb: barycentric  ([0] = position, [1] = velocity), AU
    // The Sun's geocentric position in the ICRS is obtained by
    // negating Earth's heliocentric position vector.
    double pvh[2][3];
    double pvb[2][3];
    iauEpv00(tt1, tt2, pvh, pvb);

    return Eigen::Vector3d(-pvh[0][0] * DAU,
                           -pvh[0][1] * DAU,
                           -pvh[0][2] * DAU);
}

Eigen::Vector3d moonPositionIcrs(double mjd_utc) {
    double tt1 = 0.0;
    double tt2 = 0.0;
    mjdUtcToTt2(mjd_utc, tt1, tt2);

    // iauMoon98 returns the Moon's geocentric position-velocity in
    // the ICRS at the requested TT epoch, in AU and AU/day.
    double pv[2][3];
    iauMoon98(tt1, tt2, pv);

    return Eigen::Vector3d(pv[0][0] * DAU,
                           pv[0][1] * DAU,
                           pv[0][2] * DAU);
}

}  // namespace libgnss::iers
