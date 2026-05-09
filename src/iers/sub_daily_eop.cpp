#include "libgnss++/iers/sub_daily_eop.hpp"

#include <array>
#include <cmath>

extern "C" {
#include "sofa.h"
}

namespace libgnss::iers {

namespace {

constexpr double kMjdReference   = 2400000.5;  // MJD <-> JD offset
constexpr double kMjdJ2000       = 51544.5;    // J2000.0 in MJD (TT)
constexpr double kJulianCentury  = 36525.0;
constexpr double kSecondsPerDay  = 86400.0;
constexpr double kMicroToBase    = 1.0e-6;

struct FundamentalArgs {
    double gmst{0.0};
    double l{0.0};
    double lp{0.0};
    double f{0.0};
    double d{0.0};
    double omega{0.0};
};

FundamentalArgs computeFundamentalArgs(double mjd_utc, double ut1_utc_seconds) {
    // UTC -> TAI -> TT via SOFA's leap-second table.
    double tai1 = 0.0, tai2 = 0.0, tt1 = 0.0, tt2 = 0.0;
    iauUtctai(kMjdReference, mjd_utc, &tai1, &tai2);
    iauTaitt(tai1, tai2, &tt1, &tt2);
    const double mjd_tt = (tt1 - kMjdReference) + tt2;
    const double t_centuries = (mjd_tt - kMjdJ2000) / kJulianCentury;

    // UT1 = UTC + (UT1-UTC).
    const double mjd_ut1 = mjd_utc + ut1_utc_seconds / kSecondsPerDay;

    FundamentalArgs fa;
    // GMST + π. The +π offset matches the IERS Conventions 2010
    // sub-daily-EOP table convention (table arguments are `θ + π`
    // where θ is the standard GMST), so the harmonic-series
    // amplitudes can be applied as published without extra phase
    // shifts.
    fa.gmst  = iauGmst06(kMjdReference, mjd_ut1, kMjdReference, mjd_tt) + M_PI;
    fa.l     = iauFal03(t_centuries);
    fa.lp    = iauFalp03(t_centuries);
    fa.f     = iauFaf03(t_centuries);
    fa.d     = iauFad03(t_centuries);
    fa.omega = iauFaom03(t_centuries);
    return fa;
}

inline double dotArg(const std::array<int, 6>& doodson,
                     const FundamentalArgs& fa) {
    return doodson[0] * fa.gmst
         + doodson[1] * fa.l
         + doodson[2] * fa.lp
         + doodson[3] * fa.f
         + doodson[4] * fa.d
         + doodson[5] * fa.omega;
}

// IERS Conventions 2010 Table 5.1a — pole-libration sub-diurnal terms
// (10 rows). Amplitudes in micro-arcseconds. Layout per row:
//   [0..5] : Doodson coefficients (γ, l, l', F, D, Ω) integer multipliers
//   [6..7] : xp Cos / Sin amplitude
//   [8..9] : yp Cos / Sin amplitude
constexpr std::array<std::array<double, 10>, 10> kPmLibrationRows{{
    { 1, -1,  0, -2,  0, -1,   -0.4,    0.3,   -0.3,   -0.4},
    { 1, -1,  0, -2,  0, -2,   -2.3,    1.3,   -1.3,   -2.3},
    { 1,  1,  0, -2, -2, -2,   -0.4,    0.3,   -0.3,   -0.4},
    { 1,  0,  0, -2,  0, -1,   -2.1,    1.2,   -1.2,   -2.1},
    { 1,  0,  0, -2,  0, -2,  -11.4,    6.5,   -6.5,  -11.4},
    { 1, -1,  0,  0,  0,  0,    0.8,   -0.5,    0.5,    0.8},
    { 1,  0,  0, -2,  2, -2,   -4.8,    2.7,   -2.7,   -4.8},
    { 1,  0,  0,  0,  0,  0,   14.3,   -8.2,    8.2,   14.3},
    { 1,  0,  0,  0,  0, -1,    1.9,   -1.1,    1.1,    1.9},
    { 1,  1,  0,  0,  0,  0,    0.8,   -0.4,    0.4,    0.8},
}};

// IERS Conventions 2010 Table 5.1b — UT1 / LOD libration sub-diurnal
// terms (11 rows). Amplitudes in micro-seconds. Layout per row:
//   [0..5] : Doodson coefficients (γ, l, l', F, D, Ω)
//   [6..7] : UT1 Cos / Sin amplitude
//   [8..9] : LOD Cos / Sin amplitude
constexpr std::array<std::array<double, 10>, 11> kUtLibrationRows{{
    { 2, -2,  0, -2,  0, -2,    0.05,  -0.03,   -0.3,   -0.6},
    { 2,  0,  0, -2, -2, -2,    0.06,  -0.03,   -0.4,   -0.7},
    { 2, -1,  0, -2,  0, -2,    0.35,  -0.20,   -2.4,   -4.2},
    { 2,  1,  0, -2, -2, -2,    0.07,  -0.04,   -0.5,   -0.8},
    { 2,  0,  0, -2,  0, -1,   -0.07,   0.04,    0.5,    0.8},
    { 2,  0,  0, -2,  0, -2,    1.75,  -1.01,  -12.2,  -21.3},
    { 2,  1,  0, -2,  0, -2,   -0.05,   0.03,    0.3,    0.6},
    { 2,  0, -1, -2,  2, -2,    0.05,  -0.03,   -0.3,   -0.6},
    { 2,  0,  0, -2,  2, -2,    0.76,  -0.44,   -5.5,   -9.5},
    { 2,  0,  0,  0,  0,  0,    0.21,  -0.12,   -1.5,   -2.6},
    { 2,  0,  0,  0,  0, -1,    0.06,  -0.04,   -0.4,   -0.8},
}};

// IERS Conventions 2010 Table 8.2 — ocean-tide EOP corrections
// (71 rows; Eanes-Ray model). Layout per row:
//   [0..5]  : Doodson coefficients (γ, l, l', F, D, Ω)
//   [6..7]  : xp Cos / Sin   (μas)
//   [8..9]  : yp Cos / Sin   (μas)
//   [10..11]: UT1 Cos / Sin  (μs)
constexpr std::array<std::array<double, 12>, 71> kOceanTideRows{{
    { 1,-1, 0,-2,-2,-2,   -0.05,    0.94,   -0.94,   -0.05,    0.396,  -0.078},
    { 1,-2, 0,-2, 0,-1,    0.06,    0.64,   -0.64,    0.06,    0.195,  -0.059},
    { 1,-2, 0,-2, 0,-2,    0.30,    3.42,   -3.42,    0.30,    1.034,  -0.314},
    { 1, 0, 0,-2,-2,-1,    0.08,    0.78,   -0.78,    0.08,    0.224,  -0.073},
    { 1, 0, 0,-2,-2,-2,    0.46,    4.15,   -4.15,    0.45,    1.187,  -0.387},
    { 1,-1, 0,-2, 0,-1,    1.19,    4.96,   -4.96,    1.19,    0.966,  -0.474},
    { 1,-1, 0,-2, 0,-2,    6.24,   26.31,  -26.31,    6.23,    5.118,  -2.499},
    { 1, 1, 0,-2,-2,-1,    0.24,    0.94,   -0.94,    0.24,    0.172,  -0.090},
    { 1, 1, 0,-2,-2,-2,    1.28,    4.99,   -4.99,    1.28,    0.911,  -0.475},
    { 1, 0, 0,-2, 0, 0,   -0.28,   -0.77,    0.77,   -0.28,   -0.093,   0.070},
    { 1, 0, 0,-2, 0,-1,    9.22,   25.06,  -25.06,    9.22,    3.025,  -2.280},
    { 1, 0, 0,-2, 0,-2,   48.82,  132.91, -132.90,   48.82,   16.020, -12.069},
    { 1,-2, 0, 0, 0, 0,   -0.32,   -0.86,    0.86,   -0.32,   -0.103,   0.078},
    { 1, 0, 0, 0,-2, 0,   -0.66,   -1.72,    1.72,   -0.66,   -0.194,   0.154},
    { 1,-1, 0,-2, 2,-2,   -0.42,   -0.92,    0.92,   -0.42,   -0.083,   0.074},
    { 1, 1, 0,-2, 0,-1,   -0.30,   -0.64,    0.64,   -0.30,   -0.057,   0.050},
    { 1, 1, 0,-2, 0,-2,   -1.61,   -3.46,    3.46,   -1.61,   -0.308,   0.271},
    { 1,-1, 0, 0, 0, 0,   -4.48,   -9.61,    9.61,   -4.48,   -0.856,   0.751},
    { 1,-1, 0, 0, 0,-1,   -0.90,   -1.93,    1.93,   -0.90,   -0.172,   0.151},
    { 1, 1, 0, 0,-2, 0,   -0.86,   -1.81,    1.81,   -0.86,   -0.161,   0.137},
    { 1, 0,-1,-2, 2,-2,    1.54,    3.03,   -3.03,    1.54,    0.315,  -0.189},
    { 1, 0, 0,-2, 2,-1,   -0.29,   -0.58,    0.58,   -0.29,   -0.062,   0.035},
    { 1, 0, 0,-2, 2,-2,   26.13,   51.25,  -51.25,   26.13,    5.512,  -3.095},
    { 1, 0, 1,-2, 2,-2,   -0.22,   -0.42,    0.42,   -0.22,   -0.047,   0.025},
    { 1, 0,-1, 0, 0, 0,   -0.61,   -1.20,    1.20,   -0.61,   -0.134,   0.070},
    { 1, 0, 0, 0, 0, 1,    1.54,    3.00,   -3.00,    1.54,    0.348,  -0.171},
    { 1, 0, 0, 0, 0, 0,  -77.48, -151.74,  151.74,  -77.48,  -17.620,   8.548},
    { 1, 0, 0, 0, 0,-1,  -10.52,  -20.56,   20.56,  -10.52,   -2.392,   1.159},
    { 1, 0, 0, 0, 0,-2,    0.23,    0.44,   -0.44,    0.23,    0.052,  -0.025},
    { 1, 0, 1, 0, 0, 0,   -0.61,   -1.19,    1.19,   -0.61,   -0.144,   0.065},
    { 1, 0, 0, 2,-2, 2,   -1.09,   -2.11,    2.11,   -1.09,   -0.267,   0.111},
    { 1,-1, 0, 0, 2, 0,   -0.69,   -1.43,    1.43,   -0.69,   -0.288,   0.043},
    { 1, 1, 0, 0, 0, 0,   -3.46,   -7.28,    7.28,   -3.46,   -1.610,   0.187},
    { 1, 1, 0, 0, 0,-1,   -0.69,   -1.44,    1.44,   -0.69,   -0.320,   0.037},
    { 1, 0, 0, 0, 2, 0,   -0.37,   -1.06,    1.06,   -0.37,   -0.407,  -0.005},
    { 1, 2, 0, 0, 0, 0,   -0.17,   -0.51,    0.51,   -0.17,   -0.213,  -0.005},
    { 1, 0, 0, 2, 0, 2,   -1.10,   -3.42,    3.42,   -1.09,   -1.436,  -0.037},
    { 1, 0, 0, 2, 0, 1,   -0.70,   -2.19,    2.19,   -0.70,   -0.921,  -0.023},
    { 1, 0, 0, 2, 0, 0,   -0.15,   -0.46,    0.46,   -0.15,   -0.193,  -0.005},
    { 1, 1, 0, 2, 0, 2,   -0.03,   -0.59,    0.59,   -0.03,   -0.396,  -0.024},
    { 1, 1, 0, 2, 0, 1,   -0.02,   -0.38,    0.38,   -0.02,   -0.253,  -0.015},
    { 2,-3, 0,-2, 0,-2,   -0.49,   -0.04,    0.63,    0.24,   -0.089,  -0.011},
    { 2,-1, 0,-2,-2,-2,   -1.33,   -0.17,    1.53,    0.68,   -0.224,  -0.032},
    { 2,-2, 0,-2, 0,-2,   -6.08,   -1.61,    3.13,    3.35,   -0.637,  -0.177},
    { 2, 0, 0,-2,-2,-2,   -7.59,   -2.05,    3.44,    4.23,   -0.745,  -0.222},
    { 2, 0, 1,-2,-2,-2,   -0.52,   -0.14,    0.22,    0.29,   -0.049,  -0.015},
    { 2,-1,-1,-2, 0,-2,    0.47,    0.11,   -0.10,   -0.27,    0.033,   0.013},
    { 2,-1, 0,-2, 0,-1,    2.12,    0.49,   -0.41,   -1.23,    0.141,   0.058},
    { 2,-1, 0,-2, 0,-2,  -56.87,  -12.93,   11.15,   32.88,   -3.795,  -1.556},
    { 2,-1, 1,-2, 0,-2,   -0.54,   -0.12,    0.10,    0.31,   -0.035,  -0.015},
    { 2, 1, 0,-2,-2,-2,  -11.01,   -2.40,    1.89,    6.41,   -0.698,  -0.298},
    { 2, 1, 1,-2,-2,-2,   -0.51,   -0.11,    0.08,    0.30,   -0.032,  -0.014},
    { 2,-2, 0,-2, 2,-2,    0.98,    0.11,   -0.11,   -0.58,    0.050,   0.022},
    { 2, 0,-1,-2, 0,-2,    1.13,    0.11,   -0.13,   -0.67,    0.056,   0.025},
    { 2, 0, 0,-2, 0,-1,   12.32,    1.00,   -1.41,   -7.31,    0.605,   0.266},
    { 2, 0, 0,-2, 0,-2, -330.15,  -26.96,   37.58,  195.92,  -16.195,  -7.140},
    { 2, 0, 1,-2, 0,-2,   -1.01,   -0.07,    0.11,    0.60,   -0.049,  -0.021},
    { 2,-1, 0,-2, 2,-2,    2.47,   -0.28,   -0.44,   -1.48,    0.111,   0.034},
    { 2, 1, 0,-2, 0,-2,    9.40,   -1.44,   -1.88,   -5.65,    0.425,   0.117},
    { 2,-1, 0, 0, 0, 0,   -2.35,    0.37,    0.47,    1.41,   -0.106,  -0.029},
    { 2,-1, 0, 0, 0,-1,   -1.04,    0.17,    0.21,    0.62,   -0.047,  -0.013},
    { 2, 0,-1,-2, 2,-2,   -8.51,    3.50,    3.29,    5.11,   -0.437,  -0.019},
    { 2, 0, 0,-2, 2,-2, -144.13,   63.56,   59.23,   86.56,   -7.547,  -0.159},
    { 2, 0, 1,-2, 2,-2,    1.19,   -0.56,   -0.52,   -0.72,    0.064,   0.000},
    { 2, 0, 0, 0, 0, 1,    0.49,   -0.25,   -0.23,   -0.29,    0.027,  -0.001},
    { 2, 0, 0, 0, 0, 0,  -38.48,   19.14,   17.72,   23.11,   -2.104,   0.041},
    { 2, 0, 0, 0, 0,-1,  -11.44,    5.75,    5.32,    6.87,   -0.627,   0.015},
    { 2, 0, 0, 0, 0,-2,   -1.24,    0.63,    0.58,    0.75,   -0.068,   0.002},
    { 2, 1, 0, 0, 0, 0,   -1.77,    1.79,    1.71,    1.04,   -0.146,   0.037},
    { 2, 1, 0, 0, 0,-1,   -0.77,    0.78,    0.75,    0.45,   -0.064,   0.017},
    { 2, 0, 0, 2, 0, 2,   -0.33,    0.62,    0.65,    0.19,   -0.049,   0.018},
}};

}  // namespace

SubDailyEopDelta subDailyEopCorrection(double mjd_utc,
                                       double ut1_utc_seconds) {
    const auto fa = computeFundamentalArgs(mjd_utc, ut1_utc_seconds);

    double dxp_uas = 0.0;
    double dyp_uas = 0.0;
    double dut1_us = 0.0;
    double dlod_us = 0.0;

    // Tab 5.1a: pole libration -> xp/yp only.
    for (const auto& row : kPmLibrationRows) {
        const std::array<int, 6> doodson{
            int(row[0]), int(row[1]), int(row[2]),
            int(row[3]), int(row[4]), int(row[5])};
        const double arg = dotArg(doodson, fa);
        const double s = std::sin(arg);
        const double c = std::cos(arg);
        dxp_uas += s * row[6] + c * row[7];
        dyp_uas += s * row[8] + c * row[9];
    }

    // Tab 5.1b: UT1 / LOD libration only.
    for (const auto& row : kUtLibrationRows) {
        const std::array<int, 6> doodson{
            int(row[0]), int(row[1]), int(row[2]),
            int(row[3]), int(row[4]), int(row[5])};
        const double arg = dotArg(doodson, fa);
        const double s = std::sin(arg);
        const double c = std::cos(arg);
        dut1_us += s * row[6] + c * row[7];
        dlod_us += s * row[8] + c * row[9];
    }

    // Tab 8.2: ocean-tide -> xp/yp + UT1.
    for (const auto& row : kOceanTideRows) {
        const std::array<int, 6> doodson{
            int(row[0]), int(row[1]), int(row[2]),
            int(row[3]), int(row[4]), int(row[5])};
        const double arg = dotArg(doodson, fa);
        const double s = std::sin(arg);
        const double c = std::cos(arg);
        dxp_uas += s * row[6]  + c * row[7];
        dyp_uas += s * row[8]  + c * row[9];
        dut1_us += s * row[10] + c * row[11];
    }

    SubDailyEopDelta delta;
    delta.dxp_arcsec   = dxp_uas * kMicroToBase;
    delta.dyp_arcsec   = dyp_uas * kMicroToBase;
    delta.dut1_seconds = dut1_us * kMicroToBase;
    delta.dlod_seconds = dlod_us * kMicroToBase;
    return delta;
}

}  // namespace libgnss::iers
