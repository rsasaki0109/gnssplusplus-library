#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <sstream>

namespace libgnss {
namespace tidal {

namespace {

constexpr double kPi = 3.1415926535897932;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kArcsecToRadians = kDegreesToRadians / 3600.0;
constexpr double kAstronomicalUnitMeters = 149597870691.0;
constexpr double kEarthGM = 3.986004415e14;
constexpr double kSunGM = 1.327124e20;
constexpr double kMoonGM = 4.902801e12;
constexpr double kUnixGpsEpochSeconds = 315964800.0;

double wrapRadians(double angle_rad) {
    const double wrapped = std::fmod(angle_rad, 2.0 * kPi);
    return wrapped < 0.0 ? wrapped + 2.0 * kPi : wrapped;
}

std::string trimCopy(const std::string& text) {
    const auto first = std::find_if_not(
        text.begin(), text.end(),
        [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(
        text.rbegin(), text.rend(),
        [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

std::string normalizeName(const std::string& text) {
    std::string out;
    std::istringstream iss(text);
    iss >> out;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return out;
}

// Days since 1970-01-01, Gregorian calendar.
long long daysFromCivil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return static_cast<long long>(era) * 146097LL + static_cast<long long>(doe) - 719468LL;
}

double epochToUnix(int year, int month, int day, int hour, int minute, double second) {
    const long long days = daysFromCivil(year, static_cast<unsigned>(month),
                                         static_cast<unsigned>(day));
    return static_cast<double>(days) * 86400.0 +
           static_cast<double>(hour * 3600 + minute * 60) + second;
}

void unixToEpoch(double unix_seconds, double ep[6]) {
    double integral = 0.0;
    double fractional = std::modf(unix_seconds, &integral);
    if (fractional < 0.0) {
        fractional += 1.0;
        integral -= 1.0;
    }
    const std::time_t whole_seconds = static_cast<std::time_t>(integral);
    std::tm utc_tm{};
    gmtime_r(&whole_seconds, &utc_tm);
    ep[0] = utc_tm.tm_year + 1900;
    ep[1] = utc_tm.tm_mon + 1;
    ep[2] = utc_tm.tm_mday;
    ep[3] = utc_tm.tm_hour;
    ep[4] = utc_tm.tm_min;
    ep[5] = utc_tm.tm_sec + fractional;
}

double gpsUnixSeconds(const GNSSTime& time) {
    return kUnixGpsEpochSeconds +
           static_cast<double>(time.week) * constants::SECONDS_PER_WEEK + time.tow;
}

struct LeapSecond {
    int year;
    int month;
    int day;
    int gps_minus_utc;
};

const std::array<LeapSecond, 18>& leapSeconds() {
    static const std::array<LeapSecond, 18> kLeaps = {{
        {2017, 1, 1, 18},
        {2015, 7, 1, 17},
        {2012, 7, 1, 16},
        {2009, 1, 1, 15},
        {2006, 1, 1, 14},
        {1999, 1, 1, 13},
        {1997, 7, 1, 12},
        {1996, 1, 1, 11},
        {1994, 7, 1, 10},
        {1993, 7, 1, 9},
        {1992, 7, 1, 8},
        {1991, 1, 1, 7},
        {1990, 1, 1, 6},
        {1988, 1, 1, 5},
        {1985, 7, 1, 4},
        {1983, 7, 1, 3},
        {1982, 7, 1, 2},
        {1981, 7, 1, 1},
    }};
    return kLeaps;
}

double gpsToUtcUnix(double gps_unix_seconds) {
    for (const auto& leap : leapSeconds()) {
        const double utc_candidate =
            gps_unix_seconds - static_cast<double>(leap.gps_minus_utc);
        if (utc_candidate >= epochToUnix(leap.year, leap.month, leap.day, 0, 0, 0.0)) {
            return utc_candidate;
        }
    }
    return gps_unix_seconds;
}

double utcToGpsUnix(double utc_unix_seconds) {
    for (const auto& leap : leapSeconds()) {
        if (utc_unix_seconds >= epochToUnix(leap.year, leap.month, leap.day, 0, 0, 0.0)) {
            return utc_unix_seconds + static_cast<double>(leap.gps_minus_utc);
        }
    }
    return utc_unix_seconds;
}

bool parseElevenValues(const std::string& line, std::array<double, 11>& values) {
    std::istringstream iss(line);
    for (double& value : values) {
        if (!(iss >> value)) {
            return false;
        }
    }
    return true;
}

bool readBlqRecord(std::istream& input, OceanLoadingCoefficients& coefficients) {
    std::string line;
    int row = 0;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.rfind("$$", 0) == 0) {
            continue;
        }
        std::array<double, 11> values{};
        if (!parseElevenValues(trimmed, values)) {
            continue;
        }
        switch (row) {
            case 0: coefficients.up_amplitudes_m = values; break;
            case 1: coefficients.west_amplitudes_m = values; break;
            case 2: coefficients.south_amplitudes_m = values; break;
            case 3: coefficients.up_phases_deg = values; break;
            case 4: coefficients.west_phases_deg = values; break;
            case 5: coefficients.south_phases_deg = values; break;
            default: break;
        }
        ++row;
        if (row == 6) {
            return true;
        }
    }
    return false;
}

EarthRotationValues interpolateErpAtUnixArgument(const EarthRotationParameters& erp,
                                                 double rtklib_time_argument_unix) {
    EarthRotationValues values;
    if (erp.entries.empty()) {
        return values;
    }

    // RTKLIB geterp() expects GPST and converts it with gpst2utc(). CLASLIB
    // tidedisp() passes UTC into geterp(); callers that need exact parity pass
    // that same UTC-valued argument here.
    const double utc_unix = gpsToUtcUnix(rtklib_time_argument_unix);
    const double ep2000_noon = epochToUnix(2000, 1, 1, 12, 0, 0.0);
    const double mjd = 51544.5 + (utc_unix - ep2000_noon) / 86400.0;

    const auto assign_from = [&](const EarthRotationParameters::Entry& entry, double day) {
        values.xp_rad = entry.xp_rad + entry.xpr_rad_per_day * day;
        values.yp_rad = entry.yp_rad + entry.ypr_rad_per_day * day;
        values.ut1_utc_s = entry.ut1_utc_s - entry.lod_s_per_day * day;
        values.lod_s_per_day = entry.lod_s_per_day;
        values.valid = true;
    };

    if (mjd <= erp.entries.front().mjd) {
        assign_from(erp.entries.front(), mjd - erp.entries.front().mjd);
        return values;
    }
    if (mjd >= erp.entries.back().mjd) {
        assign_from(erp.entries.back(), mjd - erp.entries.back().mjd);
        return values;
    }

    size_t lo = 0;
    size_t hi = erp.entries.size() - 1;
    while (lo < hi - 1) {
        const size_t mid = (lo + hi) / 2;
        if (mjd < erp.entries[mid].mjd) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    const auto& a = erp.entries[lo];
    const auto& b = erp.entries[hi];
    const double alpha = (a.mjd == b.mjd) ? 0.5 : (mjd - a.mjd) / (b.mjd - a.mjd);
    values.xp_rad = (1.0 - alpha) * a.xp_rad + alpha * b.xp_rad;
    values.yp_rad = (1.0 - alpha) * a.yp_rad + alpha * b.yp_rad;
    values.ut1_utc_s = (1.0 - alpha) * a.ut1_utc_s + alpha * b.ut1_utc_s;
    values.lod_s_per_day = (1.0 - alpha) * a.lod_s_per_day + alpha * b.lod_s_per_day;
    values.valid = true;
    return values;
}

void astronomicalArguments(double t, double f[5]) {
    static const double fc[5][5] = {
        {134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470},
        {357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149},
        {93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417},
        {297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169},
        {125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939},
    };

    double tt[4] = {t, t * t, t * t * t, t * t * t * t};
    for (int i = 0; i < 5; ++i) {
        f[i] = fc[i][0] * 3600.0;
        for (int j = 0; j < 4; ++j) {
            f[i] += fc[i][j + 1] * tt[j];
        }
        f[i] = std::fmod(f[i] * kArcsecToRadians, 2.0 * kPi);
    }
}

void nutationIau1980(double t, const double* f, double& dpsi, double& deps) {
    static const double nut[106][10] = {
        {   0,   0,   0,   0,   1, -6798.4, -171996, -174.2, 92025,   8.9},
        {   0,   0,   2,  -2,   2,   182.6,  -13187,   -1.6,  5736,  -3.1},
        {   0,   0,   2,   0,   2,    13.7,   -2274,   -0.2,   977,  -0.5},
        {   0,   0,   0,   0,   2, -3399.2,    2062,    0.2,  -895,   0.5},
        {   0,  -1,   0,   0,   0,  -365.3,   -1426,    3.4,    54,  -0.1},
        {   1,   0,   0,   0,   0,    27.6,     712,    0.1,    -7,   0.0},
        {   0,   1,   2,  -2,   2,   121.7,    -517,    1.2,   224,  -0.6},
        {   0,   0,   2,   0,   1,    13.6,    -386,   -0.4,   200,   0.0},
        {   1,   0,   2,   0,   2,     9.1,    -301,    0.0,   129,  -0.1},
        {   0,  -1,   2,  -2,   2,   365.2,     217,   -0.5,   -95,   0.3},
        {  -1,   0,   0,   2,   0,    31.8,     158,    0.0,    -1,   0.0},
        {   0,   0,   2,  -2,   1,   177.8,     129,    0.1,   -70,   0.0},
        {  -1,   0,   2,   0,   2,    27.1,     123,    0.0,   -53,   0.0},
        {   1,   0,   0,   0,   1,    27.7,      63,    0.1,   -33,   0.0},
        {   0,   0,   0,   2,   0,    14.8,      63,    0.0,    -2,   0.0},
        {  -1,   0,   2,   2,   2,     9.6,     -59,    0.0,    26,   0.0},
        {  -1,   0,   0,   0,   1,   -27.4,     -58,   -0.1,    32,   0.0},
        {   1,   0,   2,   0,   1,     9.1,     -51,    0.0,    27,   0.0},
        {  -2,   0,   0,   2,   0,  -205.9,     -48,    0.0,     1,   0.0},
        {  -2,   0,   2,   0,   1,  1305.5,      46,    0.0,   -24,   0.0},
        {   0,   0,   2,   2,   2,     7.1,     -38,    0.0,    16,   0.0},
        {   2,   0,   2,   0,   2,     6.9,     -31,    0.0,    13,   0.0},
        {   2,   0,   0,   0,   0,    13.8,      29,    0.0,    -1,   0.0},
        {   1,   0,   2,  -2,   2,    23.9,      29,    0.0,   -12,   0.0},
        {   0,   0,   2,   0,   0,    13.6,      26,    0.0,    -1,   0.0},
        {   0,   0,   2,  -2,   0,   173.3,     -22,    0.0,     0,   0.0},
        {  -1,   0,   2,   0,   1,    27.0,      21,    0.0,   -10,   0.0},
        {   0,   2,   0,   0,   0,   182.6,      17,   -0.1,     0,   0.0},
        {   0,   2,   2,  -2,   2,    91.3,     -16,    0.1,     7,   0.0},
        {  -1,   0,   0,   2,   1,    32.0,      16,    0.0,    -8,   0.0},
        {   0,   1,   0,   0,   1,   386.0,     -15,    0.0,     9,   0.0},
        {   1,   0,   0,  -2,   1,   -31.7,     -13,    0.0,     7,   0.0},
        {   0,  -1,   0,   0,   1,  -346.6,     -12,    0.0,     6,   0.0},
        {   2,   0,  -2,   0,   0, -1095.2,      11,    0.0,     0,   0.0},
        {  -1,   0,   2,   2,   1,     9.5,     -10,    0.0,     5,   0.0},
        {   1,   0,   2,   2,   2,     5.6,      -8,    0.0,     3,   0.0},
        {   0,  -1,   2,   0,   2,    14.2,      -7,    0.0,     3,   0.0},
        {   0,   0,   2,   2,   1,     7.1,      -7,    0.0,     3,   0.0},
        {   1,   1,   0,  -2,   0,   -34.8,      -7,    0.0,     0,   0.0},
        {   0,   1,   2,   0,   2,    13.2,       7,    0.0,    -3,   0.0},
        {  -2,   0,   0,   2,   1,  -199.8,      -6,    0.0,     3,   0.0},
        {   0,   0,   0,   2,   1,    14.8,      -6,    0.0,     3,   0.0},
        {   2,   0,   2,  -2,   2,    12.8,       6,    0.0,    -3,   0.0},
        {   1,   0,   0,   2,   0,     9.6,       6,    0.0,     0,   0.0},
        {   1,   0,   2,  -2,   1,    23.9,       6,    0.0,    -3,   0.0},
        {   0,   0,   0,  -2,   1,   -14.7,      -5,    0.0,     3,   0.0},
        {   0,  -1,   2,  -2,   1,   346.6,      -5,    0.0,     3,   0.0},
        {   2,   0,   2,   0,   1,     6.9,      -5,    0.0,     3,   0.0},
        {   1,  -1,   0,   0,   0,    29.8,       5,    0.0,     0,   0.0},
        {   1,   0,   0,  -1,   0,   411.8,      -4,    0.0,     0,   0.0},
        {   0,   0,   0,   1,   0,    29.5,      -4,    0.0,     0,   0.0},
        {   0,   1,   0,  -2,   0,   -15.4,      -4,    0.0,     0,   0.0},
        {   1,   0,  -2,   0,   0,   -26.9,       4,    0.0,     0,   0.0},
        {   2,   0,   0,  -2,   1,   212.3,       4,    0.0,    -2,   0.0},
        {   0,   1,   2,  -2,   1,   119.6,       4,    0.0,    -2,   0.0},
        {   1,   1,   0,   0,   0,    25.6,      -3,    0.0,     0,   0.0},
        {   1,  -1,   0,  -1,   0, -3232.9,      -3,    0.0,     0,   0.0},
        {  -1,  -1,   2,   2,   2,     9.8,      -3,    0.0,     1,   0.0},
        {   0,  -1,   2,   2,   2,     7.2,      -3,    0.0,     1,   0.0},
        {   1,  -1,   2,   0,   2,     9.4,      -3,    0.0,     1,   0.0},
        {   3,   0,   2,   0,   2,     5.5,      -3,    0.0,     1,   0.0},
        {  -2,   0,   2,   0,   2,  1615.7,      -3,    0.0,     1,   0.0},
        {   1,   0,   2,   0,   0,     9.1,       3,    0.0,     0,   0.0},
        {  -1,   0,   2,   4,   2,     5.8,      -2,    0.0,     1,   0.0},
        {   1,   0,   0,   0,   2,    27.8,      -2,    0.0,     1,   0.0},
        {  -1,   0,   2,  -2,   1,   -32.6,      -2,    0.0,     1,   0.0},
        {   0,  -2,   2,  -2,   1,  6786.3,      -2,    0.0,     1,   0.0},
        {  -2,   0,   0,   0,   1,   -13.7,      -2,    0.0,     1,   0.0},
        {   2,   0,   0,   0,   1,    13.8,       2,    0.0,    -1,   0.0},
        {   3,   0,   0,   0,   0,     9.2,       2,    0.0,     0,   0.0},
        {   1,   1,   2,   0,   2,     8.9,       2,    0.0,    -1,   0.0},
        {   0,   0,   2,   1,   2,     9.3,       2,    0.0,    -1,   0.0},
        {   1,   0,   0,   2,   1,     9.6,      -1,    0.0,     0,   0.0},
        {   1,   0,   2,   2,   1,     5.6,      -1,    0.0,     1,   0.0},
        {   1,   1,   0,  -2,   1,   -34.7,      -1,    0.0,     0,   0.0},
        {   0,   1,   0,   2,   0,    14.2,      -1,    0.0,     0,   0.0},
        {   0,   1,   2,  -2,   0,   117.5,      -1,    0.0,     0,   0.0},
        {   0,   1,  -2,   2,   0,  -329.8,      -1,    0.0,     0,   0.0},
        {   1,   0,  -2,   2,   0,    23.8,      -1,    0.0,     0,   0.0},
        {   1,   0,  -2,  -2,   0,    -9.5,      -1,    0.0,     0,   0.0},
        {   1,   0,   2,  -2,   0,    32.8,      -1,    0.0,     0,   0.0},
        {   1,   0,   0,  -4,   0,   -10.1,      -1,    0.0,     0,   0.0},
        {   2,   0,   0,  -4,   0,   -15.9,      -1,    0.0,     0,   0.0},
        {   0,   0,   2,   4,   2,     4.8,      -1,    0.0,     0,   0.0},
        {   0,   0,   2,  -1,   2,    25.4,      -1,    0.0,     0,   0.0},
        {  -2,   0,   2,   4,   2,     7.3,      -1,    0.0,     1,   0.0},
        {   2,   0,   2,   2,   2,     4.7,      -1,    0.0,     0,   0.0},
        {   0,  -1,   2,   0,   1,    14.2,      -1,    0.0,     0,   0.0},
        {   0,   0,  -2,   0,   1,   -13.6,      -1,    0.0,     0,   0.0},
        {   0,   0,   4,  -2,   2,    12.7,       1,    0.0,     0,   0.0},
        {   0,   1,   0,   0,   2,   409.2,       1,    0.0,     0,   0.0},
        {   1,   1,   2,  -2,   2,    22.5,       1,    0.0,    -1,   0.0},
        {   3,   0,   2,  -2,   2,     8.7,       1,    0.0,     0,   0.0},
        {  -2,   0,   2,   2,   2,    14.6,       1,    0.0,    -1,   0.0},
        {  -1,   0,   0,   0,   2,   -27.3,       1,    0.0,    -1,   0.0},
        {   0,   0,  -2,   2,   1,  -169.0,       1,    0.0,     0,   0.0},
        {   0,   1,   2,   0,   1,    13.1,       1,    0.0,     0,   0.0},
        {  -1,   0,   4,   0,   2,     9.1,       1,    0.0,     0,   0.0},
        {   2,   1,   0,  -2,   0,   131.7,       1,    0.0,     0,   0.0},
        {   2,   0,   0,   2,   0,     7.1,       1,    0.0,     0,   0.0},
        {   2,   0,   2,  -2,   1,    12.8,       1,    0.0,    -1,   0.0},
        {   2,   0,  -2,   0,   1,  -943.2,       1,    0.0,     0,   0.0},
        {   1,  -1,   0,  -2,   0,   -29.3,       1,    0.0,     0,   0.0},
        {  -1,   0,   0,   1,   1,  -388.3,       1,    0.0,     0,   0.0},
        {  -1,  -1,   0,   2,   1,    35.0,       1,    0.0,     0,   0.0},
        {   0,   1,   0,   1,   0,    27.3,       1,    0.0,     0,   0.0}
    };

    dpsi = 0.0;
    deps = 0.0;
    for (int i = 0; i < 106; ++i) {
        double angle = 0.0;
        for (int j = 0; j < 5; ++j) {
            angle += nut[i][j] * f[j];
        }
        dpsi += (nut[i][6] + nut[i][7] * t) * std::sin(angle);
        deps += (nut[i][8] + nut[i][9] * t) * std::cos(angle);
    }
    dpsi *= 1e-4 * kArcsecToRadians;
    deps *= 1e-4 * kArcsecToRadians;
}

Eigen::Matrix3d rx(double t) {
    Eigen::Matrix3d x = Eigen::Matrix3d::Zero();
    x(0, 0) = 1.0;
    x(1, 1) = std::cos(t);
    x(1, 2) = std::sin(t);
    x(2, 1) = -std::sin(t);
    x(2, 2) = std::cos(t);
    return x;
}

Eigen::Matrix3d ry(double t) {
    Eigen::Matrix3d y = Eigen::Matrix3d::Zero();
    y(1, 1) = 1.0;
    y(0, 0) = std::cos(t);
    y(0, 2) = -std::sin(t);
    y(2, 0) = std::sin(t);
    y(2, 2) = std::cos(t);
    return y;
}

Eigen::Matrix3d rz(double t) {
    Eigen::Matrix3d z = Eigen::Matrix3d::Zero();
    z(0, 0) = std::cos(t);
    z(0, 1) = std::sin(t);
    z(1, 0) = -std::sin(t);
    z(1, 1) = std::cos(t);
    z(2, 2) = 1.0;
    return z;
}

double utcToGmst(double utc_unix, double ut1_utc_s) {
    const double tut = utc_unix + ut1_utc_s;
    double ep[6] = {};
    unixToEpoch(tut, ep);
    const double ut = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
    const double tut0 = epochToUnix(static_cast<int>(ep[0]), static_cast<int>(ep[1]),
                                    static_cast<int>(ep[2]), 0, 0, 0.0);
    const double ep2000_noon = epochToUnix(2000, 1, 1, 12, 0, 0.0);
    const double t1 = (tut0 - ep2000_noon) / 86400.0 / 36525.0;
    const double t2 = t1 * t1;
    const double t3 = t2 * t1;
    const double gmst0 = 24110.54841 + 8640184.812866 * t1 +
                         0.093104 * t2 - 6.2e-6 * t3;
    const double gmst = gmst0 + 1.002737909350795 * ut;
    return std::fmod(gmst, 86400.0) * kPi / 43200.0;
}

Eigen::Matrix3d eciToEcef(double utc_unix,
                          const EarthRotationValues& erpv,
                          double& gmst_out) {
    const double ep2000_noon = epochToUnix(2000, 1, 1, 12, 0, 0.0);
    const double tgps = utcToGpsUnix(utc_unix);
    const double t = (tgps - ep2000_noon + 19.0 + 32.184) / 86400.0 / 36525.0;
    const double t2 = t * t;
    const double t3 = t2 * t;

    double f[5] = {};
    astronomicalArguments(t, f);

    const double ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * kArcsecToRadians;
    const double th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * kArcsecToRadians;
    const double z = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * kArcsecToRadians;
    const double eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) *
                       kArcsecToRadians;
    const Eigen::Matrix3d p = rz(-z) * ry(th) * rz(-ze);

    double dpsi = 0.0;
    double deps = 0.0;
    nutationIau1980(t, f, dpsi, deps);
    const Eigen::Matrix3d n = rx(-eps - deps) * rz(-dpsi) * rx(eps);

    gmst_out = utcToGmst(utc_unix, erpv.ut1_utc_s);
    double gast = gmst_out + dpsi * std::cos(eps);
    gast += (0.00264 * std::sin(f[4]) + 0.000063 * std::sin(2.0 * f[4])) *
            kArcsecToRadians;

    const Eigen::Matrix3d w = ry(-erpv.xp_rad) * rx(-erpv.yp_rad);
    return w * rz(gast) * n * p;
}

void sunMoonPositionEci(double ut1_unix, Vector3d* rsun, Vector3d* rmoon) {
    const double ep2000_noon = epochToUnix(2000, 1, 1, 12, 0, 0.0);
    const double t = (ut1_unix - ep2000_noon) / 86400.0 / 36525.0;
    double f[5] = {};
    astronomicalArguments(t, f);

    const double eps = 23.439291 - 0.0130042 * t;
    const double sine = std::sin(eps * kDegreesToRadians);
    const double cose = std::cos(eps * kDegreesToRadians);

    if (rsun != nullptr) {
        const double ms = 357.5277233 + 35999.05034 * t;
        const double ls = 280.460 + 36000.770 * t +
                          1.914666471 * std::sin(ms * kDegreesToRadians) +
                          0.019994643 * std::sin(2.0 * ms * kDegreesToRadians);
        const double r = kAstronomicalUnitMeters *
                         (1.000140612 - 0.016708617 * std::cos(ms * kDegreesToRadians) -
                          0.000139589 * std::cos(2.0 * ms * kDegreesToRadians));
        const double sinl = std::sin(ls * kDegreesToRadians);
        const double cosl = std::cos(ls * kDegreesToRadians);
        *rsun = Vector3d(r * cosl, r * cose * sinl, r * sine * sinl);
    }

    if (rmoon != nullptr) {
        const double lm = 218.32 + 481267.883 * t + 6.29 * std::sin(f[0]) -
                          1.27 * std::sin(f[0] - 2.0 * f[3]) +
                          0.66 * std::sin(2.0 * f[3]) +
                          0.21 * std::sin(2.0 * f[0]) -
                          0.19 * std::sin(f[1]) -
                          0.11 * std::sin(2.0 * f[2]);
        const double pm = 5.13 * std::sin(f[2]) + 0.28 * std::sin(f[0] + f[2]) -
                          0.28 * std::sin(f[2] - f[0]) -
                          0.17 * std::sin(f[2] - 2.0 * f[3]);
        const double rm = constants::WGS84_A /
                          std::sin((0.9508 + 0.0518 * std::cos(f[0]) +
                                    0.0095 * std::cos(f[0] - 2.0 * f[3]) +
                                    0.0078 * std::cos(2.0 * f[3]) +
                                    0.0028 * std::cos(2.0 * f[0])) *
                                   kDegreesToRadians);
        const double sinl = std::sin(lm * kDegreesToRadians);
        const double cosl = std::cos(lm * kDegreesToRadians);
        const double sinp = std::sin(pm * kDegreesToRadians);
        const double cosp = std::cos(pm * kDegreesToRadians);
        *rmoon = Vector3d(rm * cosp * cosl,
                          rm * (cose * cosp * sinl - sine * sinp),
                          rm * (sine * cosp * sinl + cose * sinp));
    }
}

void sunMoonPositionEcef(double utc_unix,
                         const EarthRotationValues& erpv,
                         Vector3d* rsun,
                         Vector3d* rmoon,
                         double* gmst) {
    Vector3d sun_eci = Vector3d::Zero();
    Vector3d moon_eci = Vector3d::Zero();
    sunMoonPositionEci(utc_unix + erpv.ut1_utc_s,
                       rsun != nullptr ? &sun_eci : nullptr,
                       rmoon != nullptr ? &moon_eci : nullptr);
    double gmst_value = 0.0;
    const Eigen::Matrix3d u = eciToEcef(utc_unix, erpv, gmst_value);
    if (rsun != nullptr) {
        *rsun = u * sun_eci;
    }
    if (rmoon != nullptr) {
        *rmoon = u * moon_eci;
    }
    if (gmst != nullptr) {
        *gmst = gmst_value;
    }
}

Vector3d enuToEcefSpherical(const Vector3d& enu, double lat, double lon) {
    const double sinp = std::sin(lat);
    const double cosp = std::cos(lat);
    const double sinl = std::sin(lon);
    const double cosl = std::cos(lon);
    return Vector3d(
        -sinl * enu.x() - sinp * cosl * enu.y() + cosp * cosl * enu.z(),
         cosl * enu.x() - sinp * sinl * enu.y() + cosp * sinl * enu.z(),
                            cosp * enu.y() + sinp * enu.z());
}

void sphericalPosition(const Vector3d& rr, double& lat, double& lon) {
    const double r = rr.norm();
    if (r <= 0.0) {
        lat = 0.0;
        lon = 0.0;
        return;
    }
    lat = std::asin(rr.z() / r);
    lon = std::atan2(rr.y(), rr.x());
}

Vector3d tidePlanet(const Vector3d& up,
                    const Vector3d& body_position_ecef,
                    double body_gm,
                    double lat,
                    double lon) {
    constexpr double kH3 = 0.292;
    constexpr double kL3 = 0.015;
    const double r = body_position_ecef.norm();
    if (r <= 0.0 || !std::isfinite(r)) {
        return Vector3d::Zero();
    }

    const Vector3d ep = body_position_ecef / r;
    const double k2 = body_gm / kEarthGM * std::pow(constants::WGS84_A, 4.0) / (r * r * r);
    const double k3 = k2 * constants::WGS84_A / r;
    const double latp = std::asin(ep.z());
    const double lonp = std::atan2(ep.y(), ep.x());
    const double cosp = std::cos(latp);
    const double sinl = std::sin(lat);
    const double cosl = std::cos(lat);

    const double p = (3.0 * sinl * sinl - 1.0) / 2.0;
    const double h2 = 0.6078 - 0.0006 * p;
    const double l2 = 0.0847 + 0.0002 * p;
    const double a = std::clamp(ep.dot(up), -1.0, 1.0);
    double dp = k2 * 3.0 * l2 * a;
    double du = k2 * (h2 * (1.5 * a * a - 0.5) - 3.0 * l2 * a * a);

    dp += k3 * kL3 * (7.5 * a * a - 1.5);
    du += k3 * (kH3 * (2.5 * a * a * a - 1.5 * a) -
                kL3 * (7.5 * a * a - 1.5) * a);

    du += 0.75 * 0.0025 * k2 * std::sin(2.0 * latp) * std::sin(2.0 * lat) *
          std::sin(lon - lonp);
    du += 0.75 * 0.0022 * k2 * cosp * cosp * cosl * cosl *
          std::sin(2.0 * (lon - lonp));

    return dp * ep + du * up;
}

Vector3d solidEarthTide(const Vector3d& receiver_position,
                        double utc_unix,
                        const EarthRotationValues& erpv) {
    if (!receiver_position.allFinite() ||
        receiver_position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);
    const Vector3d up = enuToEcefSpherical(Vector3d(0.0, 0.0, 1.0), lat, lon);
    const Vector3d north = enuToEcefSpherical(Vector3d(0.0, 1.0, 0.0), lat, lon);

    Vector3d sun = Vector3d::Zero();
    Vector3d moon = Vector3d::Zero();
    double gmst = 0.0;
    sunMoonPositionEcef(utc_unix, erpv, &sun, &moon, &gmst);

    Vector3d dr = tidePlanet(up, sun, kSunGM, lat, lon) +
                  tidePlanet(up, moon, kMoonGM, lat, lon);

    const double sin2l = std::sin(2.0 * lat);
    const double k1_radial = -0.012 * sin2l * std::sin(gmst + lon);
    dr += k1_radial * up;

    const double sinl = std::sin(lat);
    const double du = 0.1196 * (1.5 * sinl * sinl - 0.5);
    const double dn = 0.0247 * sin2l;
    dr += du * up + dn * north;
    return dr;
}

Vector3d oceanLoadingEnu(double ut1_unix, const OceanLoadingCoefficients& coefficients) {
    static const double args[11][5] = {
        {1.40519e-4,  2.0, -2.0,  0.0,  0.00},
        {1.45444e-4,  0.0,  0.0,  0.0,  0.00},
        {1.37880e-4,  2.0, -3.0,  1.0,  0.00},
        {1.45842e-4,  2.0,  0.0,  0.0,  0.00},
        {0.72921e-4,  1.0,  0.0,  0.0,  0.25},
        {0.67598e-4,  1.0, -2.0,  0.0, -0.25},
        {0.72523e-4, -1.0,  0.0,  0.0, -0.25},
        {0.64959e-4,  1.0, -3.0,  1.0, -0.25},
        {0.53234e-5,  0.0,  2.0,  0.0,  0.00},
        {0.26392e-5,  0.0,  1.0, -1.0,  0.00},
        {0.03982e-5,  2.0,  0.0,  0.0,  0.00},
    };

    double ep[6] = {};
    unixToEpoch(ut1_unix, ep);
    const double fday = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
    const double day_start = epochToUnix(static_cast<int>(ep[0]), static_cast<int>(ep[1]),
                                         static_cast<int>(ep[2]), 0, 0, 0.0);
    const double ep1975 = epochToUnix(1975, 1, 1, 0, 0, 0.0);
    const double days = (day_start - ep1975) / 86400.0 + 1.0;
    const double t = (27392.500528 + 1.000000035 * days) / 36525.0;
    const double t2 = t * t;
    const double t3 = t2 * t;

    const double a[5] = {
        fday,
        (279.69668 + 36000.768930485 * t + 3.03e-4 * t2) * kDegreesToRadians,
        (270.434358 + 481267.88314137 * t - 0.001133 * t2 + 1.9e-6 * t3) *
            kDegreesToRadians,
        (334.329653 + 4069.0340329577 * t - 0.010325 * t2 - 1.2e-5 * t3) *
            kDegreesToRadians,
        2.0 * kPi,
    };

    double up = 0.0;
    double west = 0.0;
    double south = 0.0;
    for (int i = 0; i < 11; ++i) {
        double angle = 0.0;
        for (int j = 0; j < 5; ++j) {
            angle += a[j] * args[i][j];
        }
        up += coefficients.up_amplitudes_m[i] *
              std::cos(angle - coefficients.up_phases_deg[i] * kDegreesToRadians);
        west += coefficients.west_amplitudes_m[i] *
                std::cos(angle - coefficients.west_phases_deg[i] * kDegreesToRadians);
        south += coefficients.south_amplitudes_m[i] *
                 std::cos(angle - coefficients.south_phases_deg[i] * kDegreesToRadians);
    }
    return Vector3d(-west, -south, up);
}

void meanPole(double ut1_unix, double& xp_bar_mas, double& yp_bar_mas) {
    const double ep2000 = epochToUnix(2000, 1, 1, 0, 0, 0.0);
    const double y = (ut1_unix - ep2000) / 86400.0 / 365.25;
    if (y < 3653.0 / 365.25) {
        const double y2 = y * y;
        const double y3 = y2 * y;
        xp_bar_mas = 55.974 + 1.8243 * y + 0.18413 * y2 + 0.007024 * y3;
        yp_bar_mas = 346.346 + 1.7896 * y - 0.10729 * y2 - 0.000908 * y3;
    } else {
        xp_bar_mas = 23.513 + 7.6141 * y;
        yp_bar_mas = 358.891 - 0.6287 * y;
    }
}

Vector3d poleTide(const Vector3d& receiver_position,
                  double ut1_unix,
                  const EarthRotationValues& erpv) {
    if (!erpv.valid || !receiver_position.allFinite() ||
        receiver_position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);

    double xp_bar = 0.0;
    double yp_bar = 0.0;
    meanPole(ut1_unix, xp_bar, yp_bar);

    const double m1 = erpv.xp_rad / kArcsecToRadians - xp_bar * 1e-3;
    const double m2 = -erpv.yp_rad / kArcsecToRadians + yp_bar * 1e-3;
    const double cosl = std::cos(lon);
    const double sinl = std::sin(lon);
    const Vector3d denu(
        9e-3 * std::sin(lat) * (m1 * sinl - m2 * cosl),
        -9e-3 * std::cos(2.0 * lat) * (m1 * cosl + m2 * sinl),
        -33e-3 * std::sin(2.0 * lat) * (m1 * cosl + m2 * sinl));
    return enuToEcefSpherical(denu, lat, lon);
}

}  // namespace

bool loadEarthRotationParameters(const std::string& filename,
                                 EarthRotationParameters& erp) {
    erp.entries.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(input, line)) {
        std::istringstream iss(line);
        double v[14] = {};
        int count = 0;
        for (; count < 14 && (iss >> v[count]); ++count) {}
        if (count < 5) {
            continue;
        }
        EarthRotationParameters::Entry entry;
        entry.mjd = v[0];
        entry.xp_rad = v[1] * 1e-6 * kArcsecToRadians;
        entry.yp_rad = v[2] * 1e-6 * kArcsecToRadians;
        entry.ut1_utc_s = v[3] * 1e-7;
        entry.lod_s_per_day = v[4] * 1e-7;
        if (count > 12) {
            entry.xpr_rad_per_day = v[12] * 1e-6 * kArcsecToRadians;
        }
        if (count > 13) {
            entry.ypr_rad_per_day = v[13] * 1e-6 * kArcsecToRadians;
        }
        erp.entries.push_back(entry);
    }

    std::sort(erp.entries.begin(), erp.entries.end(),
              [](const auto& a, const auto& b) { return a.mjd < b.mjd; });
    return !erp.entries.empty();
}

bool interpolateEarthRotationValues(const EarthRotationParameters& erp,
                                    const GNSSTime& time,
                                    EarthRotationValues& values) {
    values = interpolateErpAtUnixArgument(erp, gpsUnixSeconds(time));
    return values.valid;
}

bool loadOceanLoadingCoefficients(const std::string& filename,
                                  const std::string& station_name,
                                  OceanLoadingCoefficients& coefficients) {
    const std::string normalized_station = normalizeName(station_name);
    if (normalized_station.empty()) {
        return false;
    }
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.rfind("$$", 0) == 0) {
            continue;
        }
        std::istringstream header(trimmed);
        std::string name;
        header >> name;
        if (normalizeName(name) != normalized_station) {
            continue;
        }
        OceanLoadingCoefficients parsed;
        if (readBlqRecord(input, parsed)) {
            coefficients = parsed;
            return true;
        }
    }
    return false;
}

bool loadClasGridOceanLoading(const std::string& filename,
                              OceanLoadingGrid& grid) {
    grid.networks.clear();
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed.rfind("$$", 0) == 0) {
            continue;
        }
        const size_t dash = trimmed.find('-');
        if (dash == std::string::npos) {
            continue;
        }
        int network_id = 0;
        int grid_number = 0;
        try {
            network_id = std::stoi(trimmed.substr(0, dash));
            grid_number = std::stoi(trimmed.substr(dash + 1));
        } catch (const std::exception&) {
            continue;
        }
        for (int i = 0; i < 3; ++i) {
            if (!std::getline(input, line)) {
                return false;
            }
        }
        OceanLoadingCoefficients coefficients;
        if (!readBlqRecord(input, coefficients)) {
            return false;
        }
        grid.networks[network_id][grid_number] = coefficients;
    }

    return !grid.networks.empty();
}

const OceanLoadingCoefficients* findClasGridOceanLoading(
    const OceanLoadingGrid& grid,
    int network_id,
    int grid_number) {
    const auto network_it = grid.networks.find(network_id);
    if (network_it == grid.networks.end()) {
        return nullptr;
    }
    const auto grid_it = network_it->second.find(grid_number);
    return grid_it == network_it->second.end() ? nullptr : &grid_it->second;
}

double julianDateFromTime(const GNSSTime& time) {
    return 2444244.5 + static_cast<double>(time.week) * 7.0 + time.tow / 86400.0;
}

double greenwichMeanSiderealTime(const GNSSTime& time) {
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    return wrapRadians(utcToGmst(utc_unix, 0.0));
}

Vector3d rotateEciToEcef(const Vector3d& eci, double gmst_rad) {
    return rz(gmst_rad) * eci;
}

Vector3d approximateSunPositionEcef(const GNSSTime& time) {
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    EarthRotationValues erpv;
    Vector3d sun = Vector3d::Zero();
    sunMoonPositionEcef(utc_unix, erpv, &sun, nullptr, nullptr);
    return sun;
}

Vector3d approximateMoonPositionEcef(const GNSSTime& time) {
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    EarthRotationValues erpv;
    Vector3d moon = Vector3d::Zero();
    sunMoonPositionEcef(utc_unix, erpv, nullptr, &moon, nullptr);
    return moon;
}

Vector3d surfaceUpVector(const Vector3d& receiver_position) {
    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);
    return enuToEcefSpherical(Vector3d(0.0, 0.0, 1.0), lat, lon).normalized();
}

Vector3d bodyTideDisplacement(const Vector3d& receiver_position,
                              const Vector3d& body_position_ecef,
                              double body_gm_m3_s2) {
    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);
    return tidePlanet(surfaceUpVector(receiver_position), body_position_ecef,
                      body_gm_m3_s2, lat, lon);
}

Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time) {
    return calculateSolidEarthTides(receiver_position, time, nullptr);
}

Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time,
                                  const EarthRotationParameters* erp) {
    const double gpst_unix = gpsUnixSeconds(time);
    const double utc_unix = gpsToUtcUnix(gpst_unix);
    EarthRotationValues erpv;
    if (erp != nullptr) {
        erpv = interpolateErpAtUnixArgument(*erp, utc_unix);
    }
    return solidEarthTide(receiver_position, utc_unix, erpv);
}

Vector3d calculateOceanLoading(const OceanLoadingCoefficients& coefficients,
                               const Vector3d& receiver_position,
                               const GNSSTime& time) {
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);
    return enuToEcefSpherical(oceanLoadingEnu(utc_unix, coefficients), lat, lon);
}

Vector3d calculateClasGridOceanLoading(const OceanLoadingGrid& grid,
                                       const ClasGridInterpolation& interpolation,
                                       const Vector3d& receiver_position,
                                       const GNSSTime& time,
                                       const EarthRotationValues& erp_values) {
    if (interpolation.network_id <= 0 || interpolation.grid_count <= 0) {
        return Vector3d::Zero();
    }
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    const double ut1_unix = utc_unix + erp_values.ut1_utc_s;

    std::array<Vector3d, 4> denu_grid{};
    const int count = std::min(interpolation.grid_count, 4);
    for (int i = 0; i < count; ++i) {
        const auto* coefficients = findClasGridOceanLoading(
            grid, interpolation.network_id, interpolation.grid_numbers[i]);
        if (coefficients == nullptr) {
            return Vector3d::Zero();
        }
        denu_grid[i] = oceanLoadingEnu(ut1_unix, *coefficients);
    }

    Vector3d denu = Vector3d::Zero();
    if (interpolation.use_model_interpolation && count == 4) {
        for (int component = 0; component < 3; ++component) {
            double transformed[4] = {};
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    transformed[row] += interpolation.model_gmat[row * 4 + col] *
                                        denu_grid[col](component);
                }
            }
            for (int row = 0; row < 4; ++row) {
                denu(component) += interpolation.model_emat[row] * transformed[row];
            }
        }
    } else {
        double weight_sum = 0.0;
        for (int i = 0; i < count; ++i) {
            denu += interpolation.weights[i] * denu_grid[i];
            weight_sum += interpolation.weights[i];
        }
        if (std::abs(weight_sum) < 1e-12 && count == 1) {
            denu = denu_grid[0];
        }
    }

    double lat = 0.0;
    double lon = 0.0;
    sphericalPosition(receiver_position, lat, lon);
    return enuToEcefSpherical(denu, lat, lon);
}

Vector3d calculatePoleTide(const Vector3d& receiver_position,
                           const GNSSTime& time,
                           const EarthRotationParameters* erp) {
    if (erp == nullptr) {
        return Vector3d::Zero();
    }
    const double utc_unix = gpsToUtcUnix(gpsUnixSeconds(time));
    const EarthRotationValues erpv = interpolateErpAtUnixArgument(*erp, utc_unix);
    return poleTide(receiver_position, utc_unix + erpv.ut1_utc_s, erpv);
}

TideDisplacementComponents calculateTideDisplacement(
    const Vector3d& receiver_position,
    const GNSSTime& time,
    const EarthRotationParameters* erp,
    const OceanLoadingGrid* ocean_grid,
    const ClasGridInterpolation* ocean_interpolation,
    bool apply_solid,
    bool apply_ocean,
    bool apply_pole) {
    TideDisplacementComponents components;
    const double gpst_unix = gpsUnixSeconds(time);
    const double utc_unix = gpsToUtcUnix(gpst_unix);
    EarthRotationValues erpv;
    if (erp != nullptr) {
        erpv = interpolateErpAtUnixArgument(*erp, utc_unix);
    }

    if (apply_solid) {
        components.solid_ecef = solidEarthTide(receiver_position, utc_unix, erpv);
    }
    if (apply_pole && erp != nullptr) {
        components.pole_ecef = poleTide(receiver_position, utc_unix + erpv.ut1_utc_s, erpv);
    }
    if (apply_ocean && ocean_grid != nullptr && ocean_interpolation != nullptr) {
        components.ocean_ecef =
            calculateClasGridOceanLoading(*ocean_grid, *ocean_interpolation,
                                          receiver_position, time, erpv);
    }
    return components;
}

}  // namespace tidal
}  // namespace libgnss
