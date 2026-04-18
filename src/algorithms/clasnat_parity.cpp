#include <libgnss++/algorithms/clasnat_parity.hpp>
#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>

namespace libgnss::clasnat_parity {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kClasPi = 3.1415926535897932;
constexpr double kClasRadiansToDegrees = 180.0 / kClasPi;
constexpr double kIonosphereHeightM = 350000.0;

double dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm3(const double a[3]) {
    return std::sqrt(dot3(a, a));
}

bool unit3(const double a[3], double out[3]) {
    const double n = norm3(a);
    if (n <= 0.0) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        out[i] = a[i] / n;
    }
    return true;
}

void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

double interpPcvVariation(const std::array<double, kParityPcvGridSize>& var,
                          double zenith_deg) {
    const double a = zenith_deg / 5.0;
    const int i = static_cast<int>(a);
    if (i < 0) {
        return var[0];
    }
    if (i >= kParityPcvGridSize - 1) {
        return var[kParityPcvGridSize - 1];
    }
    return var[static_cast<size_t>(i)] * (1.0 - a + i) +
           var[static_cast<size_t>(i + 1)] * (a - i);
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

double embeddedGeoidHeightJapan(double latitude_rad, double longitude_rad) {
    constexpr int lon_min_deg = 120;
    constexpr int lon_max_deg = 150;
    constexpr int lat_min_deg = 20;
    constexpr int lat_max_deg = 50;
    constexpr double geoid[lon_max_deg - lon_min_deg + 1][lat_max_deg - lat_min_deg + 1] = {
#include "clas_embedded_geoid_japan.inc"
    };

    const double latitude_deg = latitude_rad * kRadiansToDegrees;
    const double longitude_deg = longitude_rad * kRadiansToDegrees;
    if (longitude_deg < lon_min_deg || longitude_deg > lon_max_deg ||
        latitude_deg < lat_min_deg || latitude_deg > lat_max_deg) {
        return 0.0;
    }

    const double lon_offset = longitude_deg - lon_min_deg;
    const double lat_offset = latitude_deg - lat_min_deg;
    const int lon0 = std::clamp(static_cast<int>(std::floor(lon_offset)),
                                0,
                                lon_max_deg - lon_min_deg);
    const int lat0 = std::clamp(static_cast<int>(std::floor(lat_offset)),
                                0,
                                lat_max_deg - lat_min_deg);
    const int lon1 = std::min(lon0 + 1, lon_max_deg - lon_min_deg);
    const int lat1 = std::min(lat0 + 1, lat_max_deg - lat_min_deg);
    const double a = lon_offset - lon0;
    const double b = lat_offset - lat0;

    return geoid[lon0][lat0] * (1.0 - a) * (1.0 - b) +
           geoid[lon1][lat0] * a * (1.0 - b) +
           geoid[lon0][lat1] * (1.0 - a) * b +
           geoid[lon1][lat1] * a * b;
}

double interpMetCoefficient(const std::array<double, 5>& values, double latitude_deg) {
    const int i = static_cast<int>(latitude_deg / 15.0);
    if (i < 1) {
        return values[0];
    }
    if (i > 4) {
        return values[4];
    }
    return values[static_cast<size_t>(i - 1)] * (1.0 - latitude_deg / 15.0 + i) +
           values[static_cast<size_t>(i)] * (latitude_deg / 15.0 - i);
}

bool standardVerticalTroposphereDelays(const GNSSTime& time,
                                       const double pos[3],
                                       double& hydrostatic_m,
                                       double& wet_m) {
    constexpr std::array<double, 5> mean_pressure = {
        1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    constexpr std::array<double, 5> mean_temperature = {
        299.65, 294.15, 283.15, 272.15, 263.65};
    constexpr std::array<double, 5> mean_water_pressure = {
        26.31, 21.79, 11.66, 6.78, 4.11};
    constexpr std::array<double, 5> mean_lapse_rate = {
        6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    constexpr std::array<double, 5> mean_water_vapor_rate = {
        2.77, 3.15, 2.57, 1.81, 1.55};
    constexpr std::array<double, 5> amp_pressure = {
        0.00, -3.75, -2.25, -1.75, -0.50};
    constexpr std::array<double, 5> amp_temperature = {
        0.00, 7.00, 11.00, 15.00, 14.50};
    constexpr std::array<double, 5> amp_water_pressure = {
        0.00, 8.85, 7.24, 5.36, 3.39};
    constexpr std::array<double, 5> amp_lapse_rate = {
        0.00e-3, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    constexpr std::array<double, 5> amp_water_vapor_rate = {
        0.00, 0.33, 0.46, 0.74, 0.30};
    constexpr double gravity = 9.80665;
    constexpr double gas_constant_dry = 287.0537625;

    const double latitude_deg = pos[0] * kRadiansToDegrees;
    if (latitude_deg < 0.0 || latitude_deg > 75.0) {
        return false;
    }

    const double doy = static_cast<double>(dayOfYearFromTime(time));
    const double seasonal = std::cos(2.0 * M_PI * (doy - 28.0) / 365.25);
    auto seasonalValues = [&](const std::array<double, 5>& mean_values,
                              const std::array<double, 5>& amp_values) {
        std::array<double, 5> values = {};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = mean_values[i] - amp_values[i] * seasonal;
        }
        return values;
    };

    const double pressure0 =
        interpMetCoefficient(seasonalValues(mean_pressure, amp_pressure), latitude_deg);
    const double temperature0 =
        interpMetCoefficient(seasonalValues(mean_temperature, amp_temperature), latitude_deg);
    const double water_pressure0 =
        interpMetCoefficient(seasonalValues(mean_water_pressure, amp_water_pressure), latitude_deg);
    const double lapse_rate =
        interpMetCoefficient(seasonalValues(mean_lapse_rate, amp_lapse_rate), latitude_deg);
    const double water_vapor_rate =
        interpMetCoefficient(seasonalValues(mean_water_vapor_rate, amp_water_vapor_rate),
                             latitude_deg);

    const double geoid_height_m = embeddedGeoidHeightJapan(pos[0], pos[1]);
    const double orthometric_height_m = pos[2] - geoid_height_m;
    const double height_scale = 1.0 - lapse_rate * orthometric_height_m / temperature0;
    const double temperature = temperature0 - lapse_rate * orthometric_height_m;
    if (temperature <= 0.0 || height_scale <= 0.0 || lapse_rate <= 0.0) {
        return false;
    }

    const double pressure =
        pressure0 * std::pow(height_scale, gravity / (gas_constant_dry * lapse_rate));
    double water_pressure =
        water_pressure0 *
        std::pow(height_scale,
                 (water_vapor_rate + 1.0) * gravity / (gas_constant_dry * lapse_rate));

    const double celsius = temperature - 273.15;
    const double saturation =
        6.11 * std::pow(temperature / 273.15, -5.3) *
        std::exp(25.2 * celsius / temperature);
    if (water_pressure > saturation) {
        water_pressure = saturation;
    }

    hydrostatic_m =
        2.2768 * pressure * 0.001 /
        (1.0 - 0.00266 * std::cos(2.0 * pos[0]) - 2.8e-7 * pos[2]);
    wet_m = 2.2768 * (1255.0 / temperature + 0.05) * water_pressure * 0.001;
    return std::isfinite(hydrostatic_m) && std::isfinite(wet_m) &&
           hydrostatic_m > 0.0 && wet_m > 0.0;
}

double claslibTime2Doy(const GNSSTime& time) {
    constexpr double kGpsEpochUnixSeconds = 315964800.0;
    const double unix_seconds =
        kGpsEpochUnixSeconds +
        static_cast<double>(time.week) * constants::SECONDS_PER_WEEK +
        time.tow;
    const double whole_seconds = std::floor(unix_seconds);
    const std::time_t utc_seconds = static_cast<std::time_t>(whole_seconds);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    const double frac_seconds = unix_seconds - whole_seconds;
    const double seconds_of_day =
        static_cast<double>(utc_tm.tm_hour * 3600 + utc_tm.tm_min * 60 + utc_tm.tm_sec) +
        frac_seconds;
    return static_cast<double>(utc_tm.tm_yday) + seconds_of_day / 86400.0 + 1.0;
}

double claslibInterp1(const double* x, const double* y, int nx, double xi) {
    int i;
    for (i = 0; i < nx - 1; ++i) {
        if (xi < x[i + 1]) {
            break;
        }
    }
    const double dx = (xi - x[i]) / (x[i + 1] - x[i]);
    return y[i] * (1.0 - dx) + y[i + 1] * dx;
}

double claslibInterpc(const double coef[], double lat) {
    const int i = static_cast<int>(lat / 15.0);
    if (i < 1) {
        return coef[0];
    }
    if (i > 4) {
        return coef[4];
    }
    return coef[i - 1] * (1.0 - lat / 15.0 + i) +
           coef[i] * (lat / 15.0 - i);
}

double claslibMapf(double el, double a, double b, double c) {
    const double sinel = std::sin(el);
    return (1.0 + a / (1.0 + b / (1.0 + c))) /
           (sinel + (a / (sinel + b / (sinel + c))));
}

int claslibGetMops(const double lat,
                   const double doy,
                   double* pre,
                   double* temp,
                   double* wpre,
                   double* ltemp,
                   double* wvap) {
    const double latdeg = lat * kClasRadiansToDegrees;
    const double interval[] = {15.0, 30.0, 45.0, 60.0, 75.0};
    const double pTave[] = {1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    const double tTave[] = {299.65, 294.15, 283.15, 272.15, 263.65};
    const double wTave[] = {26.31, 21.79, 11.66, 6.78, 4.11};
    const double tlTave[] = {6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    const double wlTave[] = {2.77, 3.15, 2.57, 1.81, 1.55};
    const double pTableS[] = {0.00, -3.75, -2.25, -1.75, -0.50};
    const double tTableS[] = {0.00, 7.00, 11.00, 15.00, 14.50};
    const double wTableS[] = {0.00, 8.85, 7.24, 5.36, 3.39};
    const double tlTableS[] = {0.00e-3, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    const double wlTableS[] = {0.00, 0.33, 0.46, 0.74, 0.30};
    double calcPsta[5] = {};
    double calcTsta[5] = {};
    double calcWsta[5] = {};
    double calcTLsta[5] = {};
    double calcWLsta[5] = {};

    for (int i = 0; i < 5; ++i) {
        const double cos_ = std::cos(2.0 * kClasPi * (doy - 28.0) / 365.25);
        calcPsta[i] = pTave[i] - pTableS[i] * cos_;
        calcTsta[i] = tTave[i] - tTableS[i] * cos_;
        calcWsta[i] = wTave[i] - wTableS[i] * cos_;
        calcTLsta[i] = tlTave[i] - tlTableS[i] * cos_;
        calcWLsta[i] = wlTave[i] - wlTableS[i] * cos_;
    }

    if (75.0 <= latdeg) {
        *pre = calcPsta[4];
        *temp = calcTsta[4];
        *wpre = calcWsta[4];
        *ltemp = calcTLsta[4];
        *wvap = calcWLsta[4];
    } else if (latdeg <= 15.0) {
        *pre = calcPsta[0];
        *temp = calcTsta[0];
        *wpre = calcWsta[0];
        *ltemp = calcTLsta[0];
        *wvap = calcWLsta[0];
    } else if (15.0 < latdeg && latdeg < 75.0) {
        *pre = claslibInterp1(interval, calcPsta, 5, latdeg);
        *temp = claslibInterp1(interval, calcTsta, 5, latdeg);
        *wpre = claslibInterp1(interval, calcWsta, 5, latdeg);
        *ltemp = claslibInterp1(interval, calcTLsta, 5, latdeg);
        *wvap = claslibInterp1(interval, calcWLsta, 5, latdeg);
    } else {
        return 1;
    }
    return 0;
}

double claslibGetTdv(const double lat, const double Hs, const double Ps) {
    return 2.2768 / (1.0 - 0.00266 * std::cos(2.0 * lat) - (2.8e-7) * Hs) *
           Ps * 0.001;
}

double claslibGetTwv(const double Ts, const double es) {
    return 2.2768 * (1255.0 / Ts + 0.05) * es * 0.001;
}

int claslibGetStTv(const GNSSTime& time,
                   const double lat,
                   const double hs,
                   const double hg,
                   double* tdv,
                   double* twv) {
    double P0, T0, e0, beta0, lambda0, Hs, Ts, Ps, es, t0, tk, tc, ET, doy;
    constexpr double g = 9.80665;
    constexpr double Rd = 287.0537625;

    doy = claslibTime2Doy(time);
    if (claslibGetMops(lat, doy, &P0, &T0, &e0, &beta0, &lambda0)) {
        return 1;
    }

    Hs = hs - hg;
    Ts = T0 - beta0 * Hs;
    Ps = P0 * std::pow(1 - beta0 * Hs / T0, g / (Rd * beta0));
    es = e0 * std::pow(1 - beta0 * Hs / T0, (lambda0 + 1.0) * g / (Rd * beta0));

    t0 = 273.15;
    tk = Ts;
    tc = tk - t0;
    ET = 6.11 * std::pow(tk / t0, -5.3) * std::exp(25.2 * tc / tk);
    if (es > ET) {
        es = ET;
    }

    *tdv = claslibGetTdv(lat, Hs + hg, Ps);
    *twv = claslibGetTwv(Ts, es);
    return 0;
}

double claslibNmf(const GNSSTime& time,
                  const double pos[],
                  const double azel[],
                  double* mapfw) {
    const double coef[][5] = {
        {1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
        {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
        {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2},
    };
    const double aht[] = {2.53E-5, 5.49E-3, 1.14E-3};

    double ah[3], aw[3];
    double el = azel[1];
    double lat = pos[0] * kClasRadiansToDegrees;
    const double hgt = pos[2];

    if (el <= 0.0) {
        if (mapfw) {
            *mapfw = 0.0;
        }
        return 0.0;
    }

    const double y = (claslibTime2Doy(time) - 28.0) / 365.25 +
                     (lat < 0.0 ? 0.5 : 0.0);
    const double cosy = std::cos(2.0 * kClasPi * y);
    lat = std::fabs(lat);

    for (int i = 0; i < 3; ++i) {
        ah[i] = claslibInterpc(coef[i], lat) - claslibInterpc(coef[i + 3], lat) * cosy;
        aw[i] = claslibInterpc(coef[i + 6], lat);
    }
    const double dm =
        (1.0 / std::sin(el) - claslibMapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

    if (mapfw) {
        *mapfw = claslibMapf(el, aw[0], aw[1], aw[2]);
    }
    return claslibMapf(el, ah[0], ah[1], ah[2]) + dm;
}

double claslibTropmapf(const GNSSTime& time,
                       const double pos[],
                       const double azel[],
                       double* mapfw) {
    if (pos[2] < -1000.0 || pos[2] > 20000.0) {
        if (mapfw) {
            *mapfw = 0.0;
        }
        return 0.0;
    }
    return claslibNmf(time, pos, azel, mapfw);
}

tidal::EarthRotationParameters oneEntryErp(const GNSSTime& time, const double erpv[5]) {
    tidal::EarthRotationParameters erp;
    tidal::EarthRotationParameters::Entry entry;
    entry.mjd = tidal::julianDateFromTime(time) - 2400000.5;
    entry.xp_rad = erpv[0];
    entry.yp_rad = erpv[1];
    entry.ut1_utc_s = erpv[2];
    entry.lod_s_per_day = erpv[3];
    entry.xpr_rad_per_day = erpv[4];
    entry.ypr_rad_per_day = 0.0;
    erp.entries.push_back(entry);
    return erp;
}

}  // namespace

bool tidedispAvailable() {
    return true;
}

bool windupcorrAvailable() {
    return true;
}

bool antmodelAvailable() {
    return true;
}

bool ionmapfAvailable() {
    return true;
}

bool prectropAvailable() {
    return true;
}

bool satposSsrAvailable() {
    return false;
}

bool corrmeasAvailable() {
    return false;
}

void tidedisp(const GNSSTime& gpst,
              const double rr[3],
              const double erpv[5],
              double disp_out[3]) {
    const Vector3d receiver_position(rr[0], rr[1], rr[2]);
    const auto erp = oneEntryErp(gpst, erpv);
    const auto components = tidal::calculateTideDisplacement(
        receiver_position, gpst, &erp, nullptr, nullptr, true, false, true);
    const Vector3d disp = components.total();
    for (int i = 0; i < 3; ++i) {
        disp_out[i] = disp(i);
    }
}

void windupcorr(const GNSSTime& /*time*/,
                const double rs[6],
                const double rr[3],
                double& phw_io) {
    double ek_vec[3] = {};
    for (int i = 0; i < 3; ++i) {
        ek_vec[i] = rr[i] - rs[i];
    }
    double ek[3] = {};
    if (!unit3(ek_vec, ek)) {
        return;
    }

    double ezs_vec[3] = {-rs[0], -rs[1], -rs[2]};
    double ezs[3] = {};
    if (!unit3(ezs_vec, ezs)) {
        return;
    }

    const double omega[3] = {0.0, 0.0, constants::OMEGA_E};
    double omega_cross_r[3] = {};
    cross3(omega, rs, omega_cross_r);
    double ess_vec[3] = {
        rs[3] + omega_cross_r[0],
        rs[4] + omega_cross_r[1],
        rs[5] + omega_cross_r[2],
    };
    double ess[3] = {};
    if (!unit3(ess_vec, ess)) {
        return;
    }

    double tmp[3] = {};
    double eys[3] = {};
    cross3(ezs, ess, tmp);
    if (!unit3(tmp, eys)) {
        return;
    }
    double exs[3] = {};
    cross3(eys, ezs, exs);

    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(Vector3d(rr[0], rr[1], rr[2]), lat, lon, h);
    (void)h;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);
    const double exr[3] = {-sin_lon, cos_lon, 0.0};
    const double eyr[3] = {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat};

    double eks[3] = {};
    double ekr[3] = {};
    cross3(ek, eys, eks);
    cross3(ek, eyr, ekr);
    double ds[3] = {};
    double dr[3] = {};
    for (int i = 0; i < 3; ++i) {
        ds[i] = exs[i] - ek[i] * dot3(ek, exs) - eks[i];
        dr[i] = exr[i] - ek[i] * dot3(ek, exr) + ekr[i];
    }
    const double denom = norm3(ds) * norm3(dr);
    if (denom <= 0.0) {
        return;
    }
    double cosp = dot3(ds, dr) / denom;
    cosp = std::clamp(cosp, -1.0, 1.0);
    double ph = std::acos(cosp) / (2.0 * M_PI);
    double drs[3] = {};
    cross3(ds, dr, drs);
    if (dot3(ek, drs) < 0.0) {
        ph = -ph;
    }
    phw_io = ph + std::floor(phw_io - ph + 0.5);
}

void antmodel(const ReceiverPcvModel& pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[kParityMaxFreq]) {
    const double cos_el = std::cos(azel[1]);
    const double e[3] = {
        std::sin(azel[0]) * cos_el,
        std::cos(azel[0]) * cos_el,
        std::sin(azel[1]),
    };
    for (int f = 0; f < kParityMaxFreq; ++f) {
        double off[3] = {};
        for (int j = 0; j < 3; ++j) {
            off[j] = pcv.offsets_m[static_cast<size_t>(f)][static_cast<size_t>(j)] + del[j];
        }
        const double pcv_m =
            opt ? interpPcvVariation(
                      pcv.variations_m[static_cast<size_t>(f)],
                      90.0 - azel[1] * kRadiansToDegrees)
                : 0.0;
        dant[f] = -dot3(off, e) + pcv_m;
    }
}

double ionmapf(const double pos[3], const double azel[2]) {
    if (pos[2] >= kIonosphereHeightM) {
        return 1.0;
    }
    const double ratio =
        (constants::WGS84_A + pos[2]) / (constants::WGS84_A + kIonosphereHeightM);
    return 1.0 /
           std::cos(std::asin(ratio * std::sin(M_PI / 2.0 - azel[1])));
}

double prectrop(const GNSSTime& time,
                const double pos[3],
                const double azel[2],
                double zwd,
                double ztd) {
    double dry_delay_m = 0.0;
    double wet_delay_m = 0.0;
    const double geoid_height_m = embeddedGeoidHeightJapan(pos[0], pos[1]);
    if (claslibGetStTv(time, pos[0], pos[2], geoid_height_m, &dry_delay_m, &wet_delay_m)) {
        return 0.0;
    }
    double wet_mapping = 0.0;
    const double dry_mapping = claslibTropmapf(time, pos, azel, &wet_mapping);
    return dry_mapping * dry_delay_m * ztd + wet_mapping * wet_delay_m * zwd;
}

bool satpos_ssr(const GNSSTime& /*teph*/,
                const GNSSTime& /*time*/,
                int /*sat*/,
                SatposSsrOutput& /*out*/) {
    return false;
}

bool corrmeas(CorrmeasOutput& /*out*/) {
    return false;
}

}  // namespace libgnss::clasnat_parity
