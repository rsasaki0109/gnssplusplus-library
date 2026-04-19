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
constexpr double kClasFreq1Hz = 1.57542E9;
constexpr double kClasFreq2Hz = 1.22760E9;
constexpr double kClasFreq5Hz = 1.17645E9;
constexpr int kParityMaxSat = 75;
constexpr int kParitySsrChannelCount = 2;
constexpr int kParityMaxGrid = 4;
constexpr int kParityMaxCode = 64;
constexpr int kParityMaxIndexSsr = 9;
constexpr int kParityCssrInvalid = -10000;
constexpr int kParityRtcmModeCssr = 5;
constexpr double kParityMaxAgeSsrBiasSeconds = 120.0;
constexpr double kParityMaxAgeSsrIonoSeconds = 120.0;
constexpr double kParityMaxAgeSsrTropSeconds = 120.0;
constexpr int kClasSysNone = 0x00;
constexpr int kClasSysGps = 0x01;
constexpr int kClasSysGal = 0x08;
constexpr int kClasSysQzs = 0x10;
constexpr int kClasSysCmp = 0x20;
constexpr int kClasMinPrnQzs = 193;
constexpr int kClasMaxPrnQzs = 199;
constexpr double kClasMuGps = 3.9860050E14;
constexpr double kClasMuGal = 3.986004418E14;
constexpr double kClasMuCmp = 3.986004418E14;
constexpr double kClasOmge = 7.2921151467E-5;
constexpr double kClasOmgeGal = 7.2921151467E-5;
constexpr double kClasOmgeCmp = 7.292115E-5;
constexpr double kClasSin5 = -0.0871557427476582;
constexpr double kClasCos5 = 0.9961946980917456;
constexpr double kClasRtolKepler = 1E-14;
constexpr int kClasMaxIterKepler = 30;
constexpr double kClasMaxDtoeGps = 7200.0;
constexpr double kClasMaxDtoeQzs = 7200.0;
constexpr double kClasMaxDtoeGal = 10800.0;
constexpr double kClasMaxDtoeCmp = 21600.0;
constexpr double kClasDefaultUraSsr = 0.15;
constexpr double kClasMaxSsrOrbitCorrectionM = 50.0;
constexpr double kClasMaxSsrClockCorrectionM = 30.0;
constexpr double kClasMaxAgeSsrOrbitSeconds = 240.0;
constexpr double kClasMaxAgeSsrClockSeconds = 120.0;
constexpr double kClasMaxAgeSsrHighRateClockSeconds = 0.0;

static unsigned char kObsFreqs[] = {
    0, 1, 1, 1, 1,  1, 1, 1, 1, 1,
    1, 1, 1, 1, 2,  2, 2, 2, 2, 2,
    2, 2, 2, 2, 3,  3, 3, 5, 5, 5,
    4, 4, 4, 4, 4,  4, 4, 6, 6, 6,
    2, 2, 4, 4, 3,  3, 3, 1, 1, 0,
};

struct ParityObs {
    GNSSTime time;
    unsigned char sat = 0;
    unsigned char code[kParityMaxFreq] = {};
    double L[kParityMaxFreq] = {};
    double P[kParityMaxFreq] = {};
};

struct ParityPrcOpt {
    int nf = kParityMaxFreq;
    int posopt[13] = {};
    ReceiverPcvModel pcvr[2] = {};
    double antdel[2][3] = {};
};

struct ParitySsat {
    unsigned char slip[kParityMaxFreq] = {};
    double phw = 0.0;
};

struct ParityStecData {
    GNSSTime time;
    unsigned char sat = 0;
    unsigned char slip = 0;
    float iono = 0.0F;
    float rate = 0.0F;
    float quality = 0.0F;
    float rms = 0.0F;
    int flag = 0;
};

struct ParityStec {
    int n = 0;
    ParityStecData data[kParityMaxSat] = {};
};

struct ParitySsr {
    GNSSTime t0[kParityMaxIndexSsr] = {};
    int nsig = 0;
    int smode[kParityMaxCode] = {};
    float cbias[kParityMaxCode] = {};
    float pbias[kParityMaxCode] = {};
};

struct ParityNav {
    double lam[kParityMaxSat][kParityMaxFreq] = {};
    ParitySsr ssr_ch[kParitySsrChannelCount][kParityMaxSat] = {};
    int rtcmmode = kParityRtcmModeCssr;
    ParityStec stec[kParityMaxGrid] = {};
};

struct ParityOsr {
    double trop = 0.0;
    double iono = 0.0;
    double relatv = 0.0;
    double cbias[kParityMaxFreq] = {};
    double pbias[kParityMaxFreq] = {};
    double antr[kParityMaxFreq] = {};
    double wupL[kParityMaxFreq] = {};
    double compL[kParityMaxFreq] = {};
};

struct CorrmeasRuntime {
    ParityObs obs;
    ParityNav nav;
    ParityPrcOpt opt;
    ParitySsat ssat;
    ParityOsr osr;
    int index[kParityMaxGrid] = {};
    double weight[kParityMaxGrid] = {};
    double Gmat[kParityMaxGrid * kParityMaxGrid] = {};
    double Emat[kParityMaxGrid] = {};
    int brk = 0;
    int pbreset[kParityMaxFreq] = {};
};

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

double timediff(const GNSSTime& a, const GNSSTime& b) {
    return a - b;
}

GNSSTime clasTimeadd(const GNSSTime& time, double sec) {
    int week = time.week;
    int whole = static_cast<int>(time.tow);
    double frac = time.tow - static_cast<double>(whole);
    frac += sec;
    const double carry = std::floor(frac);
    whole += static_cast<int>(carry);
    frac -= carry;
    while (whole < 0) {
        whole += static_cast<int>(constants::SECONDS_PER_WEEK);
        --week;
    }
    while (whole >= static_cast<int>(constants::SECONDS_PER_WEEK)) {
        whole -= static_cast<int>(constants::SECONDS_PER_WEEK);
        ++week;
    }
    return GNSSTime(week, static_cast<double>(whole) + frac);
}

double clasTimediff(const GNSSTime& a, const GNSSTime& b) {
    const int a_whole = static_cast<int>(a.tow);
    const int b_whole = static_cast<int>(b.tow);
    const double a_frac = a.tow - static_cast<double>(a_whole);
    const double b_frac = b.tow - static_cast<double>(b_whole);
    return static_cast<double>((a.week - b.week) *
                               static_cast<int>(constants::SECONDS_PER_WEEK) +
                               (a_whole - b_whole)) +
           a_frac - b_frac;
}

bool gnsstimeIsSet(const GNSSTime& time) {
    return time.week != 0 || std::fabs(time.tow) > 0.0;
}

int paritySatsys(int sat, int* prn) {
    int sys = kClasSysNone;
    int out_prn = 0;
    if (sat <= 0 || sat > kParityMaxSat) {
        out_prn = 0;
    } else if (sat <= 32) {
        sys = kClasSysGps;
        out_prn = sat;
    } else if ((sat -= 32) <= 36) {
        sys = kClasSysGal;
        out_prn = sat;
    } else if ((sat -= 36) <= 7) {
        sys = kClasSysQzs;
        out_prn = kClasMinPrnQzs + sat - 1;
    } else {
        out_prn = 0;
    }
    if (prn != nullptr) {
        *prn = out_prn;
    }
    return sys;
}

double parityMaxDtoe(int sys) {
    switch (sys) {
        case kClasSysQzs:
            return kClasMaxDtoeQzs + 1.0;
        case kClasSysGal:
            return kClasMaxDtoeGal + 1.0;
        case kClasSysCmp:
            return kClasMaxDtoeCmp + 1.0;
        default:
            return kClasMaxDtoeGps + 1.0;
    }
}

double parityVarUraeph(int ura, int sys) {
    static constexpr double ura_value[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
        96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};

    if (sys == kClasSysGal) {
        double temp = 0.0;
        if (0 <= ura && ura <= 49) {
            temp = static_cast<double>(ura);
        } else if (50 <= ura && ura <= 74) {
            temp = static_cast<double>(ura - 50) * 2.0 + 50.0;
        } else if (75 <= ura && ura <= 99) {
            temp = static_cast<double>(ura - 75) * 4.0 + 100.0;
        } else if (100 <= ura && ura <= 125) {
            temp = static_cast<double>(ura - 100) * 16.0 + 200.0;
        } else {
            temp = 6144.0 * 100.0;
        }
        return (temp / 100.0) * (temp / 100.0);
    }
    const double value =
        (ura < 0 || 14 < ura) ? 6144.0 : ura_value[static_cast<size_t>(ura)];
    return value * value;
}

double parityVarUrassr(int ura) {
    if (ura <= 0) {
        return kClasDefaultUraSsr * kClasDefaultUraSsr;
    }
    if (ura >= 63) {
        return 5.4665 * 5.4665;
    }
    const double std_m =
        (std::pow(3.0, (ura >> 3) & 7) * (1.0 + (ura & 7) / 4.0) - 1.0) *
        1E-3;
    return std_m * std_m;
}

int obsFrequencyIndex(int code) {
    if (code < 0 || code >= static_cast<int>(sizeof(kObsFreqs))) {
        return 0;
    }
    const int freq = kObsFreqs[code];
    return freq > 0 ? freq - 1 : 0;
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

int parityStecData(ParityStec* stec,
                   const GNSSTime& time,
                   int sat,
                   double* iono,
                   double* rate,
                   double* rms,
                   double* quality,
                   int* slip) {
    int k, flag;
    double tt;

    if (stec->n <= 0) {
        return 0;
    }

    for (k = flag = 0; k < stec->n; k++) {
        if (sat == stec->data[k].sat) {
            flag = 1;
            break;
        }
    }
    if (flag == 0 || stec->data[k].flag == -1) {
        return 0;
    }

    tt = timediff(time, stec->data[k].time);
    if (std::fabs(tt) > kParityMaxAgeSsrIonoSeconds) {
        stec->data[k].flag = -1;
        return 0;
    }
    *iono = stec->data[k].iono + stec->data[k].rate * tt;
    *rate = stec->data[k].rate;
    *rms = stec->data[k].rms;
    *slip = stec->data[k].slip;
    *quality = stec->data[k].quality;

    return 1;
}

int parityStecGridData(ParityNav* nav,
                       const int* index,
                       const GNSSTime& time,
                       int sat,
                       int n,
                       const double* weight,
                       const double* Gmat,
                       const double* Emat,
                       double* iono,
                       double* rate,
                       double* var,
                       int* brk) {
    int i, slip;
    double ionos[kParityMaxGrid] = {};
    double rates[kParityMaxGrid] = {};
    double rms[kParityMaxGrid] = {};
    double quals[kParityMaxGrid] = {};
    double ionos_[kParityMaxGrid] = {};
    double rates_[kParityMaxGrid] = {};
    double sqrms[kParityMaxGrid] = {};
    double sqrms_[kParityMaxGrid] = {};
    double quals_[kParityMaxGrid] = {};

    if (n <= 0 || n > kParityMaxGrid) {
        return 0;
    }

    if (n == 1) {
        if (!parityStecData(nav->stec + index[0],
                            time,
                            sat,
                            iono,
                            rate,
                            rms,
                            quals,
                            &slip)) {
            return 0;
        }
        if (slip) {
            *brk = 1;
        }
        *var = rms[0] * rms[0];
        return 1;
    } else {
        for (i = 0; i < n; i++) {
            if (!parityStecData(nav->stec + index[i],
                                time,
                                sat,
                                ionos + i,
                                rates + i,
                                rms + i,
                                quals + i,
                                &slip)) {
                return 0;
            }
            if (slip) {
                *brk = 1;
            }
        }

        *iono = *rate = *var = 0.0;

        if (n == 4 && Gmat && Emat) {
            for (i = 0; i < n; i++) {
                sqrms[i] = rms[i] * rms[i];
            }
            for (int r = 0; r < n; ++r) {
                for (int c = 0; c < n; ++c) {
                    ionos_[r] += Gmat[r * n + c] * ionos[c];
                    rates_[r] += Gmat[r * n + c] * rates[c];
                    sqrms_[r] += Gmat[r * n + c] * sqrms[c];
                    quals_[r] += Gmat[r * n + c] * quals[c];
                }
            }
            for (i = 0; i < 4; ++i) {
                *iono += Emat[i] * ionos_[i];
                *rate += Emat[i] * rates_[i];
                *var += Emat[i] * sqrms_[i];
            }
        } else {
            for (i = 0; i < n; i++) {
                *iono += ionos[i] * weight[i];
                *rate += rates[i] * weight[i];
                *var += rms[i] * rms[i] * weight[i];
            }
        }
    }

    return 1;
}

void parityCompensatedisp(const ParityNav* nav,
                          const int* index,
                          const ParityObs* obs,
                          int sat,
                          const double iono,
                          const double* pb,
                          double* compL,
                          int* pbreset,
                          const ParityPrcOpt* opt,
                          const ParitySsat ssat,
                          int ch) {
    GNSSTime time = obs->time;
    static GNSSTime t0[kParitySsrChannelCount][kParityMaxSat] = {};
    static GNSSTime tm[kParitySsrChannelCount][kParityMaxSat] = {};
    int i, k, qi, qj, isat = sat - 1, oft = isat * kParityMaxFreq, oft_b;
    static double b0[kParitySsrChannelCount][kParityMaxSat * kParityMaxFreq] = {};
    static double bm[kParitySsrChannelCount][kParityMaxSat * kParityMaxFreq] = {};
    static double iono0[kParitySsrChannelCount][kParityMaxSat] = {};
    static double ionom[kParitySsrChannelCount][kParityMaxSat] = {};
    static double coef[kParitySsrChannelCount][kParityMaxSat * kParityMaxFreq] = {};
    static int slip[kParitySsrChannelCount][kParityMaxSat * kParityMaxFreq] = {};
    const double* lam = nav->lam[obs->sat - 1];
    double disp0, dispm, dt, dgf, fi;
    int nf = opt->nf, flag, fqi, fqj;

    oft_b = isat * kParityMaxFreq;

    for (k = flag = 0; k < nav->stec[index[0]].n; k++) {
        if (sat == nav->stec[index[0]].data[k].sat) {
            flag = 1;
            break;
        }
    }
    if (flag == 1 && timediff(nav->stec[index[0]].data[k].time, t0[ch][isat]) > 0.0) {
        if (opt->posopt[5] == 1) {
            tm[ch][isat] = t0[ch][isat];
            t0[ch][isat] = nav->stec[index[0]].data[k].time;
            dt = timediff(t0[ch][isat], tm[ch][isat]);
            if (dt <= 0.0) {
                return;
            }

            for (i = 0; i < nf; i++) {
                if (b0[ch][oft_b + i] == 0.0 || iono0[ch][isat] == 0.0) {
                    b0[ch][oft_b + i] = pb[i];
                    iono0[ch][isat] = iono;
                    return;
                }
            }

            for (i = 0; i < nf; i++) {
                bm[ch][oft_b + i] = b0[ch][oft_b + i];
                b0[ch][oft_b + i] = pb[i];
            }
            ionom[ch][isat] = iono0[ch][isat];
            iono0[ch][isat] = iono;

            for (i = 1; i < nf; i++) {
                qi = 0;
                qj = i;
                fqi = obsFrequencyIndex(nav->ssr_ch[ch][isat].smode[qi]);
                fqj = obsFrequencyIndex(nav->ssr_ch[ch][isat].smode[qj]);
                fi = lam[fqj] / lam[fqi];

                if (pb[qi] == kParityCssrInvalid || pb[qj] == kParityCssrInvalid ||
                    iono == 0.0) {
                    continue;
                }
                dispm = -kClasFreq2Hz / kClasFreq1Hz * (1.0 - fi * fi) *
                            ionom[ch][isat] +
                        bm[ch][oft_b + qi] - bm[ch][oft_b + qj];
                disp0 = -kClasFreq2Hz / kClasFreq1Hz * (1.0 - fi * fi) *
                            iono0[ch][isat] +
                        b0[ch][oft_b + qi] - b0[ch][oft_b + qj];
                coef[ch][isat + (i - 1) * kParityMaxSat] = (disp0 - dispm) / dt;
            }
        } else {
            for (i = 0; i < nf; i++) {
                b0[ch][oft_b + i] = obs->L[i] * lam[i];
                slip[ch][oft + i] = 0;
            }
            tm[ch][isat] = t0[ch][isat];
            t0[ch][isat] = nav->stec[index[0]].data[k].time;
        }
    }

    dt = timediff(time, t0[ch][isat]);
    if (opt->posopt[5] == 1) {
        for (i = 1; i < nf; i++) {
            qi = 0;
            qj = i;
            fqi = obsFrequencyIndex(nav->ssr_ch[ch][isat].smode[qi]);
            fqj = obsFrequencyIndex(nav->ssr_ch[ch][isat].smode[qj]);
            fi = lam[fqj] / lam[fqi];

            if (std::fabs(coef[ch][isat + (i - 1) * kParityMaxSat] /
                          (kClasFreq2Hz / kClasFreq1Hz * (1.0 - fi * fi))) > 0.008) {
                continue;
            }
            if (pbreset[qi] || pbreset[qj]) {
                coef[ch][isat] = 0.0;
                return;
            }
            compL[qi] = compL[qi] == 0.0
                            ? (1.0 / (1.0 - fi * fi) *
                               coef[ch][isat + (i - 1) * kParityMaxSat] * dt)
                            : compL[qi];
            compL[qj] = fi * fi / (1.0 - fi * fi) *
                        coef[ch][isat + (i - 1) * kParityMaxSat] * dt;
        }
    } else {
        for (i = 0; i < nf; i++) {
            if (ssat.slip[i] > 0) {
                slip[ch][oft + i] = 1;
            }
        }
        for (i = 1; i < nf; i++) {
            qi = 0;
            qj = i;
            fi = lam[qj] / lam[qi];
            if (slip[ch][oft + qi] || slip[ch][oft + qj] ||
                obs->L[qi] * lam[qi] == 0.0 || obs->L[qj] * lam[qj] == 0.0 ||
                pbreset[qi] || pbreset[qj]) {
                continue;
            }
            dgf = obs->L[qi] * lam[qi] - obs->L[qj] * lam[qj] -
                  (b0[ch][oft_b + qi] - b0[ch][oft_b + qj]);
            compL[qi] = compL[qi] == 0.0 ? (1.0 / (1.0 - fi * fi) * dgf) : compL[qi];
            compL[qj] = fi * fi / (1.0 - fi * fi) * dgf;
        }
    }
}

int parityCorrmeasLiteral(const ParityObs* obs,
                          ParityNav* nav,
                          const double* pos,
                          const double* azel,
                          const ParityPrcOpt* opt,
                          const int* index,
                          const int n,
                          const double* weight,
                          const double* Gmat,
                          const double* Emat,
                          const ParitySsat ssat,
                          int* brk,
                          ParityOsr* osr,
                          int* pbreset,
                          int ch) {
    const double* lam = nav->lam[obs->sat - 1];
    double vari, dant[kParityMaxFreq] = {}, compL[kParityMaxFreq] = {};
    double stec = 0.0, rate, t5, t6;
    double pbias[kParityMaxFreq] = {}, cbias[kParityMaxFreq] = {};
    int i, sat, smode, nsig, nf = opt->nf;
    int flag;
    double dt;
    static GNSSTime currtime = {};

    (void)pos;

    sat = obs->sat;
    for (i = 0; i < nf; i++) {
        pbias[i] = cbias[i] = kParityCssrInvalid;
    }

    t5 = timediff(obs->time, nav->ssr_ch[ch][sat - 1].t0[4]);
    t6 = timediff(obs->time, nav->ssr_ch[ch][sat - 1].t0[5]);

    if (std::fabs(t5) > kParityMaxAgeSsrBiasSeconds ||
        std::fabs(t6) > kParityMaxAgeSsrBiasSeconds) {
        nav->ssr_ch[ch][sat - 1].t0[4] = {};
        nav->ssr_ch[ch][sat - 1].t0[5] = {};
        return 0;
    }

    switch (nav->rtcmmode) {
        case kParityRtcmModeCssr:
            nsig = nav->ssr_ch[ch][sat - 1].nsig;
            for (i = 0; i < nf; i++) {
                for (int j = 0; j < nsig; j++) {
                    smode = nav->ssr_ch[ch][sat - 1].smode[j];
                    if (obs->code[i] == smode) {
                        pbias[i] = nav->ssr_ch[ch][sat - 1].pbias[smode - 1];
                        cbias[i] = nav->ssr_ch[ch][sat - 1].cbias[smode - 1];
                        break;
                    }
                }
            }
            break;
        default:
            return 0;
    }

    if (!parityStecGridData(nav,
                            index,
                            obs->time,
                            sat,
                            n,
                            weight,
                            Gmat,
                            Emat,
                            &stec,
                            &rate,
                            &vari,
                            brk)) {
        return 0;
    }

    for (i = flag = 0; i < nav->stec[index[0]].n; i++) {
        if (sat == nav->stec[index[0]].data[i].sat) {
            flag = 1;
            break;
        }
    }
    if (flag == 1) {
        dt = timediff(nav->stec[index[0]].data[i].time, nav->ssr_ch[ch][sat - 1].t0[5]);
        nav->ssr_ch[ch][sat - 1].t0[8] = nav->stec[index[0]].data[i].time;
        if (dt < 0.0 || dt >= 30.0) {
            return 0;
        }
        dt = timediff(nav->stec[index[0]].data[i].time, nav->ssr_ch[ch][sat - 1].t0[0]);
        if (dt < -30.0 || dt > 30.0) {
            return 0;
        }
    }
    dt = timediff(nav->ssr_ch[ch][sat - 1].t0[5], nav->ssr_ch[ch][sat - 1].t0[4]);
    if ((!opt->posopt[9] && std::fabs(dt) > 0.0) ||
        (opt->posopt[9] && (dt > 0.0 || dt < -30.0))) {
        return 0;
    }

    if (timediff(obs->time, currtime) != 0.0) {
        currtime = obs->time;
    }

    if (opt->posopt[5] > 0) {
        parityCompensatedisp(nav, index, obs, sat, stec, pbias, compL, pbreset, opt, ssat, ch);
    }

    antmodel(opt->pcvr[0], opt->antdel[0], azel, opt->posopt[1], dant);

    for (i = 0; i < nf; i++) {
        osr->cbias[i] = cbias[i];
        osr->pbias[i] = pbias[i];
        osr->compL[i] = compL[i];
    }

    for (i = 0; i < nf; i++) {
        osr->wupL[i] = lam[i] * ssat.phw;
        osr->antr[i] = dant[i];
    }
    osr->iono = stec;

    return 1;
}

int parityTropData(const TropGridPoint& grid,
                   const GNSSTime& time,
                   double* ztd,
                   double* zwd,
                   double* quality,
                   int* valid) {
    const double tt = timediff(time, grid.time);
    if (std::fabs(tt) > kParityMaxAgeSsrTropSeconds) {
        return 0;
    }
    const double pos[2] = {
        grid.pos_deg[0] * kDegreesToRadians,
        grid.pos_deg[1] * kDegreesToRadians,
    };
    const double geoid_height_m = embeddedGeoidHeightJapan(pos[0], pos[1]);
    double tdvd = 0.0;
    double twvd = 0.0;
    if (claslibGetStTv(time, pos[0], 0.0, geoid_height_m, &tdvd, &twvd)) {
        return 0;
    }
    const double raw_zwd = static_cast<double>(static_cast<float>(grid.zwd));
    const double raw_ztd = static_cast<double>(static_cast<float>(grid.ztd));
    if (raw_ztd == kParityCssrInvalid || raw_zwd == kParityCssrInvalid) {
        return 0;
    }
    *ztd = (raw_ztd - raw_zwd) / tdvd;
    *zwd = raw_zwd / twvd;
    *quality = static_cast<double>(static_cast<float>(grid.quality));
    *valid = grid.valid;
    return 1;
}

int parityTropGridDataLiteral(const TropGridInput& input, TropGridOutput& out) {
    const int n = std::clamp(input.n, 0, 4);
    if (n <= 0) {
        return 0;
    }

    double zwds[4] = {};
    double ztds[4] = {};
    double quals[4] = {};
    int valid[4] = {};
    if (n == 1) {
        const int idx = std::clamp(input.index[0], 0, 3);
        double quality = 0.0;
        int valid0 = 0;
        if (!parityTropData(input.grid[idx],
                            input.time,
                            &out.ztd,
                            &out.zwd,
                            &quality,
                            &valid0)) {
            out = TropGridOutput{};
            return 0;
        }
        out.break_flag = 0;
        return 1;
    }

    for (int i = 0; i < n; ++i) {
        const int idx = std::clamp(input.index[i], 0, 3);
        if (!parityTropData(input.grid[idx],
                            input.time,
                            &ztds[i],
                            &zwds[i],
                            &quals[i],
                            &valid[i])) {
            out = TropGridOutput{};
            return 0;
        }
    }

    out.zwd = 0.0;
    out.ztd = 0.0;
    if (n == 4) {
        double zwds_g[4] = {};
        double ztds_g[4] = {};
        double quals_g[4] = {};
        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                ztds_g[r] += input.Gmat[r * n + c] * ztds[c];
                zwds_g[r] += input.Gmat[r * n + c] * zwds[c];
                quals_g[r] += input.Gmat[r * n + c] * quals[c];
            }
        }
        (void)quals_g;
        for (int i = 0; i < 4; ++i) {
            out.ztd += input.Emat[i] * ztds_g[i];
            out.zwd += input.Emat[i] * zwds_g[i];
        }
    } else {
        for (int i = 0; i < n; ++i) {
            out.zwd += zwds[i] * input.weight[i];
            out.ztd += ztds[i] * input.weight[i];
        }
    }
    int valid_count = 0;
    for (int i = 0; i < n; ++i) {
        valid_count += valid[i];
    }
    out.break_flag = valid_count == n ? 0 : 1;
    return 1;
}

bool paritySeleph(const GNSSTime& time,
                  int sat,
                  int iode,
                  const SatposBroadcastEphemeris& eph) {
    if (!eph.valid || eph.sat != sat) {
        return false;
    }
    if (iode >= 0 && eph.iode != iode) {
        return false;
    }
    const int sys = paritySatsys(sat, nullptr);
    if (sys != kClasSysGps && sys != kClasSysGal &&
        sys != kClasSysQzs && sys != kClasSysCmp) {
        return false;
    }
    return std::fabs(clasTimediff(eph.toe, time)) <= parityMaxDtoe(sys);
}

bool parityEph2pos(const GNSSTime& time,
                   const SatposBroadcastEphemeris& eph,
                   double* rs,
                   double* dts,
                   double* var) {
    double tk, M, E, Ek, sinE, cosE, u, r, i, O;
    double sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
    double xg, yg, zg, sino, coso;
    int n, sys, prn;

    if (eph.A <= 0.0) {
        rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
        return true;
    }
    tk = clasTimediff(time, eph.toe);

    switch ((sys = paritySatsys(eph.sat, &prn))) {
        case kClasSysGal:
            mu = kClasMuGal;
            omge = kClasOmgeGal;
            break;
        case kClasSysCmp:
            mu = kClasMuCmp;
            omge = kClasOmgeCmp;
            break;
        default:
            mu = kClasMuGps;
            omge = kClasOmge;
            break;
    }
    M = eph.M0 + (std::sqrt(mu / (eph.A * eph.A * eph.A)) + eph.deln) * tk;

    for (n = 0, E = M, Ek = 0.0;
         std::fabs(E - Ek) > kClasRtolKepler && n < kClasMaxIterKepler;
         n++) {
        Ek = E;
        E -= (E - eph.e * std::sin(E) - M) / (1.0 - eph.e * std::cos(E));
    }
    if (n >= kClasMaxIterKepler) {
        return false;
    }
    sinE = std::sin(E);
    cosE = std::cos(E);

    u = std::atan2(std::sqrt(1.0 - eph.e * eph.e) * sinE, cosE - eph.e) +
        eph.omg;
    r = eph.A * (1.0 - eph.e * cosE);
    i = eph.i0 + eph.idot * tk;
    sin2u = std::sin(2.0 * u);
    cos2u = std::cos(2.0 * u);
    u += eph.cus * sin2u + eph.cuc * cos2u;
    r += eph.crs * sin2u + eph.crc * cos2u;
    i += eph.cis * sin2u + eph.cic * cos2u;
    x = r * std::cos(u);
    y = r * std::sin(u);
    cosi = std::cos(i);

    if (sys == kClasSysCmp && prn <= 5) {
        O = eph.OMG0 + eph.OMGd * tk - omge * eph.toes;
        sinO = std::sin(O);
        cosO = std::cos(O);
        xg = x * cosO - y * cosi * sinO;
        yg = x * sinO + y * cosi * cosO;
        zg = y * std::sin(i);
        sino = std::sin(omge * tk);
        coso = std::cos(omge * tk);
        rs[0] = xg * coso + yg * sino * kClasCos5 + zg * sino * kClasSin5;
        rs[1] = -xg * sino + yg * coso * kClasCos5 + zg * coso * kClasSin5;
        rs[2] = -yg * kClasSin5 + zg * kClasCos5;
    } else {
        O = eph.OMG0 + (eph.OMGd - omge) * tk - omge * eph.toes;
        sinO = std::sin(O);
        cosO = std::cos(O);
        rs[0] = x * cosO - y * cosi * sinO;
        rs[1] = x * sinO + y * cosi * cosO;
        rs[2] = y * std::sin(i);
    }
    tk = clasTimediff(time, eph.toc);
    *dts = eph.f0 + eph.f1 * tk + eph.f2 * tk * tk;

    *dts -= 2.0 * std::sqrt(mu * eph.A) * eph.e * sinE /
            (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT);

    *var = parityVarUraeph(eph.sva, sys);
    return true;
}

bool parityEphpos(const GNSSTime& time,
                  const GNSSTime& teph,
                  int sat,
                  const SatposBroadcastEphemeris& eph,
                  int iode,
                  double* rs,
                  double* dts,
                  double* var,
                  int* svh) {
    double rst[3] = {};
    double dtst[1] = {};
    constexpr double tt = 1E-3;
    const int sys = paritySatsys(sat, nullptr);

    *svh = -1;
    if (sys == kClasSysGps || sys == kClasSysGal ||
        sys == kClasSysQzs || sys == kClasSysCmp) {
        if (!paritySeleph(teph, sat, iode, eph)) {
            return false;
        }
        if (!parityEph2pos(time, eph, rs, dts, var)) {
            return false;
        }
        if (!parityEph2pos(clasTimeadd(time, tt), eph, rst, dtst, var)) {
            return false;
        }
        *svh = eph.svh;
    } else {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        rs[i + 3] = (rst[i] - rs[i]) / tt;
    }
    dts[1] = (dtst[0] - dts[0]) / tt;
    return true;
}

int paritySatposSsrLiteral(const SatposSsrInput& input,
                           int opt,
                           double* rs,
                           double* dts,
                           double* var,
                           int* svh) {
    const SatposSsrCorrection& ssr = input.ssr;
    double t1, t2, t3;
    double dorb[3] = {};
    double er[3] = {};
    double ea[3] = {};
    double ec[3] = {};
    double rc[3] = {};
    double deph[3] = {};
    double dclk;
    double dant[3] = {};
    double sig[3] = {};
    double nsig[3] = {};
    int i;

    (void)opt;

    if (!gnsstimeIsSet(ssr.t0[0])) {
        return 0;
    }
    if (!gnsstimeIsSet(ssr.t0[1])) {
        return 0;
    }

    t1 = clasTimediff(input.time, ssr.t0[0]);
    t2 = clasTimediff(input.time, ssr.t0[1]);
    t3 = clasTimediff(input.time, ssr.t0[2]);

    if (std::fabs(t1) > kClasMaxAgeSsrOrbitSeconds ||
        std::fabs(t2) > kClasMaxAgeSsrClockSeconds) {
        *svh = -1;
        return 0;
    }
    if (ssr.udi[0] >= 1.0) {
        t1 -= ssr.udi[0] / 2.0;
    }
    if (ssr.udi[1] >= 1.0) {
        t2 -= ssr.udi[1] / 2.0;
    }

    for (i = 0; i < 3; i++) {
        deph[i] = ssr.deph[i] + ssr.ddeph[i] * t1;
    }
    dclk = ssr.dclk[0] + ssr.dclk[1] * t2 + ssr.dclk[2] * t2 * t2;

    if (ssr.iod[0] == ssr.iod[2] &&
        gnsstimeIsSet(ssr.t0[2]) &&
        std::fabs(t3) < kClasMaxAgeSsrHighRateClockSeconds) {
        dclk += ssr.hrclk;
    }
    if (norm3(deph) > kClasMaxSsrOrbitCorrectionM ||
        std::fabs(dclk) > kClasMaxSsrClockCorrectionM) {
        *svh = -1;
        return 0;
    }

    if (!parityEphpos(input.time,
                      input.teph,
                      input.sat,
                      input.eph,
                      ssr.iode,
                      rs,
                      dts,
                      var,
                      svh)) {
        return 0;
    }

    if (!unit3(rs + 3, ea)) {
        return 0;
    }
    cross3(rs, rs + 3, rc);
    if (!unit3(rc, ec)) {
        *svh = -1;
        return 0;
    }
    cross3(ea, ec, er);

    for (i = 0; i < 3; i++) {
        dorb[i] = -(er[i] * deph[0] + ea[i] * deph[1] + ec[i] * deph[2]);
        rs[i] += dorb[i] + dant[i];
    }

    dts[0] += dclk / constants::SPEED_OF_LIGHT;

    if (norm3(input.receiver_position) <= 0.0) {
        return 0;
    }
    for (i = 0; i < 3; i++) {
        sig[i] = rs[i] - input.receiver_position[i];
    }
    if (!unit3(sig, nsig)) {
        return 0;
    }

    *var = parityVarUrassr(ssr.ura);
    return 1;
}

CorrmeasRuntime makeCorrmeasRuntime(const CorrmeasInput& input) {
    CorrmeasRuntime runtime;
    const int nf = std::clamp(input.num_frequencies, 0, kParityMaxFreq);
    const int sat = std::clamp(input.sat, 1, kParityMaxSat);

    runtime.obs.time = input.time;
    runtime.obs.sat = static_cast<unsigned char>(sat);
    runtime.opt.nf = nf;
    runtime.opt.posopt[1] = input.antenna_pcv_option;
    runtime.opt.posopt[5] = input.compensation_option;
    runtime.opt.posopt[9] = input.phase_code_timing_option;
    runtime.opt.pcvr[0] = input.receiver_pcv;
    for (int j = 0; j < 3; ++j) {
        runtime.opt.antdel[0][j] = input.antenna_delta[j];
    }
    runtime.ssat.phw = input.phase_windup_cycles;
    runtime.osr.trop = input.trop_m;
    runtime.osr.relatv = input.relativity_m;
    runtime.index[0] = 0;
    runtime.weight[0] = 1.0;
    runtime.Emat[3] = 1.0;

    for (int f = 0; f < nf; ++f) {
        runtime.obs.code[f] = input.code[f];
        runtime.obs.L[f] = input.carrier_phase_cycles[f];
        runtime.obs.P[f] = input.pseudorange_m[f];
        runtime.nav.lam[sat - 1][f] = input.wavelength_m[f];
        runtime.ssat.slip[f] = input.slip[f];
        runtime.pbreset[f] = input.phase_bias_reset[f];
    }

    ParitySsr& ssr = runtime.nav.ssr_ch[0][sat - 1];
    ssr.t0[0] = input.orbit_time;
    ssr.t0[4] = input.code_bias_time;
    ssr.t0[5] = input.phase_bias_time;
    ssr.t0[8] = input.stec_time;
    ssr.nsig = nf;
    for (int f = 0; f < nf; ++f) {
        const int code = input.code[f];
        ssr.smode[f] = code;
        if (code > 0 && code <= kParityMaxCode) {
            ssr.cbias[code - 1] = static_cast<float>(input.code_bias_m[f]);
            ssr.pbias[code - 1] = static_cast<float>(input.phase_bias_m[f]);
        }
    }

    runtime.nav.stec[0].n = 1;
    runtime.nav.stec[0].data[0].time = input.stec_time;
    runtime.nav.stec[0].data[0].sat = static_cast<unsigned char>(sat);
    runtime.nav.stec[0].data[0].slip = 0;
    runtime.nav.stec[0].data[0].iono = static_cast<float>(input.stec_l1_m);
    runtime.nav.stec[0].data[0].rate = static_cast<float>(input.stec_rate_mps);
    runtime.nav.stec[0].data[0].rms = static_cast<float>(input.stec_rms_m);
    runtime.nav.stec[0].data[0].quality = 1.0F;
    runtime.nav.stec[0].data[0].flag = 1;

    return runtime;
}

void fillCorrmeasOutputFromOsr(const CorrmeasInput& input,
                               const ParityOsr& osr,
                               CorrmeasOutput& out) {
    const int nf = std::clamp(input.num_frequencies, 0, kParityMaxFreq);
    out = CorrmeasOutput{};
    out.num_frequencies = nf;
    out.iono = osr.iono;
    for (int f = 0; f < nf; ++f) {
        const double fi = (input.wavelength_m[0] != 0.0)
                              ? input.wavelength_m[f] / input.wavelength_m[0]
                              : 1.0;
        const double iono_scaled =
            fi * fi * kClasFreq2Hz / kClasFreq1Hz * osr.iono;
        out.code_bias[f] = osr.cbias[f];
        out.phase_bias[f] = osr.pbias[f];
        out.phase_compensation[f] = osr.compL[f];
        out.receiver_antenna[f] = osr.antr[f];
        out.windup_m[f] = osr.wupL[f];
        out.prc[f] =
            osr.trop + osr.relatv + osr.antr[f] + iono_scaled + osr.cbias[f];
        out.cpc[f] = osr.trop + osr.relatv + osr.antr[f] - iono_scaled +
                     osr.pbias[f] + osr.wupL[f] + osr.compL[f];
    }
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

bool eph2clkAvailable() {
    return true;
}

bool eph2posAvailable() {
    return true;
}

bool geodistAvailable() {
    return true;
}

bool satposSsrAvailable() {
    return true;
}

bool corrmeasAvailable() {
    return true;
}

bool satantoffAvailable() {
    return true;
}

bool compensatedispAvailable() {
    return true;
}

bool tropGridDataAvailable() {
    return true;
}

CorrmeasInput makeCorrmeasInput(int sample_index) {
    const int sample = ((sample_index % 20) + 20) % 20;
    const int epoch = sample / 4;
    const int sat_index = sample % 4;
    const double lat = 36.1037748 * kDegreesToRadians;
    const double lon = 140.0878550 * kDegreesToRadians;

    CorrmeasInput input;
    input.time = GNSSTime(2029, 230400.0 + 30.0 * epoch);
    input.sat = 1 + sat_index;
    input.num_frequencies = kParityMaxFreq;
    input.code[0] = 1;   // CODE_L1C
    input.code[1] = 17;  // CODE_L2L
    input.code[2] = 26;  // CODE_L5X
    input.wavelength_m[0] = constants::SPEED_OF_LIGHT / kClasFreq1Hz;
    input.wavelength_m[1] = constants::SPEED_OF_LIGHT / kClasFreq2Hz;
    input.wavelength_m[2] = constants::SPEED_OF_LIGHT / kClasFreq5Hz;
    input.receiver_pos[0] = lat;
    input.receiver_pos[1] = lon;
    input.receiver_pos[2] = 67.0;
    input.azel[0] = std::fmod((35.0 + sat_index * 67.0 + epoch * 4.0) *
                                  kDegreesToRadians,
                              2.0 * M_PI);
    input.azel[1] = (18.0 + sat_index * 14.0 + epoch * 1.5) * kDegreesToRadians;

    for (int f = 0; f < kParityMaxFreq; ++f) {
        const double range_m =
            20200000.0 + sample * 3750.0 + f * 950.0 + epoch * 125.0;
        input.carrier_phase_cycles[f] = range_m / input.wavelength_m[f];
        input.pseudorange_m[f] = range_m + 0.35 * f;
        input.code_bias_m[f] = 0.18 + 0.015 * f - 0.002 * sample;
        input.phase_bias_m[f] = -0.23 + 0.012 * f + 0.0015 * sample;
        input.receiver_pcv.offsets_m[static_cast<size_t>(f)] = {
            0.0015 * (f + 1) + 0.00005 * sample,
            -0.0020 * (f + 1) + 0.00003 * epoch,
            0.0450 + 0.0040 * f + 0.00004 * sat_index,
        };
        for (int i = 0; i < kParityPcvGridSize; ++i) {
            input.receiver_pcv.variations_m[static_cast<size_t>(f)][static_cast<size_t>(i)] =
                0.00015 * (f + 1) + 0.000025 * i +
                0.000003 * f * i + 0.0000004 * sample;
        }
    }

    input.antenna_delta[0] = 0.0025 + 0.0001 * sat_index;
    input.antenna_delta[1] = -0.0010 + 0.00005 * epoch;
    input.antenna_delta[2] = 0.0140 + 0.0002 * sample;
    input.antenna_pcv_option = 1;
    input.compensation_option = 0;
    input.phase_code_timing_option = 0;
    input.phase_windup_cycles = 0.125 + 0.006 * epoch + 0.002 * sat_index;
    input.stec_l1_m = 2.05 + 0.07 * sample + 0.03 * sat_index;
    input.stec_rate_mps = 0.0002 * static_cast<double>((sample % 5) - 2);
    input.stec_rms_m = 0.012 + 0.0005 * sat_index;
    input.stec_time = input.time;
    input.orbit_time = input.time;
    input.code_bias_time = input.time;
    input.phase_bias_time = input.time;
    input.trop_m = 1.34 + 0.011 * epoch + 0.006 * sat_index;
    input.relativity_m = -0.0025 + 0.0001 * sample;

    return input;
}

SatAntOffInput makeSatAntOffInput(int sample_index) {
    const int sample = ((sample_index % 20) + 20) % 20;
    const int epoch = sample / 4;
    const int sat_case = sample % 4;
    const int sats[4] = {1, 14, 33, 69};
    const double lat = 36.1037748 * kDegreesToRadians;
    const double lon = 140.0878550 * kDegreesToRadians;
    const Vector3d rr = geodetic2ecef(lat, lon, 67.0);
    const double az = std::fmod((42.0 + sat_case * 71.0 + epoch * 5.0) *
                                    kDegreesToRadians,
                                2.0 * M_PI);
    const double el = (22.0 + sat_case * 13.0 + epoch * 1.25) * kDegreesToRadians;
    const Vector3d los_enu(std::sin(az) * std::cos(el),
                           std::cos(az) * std::cos(el),
                           std::sin(el));
    const Vector3d los_ecef = enu2ecef(los_enu, lat, lon).normalized();
    const double range_m = 23250000.0 + sample * 42000.0;
    const Vector3d sat_pos = rr + range_m * los_ecef;
    const Vector3d east = enu2ecef(Vector3d(1.0, 0.0, 0.0), lat, lon).normalized();
    const Vector3d north = enu2ecef(Vector3d(0.0, 1.0, 0.0), lat, lon).normalized();
    const Vector3d sat_vel =
        2850.0 * (std::cos(az) * east - std::sin(az) * north) +
        (40.0 + sat_case * 8.0) * los_ecef;

    SatAntOffInput input;
    input.time = GNSSTime(2029, 230400.0 + epoch * 30.0);
    input.sat = sats[sat_case];
    input.rs[0] = sat_pos(0);
    input.rs[1] = sat_pos(1);
    input.rs[2] = sat_pos(2);
    input.rs[3] = sat_vel(0);
    input.rs[4] = sat_vel(1);
    input.rs[5] = sat_vel(2);
    input.wavelength_m[0] = constants::SPEED_OF_LIGHT / kClasFreq1Hz;
    input.wavelength_m[1] = constants::SPEED_OF_LIGHT / kClasFreq2Hz;
    input.wavelength_m[2] = constants::SPEED_OF_LIGHT / kClasFreq5Hz;
    for (int f = 0; f < kParityMaxFreq; ++f) {
        input.satellite_pcv.offsets_m[static_cast<size_t>(f)] = {
            0.18 + 0.010 * f + 0.001 * sample,
            -0.04 + 0.006 * f - 0.0005 * epoch,
            0.72 + 0.025 * f + 0.002 * sat_case,
        };
    }
    return input;
}

TropGridInput makeTropGridInput(int sample_index) {
    const int sample = ((sample_index % 20) + 20) % 20;
    TropGridInput input;
    input.time = GNSSTime(2029, 230400.0 + 15.0 * sample);
    input.n = (sample % 2 == 0) ? 1 : 4;
    input.weight[0] = 1.0;
    if (input.n == 4) {
        for (int i = 0; i < 4; ++i) {
            input.index[i] = i;
            input.Emat[i] = 0.10 + 0.10 * i;
            input.Gmat[i * 4 + i] = 1.0;
        }
    }
    for (int i = 0; i < 4; ++i) {
        input.grid[i].pos_deg[0] = 36.00 + 0.07 * i + 0.002 * sample;
        input.grid[i].pos_deg[1] = 140.00 + 0.05 * i - 0.001 * sample;
        input.grid[i].pos_deg[2] = 0.0;
        input.grid[i].time = input.time - (3.0 + i);
        input.grid[i].zwd = 0.18 + 0.006 * i + 0.0015 * sample;
        input.grid[i].ztd = 2.48 + 0.011 * i + 0.0020 * sample;
        input.grid[i].quality = 0.75 + 0.01 * i;
        input.grid[i].rms = 0.012 + 0.001 * i;
        input.grid[i].valid = ((sample + i) % 7 == 0) ? 0 : 1;
    }
    return input;
}

SatposSsrInput makeSatposSsrInput(int sample_index) {
    const int sample = ((sample_index % 24) + 24) % 24;
    const int epoch = sample / 6;
    const int sat_case = sample % 6;
    const int sats[6] = {1, 14, 25, 33, 59, 69};
    const double lat = 36.1037748 * kDegreesToRadians;
    const double lon = 140.0878550 * kDegreesToRadians;
    const Vector3d rr = geodetic2ecef(lat, lon, 67.0);

    SatposSsrInput input;
    input.teph = GNSSTime(2029, 230400.0 + 30.0 * epoch);
    input.time = input.teph - (0.071 + 0.002 * sat_case + 0.0005 * epoch);
    input.sat = sats[sat_case];
    input.receiver_position[0] = rr(0);
    input.receiver_position[1] = rr(1);
    input.receiver_position[2] = rr(2);

    SatposBroadcastEphemeris& eph = input.eph;
    eph.valid = true;
    eph.sat = input.sat;
    eph.iode = 48 + sample;
    eph.iodc = 140 + sample;
    eph.sva = (sat_case == 3 || sat_case == 4) ? 12 + sat_case : 2 + sat_case;
    eph.svh = 0;
    eph.week = input.teph.week;
    eph.toe = GNSSTime(input.teph.week, 230000.0 + 60.0 * epoch);
    eph.toc = eph.toe;
    eph.ttr = eph.toe;
    eph.toes = eph.toe.tow;
    eph.A = (sat_case == 3 || sat_case == 4) ? 29600000.0 : 26560000.0;
    eph.A += 1200.0 * sat_case + 350.0 * epoch;
    eph.e = 0.0065 + 0.00035 * sat_case + 0.00005 * epoch;
    eph.i0 = (54.0 + 0.7 * sat_case) * kDegreesToRadians;
    eph.OMG0 = (38.0 + 41.0 * sat_case + 1.3 * epoch) * kDegreesToRadians;
    eph.omg = (12.0 + 19.0 * sat_case + 0.5 * epoch) * kDegreesToRadians;
    eph.M0 = (78.0 + 23.0 * sat_case + 3.1 * epoch) * kDegreesToRadians;
    eph.deln = (3.5 + 0.2 * sat_case) * 1E-9;
    eph.OMGd = (-8.1 - 0.15 * sat_case) * 1E-9;
    eph.idot = (0.4 - 0.03 * sat_case) * 1E-10;
    eph.crc = 130.0 + 3.0 * sat_case;
    eph.crs = -42.0 + 1.7 * epoch;
    eph.cuc = (1.2 + 0.1 * sat_case) * 1E-6;
    eph.cus = (-0.8 + 0.05 * epoch) * 1E-6;
    eph.cic = (0.4 + 0.02 * sat_case) * 1E-6;
    eph.cis = (-0.3 + 0.01 * epoch) * 1E-6;
    eph.f0 = (2.4 + 0.07 * sat_case) * 1E-5;
    eph.f1 = (-4.0 + 0.2 * epoch) * 1E-12;
    eph.f2 = (0.3 + 0.02 * sat_case) * 1E-18;
    eph.tgd[0] = -2.3E-9 + 0.1E-9 * sat_case;
    eph.tgd[1] = 1.2E-9 - 0.05E-9 * epoch;

    SatposSsrCorrection& ssr = input.ssr;
    ssr.t0[0] = input.time - (10.0 + sat_case);
    ssr.t0[1] = input.time - (5.0 + 0.5 * epoch);
    ssr.t0[3] = ssr.t0[1];
    ssr.udi[0] = 30.0;
    ssr.udi[1] = 5.0;
    ssr.iod[0] = 3 + epoch;
    ssr.iod[1] = 3 + epoch;
    ssr.iod[2] = 2 + epoch;
    ssr.iode = eph.iode;
    ssr.ura = 10 + sat_case;
    ssr.deph[0] = 0.18 + 0.010 * sat_case + 0.002 * epoch;
    ssr.deph[1] = -0.075 + 0.006 * sat_case - 0.001 * epoch;
    ssr.deph[2] = 0.043 - 0.004 * sat_case + 0.0007 * epoch;
    ssr.ddeph[0] = (0.8 + 0.1 * sat_case) * 1E-4;
    ssr.ddeph[1] = (-0.6 + 0.05 * epoch) * 1E-4;
    ssr.ddeph[2] = (0.35 - 0.03 * sat_case) * 1E-4;
    ssr.dclk[0] = 0.42 - 0.018 * sat_case + 0.006 * epoch;
    ssr.dclk[1] = (2.0 + 0.2 * sat_case) * 1E-4;
    ssr.dclk[2] = (-0.8 + 0.1 * epoch) * 1E-6;
    ssr.hrclk = 0.0;

    return input;
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

double eph2clk(const GNSSTime& time, const SatposBroadcastEphemeris& eph) {
    double t = clasTimediff(time, eph.toc);
    for (int i = 0; i < 2; ++i) {
        t -= eph.f0 + eph.f1 * t + eph.f2 * t * t;
    }
    return eph.f0 + eph.f1 * t + eph.f2 * t * t;
}

bool eph2pos(const GNSSTime& time,
             const SatposBroadcastEphemeris& eph,
             double rs[3],
             double& dts,
             double& variance) {
    return parityEph2pos(time, eph, rs, &dts, &variance);
}

double geodist(const double rs[3], const double rr[3], double e[3]) {
    if (norm3(rs) < constants::WGS84_A) {
        return -1.0;
    }
    for (int i = 0; i < 3; ++i) {
        e[i] = rs[i] - rr[i];
    }
    const double r = norm3(e);
    for (int i = 0; i < 3; ++i) {
        e[i] /= r;
    }
    return r + kClasOmge * (rs[0] * rr[1] - rs[1] * rr[0]) /
                   constants::SPEED_OF_LIGHT;
}

bool satpos_ssr(const GNSSTime& teph,
                const GNSSTime& time,
                int sat,
                SatposSsrOutput& out) {
    SatposSsrInput input = makeSatposSsrInput(std::max(0, sat - 1));
    input.teph = teph;
    input.time = time;
    input.sat = sat;
    input.eph.sat = sat;
    input.eph.toe = teph - 400.0;
    input.eph.toc = input.eph.toe;
    input.eph.ttr = input.eph.toe;
    input.eph.toes = input.eph.toe.tow;
    input.ssr.t0[0] = time;
    input.ssr.t0[1] = time;
    input.ssr.t0[3] = time;
    return satpos_ssr(input, out);
}

bool satpos_ssr(const SatposSsrInput& input, SatposSsrOutput& out) {
    out = SatposSsrOutput{};
    return paritySatposSsrLiteral(input,
                                  input.apply_satellite_antenna_offset ? 1 : 0,
                                  out.rs,
                                  out.dts,
                                  &out.variance,
                                  &out.svh) != 0;
}

bool corrmeas(const CorrmeasInput& input, CorrmeasOutput& out) {
    CorrmeasRuntime runtime = makeCorrmeasRuntime(input);
    if (!parityCorrmeasLiteral(&runtime.obs,
                               &runtime.nav,
                               input.receiver_pos,
                               input.azel,
                               &runtime.opt,
                               runtime.index,
                               1,
                               runtime.weight,
                               runtime.Gmat,
                               runtime.Emat,
                               runtime.ssat,
                               &runtime.brk,
                               &runtime.osr,
                               runtime.pbreset,
                               0)) {
        out = CorrmeasOutput{};
        return false;
    }

    fillCorrmeasOutputFromOsr(input, runtime.osr, out);
    return true;
}

bool corrmeas(int sample_index, CorrmeasOutput& out) {
    return corrmeas(makeCorrmeasInput(sample_index), out);
}

bool corrmeas(CorrmeasOutput& out) {
    return corrmeas(0, out);
}

void satantoff(const SatAntOffInput& input, double dant[3]) {
    dant[0] = dant[1] = dant[2] = 0.0;
    const double* lam = input.wavelength_m;
    const Vector3d sun = tidal::approximateSunPositionEcef(input.time);
    double ez_vec[3] = {-input.rs[0], -input.rs[1], -input.rs[2]};
    double ez[3] = {};
    if (!unit3(ez_vec, ez)) {
        return;
    }
    double es_vec[3] = {
        sun(0) - input.rs[0],
        sun(1) - input.rs[1],
        sun(2) - input.rs[2],
    };
    double es[3] = {};
    if (!unit3(es_vec, es)) {
        return;
    }
    double ey_vec[3] = {};
    double ey[3] = {};
    cross3(ez, es, ey_vec);
    if (!unit3(ey_vec, ey)) {
        return;
    }
    double ex[3] = {};
    cross3(ey, ez, ex);

    int j = 0;
    int k = 1;
    const int sys = paritySatsys(input.sat, nullptr);
    if (kParityMaxFreq >= 3 && sys == kClasSysGal) {
        k = 2;
    }
    if (lam[j] == 0.0 || lam[k] == 0.0) {
        return;
    }
    const double gamma = (lam[k] * lam[k]) / (lam[j] * lam[j]);
    const double c1 = gamma / (gamma - 1.0);
    const double c2 = -1.0 / (gamma - 1.0);
    for (int i = 0; i < 3; ++i) {
        const auto& off1 = input.satellite_pcv.offsets_m[static_cast<size_t>(j)];
        const auto& off2 = input.satellite_pcv.offsets_m[static_cast<size_t>(k)];
        const double dant1 = off1[0] * ex[i] + off1[1] * ey[i] + off1[2] * ez[i];
        const double dant2 = off2[0] * ex[i] + off2[1] * ey[i] + off2[2] * ez[i];
        dant[i] = c1 * dant1 + c2 * dant2;
    }
}

bool compensatedisp(const CorrmeasInput& input, double compL[kParityMaxFreq]) {
    for (int f = 0; f < kParityMaxFreq; ++f) {
        compL[f] = 0.0;
    }
    CorrmeasRuntime runtime = makeCorrmeasRuntime(input);
    double pbias[kParityMaxFreq] = {};
    for (int f = 0; f < kParityMaxFreq; ++f) {
        pbias[f] = input.phase_bias_m[f];
    }
    parityCompensatedisp(&runtime.nav,
                         runtime.index,
                         &runtime.obs,
                         runtime.obs.sat,
                         input.stec_l1_m,
                         pbias,
                         compL,
                         runtime.pbreset,
                         &runtime.opt,
                         runtime.ssat,
                         0);
    return true;
}

bool trop_grid_data(const TropGridInput& input, TropGridOutput& out) {
    out = TropGridOutput{};
    return parityTropGridDataLiteral(input, out) != 0;
}

}  // namespace libgnss::clasnat_parity
