#include <libgnss++/algorithms/madoca_parity.hpp>

#include <libgnss++/core/constants.hpp>

#include <cmath>
#include <ctime>

namespace libgnss::algorithms::madoca_parity {
namespace {

constexpr double kIonosphereHeightM = 350000.0;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kD2R = kPi / 180.0;
constexpr double kR2D = 180.0 / kPi;

constexpr int kMinPrnGps = 1;
constexpr int kMaxPrnGps = 32;
constexpr int kNsatGps = kMaxPrnGps - kMinPrnGps + 1;

constexpr int kMinPrnGlo = 1;
constexpr int kMaxPrnGlo = 27;
constexpr int kNsatGlo = kMaxPrnGlo - kMinPrnGlo + 1;

constexpr int kMinPrnGal = 1;
constexpr int kMaxPrnGal = 36;
constexpr int kNsatGal = kMaxPrnGal - kMinPrnGal + 1;

constexpr int kMinPrnQzs = 193;
constexpr int kMaxPrnQzs = 202;
constexpr int kNsatQzs = kMaxPrnQzs - kMinPrnQzs + 1;

constexpr int kMinPrnCmp = 1;
constexpr int kMaxPrnCmp = 63;
constexpr int kMinPrnBds3 = 19;
constexpr int kNsatCmp = kMaxPrnCmp - kMinPrnCmp + 1;

constexpr int kMinPrnIrn = 1;
constexpr int kMaxPrnIrn = 14;
constexpr int kNsatIrn = kMaxPrnIrn - kMinPrnIrn + 1;

constexpr int kNsatLeo = 0;

constexpr int kMinPrnSbs = 120;
constexpr int kMaxPrnSbs = 158;
constexpr int kNsatSbs = kMaxPrnSbs - kMinPrnSbs + 1;

constexpr double kMuGps = 3.9860050E14;
constexpr double kMuGal = 3.986004418E14;
constexpr double kMuCmp = 3.986004418E14;
constexpr double kOmgeGal = 7.2921151467E-5;
constexpr double kOmgeCmp = 7.292115E-5;
constexpr double kSinNeg5 = -0.0871557427476582;
constexpr double kCosNeg5 = 0.9961946980917456;
constexpr double kRtolKepler = 1E-13;
constexpr int kMaxIterKepler = 30;
constexpr double kStdGalNapa = 500.0;

const double kGpsTime0[] = {1980, 1, 6, 0, 0, 0};

double sqr(double x) {
    return x * x;
}

double dot(const double* a, const double* b, int n) {
    double c = 0.0;

    while (--n >= 0) {
        c += a[n] * b[n];
    }
    return c;
}

double norm3(const double v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

GTime epoch2time(const double* ep) {
    const int doy[] = {1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
    GTime time = {};
    int days;
    int sec;
    const int year = static_cast<int>(ep[0]);
    const int mon = static_cast<int>(ep[1]);
    const int day = static_cast<int>(ep[2]);

    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
        return time;
    }

    days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 +
           (year % 4 == 0 && mon >= 3 ? 1 : 0);
    sec = static_cast<int>(std::floor(ep[5]));
    time.time = static_cast<std::int64_t>(days) * 86400 +
                static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 +
                sec;
    time.sec = ep[5] - sec;
    return time;
}

void time2epoch(GTime t, double* ep) {
    const int mday[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };
    int days;
    int sec;
    int mon;
    int day;

    days = static_cast<int>(t.time / 86400);
    sec = static_cast<int>(t.time - static_cast<std::int64_t>(days) * 86400);
    for (day = days % 1461, mon = 0; mon < 48; mon++) {
        if (day >= mday[mon]) {
            day -= mday[mon];
        } else {
            break;
        }
    }
    ep[0] = 1970 + days / 1461 * 4 + mon / 12;
    ep[1] = mon % 12 + 1;
    ep[2] = day + 1;
    ep[3] = sec / 3600;
    ep[4] = sec % 3600 / 60;
    ep[5] = sec % 60 + t.sec;
}

double timediff(GTime t1, GTime t2) {
    return std::difftime(static_cast<std::time_t>(t1.time),
                         static_cast<std::time_t>(t2.time)) +
           t1.sec - t2.sec;
}

GTime gpst2time(int week, double sec) {
    GTime t = epoch2time(kGpsTime0);

    if (sec < -1E9 || 1E9 < sec) {
        sec = 0.0;
    }
    t.time += static_cast<std::int64_t>(86400) * 7 * week + static_cast<int>(sec);
    t.sec = sec - static_cast<int>(sec);
    return t;
}

double time2gpst(GTime t, int* week) {
    GTime t0 = epoch2time(kGpsTime0);
    std::int64_t sec = t.time - t0.time;
    int w = static_cast<int>(sec / (86400 * 7));

    if (week != nullptr) {
        *week = w;
    }
    return static_cast<double>(sec - static_cast<std::int64_t>(w) * 86400 * 7) +
           t.sec;
}

GTime timeadd(GTime t, double sec) {
    double tt;

    t.sec += sec;
    tt = std::floor(t.sec);
    t.time += static_cast<int>(tt);
    t.sec -= tt;
    return t;
}

double time2doy(GTime t) {
    double ep[6];

    time2epoch(t, ep);
    ep[1] = ep[2] = 1.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}

double interpc(const double coef[], double lat) {
    int i = static_cast<int>(lat / 15.0);
    if (i < 1) {
        return coef[0];
    } else if (i > 4) {
        return coef[4];
    }
    return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}

double mapf(double el, double a, double b, double c) {
    double sinel = std::sin(el);
    return (1.0 + a / (1.0 + b / (1.0 + c))) /
           (sinel + (a / (sinel + b / (sinel + c))));
}

double nmf(GTime time, const double pos[], const double azel[], double* mapfw) {
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

    double y;
    double cosy;
    double ah[3];
    double aw[3];
    double dm;
    double el = azel[1];
    double lat = pos[0] * kR2D;
    double hgt = pos[2];
    int i;

    if (el <= 0.0) {
        if (mapfw != nullptr) {
            *mapfw = 0.0;
        }
        return 0.0;
    }
    y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);

    cosy = std::cos(2.0 * kPi * y);
    lat = std::fabs(lat);

    for (i = 0; i < 3; i++) {
        ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
        aw[i] = interpc(coef[i + 6], lat);
    }
    dm = (1.0 / std::sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

    if (mapfw != nullptr) {
        *mapfw = mapf(el, aw[0], aw[1], aw[2]);
    }

    return mapf(el, ah[0], ah[1], ah[2]) + dm;
}

double interpvar(double ang, const double* var) {
    double a = ang / 5.0;
    int i = static_cast<int>(a);
    if (i < 0) {
        return var[0];
    } else if (i >= 18) {
        return var[18];
    }
    return var[i] * (1.0 - a + i) + var[i + 1] * (a - i);
}

double var_uraeph(int sys, int ura) {
    const double ura_value[] = {
        2.4,   3.4,    4.85,   6.85,   9.65,   13.65, 24.0, 48.0,
        96.0,  192.0,  384.0,  768.0,  1536.0, 3072.0, 6144.0,
    };
    if (sys == kSysGal) {
        if (ura <= 49) {
            return sqr(ura * 0.01);
        }
        if (ura <= 74) {
            return sqr(0.5 + (ura - 50) * 0.02);
        }
        if (ura <= 99) {
            return sqr(1.0 + (ura - 75) * 0.04);
        }
        if (ura <= 125) {
            return sqr(2.0 + (ura - 100) * 0.16);
        }
        return sqr(kStdGalNapa);
    }
    return ura < 0 || 14 < ura ? sqr(6144.0) : sqr(ura_value[ura]);
}

}  // namespace

bool satnoAvailable() {
    return true;
}

bool satsysAvailable() {
    return true;
}

bool ionmapfAvailable() {
    return true;
}

bool geodistAvailable() {
    return true;
}

bool tropmodelAvailable() {
    return true;
}

bool tropmapfAvailable() {
    return true;
}

bool antmodelAvailable() {
    return true;
}

bool antmodelSAvailable() {
    return true;
}

bool eph2clkAvailable() {
    return true;
}

bool eph2posAvailable() {
    return true;
}

int satno(int sys, int prn) {
    if (prn <= 0) {
        return 0;
    }

    switch (sys) {
        case kSysGps:
            if (prn < kMinPrnGps || kMaxPrnGps < prn) {
                return 0;
            }
            return prn - kMinPrnGps + 1;
        case kSysGlo:
            if (prn < kMinPrnGlo || kMaxPrnGlo < prn) {
                return 0;
            }
            return kNsatGps + prn - kMinPrnGlo + 1;
        case kSysGal:
            if (prn < kMinPrnGal || kMaxPrnGal < prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + prn - kMinPrnGal + 1;
        case kSysQzs:
            if (prn < kMinPrnQzs || kMaxPrnQzs < prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + kNsatGal + prn - kMinPrnQzs + 1;
        case kSysCmp:
            if (prn < kMinPrnCmp || kMaxPrnCmp < prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + prn - kMinPrnCmp + 1;
        case kSysBd2:
            if (prn < kMinPrnCmp || kMinPrnBds3 <= prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + prn - kMinPrnCmp + 1;
        case kSysIrn:
            if (prn < kMinPrnIrn || kMaxPrnIrn < prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + kNsatCmp +
                   prn - kMinPrnIrn + 1;
        case kSysLeo:
            return 0;
        case kSysSbs:
            if (prn < kMinPrnSbs || kMaxPrnSbs < prn) {
                return 0;
            }
            return kNsatGps + kNsatGlo + kNsatGal + kNsatQzs + kNsatCmp +
                   kNsatIrn + kNsatLeo + prn - kMinPrnSbs + 1;
        default:
            return 0;
    }
}

int satsys(int sat, int* prn) {
    int sys = kSysNone;
    if (sat <= 0 || kMadocalibMaxSat < sat) {
        sat = 0;
    } else if (sat <= kNsatGps) {
        sys = kSysGps;
        sat += kMinPrnGps - 1;
    } else if ((sat -= kNsatGps) <= kNsatGlo) {
        sys = kSysGlo;
        sat += kMinPrnGlo - 1;
    } else if ((sat -= kNsatGlo) <= kNsatGal) {
        sys = kSysGal;
        sat += kMinPrnGal - 1;
    } else if ((sat -= kNsatGal) <= kNsatQzs) {
        sys = kSysQzs;
        sat += kMinPrnQzs - 1;
    } else if ((sat -= kNsatQzs) <= kNsatCmp) {
        sys = kSysCmp;
        sat += kMinPrnCmp - 1;
    } else if ((sat -= kNsatCmp) <= kNsatIrn) {
        sys = kSysIrn;
        sat += kMinPrnIrn - 1;
    } else if ((sat -= kNsatIrn) <= kNsatLeo) {
        sys = kSysLeo;
        sat += 0;
    } else if ((sat -= kNsatLeo) <= kNsatSbs) {
        sys = kSysSbs;
        sat += kMinPrnSbs - 1;
    } else {
        sat = 0;
    }

    if (prn != nullptr) {
        *prn = sat;
    }
    return sys;
}

double ionmapf(const double pos[3], const double azel[2]) {
    if (pos[2] >= kIonosphereHeightM) {
        return 1.0;
    }
    return 1.0 /
           std::cos(std::asin((constants::WGS84_A + pos[2]) /
                              (constants::WGS84_A + kIonosphereHeightM) *
                              std::sin(kPi / 2.0 - azel[1])));
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
    return r + constants::OMEGA_E * (rs[0] * rr[1] - rs[1] * rr[0]) /
                   constants::SPEED_OF_LIGHT;
}

double tropmodel(GTime time, const double pos[3], const double azel[2], double humi) {
    (void)time;
    const double temp0 = 15.0;
    double hgt;
    double pres;
    double temp;
    double e;
    double z;
    double trph;
    double trpw;

    if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) {
        return 0.0;
    }

    hgt = pos[2] < 0.0 ? 0.0 : pos[2];

    pres = 1013.25 * std::pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    temp = temp0 - 6.5E-3 * hgt + 273.16;
    e = 6.108 * humi * std::exp((17.15 * temp - 4684.0) / (temp - 38.45));

    z = kPi / 2.0 - azel[1];
    trph = 0.0022768 * pres /
           (1.0 - 0.00266 * std::cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) /
           std::cos(z);
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / std::cos(z);
    return trph + trpw;
}

double tropmapf(GTime time, const double pos[3], const double azel[2], double* mapfw) {
    if (pos[2] < -1000.0 || pos[2] > 20000.0) {
        if (mapfw != nullptr) {
            *mapfw = 0.0;
        }
        return 0.0;
    }
    return nmf(time, pos, azel, mapfw);
}

void antmodel(const AntennaPcv* pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[kMadocalibNFreqPcv]) {
    double e[3];
    double off[3];
    double cosel = std::cos(azel[1]);
    int i;
    int j;

    e[0] = std::sin(azel[0]) * cosel;
    e[1] = std::cos(azel[0]) * cosel;
    e[2] = std::sin(azel[1]);

    for (i = 0; i < kMadocalibNFreqPcv; i++) {
        for (j = 0; j < 3; j++) {
            off[j] = pcv->off[i][j] + del[j];
        }

        dant[i] = -dot(off, e, 3) +
                  (opt ? interpvar(90.0 - azel[1] * kR2D, pcv->var[i]) : 0.0);
    }
}

void antmodel_s(const AntennaPcv* pcv,
                double nadir,
                double dant[kMadocalibNFreqPcv]) {
    int i;

    for (i = 0; i < kMadocalibNFreqPcv; i++) {
        dant[i] = interpvar(nadir * kR2D * 5.0, pcv->var[i]);
    }
}

double eph2clk(GTime time, const BroadcastEphemeris* eph) {
    double t;
    double ts;
    int i;

    t = ts = timediff(time, eph->toc);

    for (i = 0; i < 2; i++) {
        t = ts - (eph->f0 + eph->f1 * t + eph->f2 * t * t);
    }
    return eph->f0 + eph->f1 * t + eph->f2 * t * t;
}

void eph2pos(GTime time,
             const BroadcastEphemeris* eph,
             double rs[3],
             double* dts,
             double* var) {
    double tk;
    double M;
    double E;
    double Ek;
    double sinE;
    double cosE;
    double u;
    double r;
    double i;
    double O;
    double sin2u;
    double cos2u;
    double x;
    double y;
    double sinO;
    double cosO;
    double cosi;
    double mu;
    double omge;
    double xg;
    double yg;
    double zg;
    double sino;
    double coso;
    int n;
    int sys;
    int prn;

    if (eph->A <= 0.0) {
        rs[0] = rs[1] = rs[2] = *dts = *var = 0.0;
        return;
    }
    tk = timediff(time, eph->toe);

    switch ((sys = satsys(eph->sat, &prn))) {
        case kSysGal:
            mu = kMuGal;
            omge = kOmgeGal;
            break;
        case kSysCmp:
            mu = kMuCmp;
            omge = kOmgeCmp;
            break;
        default:
            mu = kMuGps;
            omge = constants::OMEGA_E;
            break;
    }
    M = eph->M0 + (std::sqrt(mu / (eph->A * eph->A * eph->A)) + eph->deln) * tk;

    for (n = 0, E = M, Ek = 0.0;
         std::fabs(E - Ek) > kRtolKepler && n < kMaxIterKepler;
         n++) {
        Ek = E;
        E -= (E - eph->e * std::sin(E) - M) / (1.0 - eph->e * std::cos(E));
    }
    if (n >= kMaxIterKepler) {
        return;
    }
    sinE = std::sin(E);
    cosE = std::cos(E);

    u = std::atan2(std::sqrt(1.0 - eph->e * eph->e) * sinE, cosE - eph->e) +
        eph->omg;
    r = eph->A * (1.0 - eph->e * cosE);
    i = eph->i0 + eph->idot * tk;
    sin2u = std::sin(2.0 * u);
    cos2u = std::cos(2.0 * u);
    u += eph->cus * sin2u + eph->cuc * cos2u;
    r += eph->crs * sin2u + eph->crc * cos2u;
    i += eph->cis * sin2u + eph->cic * cos2u;
    x = r * std::cos(u);
    y = r * std::sin(u);
    cosi = std::cos(i);

    if (sys == kSysCmp && (prn <= 5 || prn >= 59)) {
        O = eph->OMG0 + eph->OMGd * tk - omge * eph->toes;
        sinO = std::sin(O);
        cosO = std::cos(O);
        xg = x * cosO - y * cosi * sinO;
        yg = x * sinO + y * cosi * cosO;
        zg = y * std::sin(i);
        sino = std::sin(omge * tk);
        coso = std::cos(omge * tk);
        rs[0] = xg * coso + yg * sino * kCosNeg5 + zg * sino * kSinNeg5;
        rs[1] = -xg * sino + yg * coso * kCosNeg5 + zg * coso * kSinNeg5;
        rs[2] = -yg * kSinNeg5 + zg * kCosNeg5;
    } else {
        O = eph->OMG0 + (eph->OMGd - omge) * tk - omge * eph->toes;
        sinO = std::sin(O);
        cosO = std::cos(O);
        rs[0] = x * cosO - y * cosi * sinO;
        rs[1] = x * sinO + y * cosi * cosO;
        rs[2] = y * std::sin(i);
    }
    tk = timediff(time, eph->toc);
    *dts = eph->f0 + eph->f1 * tk + eph->f2 * tk * tk;

    *dts -= 2.0 * std::sqrt(mu * eph->A) * eph->e * sinE /
            sqr(constants::SPEED_OF_LIGHT);

    *var = var_uraeph(sys, eph->sva);
}

}  // namespace libgnss::algorithms::madoca_parity
