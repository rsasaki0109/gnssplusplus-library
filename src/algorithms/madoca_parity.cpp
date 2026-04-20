#include <libgnss++/algorithms/madoca_parity.hpp>

#include <libgnss++/core/constants.hpp>

#include <cmath>

namespace libgnss::algorithms::madoca_parity {
namespace {

constexpr double kIonosphereHeightM = 350000.0;
constexpr double kPi = 3.141592653589793238462643383279502884;

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

double norm3(const double v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
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

}  // namespace libgnss::algorithms::madoca_parity
