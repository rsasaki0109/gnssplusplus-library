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

bool mcssrSelBiascodeAvailable() {
    return true;
}

bool getbituAvailable() {
    return true;
}

bool getbitsAvailable() {
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

int mcssrSelBiascode(int sys, int code) {
    switch (sys) {
        case kSysGps:
            switch (code) {
                case kCodeL1C: return kCodeL1C;  // L1C/A
                case kCodeL1P:                   // L1P
                case kCodeL1W: return kCodeL1W;  // L1 Z-tracking
                case kCodeL1S:                   // L1C(D)
                case kCodeL1L:                   // L1C(P)
                case kCodeL1X: return kCodeL1X;  // L1C(D+P)
                case kCodeL2S:                   // L2C(M)
                case kCodeL2L:                   // L2C(L)
                case kCodeL2X: return kCodeL2X;  // L2C(M+L)
                case kCodeL2P:                   // L2P
                case kCodeL2W: return kCodeL2W;  // L2 Z-tracking
                case kCodeL5I: return kCodeL5I;  // L5I
                case kCodeL5Q: return kCodeL5Q;  // L5Q
                case kCodeL5X: return kCodeL5X;  // L5X
            }
            break;
        case kSysGlo:
            switch (code) {
                case kCodeL1C:                   // G1C/A
                case kCodeL1P: return kCodeL1C;  // G1P
                case kCodeL2C:                   // G2C/A
                case kCodeL2P: return kCodeL2C;  // G2P
            }
            break;
        case kSysGal:
            switch (code) {
                case kCodeL1C:                   // E1C
                case kCodeL1B:                   // E1B
                case kCodeL1X: return kCodeL1X;  // E1B+C
                case kCodeL5I:                   // E5aI
                case kCodeL5Q:                   // E5aQ
                case kCodeL5X: return kCodeL5X;  // E5aI+Q
                case kCodeL7I:                   // E5bI
                case kCodeL7Q:                   // E5bQ
                case kCodeL7X: return kCodeL7X;  // E5bI+Q
                case kCodeL6B:                   // E6B
                case kCodeL6C:                   // E6C
                case kCodeL6X: return kCodeL6B;  // E6B+C
            }
            break;
        case kSysQzs:
            switch (code) {
                case kCodeL1C: return kCodeL1C;  // L1C/A
                case kCodeL1S:                   // L1C(D)
                case kCodeL1L:                   // L1C(P)
                case kCodeL1X: return kCodeL1X;  // L1C(D+P)
                case kCodeL2S:                   // L2C(M)
                case kCodeL2L:                   // L2C(L)
                case kCodeL2X: return kCodeL2X;  // L2C(M+L)
                case kCodeL5I:                   // L5I
                case kCodeL5Q:                   // L5Q
                case kCodeL5X: return kCodeL5X;  // L5X
                case kCodeL1E: return kCodeL1E;  // L1C/B
            }
            break;
        case kSysCmp:
            switch (code) {
                case kCodeL2I:                   // B1I
                case kCodeL2Q:                   // B1Q
                case kCodeL2X: return kCodeL2I;  // B1I+Q
                case kCodeL6I:                   // B3I
                case kCodeL6Q:                   // B3Q
                case kCodeL6X: return kCodeL6I;  // B3I+Q
                case kCodeL7I:                   // B2I
                case kCodeL7Q:                   // B2Q
                case kCodeL7X: return kCodeL7I;  // B2I+Q
                case kCodeL5D: return kCodeL5D;  // B2aI
                case kCodeL5P: return kCodeL5P;  // B2aQ
                case kCodeL5X: return kCodeL5X;  // B2aI+Q
                case kCodeL1D: return kCodeL1D;  // B1C(D)
                case kCodeL1P: return kCodeL1P;  // B1C(P)
                case kCodeL1X: return kCodeL1X;  // B1C(D+P)
                case kCodeL7D: return kCodeL7D;  // B2b
            }
            break;
    }
    return kCodeNone;
}

std::uint32_t getbitu(const std::uint8_t* buff, int pos, int len) {
    std::uint32_t bits = 0;
    for (int i = pos; i < pos + len; ++i) {
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    }
    return bits;
}

std::int32_t getbits(const std::uint8_t* buff, int pos, int len) {
    const std::uint32_t bits = getbitu(buff, pos, len);
    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) {
        return static_cast<std::int32_t>(bits);
    }
    return static_cast<std::int32_t>(bits | (~0u << len));  // extend sign
}

}  // namespace libgnss::algorithms::madoca_parity
