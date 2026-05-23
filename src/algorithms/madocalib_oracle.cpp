#include <libgnss++/external/madocalib_oracle.hpp>

#ifndef GNSSPP_HAS_MADOCALIB_ORACLE
#define GNSSPP_HAS_MADOCALIB_ORACLE 0
#endif

#if GNSSPP_HAS_MADOCALIB_ORACLE
extern "C" {
#include "rtklib.h"
}
#endif

namespace libgnss::external::madocalib_oracle {
namespace {

void zero3(double out[3]) {
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
}

}  // namespace

bool available() {
    return GNSSPP_HAS_MADOCALIB_ORACLE != 0;
}

int satno(int sys, int prn) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::satno(sys, prn);
#else
    (void)sys;
    (void)prn;
    return 0;
#endif
}

int satsys(int sat, int* prn) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::satsys(sat, prn);
#else
    (void)sat;
    if (prn != nullptr) {
        *prn = 0;
    }
    return 0;
#endif
}

double ionmapf(const double pos[3], const double azel[2]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::ionmapf(pos, azel);
#else
    (void)pos;
    (void)azel;
    return 0.0;
#endif
}

double geodist(const double rs[3], const double rr[3], double e[3]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::geodist(rs, rr, e);
#else
    (void)rs;
    (void)rr;
    zero3(e);
    return -1.0;
#endif
}

}  // namespace libgnss::external::madocalib_oracle
