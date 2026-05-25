#include <libgnss++/external/madocalib_oracle.hpp>

#include <cstdio>

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

int mcssrSelBiascode(int sys, int code) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::mcssr_sel_biascode(sys, code);
#else
    (void)sys;
    (void)code;
    return 0;
#endif
}

std::uint32_t getbitu(const std::uint8_t* buff, int pos, int len) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::getbitu(buff, pos, len);
#else
    (void)buff;
    (void)pos;
    (void)len;
    return 0;
#endif
}

std::int32_t getbits(const std::uint8_t* buff, int pos, int len) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::getbits(buff, pos, len);
#else
    (void)buff;
    (void)pos;
    (void)len;
    return 0;
#endif
}

int decodeQzssL6eFile(const char* path, libgnss::io::MadocaSsrCorrection* out,
                      int maxSat) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    using Correction = libgnss::io::MadocaSsrCorrection;
    FILE* fp = std::fopen(path, "rb");
    if (fp == nullptr) {
        return -1;
    }
    // rtcm_t is ~126 MB (embeds large obs/nav/ssr arrays); allocate on the heap.
    rtcm_t* rtcm = new rtcm_t{};
    init_rtcm(rtcm);
    double ep[6] = {2025.0, 4.0, 1.0, 0.0, 0.0, 0.0};
    init_mcssr(epoch2time(ep));  // GPS week determination only
    while (input_qzssl6ef(rtcm, fp) != -2) {
        // Corrections accumulate into rtcm->ssr until end of file.
    }
    const int n = (maxSat < MAXSAT) ? maxSat : MAXSAT;
    int count = 0;
    for (int s = 0; s < n; ++s) {
        const ssr_t& g = rtcm->ssr[s];
        Correction& o = out[s];
        o.iode = g.iode;
        o.ura = g.ura;
        for (int k = 0; k < 6; ++k) {
            o.iod[k] = g.iod[k];
            o.udi[k] = g.udi[k];
        }
        for (int k = 0; k < 3; ++k) {
            o.deph[k] = g.deph[k];
            o.ddeph[k] = g.ddeph[k];
            o.dclk[k] = g.dclk[k];
        }
        for (int k = 0; k < Correction::kMaxCode; ++k) {
            o.cbias[k] = g.cbias[k];
            o.vcbias[k] = g.vcbias[k];
            o.pbias[k] = g.pbias[k];
            o.vpbias[k] = g.vpbias[k];
            o.discnt[k] = g.discnt[k];
        }
        o.yaw_ang = g.yaw_ang;
        o.yaw_rate = g.yaw_rate;
        o.update = g.update;
        if (g.update) {
            ++count;
        }
    }
    free_rtcm(rtcm);
    delete rtcm;
    std::fclose(fp);
    return count;
#else
    (void)path;
    (void)out;
    (void)maxSat;
    return -1;
#endif
}

}  // namespace libgnss::external::madocalib_oracle
