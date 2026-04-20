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

constexpr int kNFreqPcv = libgnss::algorithms::madoca_parity::kMadocalibNFreqPcv;
constexpr int kPcvAngles = libgnss::algorithms::madoca_parity::kMadocalibPcvAngles;

void zero3(double out[3]) {
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
}

void zeroN(double* out, int n) {
    for (int i = 0; i < n; ++i) {
        out[i] = 0.0;
    }
}

#if GNSSPP_HAS_MADOCALIB_ORACLE
gtime_t toOracleTime(GTime time) {
    gtime_t out = {};
    out.time = static_cast<time_t>(time.time);
    out.sec = time.sec;
    return out;
}

pcv_t toOraclePcv(const AntennaPcv* pcv) {
    pcv_t out = {};
    for (int i = 0; i < kNFreqPcv; ++i) {
        for (int j = 0; j < 3; ++j) {
            out.off[i][j] = pcv->off[i][j];
        }
        for (int j = 0; j < kPcvAngles; ++j) {
            out.var[i][j] = pcv->var[i][j];
        }
    }
    return out;
}

eph_t toOracleEph(const BroadcastEphemeris* eph) {
    eph_t out = {};
    out.sat = eph->sat;
    out.iode = eph->iode;
    out.iodc = eph->iodc;
    out.sva = eph->sva;
    out.svh = eph->svh;
    out.week = eph->week;
    out.code = eph->code;
    out.flag = eph->flag;
    out.toe = toOracleTime(eph->toe);
    out.toc = toOracleTime(eph->toc);
    out.ttr = toOracleTime(eph->ttr);
    out.A = eph->A;
    out.e = eph->e;
    out.i0 = eph->i0;
    out.OMG0 = eph->OMG0;
    out.omg = eph->omg;
    out.M0 = eph->M0;
    out.deln = eph->deln;
    out.OMGd = eph->OMGd;
    out.idot = eph->idot;
    out.crc = eph->crc;
    out.crs = eph->crs;
    out.cuc = eph->cuc;
    out.cus = eph->cus;
    out.cic = eph->cic;
    out.cis = eph->cis;
    out.toes = eph->toes;
    out.fit = eph->fit;
    out.f0 = eph->f0;
    out.f1 = eph->f1;
    out.f2 = eph->f2;
    for (int i = 0; i < 6; ++i) {
        out.tgd[i] = eph->tgd[i];
    }
    out.Adot = eph->Adot;
    out.ndot = eph->ndot;
    return out;
}
#endif

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

double tropmodel(GTime time, const double pos[3], const double azel[2], double humi) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const gtime_t c_time = toOracleTime(time);
    return ::tropmodel(c_time, pos, azel, humi);
#else
    (void)time;
    (void)pos;
    (void)azel;
    (void)humi;
    return 0.0;
#endif
}

double tropmapf(GTime time, const double pos[3], const double azel[2], double* mapfw) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const gtime_t c_time = toOracleTime(time);
    return ::tropmapf(c_time, pos, azel, mapfw);
#else
    (void)time;
    (void)pos;
    (void)azel;
    if (mapfw != nullptr) {
        *mapfw = 0.0;
    }
    return 0.0;
#endif
}

void antmodel(const AntennaPcv* pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[kNFreqPcv]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const pcv_t c_pcv = toOraclePcv(pcv);
    ::antmodel(&c_pcv, del, azel, opt, dant);
#else
    (void)pcv;
    (void)del;
    (void)azel;
    (void)opt;
    zeroN(dant, kNFreqPcv);
#endif
}

void antmodel_s(const AntennaPcv* pcv, double nadir, double dant[kNFreqPcv]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const pcv_t c_pcv = toOraclePcv(pcv);
    ::antmodel_s(&c_pcv, nadir, dant);
#else
    (void)pcv;
    (void)nadir;
    zeroN(dant, kNFreqPcv);
#endif
}

double eph2clk(GTime time, const BroadcastEphemeris* eph) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const gtime_t c_time = toOracleTime(time);
    const eph_t c_eph = toOracleEph(eph);
    return ::eph2clk(c_time, &c_eph);
#else
    (void)time;
    (void)eph;
    return 0.0;
#endif
}

void eph2pos(GTime time,
             const BroadcastEphemeris* eph,
             double rs[3],
             double* dts,
             double* var) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    const gtime_t c_time = toOracleTime(time);
    const eph_t c_eph = toOracleEph(eph);
    ::eph2pos(c_time, &c_eph, rs, dts, var);
#else
    (void)time;
    (void)eph;
    zero3(rs);
    if (dts != nullptr) {
        *dts = 0.0;
    }
    if (var != nullptr) {
        *var = 0.0;
    }
#endif
}

}  // namespace libgnss::external::madocalib_oracle
