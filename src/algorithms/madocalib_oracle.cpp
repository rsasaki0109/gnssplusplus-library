#include <libgnss++/external/madocalib_oracle.hpp>

#include <memory>
#include <vector>

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

GTime fromOracleTime(gtime_t time) {
    GTime out = {};
    out.time = static_cast<decltype(out.time)>(time.time);
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

int mcssr_sel_biascode(int sys, int code) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    return ::mcssr_sel_biascode(sys, code);
#else
    (void)sys;
    (void)code;
    return 0;
#endif
}

int miono_get_corr(GTime time,
                   const double rr[3],
                   const MionoAreaFixture* areas,
                   int area_count,
                   MionoCorrResult* result) {
    if (result != nullptr) {
        *result = MionoCorrResult{};
    }
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (rr == nullptr || areas == nullptr || area_count <= 0 || result == nullptr) {
        return 0;
    }

    ::init_miono(toOracleTime(time));
    auto nav = std::make_unique<nav_t>();

    for (int i = 0; i < area_count; ++i) {
        const MionoAreaFixture& src = areas[i];
        if (src.region_id < 0 || MIONO_MAX_RID <= src.region_id ||
            src.area_number < 0 || MIONO_MAX_ANUM <= src.area_number) {
            continue;
        }
        miono_region_t& region = nav->pppiono.re[src.region_id];
        miono_area_t& area = region.area[src.area_number];

        region.rvalid = src.rvalid;
        region.ralert = src.ralert;
        region.narea = src.area_number + 1 > region.narea ? src.area_number + 1
                                                           : region.narea;
        area.avalid = src.avalid;
        area.sid = src.sid;
        area.type = src.type;
        for (int j = 0; j < 2; ++j) {
            area.ref[j] = src.ref[j];
            area.span[j] = src.span[j];
        }
        for (int sat = 0; sat < MAXSAT && sat < libgnss::algorithms::madoca_parity::kMadocalibMaxSat;
             ++sat) {
            area.sat[sat].t0 = toOracleTime(src.sat[sat].t0);
            area.sat[sat].sqi = src.sat[sat].sqi;
            for (int k = 0; k < 6; ++k) {
                area.sat[sat].coef[k] = src.sat[sat].coef[k];
            }
        }
    }

    const int status = ::miono_get_corr(rr, nav.get());
    result->rid = nav->pppiono.rid;
    result->area_number = nav->pppiono.anum;
    for (int sat = 0; sat < MAXSAT && sat < libgnss::algorithms::madoca_parity::kMadocalibMaxSat;
         ++sat) {
        result->t0[sat] = fromOracleTime(nav->pppiono.corr.t0[sat]);
        result->delay[sat] = nav->pppiono.corr.dly[sat];
        result->std[sat] = nav->pppiono.corr.std[sat];
    }
    return status;
#else
    (void)time;
    (void)rr;
    (void)areas;
    (void)area_count;
    return 0;
#endif
}

int satpos(GTime time,
           GTime teph,
           int sat,
           int ephopt,
           const BroadcastEphemeris* ephs,
           int eph_count,
           double rs[6],
           double dts[2],
           double* var,
           int* svh) {
    if (rs != nullptr) {
        zeroN(rs, 6);
    }
    if (dts != nullptr) {
        zeroN(dts, 2);
    }
    if (var != nullptr) {
        *var = 0.0;
    }
    if (svh != nullptr) {
        *svh = -1;
    }
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (rs == nullptr || dts == nullptr || var == nullptr || svh == nullptr ||
        ephs == nullptr || eph_count <= 0) {
        return 0;
    }

    auto nav = std::make_unique<nav_t>();
    std::vector<eph_t> c_ephs;
    c_ephs.reserve(static_cast<size_t>(eph_count));
    for (int i = 0; i < eph_count; ++i) {
        c_ephs.push_back(toOracleEph(&ephs[i]));
    }
    nav->eph = c_ephs.data();
    nav->n = static_cast<int>(c_ephs.size());
    nav->nmax = nav->n;

    return ::satpos(toOracleTime(time),
                    toOracleTime(teph),
                    sat,
                    ephopt,
                    nav.get(),
                    rs,
                    dts,
                    var,
                    svh);
#else
    (void)time;
    (void)teph;
    (void)sat;
    (void)ephopt;
    (void)ephs;
    (void)eph_count;
    return 0;
#endif
}

}  // namespace libgnss::external::madocalib_oracle
