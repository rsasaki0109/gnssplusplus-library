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
            o.t0[k].time = static_cast<long long>(g.t0[k].time);
            o.t0[k].sec = g.t0[k].sec;
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

#if GNSSPP_HAS_MADOCALIB_ORACLE
namespace {

// Mirror postpos.c init_mdcl6d()/update_qzssl6d() reset of the scratch region.
void clearMdcl6dRegion(mdcl6d_t* m) {
    m->re.rvalid = 0;
    for (int a = 0; a < MIONO_MAX_ANUM; ++a) {
        m->re.area[a].avalid = 0;
        for (int s = 0; s < MAXSAT; ++s) {
            m->re.area[a].sat[s].t0.time = 0;
        }
    }
}

}  // namespace
#endif

void* l6dCreate(const double ep[6]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    mdcl6d_t* m = new mdcl6d_t{};
    m->nbyte = 0;
    m->opt[0] = '\0';
    clearMdcl6dRegion(m);  // mirror init_mdcl6d()
    double e[6] = {ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]};
    init_miono(epoch2time(e));  // GPS week determination only
    return m;
#else
    (void)ep;
    return nullptr;
#endif
}

int l6dInputByte(void* handle, std::uint8_t data) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (handle == nullptr) {
        return 0;
    }
    return ::input_qzssl6d(reinterpret_cast<mdcl6d_t*>(handle), data);
#else
    (void)handle;
    (void)data;
    return 0;
#endif
}

void l6dRegion(void* handle, libgnss::io::MadocaIonoRegion* out, int* rid) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (handle == nullptr || out == nullptr) {
        return;
    }
    using Region = libgnss::io::MadocaIonoRegion;
    using Area = libgnss::io::MadocaIonoArea;
    const mdcl6d_t* m = reinterpret_cast<const mdcl6d_t*>(handle);
    const miono_region_t& re = m->re;
    out->rvalid = re.rvalid;
    out->ralert = re.ralert;
    out->narea = re.narea;
    const int na = (MIONO_MAX_ANUM < Region::kMaxArea) ? MIONO_MAX_ANUM : Region::kMaxArea;
    for (int a = 0; a < na; ++a) {
        const miono_area_t& ga = re.area[a];
        Area& oa = out->area[a];
        oa.avalid = ga.avalid;
        oa.sid = ga.sid;
        oa.type = ga.type;
        oa.ref[0] = ga.ref[0];
        oa.ref[1] = ga.ref[1];
        oa.span[0] = ga.span[0];
        oa.span[1] = ga.span[1];
        const int nsat = (MAXSAT < Area::kMaxSat) ? MAXSAT : Area::kMaxSat;
        for (int s = 0; s < nsat; ++s) {
            oa.sat[s].t0.time = static_cast<std::int64_t>(ga.sat[s].t0.time);
            oa.sat[s].t0.sec = ga.sat[s].t0.sec;
            oa.sat[s].sqi = ga.sat[s].sqi;
            for (int k = 0; k < 6; ++k) {
                oa.sat[s].coef[k] = ga.sat[s].coef[k];
            }
        }
    }
    if (rid != nullptr) {
        *rid = m->rid;
    }
#else
    (void)handle;
    (void)out;
    (void)rid;
#endif
}

void l6dClearRegion(void* handle) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (handle != nullptr) {
        clearMdcl6dRegion(reinterpret_cast<mdcl6d_t*>(handle));
    }
#else
    (void)handle;
#endif
}

void l6dDestroy(void* handle) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    delete reinterpret_cast<mdcl6d_t*>(handle);
#else
    (void)handle;
#endif
}

#if GNSSPP_HAS_MADOCALIB_ORACLE
namespace {

// Decoder control struct plus the persistent navigation store. nav_t embeds the
// 256-slot pppiono region table (~131 MB), so the pair is heap-allocated.
struct L6dApp {
    mdcl6d_t mdcl6d{};
    nav_t nav{};
};

}  // namespace
#endif

void* l6dAppCreate(const double ep[6]) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    L6dApp* app = new L6dApp{};
    app->mdcl6d.nbyte = 0;
    app->mdcl6d.opt[0] = '\0';
    clearMdcl6dRegion(&app->mdcl6d);  // mirror init_mdcl6d()
    double e[6] = {ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]};
    init_miono(epoch2time(e));  // GPS week determination only
    return app;
#else
    (void)ep;
    return nullptr;
#endif
}

int l6dAppInputByte(void* handle, std::uint8_t data) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (handle == nullptr) {
        return 0;
    }
    L6dApp* app = reinterpret_cast<L6dApp*>(handle);
    const int ret = ::input_qzssl6d(&app->mdcl6d, data);
    if (ret == 10 && app->mdcl6d.re.rvalid) {
        // Mirror postpos.c update_qzssl6d: persist the region then reset scratch.
        app->nav.pppiono.re[app->mdcl6d.rid] = app->mdcl6d.re;
        app->mdcl6d.re.rvalid = 0;
        for (int a = 0; a < MIONO_MAX_ANUM; ++a) {
            app->mdcl6d.re.area[a].avalid = 0;
            for (int s = 0; s < MAXSAT; ++s) {
                app->mdcl6d.re.area[a].sat[s].t0.time = 0;
            }
        }
    }
    return ret;
#else
    (void)handle;
    (void)data;
    return 0;
#endif
}

int l6dAppGetCorr(void* handle, const double rr[3],
                  libgnss::io::MadocaIonoCorr* out) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    if (handle == nullptr || out == nullptr) {
        return 0;
    }
    L6dApp* app = reinterpret_cast<L6dApp*>(handle);
    const int ret = ::miono_get_corr(rr, &app->nav);
    out->rid = app->nav.pppiono.rid;
    out->anum = app->nav.pppiono.anum;
    const int n = (MAXSAT < libgnss::io::MadocaIonoCorr::kMaxSat)
                      ? MAXSAT
                      : libgnss::io::MadocaIonoCorr::kMaxSat;
    for (int i = 0; i < n; ++i) {
        out->t0[i].time =
            static_cast<std::int64_t>(app->nav.pppiono.corr.t0[i].time);
        out->t0[i].sec = app->nav.pppiono.corr.t0[i].sec;
        out->dly[i] = app->nav.pppiono.corr.dly[i];
        out->std[i] = app->nav.pppiono.corr.std[i];
    }
    return ret;
#else
    (void)handle;
    (void)rr;
    (void)out;
    return 0;
#endif
}

void l6dAppDestroy(void* handle) {
#if GNSSPP_HAS_MADOCALIB_ORACLE
    delete reinterpret_cast<L6dApp*>(handle);
#else
    (void)handle;
#endif
}

}  // namespace libgnss::external::madocalib_oracle
