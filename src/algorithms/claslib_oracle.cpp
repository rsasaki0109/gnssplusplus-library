#include <libgnss++/external/claslib_oracle.hpp>

#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/external/claslib_bridge.hpp>

#include <algorithm>
#include <cstring>

#ifndef GNSSPP_HAS_CLASLIB_BRIDGE
#define GNSSPP_HAS_CLASLIB_BRIDGE 0
#endif

#if GNSSPP_HAS_CLASLIB_BRIDGE
extern "C" {
#include "rtklib.h"
#include "cssr2osr.h"
}
#endif

namespace libgnss::external::claslib_oracle {
namespace {

#if GNSSPP_HAS_CLASLIB_BRIDGE
gtime_t toClasTime(const GTime& time) {
    return gpst2time(time.week, time.tow);
}

erp_t makeSingleEntryErp(const GTime& time, const double erpv[5], erpd_t& storage) {
    std::memset(&storage, 0, sizeof(storage));
    storage.mjd = tidal::julianDateFromTime(time) - 2400000.5;
    storage.xp = erpv[0];
    storage.yp = erpv[1];
    storage.ut1_utc = erpv[2];
    storage.lod = erpv[3];
    storage.xpr = erpv[4];
    storage.ypr = 0.0;

    erp_t erp = {};
    erp.n = 1;
    erp.nmax = 1;
    erp.data = &storage;
    return erp;
}
#endif

void zero3(double out[3]) {
    out[0] = 0.0;
    out[1] = 0.0;
    out[2] = 0.0;
}

}  // namespace

bool available() {
    return libgnss::external::claslib::isAvailable();
}

void tidedisp(const GTime& gpst,
              const double rr[3],
              const double erpv[5],
              double disp_out[3]) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    erpd_t erp_storage = {};
    erp_t erp = makeSingleEntryErp(gpst, erpv, erp_storage);
    double zero_odisp[6 * 11] = {};
    int index[4] = {};
    double gmat[16] = {};
    double emat[4] = {};
    double weight[4] = {};
    ::tidedisp(gpst2utc(toClasTime(gpst)),
               rr,
               2,
               &erp,
               zero_odisp,
               nullptr,
               0,
               0,
               index,
               gmat,
               emat,
               weight,
               disp_out);
#else
    (void)gpst;
    (void)rr;
    (void)erpv;
    zero3(disp_out);
#endif
}

void windupcorr(const GTime& time,
                const double rs[6],
                const double rr[3],
                double& phw_io) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    double phw = phw_io;
    ::windupcorr(toClasTime(time), rs, rr, &phw);
    phw_io = phw;
#else
    (void)time;
    (void)rs;
    (void)rr;
#endif
}

void antmodel(const ReceiverPcvModel& pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[libgnss::clasnat_parity::kParityMaxFreq]) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    pcv_t clas_pcv = {};
    for (int f = 0; f < std::min(NFREQ, libgnss::clasnat_parity::kParityMaxFreq); ++f) {
        for (int j = 0; j < 3; ++j) {
            clas_pcv.off[f][j] = pcv.offsets_m[static_cast<size_t>(f)][static_cast<size_t>(j)];
        }
        for (int j = 0; j < 19; ++j) {
            clas_pcv.var[f][j] =
                pcv.variations_m[static_cast<size_t>(f)][static_cast<size_t>(j)];
        }
    }
    double full_dant[NFREQ + NEXOBS] = {};
    ::antmodel(&clas_pcv, del, azel, opt, full_dant);
    for (int f = 0; f < libgnss::clasnat_parity::kParityMaxFreq; ++f) {
        dant[f] = full_dant[f];
    }
#else
    (void)pcv;
    (void)del;
    (void)azel;
    (void)opt;
    for (int f = 0; f < libgnss::clasnat_parity::kParityMaxFreq; ++f) {
        dant[f] = 0.0;
    }
#endif
}

double ionmapf(const double pos[3], const double azel[2]) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    return ::ionmapf(pos, azel);
#else
    (void)pos;
    (void)azel;
    return 0.0;
#endif
}

double prectrop(const GTime& time,
                const double pos[3],
                const double azel[2],
                double zwd,
                double ztd) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    prcopt_t opt = prcopt_default;
    opt.nf = NFREQ;
    opt.tropopt = TROPOPT_COR;
    return ::prectrop(toClasTime(time), pos, azel, &opt, zwd, ztd);
#else
    (void)time;
    (void)pos;
    (void)azel;
    (void)zwd;
    (void)ztd;
    return 0.0;
#endif
}

bool satpos_ssr(const GTime& teph,
                const GTime& time,
                int sat,
                SatposSsrOutput& out) {
    (void)teph;
    (void)time;
    (void)sat;
    out = SatposSsrOutput{};
    return false;
}

bool corrmeas(CorrmeasOutput& out) {
    out = CorrmeasOutput{};
    return false;
}

}  // namespace libgnss::external::claslib_oracle
