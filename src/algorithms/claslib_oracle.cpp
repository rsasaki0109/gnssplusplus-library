#include <libgnss++/external/claslib_oracle.hpp>

#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/external/claslib_bridge.hpp>

#include <algorithm>
#include <array>
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

#if GNSSPP_HAS_CLASLIB_BRIDGE
void fillClasEph(const libgnss::clasnat_parity::SatposBroadcastEphemeris& input,
                 eph_t& eph) {
    std::memset(&eph, 0, sizeof(eph));
    eph.sat = input.sat;
    eph.iode = input.iode;
    eph.iodc = input.iodc;
    eph.sva = input.sva;
    eph.svh = input.svh;
    eph.week = input.week;
    eph.code = input.code;
    eph.flag = input.flag;
    eph.toe = toClasTime(input.toe);
    eph.toc = toClasTime(input.toc);
    eph.ttr = toClasTime(input.ttr);
    eph.A = input.A;
    eph.e = input.e;
    eph.i0 = input.i0;
    eph.OMG0 = input.OMG0;
    eph.omg = input.omg;
    eph.M0 = input.M0;
    eph.deln = input.deln;
    eph.OMGd = input.OMGd;
    eph.idot = input.idot;
    eph.crc = input.crc;
    eph.crs = input.crs;
    eph.cuc = input.cuc;
    eph.cus = input.cus;
    eph.cic = input.cic;
    eph.cis = input.cis;
    eph.toes = input.toes;
    eph.fit = input.fit;
    eph.f0 = input.f0;
    eph.f1 = input.f1;
    eph.f2 = input.f2;
    for (int i = 0; i < 4; ++i) {
        eph.tgd[i] = input.tgd[i];
    }
    eph.Adot = input.Adot;
    eph.ndot = input.ndot;
}

void fillClasSsr(const libgnss::clasnat_parity::SatposSsrCorrection& input,
                 ssr_t& ssr) {
    std::memset(&ssr, 0, sizeof(ssr));
    constexpr int kInputSsrCount = 9;
    for (int i = 0; i < std::min(MAX_INDEX_SSR, kInputSsrCount); ++i) {
        ssr.t0[i] = toClasTime(input.t0[i]);
        ssr.udi[i] = input.udi[i];
        ssr.iod[i] = input.iod[i];
    }
    ssr.iode = input.iode;
    ssr.iodcrc = input.iodcrc;
    ssr.ura = input.ura;
    ssr.refd = input.refd;
    for (int i = 0; i < 3; ++i) {
        ssr.deph[i] = input.deph[i];
        ssr.ddeph[i] = input.ddeph[i];
        ssr.dclk[i] = input.dclk[i];
    }
    ssr.hrclk = input.hrclk;
}
#endif

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
    auto input = libgnss::clasnat_parity::makeSatposSsrInput(std::max(0, sat - 1));
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
    return libgnss::external::claslib_oracle::satpos_ssr(input, out);
}

bool satpos_ssr(const SatposSsrInput& input, SatposSsrOutput& out) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    out = SatposSsrOutput{};
    if (input.sat <= 0 || input.sat > MAXSAT) {
        return false;
    }

    eph_t eph = {};
    fillClasEph(input.eph, eph);
    nav_t nav = {};
    nav.n = 1;
    nav.nmax = 1;
    nav.eph = &eph;
    nav.rtcmmode = RTCMMODE_CSSR;
    fillClasSsr(input.ssr, nav.ssr_ch[0][input.sat - 1]);

    clearsatcorr();
    set_cssr_ch_idx(0);
    saveposition(input.receiver_position);

    const int ephopt =
        input.apply_satellite_antenna_offset ? EPHOPT_SSRCOM : EPHOPT_SSRAPC;
    return ::satpos(toClasTime(input.time),
                    toClasTime(input.teph),
                    input.sat,
                    ephopt,
                    &nav,
                    out.rs,
                    out.dts,
                    &out.variance,
                    &out.svh) != 0;
#else
    (void)input;
    out = SatposSsrOutput{};
    return false;
#endif
}

bool corrmeas(const libgnss::clasnat_parity::CorrmeasInput& input, CorrmeasOutput& out) {
#if GNSSPP_HAS_CLASLIB_BRIDGE
    out = CorrmeasOutput{};
    const int nf = std::clamp(input.num_frequencies,
                              0,
                              libgnss::clasnat_parity::kParityMaxFreq);
    const int sat = std::clamp(input.sat, 1, MAXSAT);

    obsd_t obs = {};
    obs.time = toClasTime(input.time);
    obs.sat = static_cast<unsigned char>(sat);
    obs.rcv = 1;

    nav_t nav = {};
    nav.rtcmmode = RTCMMODE_CSSR;
    std::array<stec_t, MAX_NGRID> stec_grids = {};
    std::array<stecd_t, MAXSAT> stec_data = {};
    nav.stec = stec_grids.data();
    nav.stec[0].n = 1;
    nav.stec[0].nmax = static_cast<int>(stec_data.size());
    nav.stec[0].data = stec_data.data();
    nav.stec[0].data[0].time = toClasTime(input.stec_time);
    nav.stec[0].data[0].sat = static_cast<unsigned char>(sat);
    nav.stec[0].data[0].slip = 0;
    nav.stec[0].data[0].iono = static_cast<float>(input.stec_l1_m);
    nav.stec[0].data[0].rate = static_cast<float>(input.stec_rate_mps);
    nav.stec[0].data[0].quality = 1.0F;
    nav.stec[0].data[0].rms = static_cast<float>(input.stec_rms_m);
    nav.stec[0].data[0].flag = 1;

    ssr_t& ssr = nav.ssr_ch[0][sat - 1];
    ssr.t0[0] = toClasTime(input.orbit_time);
    ssr.t0[4] = toClasTime(input.code_bias_time);
    ssr.t0[5] = toClasTime(input.phase_bias_time);
    ssr.t0[8] = toClasTime(input.stec_time);
    ssr.nsig = nf;

    for (int f = 0; f < nf; ++f) {
        obs.code[f] = input.code[f];
        obs.L[f] = input.carrier_phase_cycles[f];
        obs.P[f] = input.pseudorange_m[f];
        nav.lam[sat - 1][f] = input.wavelength_m[f];
        const int code = input.code[f];
        ssr.smode[f] = code;
        if (code > 0 && code <= MAXCODE) {
            ssr.cbias[code - 1] = static_cast<float>(input.code_bias_m[f]);
            ssr.pbias[code - 1] = static_cast<float>(input.phase_bias_m[f]);
        }
    }

    prcopt_t opt = prcopt_default;
    opt.nf = nf;
    opt.posopt[1] = input.antenna_pcv_option;
    opt.posopt[5] = input.compensation_option;
    opt.posopt[9] = input.phase_code_timing_option;
    for (int f = 0; f < std::min(NFREQ, libgnss::clasnat_parity::kParityMaxFreq); ++f) {
        for (int j = 0; j < 3; ++j) {
            opt.pcvr[0].off[f][j] =
                input.receiver_pcv.offsets_m[static_cast<size_t>(f)][static_cast<size_t>(j)];
        }
        for (int j = 0; j < 19; ++j) {
            opt.pcvr[0].var[f][j] =
                input.receiver_pcv.variations_m[static_cast<size_t>(f)][static_cast<size_t>(j)];
        }
    }
    for (int j = 0; j < 3; ++j) {
        opt.antdel[0][j] = input.antenna_delta[j];
    }

    ssat_t ssat = {};
    ssat.phw = input.phase_windup_cycles;
    int pbreset[NFREQ] = {};
    for (int f = 0; f < std::min(NFREQ, nf); ++f) {
        ssat.slip[f] = input.slip[f];
        pbreset[f] = input.phase_bias_reset[f];
    }

    osrd_t osr = {};
    osr.trop = input.trop_m;
    osr.relatv = input.relativity_m;
    int index[MAX_NGRID] = {};
    double weight[MAX_NGRID] = {};
    double Gmat[MAX_NGRID * MAX_NGRID] = {};
    double Emat[MAX_NGRID] = {};
    int brk = 0;
    weight[0] = 1.0;
    Emat[3] = 1.0;

    if (!::corrmeas(&obs,
                    &nav,
                    input.receiver_pos,
                    input.azel,
                    &opt,
                    index,
                    1,
                    weight,
                    Gmat,
                    Emat,
                    ssat,
                    &brk,
                    &osr,
                    pbreset,
                    0)) {
        return false;
    }

    out.num_frequencies = nf;
    out.iono = osr.iono;
    for (int f = 0; f < nf; ++f) {
        const double fi = input.wavelength_m[0] != 0.0
                              ? input.wavelength_m[f] / input.wavelength_m[0]
                              : 1.0;
        const double iono_scaled = fi * fi * FREQ2 / FREQ1 * osr.iono;
        out.code_bias[f] = osr.cbias[f];
        out.phase_bias[f] = osr.pbias[f];
        out.phase_compensation[f] = osr.compL[f];
        out.receiver_antenna[f] = osr.antr[f];
        out.windup_m[f] = osr.wupL[f];
        out.prc[f] = osr.trop + osr.relatv + osr.antr[f] + iono_scaled + osr.cbias[f];
        out.cpc[f] = osr.trop + osr.relatv + osr.antr[f] - iono_scaled +
                     osr.pbias[f] + osr.wupL[f] + osr.compL[f];
    }
    return true;
#else
    (void)input;
    out = CorrmeasOutput{};
    return false;
#endif
}

bool corrmeas(int sample_index, CorrmeasOutput& out) {
    const auto input = libgnss::clasnat_parity::makeCorrmeasInput(sample_index);
    return libgnss::external::claslib_oracle::corrmeas(input, out);
}

bool corrmeas(CorrmeasOutput& out) {
    return libgnss::external::claslib_oracle::corrmeas(0, out);
}

}  // namespace libgnss::external::claslib_oracle
