#include "ppp_clasnat_zdres.hpp"

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <vector>

namespace libgnss::ppp_clasnat_zdres {

namespace {

constexpr double OMGE = constants::OMEGA_E;
constexpr int kResidualFreqCount = 2;

double sqr(double x) {
    return x * x;
}

void trace(int, const char*, ...) {}

const char* time_str(gtime_t, int) {
    return "";
}

double timediff(gtime_t t1, gtime_t t2) {
    return (t1.week - t2.week) * constants::SECONDS_PER_WEEK + (t1.tow - t2.tow);
}

double time2gpst(gtime_t t, int* week) {
    if (week) *week = t.week;
    return t.tow;
}

gtime_t timeadd(gtime_t t, double sec) {
    t.tow += sec;
    while (t.tow < 0.0) {
        t.tow += constants::SECONDS_PER_WEEK;
        --t.week;
    }
    while (t.tow >= constants::SECONDS_PER_WEEK) {
        t.tow -= constants::SECONDS_PER_WEEK;
        ++t.week;
    }
    return t;
}

gtime_t gpst2utc(gtime_t t) {
    return t;
}

gtime_t toClasTime(const GNSSTime& time) {
    return {time.week, time.tow};
}

int clasSatNo(const SatelliteId& satellite) {
    const int prn = static_cast<int>(satellite.prn);
    switch (satellite.system) {
        case GNSSSystem::GPS:
            return prn >= 1 && prn <= 32 ? prn : 0;
        case GNSSSystem::Galileo:
            return prn >= 1 && prn <= 36 ? 32 + prn : 0;
        case GNSSSystem::QZSS: {
            const int qprn = prn >= 193 ? prn - 192 : prn;
            return qprn >= 1 && qprn <= 7 ? 68 + qprn : 0;
        }
        default:
            return 0;
    }
}

SatelliteId satelliteFromClasSatNo(int sat) {
    if (sat >= 1 && sat <= 32) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat));
    }
    if (sat >= 33 && sat <= 68) {
        return SatelliteId(GNSSSystem::Galileo, static_cast<uint8_t>(sat - 32));
    }
    if (sat >= 69 && sat <= 75) {
        return SatelliteId(GNSSSystem::QZSS, static_cast<uint8_t>(sat - 68));
    }
    return SatelliteId{};
}

int satsys(int sat, int* prn) {
    if (sat >= 1 && sat <= 32) {
        if (prn) *prn = sat;
        return SYS_GPS;
    }
    if (sat >= 33 && sat <= 68) {
        if (prn) *prn = sat - 32;
        return SYS_GAL;
    }
    if (sat >= 69 && sat <= 75) {
        if (prn) *prn = MINPRNQZS + (sat - 69);
        return SYS_QZS;
    }
    if (prn) *prn = 0;
    return 0;
}

double dot(const double* a, const double* b, int n) {
    double c = 0.0;
    while (--n >= 0) c += a[n] * b[n];
    return c;
}

double norm(const double* a, int n) {
    return std::sqrt(dot(a, a, n));
}

void ecef2pos(const double* r, double* pos) {
    double lat = 0.0, lon = 0.0, h = 0.0;
    ecef2geodetic(Vector3d(r[0], r[1], r[2]), lat, lon, h);
    pos[0] = lat;
    pos[1] = lon;
    pos[2] = h;
}

double geodist(const double* rs, const double* rr, double* e) {
    double r = 0.0;
    for (int i = 0; i < 3; ++i) {
        e[i] = rs[i] - rr[i];
        r += e[i] * e[i];
    }
    r = std::sqrt(r);
    if (r <= 0.0) return -1.0;
    for (int i = 0; i < 3; ++i) e[i] /= r;
    return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}

double satazel(const double* pos, const double* e, double* azel) {
    const double sinp = std::sin(pos[0]);
    const double cosp = std::cos(pos[0]);
    const double sinl = std::sin(pos[1]);
    const double cosl = std::cos(pos[1]);
    const double enu[3] = {
        -sinl * e[0] + cosl * e[1],
        -sinp * cosl * e[0] - sinp * sinl * e[1] + cosp * e[2],
        cosp * cosl * e[0] + cosp * sinl * e[1] + sinp * e[2],
    };
    azel[0] = std::atan2(enu[0], enu[1]);
    if (azel[0] < 0.0) azel[0] += 2.0 * PI;
    azel[1] = std::asin(std::clamp(enu[2], -1.0, 1.0));
    return azel[1];
}

double geoidh(const double*) {
    return 0.0;
}

int get_stTv(gtime_t, double, double, double, double* tdvd, double* twvd) {
    *tdvd = 1.0;
    *twvd = 1.0;
    return 0;
}

double tropmapf(gtime_t, const double*, const double* azel, double* m_w) {
    const double sinel = std::max(std::sin(azel[1]), 1e-3);
    *m_w = 1.0 / sinel;
    return *m_w;
}

int trop_grid_data(nav_t*, const int*, gtime_t, int, const double*, const double*,
                   const double*, double* zwd, double* ztd, int* tbrk) {
    *zwd = 0.0;
    *ztd = 0.0;
    if (tbrk) *tbrk = 0;
    return 1;
}

void tidedisp(gtime_t, const double*, int, const erp_t*, const double*,
              void*, int, int, const int*, const double*, const double*,
              const double*, double* disp) {
    disp[0] = disp[1] = disp[2] = 0.0;
}

int satsigexclude(const obsd_t*, int svh, const prcopt_t*) {
    return svh < 0;
}

double shapiro(const double*, const double*) {
    return 0.0;
}

void windupcorr(gtime_t, const double*, const double*, double*) {}

int selfreqpair(const int sat, const prcopt_t *opt,const obsd_t *obs)
{
    int optf,sys;
    
    sys=satsys(sat,NULL);
    optf=(sys==SYS_GPS)?opt->posopt[10]:((sys==SYS_QZS)?opt->posopt[12]:0);
    
    if (NFREQ==1||optf==POSL1) {
         return 0;
    }else {
        if ((satsys(sat,NULL))&(SYS_GAL)) {/* GAL */
            if(obs->L[2]!=0.0||obs->P[2]!=0.0) return 2; 
            return 0; 
        }
        if (optf==POSL1L2L5) return 1+2;
        if (optf==POSL1L5) return 2;
        if (optf==POSL1L5_L2&&obs->L[2]!=0.0&&obs->P[2]!=0.0) return 2;
        if (optf==POSL1L2) {
             if (obs->L[1]!=0.0||obs->P[1]!=0.0) return 1;
             return 0;
        }
        return 1;
    }
}

/* precise tropospheric model ------------------------------------------------*/
double prectrop(gtime_t time, const double *pos, const double *azel,
                const prcopt_t *opt, const double zwd, double ztd)
{
    double m_d,m_w,tdvd,twvd,gh;

    (void)opt;
    gh=geoidh(pos);
    if (get_stTv(time,pos[0],pos[2],gh,&tdvd,&twvd)) return 0.0;

    /* mapping function */
    m_d=tropmapf(time,pos,azel,&m_w);

    return m_d*tdvd*ztd+m_w*twvd*zwd;

}

void antmodel(const pcv_t*, const double*, const double*, int, double* dant) {
    for (int i = 0; i < NFREQ + NEXOBS; ++i) dant[i] = 0.0;
}

void compensatedisp(const nav_t*, const int*, const obsd_t*, int,
                    const double, const double*, double* compL, int*,
                    const prcopt_t*, const ssat_t, int) {
    for (int i = 0; i < NFREQ + NEXOBS; ++i) compL[i] = 0.0;
}

void getorbitclock(double, int sat, double* orb, double* clk, int ch) {
    (void)sat;
    (void)ch;
    *orb = 0.0;
    *clk = 0.0;
}

double adjust_cpc(gtime_t, int, ssr_t*, int, double cpc, double*, int*, int) {
    return cpc;
}

double adjust_prc(gtime_t, int, ssr_t*, int, double prc, double*, int*, int) {
    return prc;
}

void adjust_r_dts(double*, double*, gtime_t, int, nav_t*, const double*, gtime_t, int) {}

/* ionosphere, satellite code/phase bias and phase windup corrected measurements */
int corrmeas(const obsd_t *obs, nav_t *nav, const double *pos,
             const double *azel, const prcopt_t *opt,
             const int *index, const int n, const double *weight,
             const double *Gmat, const double *Emat, const ssat_t ssat,
             int *brk, osrd_t *osr, int *pbreset, int ch)
{
    const double *lam=nav->lam[obs->sat-1];
    double iono = 0.0;
    int i,sat,nf=opt->nf;
    int any = 0;

    (void)pos;
    (void)azel;
    (void)index;
    (void)n;
    (void)weight;
    (void)Gmat;
    (void)Emat;
    (void)ssat;
    (void)brk;
    (void)pbreset;

    sat=obs->sat;

    for (i=0;i<nf;i++) {
        const bridge_freq_t& bridge = nav->ssr_ch[ch][sat-1].bridge[i];
        osr->cbias[i]=CSSRINVALID;
        osr->pbias[i]=CSSRINVALID;
        osr->compL[i]=0.0;
        osr->wupL[i]=0.0;
        osr->antr[i]=0.0;
        if (!bridge.valid || lam[0] == 0.0 || lam[i] == 0.0) continue;
        const double fi = lam[i] / lam[0];
        const double iono_scaled = fi * fi * FREQ2 / FREQ1 * bridge.iono;
        osr->antr[i]=bridge.antr;
        osr->wupL[i]=bridge.wupL;
        osr->compL[i]=bridge.compL;
        osr->cbias[i]=bridge.PRC - osr->trop - osr->relatv -
                      osr->antr[i] - iono_scaled;
        osr->pbias[i]=bridge.CPC - osr->trop - osr->relatv -
                      osr->antr[i] + iono_scaled -
                      osr->wupL[i] - osr->compL[i];
        iono = bridge.iono;
        any = 1;
    }
    osr->iono=iono;
    return any;
}

double finiteOrZero(double value) {
    return std::isfinite(value) ? value : 0.0;
}

double elevationWeight(double elevation_rad) {
    const double sin_el = std::max(std::sin(elevation_rad), 1e-3);
    return 1.0 / (sin_el * sin_el);
}

double clasCodeVarianceForRow(GNSSSystem system, double elevation_rad) {
    constexpr double kCodePhaseRatio = 50.0;
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kCodePhaseRatio * kPhaseSigmaM;
    const double b = system_factor * kCodePhaseRatio * kPhaseElevationSigmaM;
    return a * a + b * b * elevationWeight(elevation_rad);
}

double clasPhaseVarianceForRow(GNSSSystem system, double elevation_rad, int freq_index) {
    constexpr double kPhaseSigmaM = 0.01;
    constexpr double kPhaseElevationSigmaM = 0.005;
    const double system_factor = system == GNSSSystem::GLONASS ? 1.5 : 1.0;
    const double a = system_factor * kPhaseSigmaM;
    const double b = system_factor * kPhaseElevationSigmaM;
    double variance = a * a + b * b * elevationWeight(elevation_rad);
    if (freq_index == 1) {
        variance *= std::pow(2.55 / 1.55, 2);
    }
    return variance;
}

double claslibIonoMappingFunction(const Vector3d& receiver_position, double elevation_rad) {
    constexpr double kIonosphereHeightM = 350000.0;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    (void)lat;
    (void)lon;
    if (height >= kIonosphereHeightM) {
        return 1.0;
    }
    const double ratio =
        (constants::WGS84_A + height) /
        (constants::WGS84_A + kIonosphereHeightM);
    const double arg = std::clamp(ratio * std::cos(elevation_rad), -1.0, 1.0);
    const double denom = std::cos(std::asin(arg));
    return denom > 0.0 ? 1.0 / denom : 1.0;
}

int clasSystemOrder(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return 0;
        case GNSSSystem::GLONASS: return 1;
        case GNSSSystem::Galileo: return 2;
        case GNSSSystem::BeiDou: return 3;
        case GNSSSystem::QZSS: return 4;
        default: return 6;
    }
}

int clasResidualGroup(const OSRCorrection& osr, int signal_index) {
    if (osr.satellite.system == GNSSSystem::GPS && signal_index == 1 &&
        osr.signals[signal_index] == SignalType::GPS_L2C) {
        return 5;
    }
    return clasSystemOrder(osr.satellite.system);
}

SatelliteId ambiguitySatelliteForFreq(const SatelliteId& satellite, int freq_index) {
    int base_prn = static_cast<int>(satellite.prn);
    if (satellite.system == GNSSSystem::QZSS && base_prn >= 193 && base_prn <= 199) {
        base_prn -= 192;
    }
    if (freq_index <= 0) {
        return SatelliteId(satellite.system, static_cast<uint8_t>(base_prn));
    }
    const int prn = base_prn + 100 * freq_index;
    return SatelliteId(
        satellite.system,
        static_cast<uint8_t>(std::min(255, std::max(0, prn))));
}

int ionoIndex(const SatelliteId& satellite) {
    const int satno = clasSatNo(satellite);
    return satno > 0 ? ppp_claslib_full::kClasNp + satno - 1 : -1;
}

int ambIndex(const SatelliteId& satellite, int freq_index) {
    const int satno = clasSatNo(satellite);
    if (satno <= 0 || freq_index < 0 || freq_index >= ppp_claslib_full::kClasNfreq) {
        return -1;
    }
    return ppp_claslib_full::kClasAmbStart +
           ppp_claslib_full::kClasMaxSat * freq_index + satno - 1;
}

double clasCarrierWavelength(int freq_index) {
    switch (freq_index) {
        case 0: return constants::GPS_L1_WAVELENGTH;
        case 1: return constants::GPS_L2_WAVELENGTH;
        case 2: return constants::GPS_L5_WAVELENGTH;
        default: return 0.0;
    }
}

int clasFrequencySlot(const OSRCorrection& osr, int signal_index) {
    if (signal_index < 0 || signal_index >= osr.num_frequencies ||
        signal_index >= OSR_MAX_FREQ) {
        return -1;
    }
    if (osr.satellite.system == GNSSSystem::Galileo &&
        osr.signals[signal_index] == SignalType::GAL_E5A) {
        return 2;
    }
    return signal_index;
}

bool rowLessClasOrder(const ClasnatMeasurementRow& lhs,
                      const ClasnatMeasurementRow& rhs) {
    const int lhs_group = clasSystemOrder(lhs.satellite.system);
    const int rhs_group = clasSystemOrder(rhs.satellite.system);
    if (lhs_group != rhs_group) return lhs_group < rhs_group;
    if (lhs.freq_index != rhs.freq_index) return lhs.freq_index < rhs.freq_index;
    if (lhs.is_phase != rhs.is_phase) return lhs.is_phase && !rhs.is_phase;
    if (lhs.satellite.system != rhs.satellite.system) {
        return lhs.satellite.system < rhs.satellite.system;
    }
    return lhs.satellite.prn < rhs.satellite.prn;
}

struct ZdRow {
    const OSRCorrection* osr = nullptr;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    int signal_index = 0;
    int freq_index = 0;
    int group = 0;
    bool is_phase = false;
    bool valid = false;
    double y = 0.0;
    double variance = 0.0;
    Eigen::RowVectorXd H_base;
    ppp_clas::MeasurementRow::ModelComponents components;
};

bool rowLessZdOrder(const ZdRow& lhs, const ZdRow& rhs) {
    if (lhs.group != rhs.group) return lhs.group < rhs.group;
    if (lhs.freq_index != rhs.freq_index) return lhs.freq_index < rhs.freq_index;
    if (lhs.is_phase != rhs.is_phase) return lhs.is_phase && !rhs.is_phase;
    if (lhs.satellite.system != rhs.satellite.system) {
        return lhs.satellite.system < rhs.satellite.system;
    }
    return lhs.satellite.prn < rhs.satellite.prn;
}

ppp_clas::MeasurementRow::ModelComponents subtractComponents(
    const ppp_clas::MeasurementRow::ModelComponents& lhs,
    const ppp_clas::MeasurementRow::ModelComponents& rhs) {
    ppp_clas::MeasurementRow::ModelComponents out;
    out.valid = lhs.valid && rhs.valid;
    out.y = lhs.y - rhs.y;
    out.obs = lhs.obs - rhs.obs;
    out.rho = lhs.rho - rhs.rho;
    out.dts = lhs.dts - rhs.dts;
    out.dtr = lhs.dtr - rhs.dtr;
    out.rel = lhs.rel - rhs.rel;
    out.sagnac = lhs.sagnac - rhs.sagnac;
    out.trop_dry = lhs.trop_dry - rhs.trop_dry;
    out.trop_wet = lhs.trop_wet - rhs.trop_wet;
    out.trop_model = lhs.trop_model - rhs.trop_model;
    out.trop_grid = lhs.trop_grid - rhs.trop_grid;
    out.iono_grid = lhs.iono_grid - rhs.iono_grid;
    out.iono_state_term = lhs.iono_state_term - rhs.iono_state_term;
    out.prc = lhs.prc - rhs.prc;
    out.cpc = lhs.cpc - rhs.cpc;
    out.osr_corr = lhs.osr_corr - rhs.osr_corr;
    out.code_bias = lhs.code_bias - rhs.code_bias;
    out.phase_bias = lhs.phase_bias - rhs.phase_bias;
    out.phase_comp = lhs.phase_comp - rhs.phase_comp;
    out.windup = lhs.windup - rhs.windup;
    out.pco_rcv = lhs.pco_rcv - rhs.pco_rcv;
    out.pco_sat = lhs.pco_sat - rhs.pco_sat;
    out.pcv_rcv = lhs.pcv_rcv - rhs.pcv_rcv;
    out.pcv_sat = lhs.pcv_sat - rhs.pcv_sat;
    out.tide_solid = lhs.tide_solid - rhs.tide_solid;
    out.tide_ocean = lhs.tide_ocean - rhs.tide_ocean;
    out.tide_pole = lhs.tide_pole - rhs.tide_pole;
    out.amb_m = lhs.amb_m - rhs.amb_m;
    out.modeled_sum = lhs.modeled_sum - rhs.modeled_sum;
    return out;
}

void initx(ppp_claslib_full::ClaslibRtkState& rtk, int index,
           double value, double variance) {
    if (index < 0 || index >= ppp_claslib_full::kClasNx) {
        return;
    }
    rtk.x(index) = value;
    rtk.P.row(index).setZero();
    rtk.P.col(index).setZero();
    rtk.P(index, index) = variance;
}

SignalType signalForClasFreq(const OSRCorrection& osr, int freq_slot) {
    for (int i = 0; i < osr.num_frequencies && i < OSR_MAX_FREQ; ++i) {
        if (clasFrequencySlot(osr, i) == freq_slot) {
            return osr.signals[i];
        }
    }
    return SignalType::SIGNAL_TYPE_COUNT;
}

void convertInputs(const ObservationData& obs,
                   const std::vector<OSRCorrection>& osr_corrections,
                   const Vector3d& receiver_position,
                   double receiver_clock_bias_m,
                   std::vector<obsd_t>& obs_org,
                   std::vector<double>& rs,
                   std::vector<double>& dts,
                   std::vector<double>& vare,
                   std::vector<int>& svh,
                   nav_t& nav,
                   prcopt_t& opt,
                   sol_t& sol,
                   grid_t& grid,
                   ssat_t* ssat,
                   std::array<std::array<const OSRCorrection*, NFREQ>, MAXSAT>& osr_by_freq,
                   std::array<std::array<int, NFREQ>, MAXSAT>& signal_index_by_freq) {
    opt.nf = NFREQ;
    opt.elmin = 15.0 * D2R;
    opt.posopt[9] = 1;
    opt.posopt[10] = POSL1L5_L2;
    opt.posopt[12] = POSL1L5_L2;
    sol.time = toClasTime(obs.time);
    sol.network = 1;
    sol.dtr[0] = receiver_clock_bias_m / CLIGHT;
    grid.network = 1;
    grid.num = 1;
    grid.index[0] = 0;
    grid.weight[0] = 1.0;

    for (auto& row : osr_by_freq) {
        row.fill(nullptr);
    }
    for (auto& row : signal_index_by_freq) {
        row.fill(-1);
    }

    std::vector<const OSRCorrection*> sorted;
    sorted.reserve(osr_corrections.size());
    for (const auto& osr : osr_corrections) {
        if (!osr.valid || osr.elevation < opt.elmin) {
            continue;
        }
        const int sat = clasSatNo(osr.satellite);
        if (sat <= 0 || sat > MAXSAT) {
            continue;
        }
        sorted.push_back(&osr);
    }
    std::sort(sorted.begin(), sorted.end(), [](const OSRCorrection* lhs,
                                               const OSRCorrection* rhs) {
        return clasSatNo(lhs->satellite) < clasSatNo(rhs->satellite);
    });

    for (const OSRCorrection* osr : sorted) {
        const int sat = clasSatNo(osr->satellite);
        obsd_t obsd;
        obsd.time = toClasTime(obs.time);
        obsd.sat = static_cast<unsigned char>(sat);
        obsd.rcv = 1;
        obsd.osr_i = static_cast<int>(obs_org.size());
        ssat[sat - 1].sys = static_cast<unsigned char>(satsys(sat, nullptr));
        ssat[sat - 1].azel[0] = osr->azimuth;
        ssat[sat - 1].azel[1] = osr->elevation;
        ssat[sat - 1].phw = osr->windup_cycles;

        for (int i = 0; i < osr->num_frequencies && i < OSR_MAX_FREQ; ++i) {
            const int f = clasFrequencySlot(*osr, i);
            if (f < 0 || f >= NFREQ) {
                continue;
            }
            const Observation* raw = obs.getObservation(osr->satellite, osr->signals[i]);
            if (!raw || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange) ||
                raw->carrier_phase == 0.0 || raw->pseudorange == 0.0 ||
                osr->wavelengths[i] <= 0.0) {
                continue;
            }
            const int code = f + 1;
            obsd.L[f] = raw->carrier_phase;
            obsd.P[f] = raw->pseudorange;
            obsd.D[f] = static_cast<float>(raw->doppler);
            obsd.SNR[f] = static_cast<unsigned char>(
                std::clamp(static_cast<int>(std::lround(raw->snr / 0.25)), 0, 255));
            obsd.LLI[f] = raw->lli;
            obsd.code[f] = static_cast<unsigned char>(code);
            nav.lam[sat - 1][f] = osr->wavelengths[i];
            nav.ssr_ch[0][sat - 1].nsig = std::max(nav.ssr_ch[0][sat - 1].nsig, f + 1);
            nav.ssr_ch[0][sat - 1].smode[f] = code;
            nav.ssr_ch[0][sat - 1].t0[0] = toClasTime(osr->clock_reference_time);
            nav.ssr_ch[0][sat - 1].t0[4] = toClasTime(osr->phase_bias_reference_time);
            nav.ssr_ch[0][sat - 1].t0[5] = toClasTime(osr->phase_bias_reference_time);
            nav.ssr_ch[0][sat - 1].t0[8] = toClasTime(osr->atmos_reference_time);
            if (nav.ssr_ch[0][sat - 1].t0[0].week == 0) nav.ssr_ch[0][sat - 1].t0[0] = toClasTime(obs.time);
            if (nav.ssr_ch[0][sat - 1].t0[4].week == 0) nav.ssr_ch[0][sat - 1].t0[4] = toClasTime(obs.time);
            if (nav.ssr_ch[0][sat - 1].t0[5].week == 0) nav.ssr_ch[0][sat - 1].t0[5] = toClasTime(obs.time);
            if (nav.ssr_ch[0][sat - 1].t0[8].week == 0) nav.ssr_ch[0][sat - 1].t0[8] = toClasTime(obs.time);
            nav.ssr_ch[0][sat - 1].bridge[f].signal_index = i;
            nav.ssr_ch[0][sat - 1].bridge[f].PRC = osr->PRC[i];
            nav.ssr_ch[0][sat - 1].bridge[f].CPC = osr->CPC[i];
            nav.ssr_ch[0][sat - 1].bridge[f].iono = osr->iono_l1_m;
            nav.ssr_ch[0][sat - 1].bridge[f].antr = osr->receiver_antenna_m[i];
            nav.ssr_ch[0][sat - 1].bridge[f].wupL = osr->windup_m[i];
            nav.ssr_ch[0][sat - 1].bridge[f].compL = osr->phase_compensation_m[i];
            nav.ssr_ch[0][sat - 1].bridge[f].orb = osr->orbit_projection_m;
            nav.ssr_ch[0][sat - 1].bridge[f].clk = osr->clock_correction_m;
            nav.ssr_ch[0][sat - 1].bridge[f].valid = true;
            osr_by_freq[static_cast<size_t>(sat - 1)][static_cast<size_t>(f)] = osr;
            signal_index_by_freq[static_cast<size_t>(sat - 1)][static_cast<size_t>(f)] = i;
        }
        if (obsd.P[0] == 0.0 && obsd.L[0] == 0.0) {
            continue;
        }
        nav.stec[0].data[nav.stec[0].n].sat = sat;
        nav.stec[0].data[nav.stec[0].n].time = toClasTime(obs.time);
        nav.stec[0].n++;
        obs_org.push_back(obsd);
        rs.push_back(osr->satellite_position.x());
        rs.push_back(osr->satellite_position.y());
        rs.push_back(osr->satellite_position.z());
        rs.push_back(osr->satellite_velocity.x());
        rs.push_back(osr->satellite_velocity.y());
        rs.push_back(osr->satellite_velocity.z());
        dts.push_back(osr->satellite_clock_bias_s);
        dts.push_back(0.0);
        vare.push_back(0.0);
        svh.push_back(0);
    }

    (void)receiver_position;
}

std::vector<ZdRow> buildZeroDifferenceRowsFromClas(
    const ObservationData& obs,
    const std::vector<obsd_t>& obs_org,
    const std::vector<double>& y,
    const std::vector<double>& e,
    const std::vector<double>& azel,
    const std::array<std::array<const OSRCorrection*, NFREQ>, MAXSAT>& osr_by_freq,
    const std::array<std::array<int, NFREQ>, MAXSAT>& signal_index_by_freq,
    const Vector3d& receiver_position,
    double receiver_clock_bias_m) {
    std::vector<ZdRow> rows;
    const int nf = NFREQ;
    for (int i = 0; i < static_cast<int>(obs_org.size()); ++i) {
        const int sat = obs_org[static_cast<size_t>(i)].sat;
        if (sat <= 0 || sat > MAXSAT) {
            continue;
        }
        for (int f = 0; f < NFREQ; ++f) {
            const OSRCorrection* osr =
                osr_by_freq[static_cast<size_t>(sat - 1)][static_cast<size_t>(f)];
            const int signal_index =
                signal_index_by_freq[static_cast<size_t>(sat - 1)][static_cast<size_t>(f)];
            if (!osr || signal_index < 0 ||
                obs_org[static_cast<size_t>(i)].L[f] == 0.0 ||
                obs_org[static_cast<size_t>(i)].P[f] == 0.0) {
                continue;
            }
            const double phase_y = y[static_cast<size_t>(nf * i * 2 + f)];
            const double code_y = y[static_cast<size_t>(nf * i * 2 + nf + f)];
            const double rho_sagnac =
                libgnss::geodist(osr->satellite_position, receiver_position);
            const double rho_no_sagnac =
                (osr->satellite_position - receiver_position).norm();
            const double sat_clk_m = CLIGHT * osr->satellite_clock_bias_s;
            const double fi_for_grid =
                (osr->wavelengths[0] > 0.0 && osr->wavelengths[signal_index] > 0.0)
                    ? osr->wavelengths[signal_index] / osr->wavelengths[0]
                    : 1.0;
            const double iono_grid_m =
                fi_for_grid * fi_for_grid * FREQ2 / FREQ1 * osr->iono_l1_m;
            const double tide_applied_m =
                osr->tide_geometry_m != 0.0 ? osr->tide_geometry_m
                                            : osr->solid_earth_tide_m;
            Eigen::RowVectorXd h_base =
                Eigen::RowVectorXd::Zero(ppp_claslib_full::kClasNx);
            h_base(0) = -e[static_cast<size_t>(i * 3 + 0)];
            h_base(1) = -e[static_cast<size_t>(i * 3 + 1)];
            h_base(2) = -e[static_cast<size_t>(i * 3 + 2)];

            if (std::isfinite(phase_y) && phase_y != 0.0) {
                ZdRow phase_row;
                phase_row.osr = osr;
                phase_row.satellite = osr->satellite;
                phase_row.ambiguity_satellite =
                    ambiguitySatelliteForFreq(osr->satellite, f);
                phase_row.signal_index = signal_index;
                phase_row.freq_index = f;
                phase_row.group = clasResidualGroup(*osr, signal_index);
                phase_row.is_phase = true;
                phase_row.valid = true;
                phase_row.y = phase_y;
                phase_row.variance =
                    clasPhaseVarianceForRow(osr->satellite.system, azel[static_cast<size_t>(i * 2 + 1)], f);
                phase_row.H_base = h_base;
                phase_row.components.valid = true;
                phase_row.components.y = phase_y;
                phase_row.components.obs =
                    obs_org[static_cast<size_t>(i)].L[f] * osr->wavelengths[signal_index];
                phase_row.components.rho = rho_no_sagnac;
                phase_row.components.dts = sat_clk_m;
                phase_row.components.dtr = receiver_clock_bias_m;
                phase_row.components.rel = osr->relativity_correction_m;
                phase_row.components.sagnac = rho_sagnac - rho_no_sagnac;
                phase_row.components.trop_grid = osr->trop_correction_m;
                phase_row.components.iono_grid = -iono_grid_m;
                phase_row.components.cpc = osr->CPC[signal_index];
                phase_row.components.osr_corr = osr->CPC[signal_index];
                phase_row.components.phase_bias = osr->phase_bias_m[signal_index];
                phase_row.components.phase_comp = osr->phase_compensation_m[signal_index];
                phase_row.components.windup = osr->windup_m[signal_index];
                phase_row.components.pcv_rcv = osr->receiver_antenna_m[signal_index];
                phase_row.components.tide_solid = tide_applied_m;
                phase_row.components.modeled_sum =
                    phase_row.components.obs - phase_y;
                rows.push_back(phase_row);
            }

            if (std::isfinite(code_y) && code_y != 0.0) {
                ZdRow code_row;
                code_row.osr = osr;
                code_row.satellite = osr->satellite;
                code_row.signal_index = signal_index;
                code_row.freq_index = f;
                code_row.group = clasResidualGroup(*osr, signal_index);
                code_row.is_phase = false;
                code_row.valid = true;
                code_row.y = code_y;
                code_row.variance =
                    clasCodeVarianceForRow(osr->satellite.system, azel[static_cast<size_t>(i * 2 + 1)]);
                code_row.H_base = h_base;
                code_row.components.valid = true;
                code_row.components.y = code_y;
                code_row.components.obs = obs_org[static_cast<size_t>(i)].P[f];
                code_row.components.rho = rho_no_sagnac;
                code_row.components.dts = sat_clk_m;
                code_row.components.dtr = receiver_clock_bias_m;
                code_row.components.rel = osr->relativity_correction_m;
                code_row.components.sagnac = rho_sagnac - rho_no_sagnac;
                code_row.components.trop_grid = osr->trop_correction_m;
                code_row.components.iono_grid = iono_grid_m;
                code_row.components.prc = osr->PRC[signal_index];
                code_row.components.osr_corr = osr->PRC[signal_index];
                code_row.components.code_bias = osr->code_bias_m[signal_index];
                code_row.components.pcv_rcv = osr->receiver_antenna_m[signal_index];
                code_row.components.tide_solid = tide_applied_m;
                code_row.components.modeled_sum =
                    code_row.components.obs - code_y;
                rows.push_back(code_row);
            }
        }
    }
    std::sort(rows.begin(), rows.end(), rowLessZdOrder);
    return rows;
}

ClasnatMeasurementBuild formSingleDifferenceRows(
    const ObservationData& obs,
    const std::vector<ZdRow>& zd_rows,
    ppp_claslib_full::ClaslibRtkState& rtk,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position) {
    ClasnatMeasurementBuild build;
    struct GroupKey {
        int group = 0;
        int freq_index = 0;
        bool is_phase = false;
        bool operator<(const GroupKey& rhs) const {
            if (group != rhs.group) return group < rhs.group;
            if (freq_index != rhs.freq_index) return freq_index < rhs.freq_index;
            return is_phase > rhs.is_phase;
        }
    };
    std::map<GroupKey, std::vector<const ZdRow*>> groups;
    for (const auto& row : zd_rows) {
        if (row.valid) {
            groups[{row.group, row.freq_index, row.is_phase}].push_back(&row);
        }
    }

    for (auto& [key, members] : groups) {
        if (members.size() < 2) {
            continue;
        }
        const ZdRow* ref = nullptr;
        for (const ZdRow* row : members) {
            if (ref == nullptr || row->osr->elevation >= ref->osr->elevation) {
                ref = row;
            }
        }
        if (ref == nullptr) {
            continue;
        }
        for (const ZdRow* sat : members) {
            if (sat == ref) {
                continue;
            }
            ClasnatMeasurementRow row;
            row.H = ref->H_base - sat->H_base;
            row.residual = ref->y - sat->y;
            row.variance = ref->variance + sat->variance;
            row.reference_variance = ref->variance;
            row.satellite = sat->satellite;
            row.ambiguity_satellite =
                key.is_phase ? sat->ambiguity_satellite : SatelliteId{};
            row.reference_satellite = ref->satellite;
            row.is_phase = key.is_phase;
            row.signal_index = sat->signal_index;
            row.freq_index = key.freq_index;
            row.components = subtractComponents(ref->components, sat->components);

            if (config.estimate_ionosphere) {
                const double ref_fi =
                    ref->osr->wavelengths[ref->signal_index] / constants::GPS_L1_WAVELENGTH;
                const double sat_fi =
                    sat->osr->wavelengths[sat->signal_index] / constants::GPS_L1_WAVELENGTH;
                const double sign = key.is_phase ? -1.0 : 1.0;
                const double ref_coeff =
                    sign * ref_fi * ref_fi *
                    claslibIonoMappingFunction(receiver_position, ref->osr->elevation);
                const double sat_coeff =
                    sign * sat_fi * sat_fi *
                    claslibIonoMappingFunction(receiver_position, sat->osr->elevation);
                const auto ref_iono_it =
                    rtk.ionosphere_indices.find(ref->satellite);
                const auto sat_iono_it =
                    rtk.ionosphere_indices.find(sat->satellite);
                const bool ref_has_iono =
                    ref_iono_it != rtk.ionosphere_indices.end() &&
                    ref_iono_it->second >= 0 &&
                    ref_iono_it->second < ppp_claslib_full::kClasNx;
                const bool sat_has_iono =
                    sat_iono_it != rtk.ionosphere_indices.end() &&
                    sat_iono_it->second >= 0 &&
                    sat_iono_it->second < ppp_claslib_full::kClasNx;
                const double ref_state =
                    ref_has_iono ? rtk.x(ref_iono_it->second) : 0.0;
                const double sat_state =
                    sat_has_iono ? rtk.x(sat_iono_it->second) : 0.0;
                const double iono_state_term =
                    ref_coeff * ref_state - sat_coeff * sat_state;
                row.residual -= iono_state_term;
                row.components.iono_state_term =
                    finiteOrZero(row.components.iono_state_term) + iono_state_term;
                if (ref_has_iono) {
                    row.H(ref_iono_it->second) = ref_coeff;
                }
                if (sat_has_iono) {
                    row.H(sat_iono_it->second) = -sat_coeff;
                }
            }

            if (key.is_phase) {
                const auto ref_amb_it =
                    rtk.ambiguity_indices.find(ref->ambiguity_satellite);
                const auto sat_amb_it =
                    rtk.ambiguity_indices.find(sat->ambiguity_satellite);
                const bool ref_has_amb =
                    ref_amb_it != rtk.ambiguity_indices.end() &&
                    ref_amb_it->second >= 0 &&
                    ref_amb_it->second < ppp_claslib_full::kClasNx;
                const bool sat_has_amb =
                    sat_amb_it != rtk.ambiguity_indices.end() &&
                    sat_amb_it->second >= 0 &&
                    sat_amb_it->second < ppp_claslib_full::kClasNx;
                if (!sat_has_amb) {
                    continue;
                }
                const double ref_amb_state =
                    ref_has_amb ? rtk.x(ref_amb_it->second) : 0.0;
                const double sat_amb_state = rtk.x(sat_amb_it->second);
                const double ref_lambda = ref->osr->wavelengths[ref->signal_index];
                const double sat_lambda = sat->osr->wavelengths[sat->signal_index];
                const double amb_state_term =
                    ref_lambda * ref_amb_state - sat_lambda * sat_amb_state;
                row.residual -= amb_state_term;
                row.components.amb_m =
                    finiteOrZero(row.components.amb_m) + amb_state_term;
                if (ref_has_amb) {
                    row.H(ref_amb_it->second) = ref_lambda;
                    build.observed_ambiguities.insert(ref->ambiguity_satellite);
                }
                row.H(sat_amb_it->second) = -sat_lambda;
                build.observed_ambiguities.insert(sat->ambiguity_satellite);

                const Observation* raw =
                    obs.getObservation(
                        sat->satellite, sat->osr->signals[sat->signal_index]);
                if (raw && raw->loss_of_lock) {
                    initx(rtk, sat_amb_it->second, 0.0, 0.0);
                    auto& ambiguity = rtk.ambiguity_states[sat->ambiguity_satellite];
                    ambiguity.lock_count = -5;
                    ambiguity.needs_reinitialization = true;
                }
            }
            row.components.y = row.residual;
            row.components.modeled_sum =
                finiteOrZero(row.components.obs) - row.residual;
            build.rows.push_back(row);
        }
    }
    return build;
}

}  // namespace

/* zero differenced phase/code residuals  ----------------------------------
* args   : obsd_t  *obs      I   observation data for an epoch
*                                obs[i].rcv=1:rover,2:reference
*                                sorted by receiver and satellte
*          int     n         I   number of observation data
*          double  *rs       I   satellite positions and velocities (ecef) 
*          double  *dts      I   satellite clocks
*          double  *vare     I   sat position and clock error variances (m^2)
*          double  *svh      I   sat health flag (-1:correction not available)
*          nav_t   *nav      I   navigation messages
*          double  *x        O   float states
*          double  *y        O   residuals
*          double  *e        O   line-of-sight vector (ecef) 
*          double  *azel     O   azimath/elevation
*          rtk_t   *rtk      I   rtk control/result struct
*          int     osrlog    I   output osrlog on/off flag
*          double  *cpc      O   carrier phase correction
*          gtime_t *pt0      I/O cssr-pbias correction time
*          grid_t  *grid     I/O grid infomation
*          ssat_t *ssat      I   satellite status
*          procopt_t *opt    I   processing options
*          sol_t *sol        I   solution data
*          int     ch        I   L6 channel to use
* return : nv    number of residuals
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*-----------------------------------------------------------------------------*/
int zdres(const obsd_t *obs_org,
          int n, const double *rs, const double *dts,
          const double *vare, const int *svh, nav_t *nav,
          double *x, double *y, 
          double *e, double *azel, rtk_t *rtk,
          int osrlog, double *cpc, gtime_t *pt0, grid_t *grid,
          ssat_t *ssat, prcopt_t *opt, sol_t *sol, osrd_t *osr, int ch)
{
    double r,rr[3],disp[3]={0},pos[3],meas[NFREQ*2],zwd,ztd;
    double *lam,fi;
    int i,j,f,qj,sat,sys,brk,nf=opt->nf, nftmp;
    int tbrk=0;
    double isb[NFREQ][2]={0};
    double modl[NFREQ * 2];
    obsd_t *obs=NULL;
    double isb_by_prn[NFREQ];
    double tow;
    static double pbias_ofst[MAXSAT*(NFREQ+NEXOBS)]={0};
    double dcpc;
    int nv=0;
    double ydif=0.0;

    osrd_t osrtmp[MAXOBS]={{{0}}};
    osr = osrtmp;

    (void)vare;
    (void)rtk;
    (void)isb;
    (void)isb_by_prn;
    (void)ydif;

    if (n<=0) return 0;

    for (i=0;i<n*nf*2;i++) y[i] = 0.0;

    trace(3,"zdres : n=%d\n",n);

    for (i=0;i<3;i++) rr[i]=x[i];

    if (norm(rr,3)<=0.0) return 0;
    
    if (!(obs=(obsd_t *)malloc(sizeof(obsd_t)*n))) return 0;
    for (i=0;i<n&&i<MAXOBS;i++) {
        obs[i]=obs_org[i];
    }

    sol->network = grid->network;
    ecef2pos(rr,pos);
    if (!trop_grid_data(nav, grid->index, obs[0].time, grid->num, grid->weight, grid->Gmat, grid->Emat, &zwd, &ztd, &tbrk)) { /* zenith trop delay */
        trace(2,"trop correction error: time=%s \n", time_str(obs[0].time,2));
        free(obs);
        return 0;
    }

    /* earth tides correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rr,opt->tidecorr,&nav->erp,opt->odisp[0],
                 nav->oload[grid->network-1],grid->network,grid->num,grid->index,grid->Gmat,grid->Emat,grid->weight,disp);
        for (i=0;i<3;i++) rr[i]+=disp[i];
        ecef2pos(rr,pos);
    }

    /* check invalid grid */
    nav->invtrop=tbrk;

    for (i=0;i<n&&i<MAXOBS;i++) {
        int iodeflag = FALSE, prn;
        double tmp_r=-1.0, tmp_dts=0.0;
        sat=obs[i].sat;
        lam=nav->lam[sat-1];
        sys=satsys(sat,&prn);
        osr[i].sat=sat;
        
        for (j=0;j<nf*2;j++) meas[j]=modl[j]=0.0;

        /* geometric distance/azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr,e+i*3))<=0.0) {
            continue;
        }
        if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) {
            trace(3,"elevation mask rejection: sat=%2d,el=%4.1f\n",sat,azel[i*2+1]*R2D);
            continue;
        }

        /* excluded satellite & signal */
        if (satsigexclude(&obs[i],svh[i],opt)) continue;

        /* shapiro time delay correction */
        if (opt->posopt[7]) osr[i].relatv=shapiro(rs+i*6,rr);

        /* tropospheric delay correction */
        osr[i].trop=prectrop(obs[i].time,pos,azel+i*2,opt,zwd,ztd);

        /* phase windup correction */
        windupcorr(sol->time,rs+i*6,rr,&ssat[sat-1].phw);

        /* ionosphere , satellite code/phase bias and phase windup corrected measurements */
        if (!corrmeas(obs+i, nav, pos, azel+i*2, opt,
                      grid->index,grid->num,grid->weight,grid->Gmat,grid->Emat,ssat[sat-1],&brk,osr+i,ssat[sat-1].pbreset,ch)) {
            continue;
        }

        qj=selfreqpair(sat,opt,obs+i);
        nftmp = nf;
        for (j=0;j<nf;j++) {
            f=j;
            if (f!=0&&f!=qj) continue;
            fi=lam[f]/lam[0];
            if (osr[i].pbias[j]==CSSRINVALID||osr[i].cbias[j]==CSSRINVALID) {
                continue;
            }
            osr[i].PRC[j]=osr[i].trop+osr[i].relatv+osr[i].antr[f]+fi*fi*FREQ2/FREQ1*osr[i].iono
                            +osr[i].cbias[j];
            osr[i].CPC[j]=osr[i].trop+osr[i].relatv+osr[i].antr[f]-fi*fi*FREQ2/FREQ1*osr[i].iono
                            +osr[i].pbias[j]+osr[i].wupL[f]+osr[i].compL[j];
            if (!opt->posopt[9]) {
                osr[i].CPC[j] = adjust_cpc(obs[i].time, sat, &nav->ssr_ch[ch][sat-1], f, osr[i].CPC[j], &osr[i].sis, &iodeflag, ch);
                osr[i].PRC[j] = adjust_prc(obs[i].time, sat, &nav->ssr_ch[ch][sat-1], f, osr[i].PRC[j], &osr[i].sis, &iodeflag, ch);
            }
            {
                double prc_used=osr[i].PRC[j], cpc_used=osr[i].CPC[j];
                double range_used=r, dts_used=dts[i*2];
                double rho0=sqrt(sqr(rs[i*6+0]-rr[0])+sqr(rs[i*6+1]-rr[1])+sqr(rs[i*6+2]-rr[2]));
                double sagnac_used=r-rho0;
            if (iodeflag == TRUE) {
                if (tmp_r<0.0) {
                    /* transmission time by satellite clock */
                    gtime_t dtsat = timeadd(obs[i].time,-obs[i].P[0]/CLIGHT);
                    tmp_r = r;
                    adjust_r_dts(&tmp_r, &tmp_dts, obs[i].time, sat, nav, rr, dtsat, ch);
                    trace(4, "adjust_r_dts: tow=%.1f, sat=%d, r=%.3f --> %.3f, dts=%.16f --> %.16f\n",
                          time2gpst(obs[i].time, NULL), sat, r, tmp_r, dts[i*2], tmp_dts);
                }

                modl[j]   =tmp_r-CLIGHT*tmp_dts+osr[i].CPC[j];
                modl[nftmp+j]=tmp_r-CLIGHT*tmp_dts+osr[i].PRC[j];
                range_used=tmp_r;
                dts_used=tmp_dts;
                osr[i].CPC[j]+=tmp_r-CLIGHT*tmp_dts-(r-CLIGHT*dts[i*2]);
                osr[i].PRC[j]+=tmp_r-CLIGHT*tmp_dts-(r-CLIGHT*dts[i*2]);
            } else {
                modl[j]      =r-CLIGHT*dts[i*2]+osr[i].CPC[j];
                modl[nftmp+j]=r-CLIGHT*dts[i*2]+osr[i].PRC[j];
            }
            if (range_used!=r) {
                sagnac_used=range_used-rho0;
            }
            osr[i].p[j] = modl[nftmp+j];
            osr[i].c[j] = modl[j];
            (void)prc_used;
            (void)cpc_used;
            (void)dts_used;
            (void)sagnac_used;
            }

            /* repair cycle slip of pbias */
            tow = time2gpst(obs[i].time, NULL);
            getorbitclock(tow, sat, &osr[i].orb, &osr[i].clk,ch);
            if (nav->filreset == FALSE&&
                fabs(timediff(nav->ssr_ch[ch][sat-1].t0[5],pt0[sat-1]))>0.0&&
                fabs(timediff(nav->ssr_ch[ch][sat-1].t0[5],pt0[sat-1]))<120.0) {
                dcpc=osr[i].orb-osr[i].clk+osr[i].CPC[j]-cpc[j*MAXSAT+sat-1];
                if (dcpc>=95.0*lam[f]&&dcpc<105.0*lam[f]) {
                    pbias_ofst[j*MAXSAT+sat-1]-=100.0;

                    x[ambIndex(satelliteFromClasSatNo(sat), f)]-=100.0;
                    trace(2,"pbias slip detected t=%s sat=%2d f=%1d dcpc[cycle]=%.1f\n",time_str(obs[i].time,0),sat,f,dcpc/lam[f]);
                } else if (dcpc<=-95.0*lam[f]&&dcpc>-105.0*lam[f]) {
                    pbias_ofst[j*MAXSAT+sat-1]+=100.0;
                    x[ambIndex(satelliteFromClasSatNo(sat), f)]+=100.0;
                    trace(2,"pbias slip detected t=%s sat=%2d f=%1d dcpc[cycle]=%.1f\n",time_str(obs[i].time,0),sat,f,dcpc/lam[f]);
                }
            }else{
                pbias_ofst[j*MAXSAT+sat-1]=0.0;
            }
            cpc[j*MAXSAT+sat-1]=osr[i].orb-osr[i].clk+osr[i].CPC[j];
        }
        /* isb correction */
        if (opt->isb==1) {
            for (f=0;f<nf;f++) meas[f]=meas[f]-isb[f][0]; /* L */

            if (opt->isbbyprn) {
                for (f=0;f<nf;f++) meas[f+nf]=meas[f+nf]-isb_by_prn[f]; /* P */
             } else {
                for (f=0;f<nf;f++) meas[f+nf]=meas[f+nf]-isb[f][1]; /* P */
            }
        }
        pt0[sat-1]=nav->ssr_ch[ch][sat-1].t0[5];


        /* measurementd phase and code */
        for (f=0;f<nf;f++) {
            if (lam[f]==0.0||obs[i].L[f]==0.0||obs[i].P[f]==0.0) continue;
            meas[f   ]+=obs[i].L[f]*lam[f];
            meas[f+nf]+=obs[i].P[f];
        }
        
        for (j=0;j<2;j++) { /* for phase and code */
            for (f=0;f<nf;f++) {
                if (meas[nf*j+f]==0.0) continue;
                if (j==0) ssat[sat-1].code[f]=obs[i].code[f];
                if (f>0&&!(f&qj)) continue;
                if (f==1&&(satsys(sat,NULL)==SYS_GAL)) continue;
                if (j==0&&(osr[i].pbias[f]==CSSRINVALID)) {
                    trace(2,"invalid pbias sat=%2d f=%1d\n",sat,f);
                    continue;
                }
                if (j==1&&(osr[i].cbias[f]==CSSRINVALID)) {
                    trace(2,"invalid cbias sat=%2d f=%1d \n",sat,f);
                    continue;
                }
                y[nf*i*2+nf*j+f]=meas[nf*j+f]-modl[nf*j+f];
            }
            nv+=nf;
        }
      
        if (sys == SYS_GAL) {
            osr[i].antr[1] = 0.0;
            osr[i].wupL[1] = 0.0;
            osr[i].CPC[1]  = 0.0;
            osr[i].PRC[1]  = 0.0;
        }
        if (osrlog) {
            tow = time2gpst(obs[i].time, NULL);
            getorbitclock(tow, sat, &osr[i].orb, &osr[i].clk,ch);
            trace(3, "OSRRES(ch%d),%.1f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
                  "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.9f,%.9f,%.3f\n",
                  ch, tow, sys, prn,
                  osr[i].pbias[0], osr[i].pbias[1], osr[i].pbias[2],
                  osr[i].cbias[0], osr[i].cbias[1], osr[i].cbias[2],
                  osr[i].trop,
                  osr[i].iono,
                  osr[i].antr[0], osr[i].antr[1], osr[i].antr[2],
                  osr[i].relatv,
                  osr[i].wupL[0], osr[i].wupL[1], osr[i].wupL[2],
                  osr[i].compL[0], osr[i].compL[1], osr[i].compL[2],
                  osr[i].sis,
                  osr[i].CPC[0], osr[i].CPC[1], osr[i].CPC[2],
                  osr[i].PRC[0], osr[i].PRC[1], osr[i].PRC[2],
                  osr[i].orb, osr[i].clk,
                  pos[0]*R2D,pos[1]*R2D,pos[2]);
        }
    }

    free(obs);

    return nv;
}

ClasnatMeasurementBuild buildEpochMeasurementsClasnatZdres(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_claslib_full::ClaslibRtkState& rtk,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_bias_m,
    double trop_zenith_m) {
    (void)trop_zenith_m;
    std::vector<obsd_t> obs_org;
    std::vector<double> rs;
    std::vector<double> dts;
    std::vector<double> vare;
    std::vector<int> svh;
    nav_t nav;
    prcopt_t opt;
    sol_t sol;
    grid_t grid;
    rtk_t rtk_clas;
    std::array<ssat_t, MAXSAT> ssat{};
    std::array<std::array<const OSRCorrection*, NFREQ>, MAXSAT> osr_by_freq{};
    std::array<std::array<int, NFREQ>, MAXSAT> signal_index_by_freq{};

    convertInputs(
        obs,
        epoch_context.osr_corrections,
        receiver_position,
        receiver_clock_bias_m,
        obs_org,
        rs,
        dts,
        vare,
        svh,
        nav,
        opt,
        sol,
        grid,
        ssat.data(),
        osr_by_freq,
        signal_index_by_freq);

    if (obs_org.empty()) {
        return {};
    }

    std::vector<double> x(static_cast<size_t>(ppp_claslib_full::kClasNx), 0.0);
    for (int i = 0; i < std::min<int>(rtk.x.size(), ppp_claslib_full::kClasNx); ++i) {
        x[static_cast<size_t>(i)] = rtk.x(i);
    }
    x[0] = receiver_position.x();
    x[1] = receiver_position.y();
    x[2] = receiver_position.z();

    const int n = static_cast<int>(obs_org.size());
    std::vector<double> y(static_cast<size_t>(n * NFREQ * 2), 0.0);
    std::vector<double> e(static_cast<size_t>(n * 3), 0.0);
    std::vector<double> azel(static_cast<size_t>(n * 2), 0.0);
    std::vector<double> cpc(static_cast<size_t>(NFREQ * MAXSAT), 0.0);
    std::vector<gtime_t> pt0(static_cast<size_t>(MAXSAT));

    rtk_clas.opt = opt;
    rtk_clas.sol = sol;
    zdres(obs_org.data(),
          n,
          rs.data(),
          dts.data(),
          vare.data(),
          svh.data(),
          &nav,
          x.data(),
          y.data(),
          e.data(),
          azel.data(),
          &rtk_clas,
          FALSE,
          cpc.data(),
          pt0.data(),
          &grid,
          ssat.data(),
          &opt,
          &sol,
          nullptr,
          0);

    const std::vector<ZdRow> zd_rows =
        buildZeroDifferenceRowsFromClas(
            obs,
            obs_org,
            y,
            e,
            azel,
            osr_by_freq,
            signal_index_by_freq,
            receiver_position,
            receiver_clock_bias_m);
    return formSingleDifferenceRows(obs, zd_rows, rtk, config, receiver_position);
}

}  // namespace libgnss::ppp_clasnat_zdres
