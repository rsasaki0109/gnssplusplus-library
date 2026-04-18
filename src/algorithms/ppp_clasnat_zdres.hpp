#pragma once

#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_claslib_full.hpp>
#include <libgnss++/algorithms/ppp_osr_types.hpp>
#include <libgnss++/algorithms/ppp_shared.hpp>
#include <libgnss++/core/observation.hpp>

#include <array>
#include <set>
#include <vector>

namespace libgnss::ppp_clasnat_zdres {

#ifdef NFREQ
#undef NFREQ
#endif

constexpr double PI = 3.1415926535897932;  /* pi */
constexpr double D2R = PI / 180.0;          /* deg to rad */
constexpr double R2D = 180.0 / PI;          /* rad to deg */
constexpr double CLIGHT = 299792458.0;      /* speed of light (m/s) */
constexpr double FREQ1 = 1.57542E9;         /* L1/E1  frequency (Hz) */
constexpr double FREQ2 = 1.22760E9;         /* L2     frequency (Hz) */

constexpr int NFREQ = ppp_claslib_full::kClasNfreq;
constexpr int NEXOBS = 0;
constexpr int MAXSAT = ppp_claslib_full::kClasMaxSat;
constexpr int MAXOBS = 64;
constexpr int SSR_CH_NUM = 2;
constexpr int MAX_NGRID = 4;                /* number of grids for interpolation */
constexpr int MAX_INDEX_SSR = 9;
constexpr int MAXCODE = 64;
constexpr int CSSRINVALID = -10000;         /* invalid value */
constexpr int RTCMMODE_CSSR = 5;
constexpr int TRUE = 1;
constexpr int FALSE = 0;

constexpr int SYS_GPS = 0x01;               /* navigation system: GPS */
constexpr int SYS_GAL = 0x08;               /* navigation system: Galileo */
constexpr int SYS_QZS = 0x10;               /* navigation system: QZSS */
constexpr int MINPRNQZS = 193;              /* min satellite PRN number of QZSS */

constexpr int POSL1 = 1;                    /* L1 single freq positioning */
constexpr int POSL1L2 = 2;                  /* L1+L2 dual freq positioning */
constexpr int POSL1L5 = 3;                  /* L1+L5 dual freq positioning */
constexpr int POSL1L2L5 = 4;                /* L1+L2+L5 triple freq positioning */
constexpr int POSL1L5_L2 = 5;               /* L1+L5(or L2) dual freq positioning */

struct gtime_t {        /* time struct */
    int week = 0;       /* GPS week */
    double tow = 0.0;   /* time of week (s) */
};

struct obsd_t {         /* observation data record */
    gtime_t time;       /* receiver sampling time (GPST) */
    unsigned char sat = 0, rcv = 0; /* satellite/receiver number */
    unsigned char SNR[NFREQ + NEXOBS] = {}; /* signal strength (0.25 dBHz) */
    unsigned char LLI[NFREQ + NEXOBS] = {}; /* loss of lock indicator */
    unsigned char code[NFREQ + NEXOBS] = {}; /* code indicator (CODE_???) */
    unsigned short iod = 0; /* iod for correction */
    double L[NFREQ + NEXOBS] = {}; /* observation data carrier-phase (cycle) */
    double P[NFREQ + NEXOBS] = {}; /* observation data pseudorange (m) */
    float D[NFREQ + NEXOBS] = {}; /* observation data doppler frequency (Hz) */
    double phasecorr = 0.0; /* phase correction */
    int facility = 0;      /* L6 facility(clas 2ch VRS) */
    int osr_i = -1;
};

struct pcv_t {          /* antenna parameter type */
    int sat = 0;        /* satellite number (0:receiver) */
    double off[NFREQ][3] = {}; /* phase center offset e/n/u or x/y/z (m) */
    double var[NFREQ][19] = {}; /* phase center variation (m) */
};

struct erp_t {};

struct prcopt_t {       /* processing options type */
    int nf = NFREQ;
    double elmin = 15.0 * D2R;
    int posopt[13] = {};
    int tidecorr = 0;
    pcv_t pcvr[2] = {};
    double antdel[2][3] = {};
    double odisp[2][6 * 11] = {};
    int isb = 0;
    int phasshft = 0;
    int isbbyprn = 0;
    char rectype[2][64] = {};
};

struct sol_t {          /* solution type */
    gtime_t time;       /* time (GPST) */
    double rr[6] = {};  /* position/velocity (m|m/s) */
    double dtr[6] = {}; /* receiver clock bias to time systems (s) */
    unsigned char stat = 0;
    unsigned char ns = 0;
    double chisq = 0.0;
    int network = 0;
};

struct ssat_t {         /* satellite status type */
    unsigned char sys = 0;  /* navigation system */
    unsigned char vs = 0;   /* valid satellite flag single */
    double azel[2] = {};    /* azimuth/elevation angles {az,el} (rad) */
    double resp[NFREQ] = {}; /* residuals of pseudorange (m) */
    double resc[NFREQ] = {}; /* residuals of carrier-phase (m) */
    unsigned char vsat[NFREQ] = {}; /* valid satellite flag */
    unsigned char vch[NFREQ] = {}; /* valid satellite channel flag (for debug only)*/
    unsigned char snr[NFREQ] = {}; /* signal strength (0.25 dBHz) */
    unsigned char fix[NFREQ] = {}; /* ambiguity fix flag (1:fix,2:float,3:hold) */
    unsigned char slip[NFREQ] = {}; /* cycle-slip flag */
    int lock[NFREQ] = {};   /* lock counter of phase */
    unsigned int outc[NFREQ] = {}; /* obs outage counter of phase */
    unsigned int slipc[NFREQ] = {}; /* cycle-slip counter */
    unsigned int rejc[NFREQ * 2] = {}; /* reject counter */
    double gf = 0.0;        /* geometry-free phase L1-L2 (m) */
    double gf2 = 0.0;       /* geometry-free phase L1-L5 (m) */
    double phw = 0.0;       /* phase windup (cycle) */
    gtime_t pt[2][NFREQ] = {}; /* previous carrier-phase time */
    double ph[2][NFREQ] = {}; /* previous carrier-phase observable (cycle) */
    double pbias[NFREQ] = {}; /* previous phase bias correction (ssr) [m] */
    double pbiasl0[NFREQ] = {}; /* previous l0 phase bias correction (ssr) [m] */
    double l0bias = 0.0;    /* previous l0 phase bias correction (ssr) [m] */
    int pbreset[NFREQ] = {}; /* phase bias reset flag */
    int outage = 0;         /* out age flag */
    unsigned char code[NFREQ + NEXOBS] = {}; /* code indicator (CODE_???) */
};

struct osrd_t {
    unsigned char sat = 0;         /* satellite number */
    unsigned char refsat = 0;      /* reference satellite number */
    int iode = 0;                  /* IODE */
    double clk = 0.0;              /* satellite time correction by SSR */
    double orb = 0.0;              /* satellite position correction by SSR */
    double trop = 0.0;             /* troposphere delay correction by SSR */
    double iono = 0.0;             /* ionosphere correction by SSR */
    double age = 0.0;              /* SSR age */
    double cbias[NFREQ + NEXOBS] = {}; /* satellite code bias by SSR */
    double pbias[NFREQ + NEXOBS] = {}; /* satellite phase bias by SSR */
    double l0bias = 0.0;           /* l0 phase bias by SSR */
    double discontinuity[NFREQ + NEXOBS] = {}; /* discontinuity indicator */
    double rho = 0.0;              /* distance between satellite and receiver */
    double dts = 0.0;              /* satellite clock correction from broadcast clock parameters */
    double relatv = 0.0;           /* relativity delay (shapiro time delay) */
    double earthtide = 0.0;        /* earthtide correction (solid earth tide and elimate permanent deformation) */
    double antr[NFREQ + NEXOBS] = {}; /* receiver antenna PCV */
    double wupL[NFREQ + NEXOBS] = {}; /* windup correction (m) */
    double compL[NFREQ + NEXOBS] = {}; /* compensate time variation */
    double CPC[NFREQ + NEXOBS] = {}; /* carrier-phase correction (m) */
    double PRC[NFREQ + NEXOBS] = {}; /* pseudorange correction (m) */
    double dCPC[NFREQ + NEXOBS] = {}; /* SD carrier-phase correction (m) */
    double dPRC[NFREQ + NEXOBS] = {}; /* SD pseudorange correction (m) */
    double c[NFREQ + NEXOBS] = {};    /* carrier-phase from measurement model(m) */
    double p[NFREQ + NEXOBS] = {};    /* pseudorange from measurement model(m) */
    double resc[NFREQ + NEXOBS] = {}; /* residual carrier-phase (m) */
    double resp[NFREQ + NEXOBS] = {}; /* residual pseudorange (m) */
    double dresc[NFREQ + NEXOBS] = {}; /* SD carrier-phase residual (m) */
    double dresp[NFREQ + NEXOBS] = {}; /* SD pseudorange residual (m) */
    double ddisp = 0.0;               /* SD disperse residual  (m) */
    double dL0 = 0.0;                 /* SD non-disperse residual (m) */
    double sis = 0.0;
};

struct bridge_freq_t {
    int signal_index = -1;
    double PRC = 0.0;
    double CPC = 0.0;
    double iono = 0.0;
    double antr = 0.0;
    double wupL = 0.0;
    double compL = 0.0;
    double orb = 0.0;
    double clk = 0.0;
    bool valid = false;
};

struct ssr_t {         /* SSR correction type */
    gtime_t t0[MAX_INDEX_SSR] = {}; /* epoch time (GPST) {eph,clk,hrclk,ura,cbias,pbias,vtec,trop,stec} */
    int nsig = 0;       /* number of signals used for ssrg cssr */
    int smode[MAXCODE] = {}; /* signnal mode for ssrg ssrig ssrcg*/
    double pbias[MAXCODE] = {};
    double cbias[MAXCODE] = {};
    bridge_freq_t bridge[NFREQ] = {};
};

struct stec_grid_point_t {
    int sat = 0;
    gtime_t time;
};

struct stec_t {
    int n = 0;
    stec_grid_point_t data[MAXSAT] = {};
};

struct nav_t {         /* navigation data type */
    double lam[MAXSAT][NFREQ] = {}; /* carrier wave lengths (m) */
    ssr_t ssr_ch[SSR_CH_NUM][MAXSAT] = {}; /* SSR corrections(2channel) */
    int rtcmmode = RTCMMODE_CSSR;   /* rtcm mode (5:cssr) */
    erp_t erp;                      /* earth rotation parameters */
    void* oload[32] = {};
    stec_t stec[MAX_NGRID] = {};
    int ionoreset = FALSE;
    int filreset = FALSE;
    int invtrop = 0;        /* valid/invalid flag of trop correction (0:valid 1:invalid) */
};

struct grid_t {
    double Gmat[MAX_NGRID * MAX_NGRID] = {};
    double weight[MAX_NGRID] = {};
    double Emat[MAX_NGRID] = {};
    int index[MAX_NGRID] = {};
    int network = 1;
    int num = 1;
};

struct rtk_t {         /* RTK control/result type */
    sol_t sol;         /* RTK solution */
    double rb[6] = {}; /* base position/velocity (ecef) (m|m/s) */
    int nx = ppp_claslib_full::kClasNx, na = ppp_claslib_full::kClasNx;
    double tt = 0.0;   /* time difference between current and previous (s) */
    double* x = nullptr;
    double* P = nullptr;
    double* Q = nullptr;
    double* xa = nullptr;
    double* Pa = nullptr;
    int nfix = 0;
    ssat_t ssat[MAXSAT] = {}; /* satellite status */
    prcopt_t opt;       /* processing options */
};

struct ClasnatMeasurementRow {
    Eigen::RowVectorXd H;
    double residual = 0.0;
    double variance = 0.0;
    double reference_variance = 0.0;
    SatelliteId satellite;
    SatelliteId ambiguity_satellite;
    SatelliteId reference_satellite;
    bool is_phase = false;
    int signal_index = 0;
    int freq_index = 0;
    ppp_clas::MeasurementRow::ModelComponents components;
};

struct ClasnatMeasurementBuild {
    std::vector<ClasnatMeasurementRow> rows;
    std::set<SatelliteId> observed_ambiguities;
};

int zdres(const obsd_t *obs_org,
          int n, const double *rs, const double *dts,
          const double *vare, const int *svh, nav_t *nav,
          double *x, double *y,
          double *e, double *azel, rtk_t *rtk,
          int osrlog, double *cpc, gtime_t *pt0, grid_t *grid,
          ssat_t *ssat, prcopt_t *opt, sol_t *sol, osrd_t *osr, int ch);

ClasnatMeasurementBuild buildEpochMeasurementsClasnatZdres(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_claslib_full::ClaslibRtkState& rtk,
    const ppp_shared::PPPConfig& config,
    const Vector3d& receiver_position,
    double receiver_clock_bias_m,
    double trop_zenith_m);

}  // namespace libgnss::ppp_clasnat_zdres
