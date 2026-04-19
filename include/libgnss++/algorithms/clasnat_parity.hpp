#pragma once

#include <libgnss++/core/types.hpp>

#include <array>

namespace libgnss::clasnat_parity {

constexpr int kParityMaxFreq = 3;
constexpr int kParityPcvGridSize = 19;

using PcvOffsets = std::array<std::array<double, 3>, kParityMaxFreq>;
using PcvVariations = std::array<std::array<double, kParityPcvGridSize>, kParityMaxFreq>;

struct ReceiverPcvModel {
    PcvOffsets offsets_m{};
    PcvVariations variations_m{};
};

struct SatposSsrOutput {
    double rs[6] = {};
    double dts[2] = {};
    double variance = 0.0;
    int svh = 0;
};

struct SatposBroadcastEphemeris {
    int sat = 0;
    int iode = 0;
    int iodc = 0;
    int sva = 0;
    int svh = 0;
    int week = 0;
    int code = 0;
    int flag = 0;
    GNSSTime toe;
    GNSSTime toc;
    GNSSTime ttr;
    double A = 0.0;
    double e = 0.0;
    double i0 = 0.0;
    double OMG0 = 0.0;
    double omg = 0.0;
    double M0 = 0.0;
    double deln = 0.0;
    double OMGd = 0.0;
    double idot = 0.0;
    double crc = 0.0;
    double crs = 0.0;
    double cuc = 0.0;
    double cus = 0.0;
    double cic = 0.0;
    double cis = 0.0;
    double toes = 0.0;
    double fit = 0.0;
    double f0 = 0.0;
    double f1 = 0.0;
    double f2 = 0.0;
    double tgd[4] = {};
    double Adot = 0.0;
    double ndot = 0.0;
    bool valid = false;
};

struct SatposSsrCorrection {
    GNSSTime t0[9] = {};
    double udi[9] = {};
    int iod[9] = {};
    int iode = -1;
    int iodcrc = 0;
    int ura = 0;
    int refd = 0;
    double deph[3] = {};
    double ddeph[3] = {};
    double dclk[3] = {};
    double hrclk = 0.0;
};

struct SatposSsrInput {
    GNSSTime teph;
    GNSSTime time;
    int sat = 0;
    SatposBroadcastEphemeris eph;
    SatposSsrCorrection ssr;
    double receiver_position[3] = {};
    bool apply_satellite_antenna_offset = false;
};

struct CorrmeasOutput {
    int num_frequencies = kParityMaxFreq;
    double prc[kParityMaxFreq] = {};
    double cpc[kParityMaxFreq] = {};
    double iono = 0.0;
    double code_bias[kParityMaxFreq] = {};
    double phase_bias[kParityMaxFreq] = {};
    double phase_compensation[kParityMaxFreq] = {};
    double receiver_antenna[kParityMaxFreq] = {};
    double windup_m[kParityMaxFreq] = {};
};

struct CorrmeasInput {
    GNSSTime time;
    int sat = 1;
    int num_frequencies = kParityMaxFreq;
    unsigned char code[kParityMaxFreq] = {};
    double carrier_phase_cycles[kParityMaxFreq] = {};
    double pseudorange_m[kParityMaxFreq] = {};
    double wavelength_m[kParityMaxFreq] = {};
    double receiver_pos[3] = {};
    double azel[2] = {};
    ReceiverPcvModel receiver_pcv;
    double antenna_delta[3] = {};
    int antenna_pcv_option = 1;
    int compensation_option = 0;
    int phase_code_timing_option = 0;
    double phase_windup_cycles = 0.0;
    double stec_l1_m = 0.0;
    double stec_rate_mps = 0.0;
    double stec_rms_m = 0.0;
    GNSSTime stec_time;
    GNSSTime orbit_time;
    GNSSTime code_bias_time;
    GNSSTime phase_bias_time;
    double code_bias_m[kParityMaxFreq] = {};
    double phase_bias_m[kParityMaxFreq] = {};
    double trop_m = 0.0;
    double relativity_m = 0.0;
    unsigned char slip[kParityMaxFreq] = {};
    int phase_bias_reset[kParityMaxFreq] = {};
};

CorrmeasInput makeCorrmeasInput(int sample_index);
SatposSsrInput makeSatposSsrInput(int sample_index);

bool tidedispAvailable();
bool windupcorrAvailable();
bool antmodelAvailable();
bool ionmapfAvailable();
bool prectropAvailable();
bool satposSsrAvailable();
bool corrmeasAvailable();

void tidedisp(const GNSSTime& gpst,
              const double rr[3],
              const double erpv[5],
              double disp_out[3]);

void windupcorr(const GNSSTime& time,
                const double rs[6],
                const double rr[3],
                double& phw_io);

void antmodel(const ReceiverPcvModel& pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[kParityMaxFreq]);

double ionmapf(const double pos[3], const double azel[2]);

double prectrop(const GNSSTime& time,
                const double pos[3],
                const double azel[2],
                double zwd,
                double ztd);

bool satpos_ssr(const GNSSTime& teph,
                const GNSSTime& time,
                int sat,
                SatposSsrOutput& out);
bool satpos_ssr(const SatposSsrInput& input, SatposSsrOutput& out);

bool corrmeas(const CorrmeasInput& input, CorrmeasOutput& out);
bool corrmeas(int sample_index, CorrmeasOutput& out);
bool corrmeas(CorrmeasOutput& out);

}  // namespace libgnss::clasnat_parity
