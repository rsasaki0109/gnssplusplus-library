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

struct CorrmeasOutput {
    double prc[kParityMaxFreq] = {};
    double cpc[kParityMaxFreq] = {};
    double iono = 0.0;
    double receiver_antenna[kParityMaxFreq] = {};
    double windup_m[kParityMaxFreq] = {};
};

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

bool corrmeas(CorrmeasOutput& out);

}  // namespace libgnss::clasnat_parity
