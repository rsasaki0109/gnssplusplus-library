#pragma once

#include <libgnss++/algorithms/clasnat_parity.hpp>
#include <libgnss++/core/types.hpp>

namespace libgnss::external::claslib_oracle {

using GTime = GNSSTime;
using ReceiverPcvModel = libgnss::clasnat_parity::ReceiverPcvModel;
using SatposSsrOutput = libgnss::clasnat_parity::SatposSsrOutput;
using CorrmeasOutput = libgnss::clasnat_parity::CorrmeasOutput;

bool available();

void tidedisp(const GTime& gpst,
              const double rr[3],
              const double erpv[5],
              double disp_out[3]);

void windupcorr(const GTime& time,
                const double rs[6],
                const double rr[3],
                double& phw_io);

void antmodel(const ReceiverPcvModel& pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[libgnss::clasnat_parity::kParityMaxFreq]);

double ionmapf(const double pos[3], const double azel[2]);

double prectrop(const GTime& time,
                const double pos[3],
                const double azel[2],
                double zwd,
                double ztd);

bool satpos_ssr(const GTime& teph,
                const GTime& time,
                int sat,
                SatposSsrOutput& out);

bool corrmeas(const libgnss::clasnat_parity::CorrmeasInput& input, CorrmeasOutput& out);
bool corrmeas(int sample_index, CorrmeasOutput& out);
bool corrmeas(CorrmeasOutput& out);

}  // namespace libgnss::external::claslib_oracle
