#pragma once

#include <libgnss++/algorithms/clasnat_parity.hpp>
#include <libgnss++/core/types.hpp>

namespace libgnss::external::claslib_oracle {

using GTime = GNSSTime;
using ReceiverPcvModel = libgnss::clasnat_parity::ReceiverPcvModel;
using SatposBroadcastEphemeris = libgnss::clasnat_parity::SatposBroadcastEphemeris;
using SatposSsrOutput = libgnss::clasnat_parity::SatposSsrOutput;
using SatposSsrInput = libgnss::clasnat_parity::SatposSsrInput;
using CorrmeasOutput = libgnss::clasnat_parity::CorrmeasOutput;
using SatAntOffInput = libgnss::clasnat_parity::SatAntOffInput;
using TropGridInput = libgnss::clasnat_parity::TropGridInput;
using TropGridOutput = libgnss::clasnat_parity::TropGridOutput;
using StecGridInput = libgnss::clasnat_parity::StecGridInput;
using StecGridOutput = libgnss::clasnat_parity::StecGridOutput;
using TropmodelInput = libgnss::clasnat_parity::TropmodelInput;
using FilterInput = libgnss::clasnat_parity::FilterInput;
using FilterOutput = libgnss::clasnat_parity::FilterOutput;
using LambdaInput = libgnss::clasnat_parity::LambdaInput;
using LambdaOutput = libgnss::clasnat_parity::LambdaOutput;

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

double eph2clk(const GTime& time, const SatposBroadcastEphemeris& eph);

bool eph2pos(const GTime& time,
             const SatposBroadcastEphemeris& eph,
             double rs[3],
             double& dts,
             double& variance);

double geodist(const double rs[3], const double rr[3], double e[3]);

bool satpos_ssr(const GTime& teph,
                const GTime& time,
                int sat,
                SatposSsrOutput& out);
bool satpos_ssr(const SatposSsrInput& input, SatposSsrOutput& out);

bool corrmeas(const libgnss::clasnat_parity::CorrmeasInput& input, CorrmeasOutput& out);
bool corrmeas(int sample_index, CorrmeasOutput& out);
bool corrmeas(CorrmeasOutput& out);

void satantoff(const SatAntOffInput& input, double dant[3]);
bool compensatedisp(const libgnss::clasnat_parity::CorrmeasInput& input,
                    double compL[libgnss::clasnat_parity::kParityMaxFreq]);
bool trop_grid_data(const TropGridInput& input, TropGridOutput& out);
bool stec_grid_data(const StecGridInput& input, StecGridOutput& out);
double tropmodel(const TropmodelInput& input);
bool filter_update(const FilterInput& input, FilterOutput& out);
bool lambda_search(const LambdaInput& input, LambdaOutput& out);

}  // namespace libgnss::external::claslib_oracle
