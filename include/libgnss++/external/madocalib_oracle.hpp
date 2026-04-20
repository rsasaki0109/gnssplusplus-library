#pragma once

#include <libgnss++/algorithms/madoca_parity.hpp>

namespace libgnss::external::madocalib_oracle {

using AntennaPcv = libgnss::algorithms::madoca_parity::AntennaPcv;
using BroadcastEphemeris = libgnss::algorithms::madoca_parity::BroadcastEphemeris;
using GTime = libgnss::algorithms::madoca_parity::GTime;

bool available();

int satno(int sys, int prn);
int satsys(int sat, int* prn);

double ionmapf(const double pos[3], const double azel[2]);
double geodist(const double rs[3], const double rr[3], double e[3]);
double tropmodel(GTime time, const double pos[3], const double azel[2], double humi);
double tropmapf(GTime time, const double pos[3], const double azel[2], double* mapfw);
void antmodel(const AntennaPcv* pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[libgnss::algorithms::madoca_parity::kMadocalibNFreqPcv]);
void antmodel_s(
    const AntennaPcv* pcv,
    double nadir,
    double dant[libgnss::algorithms::madoca_parity::kMadocalibNFreqPcv]);
double eph2clk(GTime time, const BroadcastEphemeris* eph);
void eph2pos(GTime time,
             const BroadcastEphemeris* eph,
             double rs[3],
             double* dts,
             double* var);

}  // namespace libgnss::external::madocalib_oracle
