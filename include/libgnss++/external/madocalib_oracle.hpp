#pragma once

#include <libgnss++/algorithms/madoca_parity.hpp>

namespace libgnss::external::madocalib_oracle {

using AntennaPcv = libgnss::algorithms::madoca_parity::AntennaPcv;
using BroadcastEphemeris = libgnss::algorithms::madoca_parity::BroadcastEphemeris;
using GTime = libgnss::algorithms::madoca_parity::GTime;
using MionoAreaFixture = libgnss::algorithms::madoca_parity::MionoAreaFixture;
using MionoCorrResult = libgnss::algorithms::madoca_parity::MionoCorrResult;

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
int mcssr_sel_biascode(int sys, int code);
int miono_get_corr(GTime time,
                   const double rr[3],
                   const MionoAreaFixture* areas,
                   int area_count,
                   MionoCorrResult* result);
int satpos(GTime time,
           GTime teph,
           int sat,
           int ephopt,
           const BroadcastEphemeris* ephs,
           int eph_count,
           double rs[6],
           double dts[2],
           double* var,
           int* svh);

}  // namespace libgnss::external::madocalib_oracle
