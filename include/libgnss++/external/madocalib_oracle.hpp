#pragma once

#include <libgnss++/algorithms/madoca_parity.hpp>

#include <string>
#include <vector>

namespace libgnss::external::madocalib_oracle {

using AntennaPcv = libgnss::algorithms::madoca_parity::AntennaPcv;
using BroadcastEphemeris = libgnss::algorithms::madoca_parity::BroadcastEphemeris;
using GTime = libgnss::algorithms::madoca_parity::GTime;
using MionoAreaFixture = libgnss::algorithms::madoca_parity::MionoAreaFixture;
using MionoCorrResult = libgnss::algorithms::madoca_parity::MionoCorrResult;

struct SsrCorrectionSnapshot {
    int sat = 0;
    GTime t0[6] = {};
    double deph[3] = {};
    double dclk[3] = {};
    int iod[6] = {};
    int iode = 0;
    int ura = 0;
    double cbias[libgnss::algorithms::madoca_parity::kMadocalibMaxCode] = {};
    double pbias[libgnss::algorithms::madoca_parity::kMadocalibMaxCode] = {};
    int vcbias[libgnss::algorithms::madoca_parity::kMadocalibMaxCode] = {};
    int vpbias[libgnss::algorithms::madoca_parity::kMadocalibMaxCode] = {};
    bool orbit_valid = false;
    bool clock_valid = false;
    bool code_bias_valid = false;
    bool phase_bias_valid = false;
    bool ura_valid = false;
};

bool available();
std::string rootDirectory();

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
std::vector<MionoCorrResult> decode_l6d_file(const std::string& path,
                                             int gps_week,
                                             const double rr[3]);
std::vector<MionoCorrResult> decode_l6d_files(const std::vector<std::string>& paths,
                                              int gps_week,
                                              const double rr[3]);
std::vector<SsrCorrectionSnapshot> decode_l6e_file(const std::string& path,
                                                   int gps_week);
std::vector<SsrCorrectionSnapshot> decode_l6e_files(const std::vector<std::string>& paths,
                                                    int gps_week);
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
