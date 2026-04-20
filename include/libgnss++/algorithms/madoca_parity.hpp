#pragma once

#include <cstdint>

namespace libgnss::algorithms::madoca_parity {

inline constexpr double kOracleTolerance = 1e-6;

inline constexpr int kSysNone = 0x000;
inline constexpr int kSysGps = 0x001;
inline constexpr int kSysSbs = 0x002;
inline constexpr int kSysGlo = 0x004;
inline constexpr int kSysGal = 0x008;
inline constexpr int kSysQzs = 0x010;
inline constexpr int kSysCmp = 0x020;
inline constexpr int kSysIrn = 0x040;
inline constexpr int kSysLeo = 0x080;
inline constexpr int kSysBd2 = 0x100;

inline constexpr int kMadocalibMaxSat = 221;
inline constexpr int kMadocalibNFreqPcv = 12;
inline constexpr int kMadocalibPcvAngles = 19;

struct GTime {
    std::int64_t time = 0;
    double sec = 0.0;
};

struct AntennaPcv {
    double off[kMadocalibNFreqPcv][3] = {};
    double var[kMadocalibNFreqPcv][kMadocalibPcvAngles] = {};
};

struct BroadcastEphemeris {
    int sat = 0;
    int iode = 0;
    int iodc = 0;
    int sva = 0;
    int svh = 0;
    int week = 0;
    int code = 0;
    int flag = 0;
    GTime toe = {};
    GTime toc = {};
    GTime ttr = {};
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
    double tgd[6] = {};
    double Adot = 0.0;
    double ndot = 0.0;
};

bool satnoAvailable();
bool satsysAvailable();
bool ionmapfAvailable();
bool geodistAvailable();
bool tropmodelAvailable();
bool tropmapfAvailable();
bool antmodelAvailable();
bool antmodelSAvailable();
bool eph2clkAvailable();
bool eph2posAvailable();

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
              double dant[kMadocalibNFreqPcv]);
void antmodel_s(const AntennaPcv* pcv, double nadir, double dant[kMadocalibNFreqPcv]);
double eph2clk(GTime time, const BroadcastEphemeris* eph);
void eph2pos(GTime time,
             const BroadcastEphemeris* eph,
             double rs[3],
             double* dts,
             double* var);

}  // namespace libgnss::algorithms::madoca_parity
