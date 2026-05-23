#pragma once

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

bool satnoAvailable();
bool satsysAvailable();
bool ionmapfAvailable();
bool geodistAvailable();

int satno(int sys, int prn);
int satsys(int sat, int* prn);

double ionmapf(const double pos[3], const double azel[2]);
double geodist(const double rs[3], const double rr[3], double e[3]);

}  // namespace libgnss::algorithms::madoca_parity
