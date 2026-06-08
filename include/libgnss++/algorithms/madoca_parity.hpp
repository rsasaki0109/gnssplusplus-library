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

// Observation code identifiers (RTKLIB rtklib.h CODE_* enum). Only the codes
// referenced by mcssr_sel_biascode are mirrored here. Values must match RTKLIB
// exactly so bias-code selection parity holds against the MADOCALIB oracle.
inline constexpr int kCodeNone = 0;
inline constexpr int kCodeL1C = 1;
inline constexpr int kCodeL1P = 2;
inline constexpr int kCodeL1W = 3;
inline constexpr int kCodeL1S = 7;
inline constexpr int kCodeL1L = 8;
inline constexpr int kCodeL1E = 9;
inline constexpr int kCodeL1B = 11;
inline constexpr int kCodeL1X = 12;
inline constexpr int kCodeL2C = 14;
inline constexpr int kCodeL2S = 16;
inline constexpr int kCodeL2L = 17;
inline constexpr int kCodeL2X = 18;
inline constexpr int kCodeL2P = 19;
inline constexpr int kCodeL2W = 20;
inline constexpr int kCodeL5I = 24;
inline constexpr int kCodeL5Q = 25;
inline constexpr int kCodeL5X = 26;
inline constexpr int kCodeL7I = 27;
inline constexpr int kCodeL7Q = 28;
inline constexpr int kCodeL7X = 29;
inline constexpr int kCodeL6B = 31;
inline constexpr int kCodeL6C = 32;
inline constexpr int kCodeL6X = 33;
inline constexpr int kCodeL2I = 40;
inline constexpr int kCodeL2Q = 41;
inline constexpr int kCodeL6I = 42;
inline constexpr int kCodeL6Q = 43;
inline constexpr int kCodeL1D = 56;
inline constexpr int kCodeL5D = 57;
inline constexpr int kCodeL5P = 58;
inline constexpr int kCodeL7D = 61;

bool satnoAvailable();
bool satsysAvailable();
bool ionmapfAvailable();
bool geodistAvailable();
bool mcssrSelBiascodeAvailable();
bool getbituAvailable();
bool getbitsAvailable();

int satno(int sys, int prn);
int satsys(int sat, int* prn);

double ionmapf(const double pos[3], const double azel[2]);
double geodist(const double rs[3], const double rr[3], double e[3]);

// Maps an observation code to the MADOCA-PPP code/phase bias code for a given
// navigation system (IS-QZSS-MDC-004 section 5.5.3.1). Returns kCodeNone for
// codes outside the MADOCA-PPP applicable signal set.
int mcssrSelBiascode(int sys, int code);

// Extract len bits (len<=32) from byte buffer starting at bit position pos
// (MSB-first within each byte), as unsigned/sign-extended values. Mirrors
// RTKLIB getbitu/getbits, the foundation of the L6E/L6D frame decoders.
std::uint32_t getbitu(const std::uint8_t* buff, int pos, int len);
std::int32_t getbits(const std::uint8_t* buff, int pos, int len);

}  // namespace libgnss::algorithms::madoca_parity
