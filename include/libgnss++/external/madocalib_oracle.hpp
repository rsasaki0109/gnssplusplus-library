#pragma once

#include <cstdint>

#include <libgnss++/io/madoca_l6.hpp>

namespace libgnss::external::madocalib_oracle {

bool available();

int satno(int sys, int prn);
int satsys(int sat, int* prn);

double ionmapf(const double pos[3], const double azel[2]);
double geodist(const double rs[3], const double rr[3], double e[3]);

int mcssrSelBiascode(int sys, int code);

std::uint32_t getbitu(const std::uint8_t* buff, int pos, int len);
std::int32_t getbits(const std::uint8_t* buff, int pos, int len);

// Decode a QZSS L6E file through MADOCALIB and copy the per-satellite SSR
// correction values into out[0..maxSat-1]. Returns the number of satellites
// with an update flag set, or -1 when the oracle is not linked or the file
// cannot be opened.
int decodeQzssL6eFile(const char* path, libgnss::io::MadocaSsrCorrection* out,
                      int maxSat);

}  // namespace libgnss::external::madocalib_oracle
