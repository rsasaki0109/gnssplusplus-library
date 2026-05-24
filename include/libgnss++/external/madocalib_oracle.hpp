#pragma once

#include <cstdint>

#include <libgnss++/io/madoca_l6.hpp>
#include <libgnss++/io/madoca_l6d.hpp>

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

// QZSS L6D (wide area ionosphere) lockstep stepping interface. These mirror the
// MADOCALIB postpos.c update_qzssl6d() glue so the native decoder can be driven
// byte-for-byte alongside the oracle and compared after each decoded message.
// l6dCreate heap-allocates an mdcl6d_t and calls init_miono(); the handle is
// opaque (nullptr when the oracle is not linked). Always pair with l6dDestroy.
void* l6dCreate(const double ep[6]);
int l6dInputByte(void* handle, std::uint8_t data);
void l6dRegion(void* handle, libgnss::io::MadocaIonoRegion* out, int* rid);
void l6dClearRegion(void* handle);
void l6dDestroy(void* handle);

}  // namespace libgnss::external::madocalib_oracle
