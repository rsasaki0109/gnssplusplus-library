#pragma once

namespace libgnss::external::madocalib_oracle {

bool available();

int satno(int sys, int prn);
int satsys(int sat, int* prn);

double ionmapf(const double pos[3], const double azel[2]);
double geodist(const double rs[3], const double rr[3], double e[3]);

}  // namespace libgnss::external::madocalib_oracle
