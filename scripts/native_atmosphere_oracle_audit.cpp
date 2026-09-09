// Synthetic diagnostic: no dataset or saved positioning input.
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
extern "C" {
#include "rtklib.h"
}
int main() {
    const double ion[8] = {0.1118e-7,-0.7451e-8,-0.5961e-7,0.1192e-6,
                           0.1167e6,-0.2294e6,-0.1311e6,0.1049e7};
    double high_iono = 0, high_trop = 0, low_iono = 0, low_trop = 0;
    std::size_t high_count = 0, low_count = 0;
    for (double lat : {-60., 0., 37., 60.})
    for (double height : {-50., 0., 100., 1000.})
    for (double elevation : {-1., 0., 1., 3., 5., 15., 45., 90.})
    for (double azimuth : {0., 90., 180., 270.})
    for (double tow : {0., 246598., 604799.}) {
        double pos[3] = {lat*M_PI/180, -122*M_PI/180, height};
        double azel[2] = {azimuth*M_PI/180, elevation*M_PI/180};
        const auto ecef = libgnss::geodetic2ecef(pos[0],pos[1],pos[2]);
        const auto time = gpst2time(2172,tow);
        const double di = std::abs(libgnss::models::ionoDelayKlobuchar(
            pos[0],pos[1],azel[0],azel[1],tow,ion,ion+4)-ionmodel(time,ion,pos,azel));
        const double dt = std::abs(libgnss::models::tropDelaySaastamoinen(ecef,azel[1])-
                                  tropmodel(time,pos,azel,0.7));
        if (!std::isfinite(di) || !std::isfinite(dt)) return 2;
        if (elevation >= 5) {
            ++high_count; high_iono=std::max(high_iono,di); high_trop=std::max(high_trop,dt);
        } else {
            ++low_count; low_iono=std::max(low_iono,di); low_trop=std::max(low_trop,dt);
        }
    }
    std::cout << std::setprecision(17)
              << "{\"at_least_5deg_samples\":" << high_count
              << ",\"maximum_iono_difference_m\":" << high_iono
              << ",\"maximum_trop_difference_m\":" << high_trop
              << ",\"below_5deg_samples\":" << low_count
              << ",\"low_maximum_iono_difference_m\":" << low_iono
              << ",\"low_maximum_trop_difference_m\":" << low_trop << "}\n";
    return high_iono <= 1e-6 && high_trop <= 1e-6 ? 0 : 3;
}
