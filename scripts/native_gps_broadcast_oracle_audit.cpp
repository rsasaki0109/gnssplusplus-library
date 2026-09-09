// Diagnostic only: raw broadcast navigation, no receiver solutions or truth.
// Link the separately compiled external/madocalib ephemeris.c and rtkcmn.c.
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/core/navigation.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
extern "C" {
#include "rtklib.h"
}

int main(int argc, char** argv) {
    const bool galileo=argc==3 && std::string(argv[2])=="galileo";
    const bool glonass=argc==3 && std::string(argv[2])=="glonass";
    if (argc != 2 && !galileo && !glonass) return 2;
    const auto system=glonass ? libgnss::GNSSSystem::GLONASS :
        (galileo ? libgnss::GNSSSystem::Galileo : libgnss::GNSSSystem::GPS);
    const int oracle_system=glonass ? SYS_GLO : (galileo ? SYS_GAL : SYS_GPS);
    libgnss::io::RINEXReader reader;
    libgnss::NavigationData nav;
    if (!reader.open(argv[1]) || !reader.readNavigationData(nav)) return 3;
    std::size_t count = 0;
    double max_position_m = 0, max_clock_m = 0;
    for (int prn = 1; prn <= (glonass ? 27 : (galileo ? 36 : 32)); ++prn) {
        for (const auto& e : nav.getEphemeris({system, static_cast<uint8_t>(prn)})) {
            if (!e.valid) continue;
            eph_t oracle{};
            oracle.sat = satno(oracle_system, prn);
            if (!oracle.sat || satsys(oracle.sat,nullptr)!=oracle_system) return 8;
            oracle.toe = gpst2time(e.toe.week, e.toe.tow);
            oracle.toc = gpst2time(e.toc.week, e.toc.tow);
            oracle.toes = e.toes != 0 ? e.toes : e.toe.tow;
            oracle.A = e.sqrt_a * e.sqrt_a;
            oracle.e = e.e; oracle.i0 = e.i0; oracle.OMG0 = e.omega0;
            oracle.omg = e.omega; oracle.M0 = e.m0; oracle.deln = e.delta_n;
            oracle.OMGd = e.omega_dot; oracle.idot = e.idot;
            oracle.cuc = e.cuc; oracle.cus = e.cus;
            oracle.crc = e.crc; oracle.crs = e.crs;
            oracle.cic = e.cic; oracle.cis = e.cis;
            oracle.f0 = e.af0; oracle.f1 = e.af1; oracle.f2 = e.af2;
            geph_t glo{};
            glo.sat=oracle.sat;glo.toe=oracle.toe;
            glo.taun=e.glonass_taun;glo.gamn=e.glonass_gamn;
            for (int i=0;i<3;++i) {
                glo.pos[i]=e.glonass_position[i];glo.vel[i]=e.glonass_velocity[i];
                glo.acc[i]=e.glonass_acceleration[i];
            }
            const double interval=glonass ? 1800.0 : 3600.0;
            for (double offset : {-interval, 0.0, interval}) {
                const auto time = e.toe + offset;
                libgnss::Vector3d p, v;
                double clock, drift, reference[3], reference_clock, variance;
                if (!e.calculateSatelliteState(time, p, v, clock, drift)) return 4;
                if (glonass) geph2pos(gpst2time(time.week,time.tow),&glo,reference,&reference_clock,&variance);
                else eph2pos(gpst2time(time.week, time.tow), &oracle,
                            reference, &reference_clock, &variance);
                const double dp = (p - libgnss::Vector3d(reference[0], reference[1], reference[2])).norm();
                const double dc = std::abs(clock - reference_clock) * 299792458.0;
                if (!std::isfinite(dp) || !std::isfinite(dc)) return 5;
                max_position_m = std::max(max_position_m, dp);
                max_clock_m = std::max(max_clock_m, dc);
                ++count;
            }
        }
    }
    if (count == 0) return 6;
    std::cout << std::setprecision(17)
              << "{\"samples\":" << count << ",\"maximum_position_difference_m\":"
              << max_position_m << ",\"maximum_clock_difference_m\":"
              << max_clock_m << "}\n";
    // Fixed diagnostic threshold, not a positioning accuracy requirement.
    return max_position_m <= 0.001 && max_clock_m <= 0.001 ? 0 : 7;
}
