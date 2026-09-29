#pragma once

// RTKLIB rtkcmn.c GPST time helpers shared by the MADOCA-PPP L6E (madoca_l6.cpp)
// and L6D (madoca_l6d.cpp) decoders for epoch reconstruction.

#include <libgnss++/io/madoca_l6.hpp>  // MadocaGtime

#include <cmath>
#include <cstdint>

namespace libgnss::io::madoca_time_internal {

inline constexpr double kGpst0[6] = {1980.0, 1.0, 6.0, 0.0, 0.0, 0.0};

inline MadocaGtime epoch2time(const double* ep) {
    static const int doy[] = {1, 32, 60, 91, 121, 152, 182,
                              213, 244, 274, 305, 335};
    MadocaGtime time;
    const int year = static_cast<int>(ep[0]);
    const int mon = static_cast<int>(ep[1]);
    const int day = static_cast<int>(ep[2]);
    if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) {
        return time;
    }
    const int days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] +
                     day - 2 + ((year % 4 == 0 && mon >= 3) ? 1 : 0);
    const int sec = static_cast<int>(std::floor(ep[5]));
    time.time = static_cast<std::int64_t>(days) * 86400 +
                static_cast<int>(ep[3]) * 3600 + static_cast<int>(ep[4]) * 60 + sec;
    time.sec = ep[5] - sec;
    return time;
}

inline double time2gpst(MadocaGtime t, int* week) {
    const MadocaGtime t0 = epoch2time(kGpst0);
    const std::int64_t sec = t.time - t0.time;
    const int w = static_cast<int>(sec / (86400 * 7));
    if (week != nullptr) {
        *week = w;
    }
    return static_cast<double>(sec - static_cast<std::int64_t>(w) * 86400 * 7) + t.sec;
}

inline MadocaGtime gpst2time(int week, double sec) {
    MadocaGtime t = epoch2time(kGpst0);
    if (sec < -1e9 || 1e9 < sec) {
        sec = 0.0;
    }
    t.time += static_cast<std::int64_t>(86400) * 7 * week + static_cast<int>(sec);
    t.sec = sec - static_cast<int>(sec);
    return t;
}

inline void adjweek(MadocaGtime* gt, double tow) {
    // gt is seeded with a nonzero reference epoch, so the RTKLIB
    // "if (gt->time == 0) get cpu time" branch never applies here.
    int week;
    const double tow_p = time2gpst(*gt, &week);
    if (tow < tow_p - 302400.0) {
        tow += 604800.0;
    } else if (tow > tow_p + 302400.0) {
        tow -= 604800.0;
    }
    *gt = gpst2time(week, tow);
}

}  // namespace libgnss::io::madoca_time_internal
