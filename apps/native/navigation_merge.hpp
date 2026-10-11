#pragma once

// Broadcast-navigation merge shared by gnss_replay and gnss_live.

#include <libgnss++/core/navigation.hpp>

#include <cstddef>

namespace libgnss_apps {

inline void mergeNavigationData(libgnss::NavigationData& dst, const libgnss::NavigationData& src) {
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            dst.addEphemeris(eph);
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
}

// Merge navigation that arrived on a live stream (RTCM 1019/1020/...). Casters rebroadcast every
// ephemeris every 30-60 s, so the plain merge above grows the database and the per-satellite
// lookup cost without bound; here a record already held (same IODE/toe/toc) is skipped.
// Returns the number of ephemerides actually added.
inline size_t mergeNavigationDataIfNew(libgnss::NavigationData& dst, const libgnss::NavigationData& src) {
    size_t added = 0;
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            if (dst.addEphemerisIfNew(eph)) {
                ++added;
            }
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
    return added;
}

}  // namespace libgnss_apps
