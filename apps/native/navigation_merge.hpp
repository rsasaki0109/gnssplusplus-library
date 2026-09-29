#pragma once

// Broadcast-navigation merge shared by gnss_replay and gnss_live.

#include <libgnss++/core/navigation.hpp>

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

}  // namespace libgnss_apps
