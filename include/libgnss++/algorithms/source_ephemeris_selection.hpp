#pragma once
#include <libgnss++/core/navigation.hpp>
#include <cmath>
#include <stdexcept>

namespace libgnss::source_transmission_clock {
// Pinned MALIB defaults, broadcast/IODE-agnostic, Galileo I/NAV only.
// Input order is authoritative: equal-age ties choose the last record.
inline const Ephemeris* selectBroadcastMessage(
    const std::vector<Ephemeris>& records, const SatelliteId& satellite,
    const GNSSTime& selection_time) {
    if (!std::isfinite(selection_time.tow) || selection_time.tow < 0 ||
        selection_time.tow >= 604800 || satellite.prn == 0)
        throw std::invalid_argument("invalid broadcast selection key");
    double limit;
    switch (satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
        case GNSSSystem::NavIC: limit = 7201; break;
        case GNSSSystem::Galileo: limit = 14400; break;
        case GNSSSystem::BeiDou: limit = 21601; break;
        case GNSSSystem::GLONASS: limit = 1800; break;
        case GNSSSystem::SBAS: limit = 360; break;
        default: throw std::invalid_argument("unsupported broadcast selection system");
    }
    const Ephemeris* selected = nullptr;
    double closest = limit+1;
    for (const auto& eph : records) {
        if (!(eph.satellite == satellite) || !eph.valid) continue;
        if (!std::isfinite(eph.toe.tow) || eph.toe.tow < 0 || eph.toe.tow >= 604800)
            throw std::invalid_argument("invalid broadcast toe");
        const double delta = eph.toe-selection_time;
        if (satellite.system == GNSSSystem::Galileo &&
            (!(eph.data_source_code & (1 << 9)) || delta >= 0)) continue;
        const double age = std::abs(delta);
        if (age > limit) continue;
        if (age <= closest) { selected = &eph; closest = age; }
    }
    return selected;
}
} // namespace libgnss::source_transmission_clock
