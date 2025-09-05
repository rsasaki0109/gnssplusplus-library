#include <libgnss++/core/types.hpp>
#include <sstream>
#include <iomanip>

namespace libgnss {

std::string SatelliteId::toString() const {
    char system_char;
    switch (system) {
        case GNSSSystem::GPS:     system_char = 'G'; break;
        case GNSSSystem::GLONASS: system_char = 'R'; break;
        case GNSSSystem::Galileo: system_char = 'E'; break;
        case GNSSSystem::BeiDou:  system_char = 'C'; break;
        case GNSSSystem::QZSS:    system_char = 'J'; break;
        case GNSSSystem::SBAS:    system_char = 'S'; break;
        case GNSSSystem::NavIC:   system_char = 'I'; break;
        default:                  system_char = 'U'; break;
    }
    std::string prn_str = std::to_string(prn);
    if (prn_str.length() < 2) {
        prn_str = "0" + prn_str;
    }
    return system_char + prn_str;
}

std::chrono::system_clock::time_point GNSSTime::toSystemTime() const {
    // GPS epoch: January 6, 1980 00:00:00 UTC
    const auto gps_epoch = std::chrono::system_clock::from_time_t(315964800);
    const auto duration = std::chrono::seconds(static_cast<long long>(week * 604800 + tow));
    return gps_epoch + duration;
}

GNSSTime GNSSTime::fromSystemTime(const std::chrono::system_clock::time_point& tp) {
    // GPS epoch: January 6, 1980 00:00:00 UTC
    const auto gps_epoch = std::chrono::system_clock::from_time_t(315964800);
    const auto duration = tp - gps_epoch;
    const auto total_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    
    int week = static_cast<int>(total_seconds / 604800);
    double tow = static_cast<double>(total_seconds % 604800);
    
    return GNSSTime(week, tow);
}

} // namespace libgnss
