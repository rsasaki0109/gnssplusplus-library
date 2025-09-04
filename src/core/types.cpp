#include <libgnss++/core/types.hpp>
#include <sstream>
#include <iomanip>

namespace libgnss {

std::string SatelliteId::toString() const {
    std::ostringstream oss;
    
    switch (system) {
        case GNSSSystem::GPS:
            oss << "G" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::GLONASS:
            oss << "R" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::Galileo:
            oss << "E" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::BeiDou:
            oss << "C" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::QZSS:
            oss << "J" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::SBAS:
            oss << "S" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        case GNSSSystem::NavIC:
            oss << "I" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
        default:
            oss << "U" << std::setfill('0') << std::setw(2) << static_cast<int>(prn);
            break;
    }
    
    return oss.str();
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
