#pragma once

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <cstddef>
#include <string>

namespace libgnss_apps {

inline bool isPrimaryPdSignal(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GAL_E1:
        case libgnss::SignalType::BDS_B1I:
        case libgnss::SignalType::BDS_B1C:
        case libgnss::SignalType::QZS_L1CA:
            return true;
        default:
            return false;
    }
}

inline std::string signalName(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA: return "GPS_L1CA";
        case libgnss::SignalType::GLO_L1CA: return "GLO_L1CA";
        case libgnss::SignalType::GAL_E1: return "GAL_E1";
        case libgnss::SignalType::BDS_B1I: return "BDS_B1I";
        case libgnss::SignalType::BDS_B1C: return "BDS_B1C";
        case libgnss::SignalType::QZS_L1CA: return "QZS_L1CA";
        default: return "UNKNOWN";
    }
}

inline bool isHealthyForPositioning(const libgnss::Observation& observation,
                                    const libgnss::Ephemeris& eph) {
    int sv_health = static_cast<int>(eph.health);
    if (observation.satellite.system == libgnss::GNSSSystem::QZSS) {
        sv_health &= 0xFE;
    }
    return sv_health == 0;
}

inline std::size_t clockGroup(libgnss::GNSSSystem system) {
    switch (system) {
        case libgnss::GNSSSystem::GPS:
            return 0U;
        case libgnss::GNSSSystem::GLONASS:
            return 1U;
        case libgnss::GNSSSystem::Galileo:
            return 2U;
        case libgnss::GNSSSystem::QZSS:
            return 3U;
        case libgnss::GNSSSystem::BeiDou:
            return 4U;
        default:
            return 0U;
    }
}

inline double groupDelayCorrectionMeters(const libgnss::Observation& observation,
                                         const libgnss::Ephemeris& eph) {
    switch (observation.satellite.system) {
        case libgnss::GNSSSystem::GPS:
        case libgnss::GNSSSystem::QZSS:
        case libgnss::GNSSSystem::Galileo:
            return eph.tgd * libgnss::constants::SPEED_OF_LIGHT;
        case libgnss::GNSSSystem::BeiDou:
            switch (observation.signal) {
                case libgnss::SignalType::BDS_B1I:
                case libgnss::SignalType::BDS_B1C:
                    return eph.tgd * libgnss::constants::SPEED_OF_LIGHT;
                case libgnss::SignalType::BDS_B2I:
                case libgnss::SignalType::BDS_B2A:
                    return eph.tgd_secondary * libgnss::constants::SPEED_OF_LIGHT;
                default:
                    return 0.0;
            }
        default:
            return 0.0;
    }
}

inline double sagnacRangeCorrection(const libgnss::Vector3d& satellite_position,
                                    const libgnss::Vector3d& receiver_position) {
    return libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
           (satellite_position(0) * receiver_position(1) -
            satellite_position(1) * receiver_position(0));
}

}  // namespace libgnss_apps
