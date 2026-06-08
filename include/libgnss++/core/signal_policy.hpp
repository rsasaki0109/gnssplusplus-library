#pragma once

#include "types.hpp"

#include <string>

namespace libgnss::signal_policy {

inline bool isBeiDouGeoSatellite(const SatelliteId& sat) {
    if (sat.system != GNSSSystem::BeiDou) {
        return false;
    }
    return (sat.prn >= 1 && sat.prn <= 5) || (sat.prn >= 59 && sat.prn <= 63);
}

inline SignalType primarySignalForSystem(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return SignalType::GPS_L1CA;
        case GNSSSystem::GLONASS: return SignalType::GLO_L1CA;
        case GNSSSystem::Galileo: return SignalType::GAL_E1;
        case GNSSSystem::BeiDou: return SignalType::BDS_B1I;
        case GNSSSystem::QZSS: return SignalType::QZS_L1CA;
        case GNSSSystem::NavIC: return SignalType::GPS_L5;
        default: return SignalType::GPS_L1CA;
    }
}

inline SignalType secondarySignalForSystem(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return SignalType::GPS_L2C;
        case GNSSSystem::GLONASS: return SignalType::GLO_L2CA;
        case GNSSSystem::Galileo: return SignalType::GAL_E5A;
        case GNSSSystem::BeiDou: return SignalType::BDS_B2I;
        case GNSSSystem::QZSS: return SignalType::QZS_L2C;
        case GNSSSystem::NavIC: return SignalType::GPS_L5;
        default: return SignalType::GPS_L2C;
    }
}

inline int rinexBand(const std::string& obs_type) {
    if (obs_type.size() < 2 || obs_type[1] < '0' || obs_type[1] > '9') {
        return -1;
    }
    return obs_type[1] - '0';
}

inline bool trySignalForObservationType(GNSSSystem system,
                                        const std::string& obs_type,
                                        SignalType& signal) {
    const int band = rinexBand(obs_type);
    switch (system) {
        case GNSSSystem::GPS:
            if (band == 1) { signal = SignalType::GPS_L1CA; return true; }
            if (band == 2) { signal = SignalType::GPS_L2C; return true; }
            if (band == 5) { signal = SignalType::GPS_L5; return true; }
            break;
        case GNSSSystem::GLONASS:
            if (band == 1) { signal = SignalType::GLO_L1CA; return true; }
            if (band == 2) { signal = SignalType::GLO_L2CA; return true; }
            if (band == 3) { signal = SignalType::GLO_L2P; return true; }
            break;
        case GNSSSystem::Galileo:
            if (band == 1) { signal = SignalType::GAL_E1; return true; }
            if (band == 5) { signal = SignalType::GAL_E5A; return true; }
            if (band == 6) { signal = SignalType::GAL_E6; return true; }
            if (band == 7 || band == 8) { signal = SignalType::GAL_E5B; return true; }
            break;
        case GNSSSystem::BeiDou:
            if (band == 1) { signal = SignalType::BDS_B1C; return true; }
            if (band == 2) { signal = SignalType::BDS_B1I; return true; }
            if (band == 5) { signal = SignalType::BDS_B2A; return true; }
            if (band == 6) { signal = SignalType::BDS_B3I; return true; }
            if (band == 7) { signal = SignalType::BDS_B2I; return true; }
            if (band == 8) { signal = SignalType::BDS_B2A; return true; }
            break;
        case GNSSSystem::QZSS:
            if (band == 1) { signal = SignalType::QZS_L1CA; return true; }
            if (band == 2) { signal = SignalType::QZS_L2C; return true; }
            if (band == 5) { signal = SignalType::QZS_L5; return true; }
            break;
        case GNSSSystem::NavIC:
            if (band == 5) { signal = SignalType::GPS_L5; return true; }
            break;
        default:
            break;
    }
    return false;
}

inline SignalType signalForObservationType(GNSSSystem system,
                                           const std::string& obs_type,
                                           bool primary_fallback) {
    SignalType signal = primary_fallback ? primarySignalForSystem(system)
                                         : secondarySignalForSystem(system);
    if (trySignalForObservationType(system, obs_type, signal)) {
        return signal;
    }
    return primary_fallback ? primarySignalForSystem(system) : secondarySignalForSystem(system);
}

inline bool isPrimarySignal(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            return signal == SignalType::GPS_L1CA;
        case GNSSSystem::GLONASS:
            return signal == SignalType::GLO_L1CA || signal == SignalType::GLO_L1P;
        case GNSSSystem::Galileo:
            return signal == SignalType::GAL_E1;
        case GNSSSystem::BeiDou:
            return signal == SignalType::BDS_B1I || signal == SignalType::BDS_B1C;
        case GNSSSystem::QZSS:
            return signal == SignalType::QZS_L1CA;
        case GNSSSystem::NavIC:
            return signal == SignalType::GPS_L5;
        default:
            return false;
    }
}

inline bool isSecondarySignal(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            return signal == SignalType::GPS_L2C || signal == SignalType::GPS_L5;
        case GNSSSystem::GLONASS:
            return signal == SignalType::GLO_L2CA || signal == SignalType::GLO_L2P;
        case GNSSSystem::Galileo:
            return signal == SignalType::GAL_E5A ||
                   signal == SignalType::GAL_E5B ||
                   signal == SignalType::GAL_E6;
        case GNSSSystem::BeiDou:
            return signal == SignalType::BDS_B2I ||
                   signal == SignalType::BDS_B3I ||
                   signal == SignalType::BDS_B2A;
        case GNSSSystem::QZSS:
            return signal == SignalType::QZS_L2C ||
                   signal == SignalType::QZS_L5;
        default:
            return false;
    }
}

inline int signalPriority(GNSSSystem system, SignalType signal, bool primary) {
    if (primary) {
        switch (system) {
            case GNSSSystem::GPS:
                return signal == SignalType::GPS_L1CA ? 0 : 100;
            case GNSSSystem::GLONASS:
                if (signal == SignalType::GLO_L1CA) return 0;
                if (signal == SignalType::GLO_L1P) return 1;
                return 100;
            case GNSSSystem::Galileo:
                return signal == SignalType::GAL_E1 ? 0 : 100;
            case GNSSSystem::BeiDou:
                if (signal == SignalType::BDS_B1I) return 0;
                if (signal == SignalType::BDS_B1C) return 1;
                return 100;
            case GNSSSystem::QZSS:
                return signal == SignalType::QZS_L1CA ? 0 : 100;
            case GNSSSystem::NavIC:
                return signal == SignalType::GPS_L5 ? 0 : 100;
            default:
                return 100;
        }
    }

    switch (system) {
        case GNSSSystem::GPS:
            if (signal == SignalType::GPS_L2C) return 0;
            if (signal == SignalType::GPS_L5) return 1;
            return 100;
        case GNSSSystem::GLONASS:
            if (signal == SignalType::GLO_L2CA) return 0;
            if (signal == SignalType::GLO_L2P) return 1;
            return 100;
        case GNSSSystem::Galileo:
            if (signal == SignalType::GAL_E5A) return 0;
            if (signal == SignalType::GAL_E5B) return 1;
            if (signal == SignalType::GAL_E6) return 2;
            return 100;
        case GNSSSystem::BeiDou:
            if (signal == SignalType::BDS_B2I) return 0;
            if (signal == SignalType::BDS_B3I) return 1;
            if (signal == SignalType::BDS_B2A) return 2;
            return 100;
        case GNSSSystem::QZSS:
            if (signal == SignalType::QZS_L2C) return 0;
            if (signal == SignalType::QZS_L5) return 1;
            return 100;
        default:
            return 100;
    }
}

inline int observationPriority(GNSSSystem system, const std::string& obs_type, bool primary) {
    SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
    if (!trySignalForObservationType(system, obs_type, signal)) {
        return 100;
    }
    if (primary && !isPrimarySignal(system, signal)) {
        return 100;
    }
    if (!primary && !isSecondarySignal(system, signal)) {
        return 100;
    }
    return signalPriority(system, signal, primary);
}

}  // namespace libgnss::signal_policy
