#pragma once

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::ppp_internal {

inline constexpr double kDefaultZenithDelayMeters = 2.3;

inline std::string trimCopy(const std::string& text) {
    const auto is_not_space = [](unsigned char ch) {
        return !std::isspace(ch);
    };
    const auto first_it = std::find_if(text.begin(), text.end(), is_not_space);
    if (first_it == text.end()) {
        return "";
    }
    const auto last_it = std::find_if(text.rbegin(), text.rend(), is_not_space).base();
    return std::string(first_it, last_it);
}

inline std::string normalizeAntennaType(const std::string& antenna_type) {
    std::string normalized = trimCopy(antenna_type);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return normalized;
}

inline bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

inline const char* signalFamilyName(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "GPS_L1CA";
        case SignalType::GPS_L1P: return "GPS_L1P";
        case SignalType::GPS_L2P: return "GPS_L2P";
        case SignalType::GPS_L2C: return "GPS_L2C";
        case SignalType::GPS_L5: return "GPS_L5";
        case SignalType::GLO_L1CA: return "GLO_L1CA";
        case SignalType::GLO_L1P: return "GLO_L1P";
        case SignalType::GLO_L2CA: return "GLO_L2CA";
        case SignalType::GLO_L2P: return "GLO_L2P";
        case SignalType::GAL_E1: return "GAL_E1";
        case SignalType::GAL_E5A: return "GAL_E5A";
        case SignalType::GAL_E5B: return "GAL_E5B";
        case SignalType::GAL_E6: return "GAL_E6";
        case SignalType::BDS_B1I: return "BDS_B1I";
        case SignalType::BDS_B2I: return "BDS_B2I";
        case SignalType::BDS_B3I: return "BDS_B3I";
        case SignalType::BDS_B1C: return "BDS_B1C";
        case SignalType::BDS_B2A: return "BDS_B2A";
        case SignalType::QZS_L1CA: return "QZS_L1CA";
        case SignalType::QZS_L2C: return "QZS_L2C";
        case SignalType::QZS_L5: return "QZS_L5";
        case SignalType::SIGNAL_TYPE_COUNT: return "UNKNOWN";
    }
    return "UNKNOWN";
}

inline std::vector<SignalType> primarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L1CA};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L1CA, SignalType::GLO_L1P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E1};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B1I, SignalType::BDS_B1C};
        case GNSSSystem::QZSS: return {SignalType::QZS_L1CA};
        default: return {};
    }
}

inline std::vector<SignalType> secondarySignals(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return {SignalType::GPS_L2C, SignalType::GPS_L5};
        case GNSSSystem::GLONASS: return {SignalType::GLO_L2CA, SignalType::GLO_L2P};
        case GNSSSystem::Galileo: return {SignalType::GAL_E5A, SignalType::GAL_E5B, SignalType::GAL_E6};
        case GNSSSystem::BeiDou: return {SignalType::BDS_B2I, SignalType::BDS_B2A, SignalType::BDS_B3I};
        case GNSSSystem::QZSS: return {SignalType::QZS_L2C, SignalType::QZS_L5};
        default: return {};
    }
}

inline const Observation* findObservationForSignals(const ObservationData& obs,
                                                    const SatelliteId& sat,
                                                    const std::vector<SignalType>& candidates) {
    for (const auto signal : candidates) {
        const Observation* candidate = obs.getObservation(sat, signal);
        if (candidate == nullptr || !candidate->valid || !candidate->has_pseudorange) {
            continue;
        }
        if (candidate->pseudorange <= 0.0 || !std::isfinite(candidate->pseudorange)) {
            continue;
        }
        return candidate;
    }
    return nullptr;
}

inline const Observation* findCarrierObservationForSignals(const ObservationData& obs,
                                                           const SatelliteId& sat,
                                                           const std::vector<SignalType>& candidates) {
    return ppp_utils::findCarrierObservation(obs, sat, candidates);
}

inline Vector3d ssrRacToEcef(const Vector3d& position_ecef,
                             const Vector3d& velocity_ecef,
                             const Vector3d& rac_correction) {
    return ppp_utils::ssrRacToEcef(position_ecef, velocity_ecef, rac_correction);
}

inline double safeVariance(double variance, double floor_value) {
    if (!std::isfinite(variance) || variance <= 0.0) {
        return floor_value;
    }
    return std::max(variance, floor_value);
}

}  // namespace libgnss::ppp_internal
