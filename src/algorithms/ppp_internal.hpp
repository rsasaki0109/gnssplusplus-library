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
    const size_t first = text.find_first_not_of(' ');
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = text.find_last_not_of(' ');
    return text.substr(first, last - first + 1);
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
