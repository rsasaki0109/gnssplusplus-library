#pragma once

#include <libgnss++/algorithms/madoca_parity.hpp>
#include <libgnss++/core/signals.hpp>

#include <cstdint>
#include <string_view>

namespace libgnss::algorithms::ppp_bias_identity {

struct ObservationCodeMapEntry {
    std::string_view observation_type;
    int rtklib_code;
};

inline constexpr ObservationCodeMapEntry kObservationCodeMap[] = {
    {"C1C", madoca_parity::kCodeL1C}, {"L1C", madoca_parity::kCodeL1C},
    {"C1P", madoca_parity::kCodeL1P}, {"L1P", madoca_parity::kCodeL1P},
    {"C1W", madoca_parity::kCodeL1W}, {"L1W", madoca_parity::kCodeL1W},
    {"C1S", madoca_parity::kCodeL1S}, {"L1S", madoca_parity::kCodeL1S},
    {"C1L", madoca_parity::kCodeL1L}, {"L1L", madoca_parity::kCodeL1L},
    {"C1E", madoca_parity::kCodeL1E}, {"L1E", madoca_parity::kCodeL1E},
    {"C1B", madoca_parity::kCodeL1B}, {"L1B", madoca_parity::kCodeL1B},
    {"C1X", madoca_parity::kCodeL1X}, {"L1X", madoca_parity::kCodeL1X},
    {"C2C", madoca_parity::kCodeL2C}, {"L2C", madoca_parity::kCodeL2C},
    {"C2S", madoca_parity::kCodeL2S}, {"L2S", madoca_parity::kCodeL2S},
    {"C2L", madoca_parity::kCodeL2L}, {"L2L", madoca_parity::kCodeL2L},
    {"C2X", madoca_parity::kCodeL2X}, {"L2X", madoca_parity::kCodeL2X},
    {"C2P", madoca_parity::kCodeL2P}, {"L2P", madoca_parity::kCodeL2P},
    {"C2W", madoca_parity::kCodeL2W}, {"L2W", madoca_parity::kCodeL2W},
    {"C5I", madoca_parity::kCodeL5I}, {"L5I", madoca_parity::kCodeL5I},
    {"C5Q", madoca_parity::kCodeL5Q}, {"L5Q", madoca_parity::kCodeL5Q},
    {"C5X", madoca_parity::kCodeL5X}, {"L5X", madoca_parity::kCodeL5X},
    {"C5D", madoca_parity::kCodeL5D}, {"L5D", madoca_parity::kCodeL5D},
    {"C5P", madoca_parity::kCodeL5P}, {"L5P", madoca_parity::kCodeL5P},
    {"C6B", madoca_parity::kCodeL6B}, {"L6B", madoca_parity::kCodeL6B},
    {"C6C", madoca_parity::kCodeL6C}, {"L6C", madoca_parity::kCodeL6C},
    {"C6X", madoca_parity::kCodeL6X}, {"L6X", madoca_parity::kCodeL6X},
    {"C6I", madoca_parity::kCodeL6I}, {"L6I", madoca_parity::kCodeL6I},
    {"C6Q", madoca_parity::kCodeL6Q}, {"L6Q", madoca_parity::kCodeL6Q},
    {"C7I", madoca_parity::kCodeL7I}, {"L7I", madoca_parity::kCodeL7I},
    {"C7Q", madoca_parity::kCodeL7Q}, {"L7Q", madoca_parity::kCodeL7Q},
    {"C7X", madoca_parity::kCodeL7X}, {"L7X", madoca_parity::kCodeL7X},
    {"C7D", madoca_parity::kCodeL7D}, {"L7D", madoca_parity::kCodeL7D},
};

inline int rtklibSystemForGnss(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS: return madoca_parity::kSysGps;
        case GNSSSystem::Galileo: return madoca_parity::kSysGal;
        case GNSSSystem::QZSS: return madoca_parity::kSysQzs;
        default: return madoca_parity::kSysNone;
    }
}

inline int rtklibCodeForObservationType(std::string_view observation_type) {
    for (const auto& entry : kObservationCodeMap) {
        if (observation_type == entry.observation_type) {
            return entry.rtklib_code;
        }
    }
    return madoca_parity::kCodeNone;
}

inline std::uint8_t madocaBiasIdentityIdForObservation(GNSSSystem system,
                                                       SignalType signal,
                                                       std::string_view observation_type,
                                                       bool exact_bias_identity) {
    if (!exact_bias_identity) {
        return rtcmSsrSignalId(system, signal);
    }
    const int rtklib_system = rtklibSystemForGnss(system);
    if (rtklib_system == madoca_parity::kSysNone) {
        return rtcmSsrSignalId(system, signal);
    }
    const int rtklib_code = rtklibCodeForObservationType(observation_type);
    const int bias_code = madoca_parity::mcssrSelBiascode(rtklib_system, rtklib_code);
    if (bias_code <= 0 || bias_code > 255) {
        return 0U;
    }
    return static_cast<std::uint8_t>(bias_code);
}

inline std::uint8_t rtcmSsrSignalIdForRtklibCode(GNSSSystem system, int code) {
    switch (system) {
        case GNSSSystem::GPS:
            switch (code) {
                case madoca_parity::kCodeL1C: return 2;
                case madoca_parity::kCodeL1P:
                case madoca_parity::kCodeL1W: return 3;
                case madoca_parity::kCodeL2S:
                case madoca_parity::kCodeL2L:
                case madoca_parity::kCodeL2X: return 8;
                case madoca_parity::kCodeL2P:
                case madoca_parity::kCodeL2W: return 9;
                case madoca_parity::kCodeL5I:
                case madoca_parity::kCodeL5Q:
                case madoca_parity::kCodeL5X: return 22;
            }
            break;
        case GNSSSystem::GLONASS:
            switch (code) {
                case madoca_parity::kCodeL1C: return 2;
                case madoca_parity::kCodeL1P: return 3;
                case madoca_parity::kCodeL2C: return 8;
                case madoca_parity::kCodeL2P: return 9;
            }
            break;
        case GNSSSystem::Galileo:
            switch (code) {
                case madoca_parity::kCodeL1C:
                case madoca_parity::kCodeL1X: return 2;
                case madoca_parity::kCodeL6B:
                case madoca_parity::kCodeL6C:
                case madoca_parity::kCodeL6X: return 8;
                case madoca_parity::kCodeL7I:
                case madoca_parity::kCodeL7Q:
                case madoca_parity::kCodeL7X: return 14;
                case madoca_parity::kCodeL5I:
                case madoca_parity::kCodeL5Q:
                case madoca_parity::kCodeL5X: return 22;
            }
            break;
        case GNSSSystem::BeiDou:
            switch (code) {
                case madoca_parity::kCodeL2I:
                case madoca_parity::kCodeL2Q: return 2;
                case madoca_parity::kCodeL6I:
                case madoca_parity::kCodeL6Q: return 8;
                case madoca_parity::kCodeL7I:
                case madoca_parity::kCodeL7Q:
                case madoca_parity::kCodeL7X: return 14;
            }
            break;
        case GNSSSystem::QZSS:
            switch (code) {
                case madoca_parity::kCodeL1C:
                case madoca_parity::kCodeL1L:
                case madoca_parity::kCodeL1X: return 2;
                case madoca_parity::kCodeL2X: return 8;
                case madoca_parity::kCodeL5I:
                case madoca_parity::kCodeL5Q:
                case madoca_parity::kCodeL5X: return 22;
            }
            break;
        default:
            break;
    }
    return 0U;
}

inline std::uint8_t rtcmSsrSignalIdForObservation(GNSSSystem system,
                                                  SignalType signal,
                                                  std::string_view observation_type,
                                                  bool exact_bias_identity) {
    if (!exact_bias_identity) {
        return rtcmSsrSignalId(system, signal);
    }
    const int rtklib_system = rtklibSystemForGnss(system);
    if (rtklib_system == madoca_parity::kSysNone) {
        return rtcmSsrSignalId(system, signal);
    }
    const int rtklib_code = rtklibCodeForObservationType(observation_type);
    const int bias_code = madoca_parity::mcssrSelBiascode(rtklib_system, rtklib_code);
    return rtcmSsrSignalIdForRtklibCode(system, bias_code);
}

}  // namespace libgnss::algorithms::ppp_bias_identity
