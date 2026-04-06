#pragma once

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

namespace libgnss {

/// Return the carrier frequency in Hz for a given signal type.
/// For GLONASS FDMA signals, pass the satellite's Ephemeris to get the
/// channel-specific frequency; without it the base frequency is returned.
inline double signalFrequencyHz(SignalType signal, const Ephemeris* eph = nullptr) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::QZS_L1CA:
            return constants::GPS_L1_FREQ;
        case SignalType::GPS_L2C:
        case SignalType::GPS_L2P:
        case SignalType::QZS_L2C:
            return constants::GPS_L2_FREQ;
        case SignalType::GPS_L5:
        case SignalType::QZS_L5:
            return constants::GPS_L5_FREQ;
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS) {
                return constants::GLO_L1_BASE_FREQ +
                       eph->glonass_frequency_channel * constants::GLO_L1_STEP_FREQ;
            }
            return constants::GLO_L1_BASE_FREQ;
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS) {
                return constants::GLO_L2_BASE_FREQ +
                       eph->glonass_frequency_channel * constants::GLO_L2_STEP_FREQ;
            }
            return constants::GLO_L2_BASE_FREQ;
        case SignalType::GAL_E1:
            return constants::GAL_E1_FREQ;
        case SignalType::GAL_E5A:
            return constants::GAL_E5A_FREQ;
        case SignalType::GAL_E5B:
            return constants::GAL_E5B_FREQ;
        case SignalType::GAL_E6:
            return constants::GAL_E6_FREQ;
        case SignalType::BDS_B1I:
            return constants::BDS_B1I_FREQ;
        case SignalType::BDS_B2I:
            return constants::BDS_B2I_FREQ;
        case SignalType::BDS_B3I:
            return constants::BDS_B3I_FREQ;
        case SignalType::BDS_B1C:
            return constants::BDS_B1C_FREQ;
        case SignalType::BDS_B2A:
            return constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}

/// RTCM SSR signal ID for code/phase bias lookup.
inline uint8_t rtcmSsrSignalId(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GPS:
            switch (signal) {
                case SignalType::GPS_L1CA: return 2U;
                case SignalType::GPS_L1P: return 3U;
                case SignalType::GPS_L2C: return 8U;
                case SignalType::GPS_L2P: return 9U;
                case SignalType::GPS_L5: return 22U;
                default: return 0U;
            }
        case GNSSSystem::GLONASS:
            switch (signal) {
                case SignalType::GLO_L1CA: return 2U;
                case SignalType::GLO_L1P: return 3U;
                case SignalType::GLO_L2CA: return 8U;
                case SignalType::GLO_L2P: return 9U;
                default: return 0U;
            }
        case GNSSSystem::Galileo:
            switch (signal) {
                case SignalType::GAL_E1: return 2U;
                case SignalType::GAL_E6: return 8U;
                case SignalType::GAL_E5B: return 14U;
                case SignalType::GAL_E5A: return 22U;
                default: return 0U;
            }
        case GNSSSystem::BeiDou:
            switch (signal) {
                case SignalType::BDS_B1I: return 2U;
                case SignalType::BDS_B3I: return 8U;
                case SignalType::BDS_B2I: return 14U;
                default: return 0U;
            }
        case GNSSSystem::QZSS:
            switch (signal) {
                case SignalType::QZS_L1CA: return 2U;
                case SignalType::QZS_L2C: return 8U;
                case SignalType::QZS_L5: return 22U;
                default: return 0U;
            }
        default:
            return 0U;
    }
}

/// Return the carrier wavelength in meters for a given signal type.
inline double signalWavelengthMeters(SignalType signal, const Ephemeris* eph = nullptr) {
    const double frequency = signalFrequencyHz(signal, eph);
    return frequency > 0.0 ? constants::SPEED_OF_LIGHT / frequency : 0.0;
}

/// Return the carrier frequency in Hz for an Observation (uses GLONASS channel if available).
inline double signalFrequencyHz(const Observation& observation) {
    if (observation.satellite.system == GNSSSystem::GLONASS &&
        observation.has_glonass_frequency_channel) {
        switch (observation.signal) {
            case SignalType::GLO_L1CA:
            case SignalType::GLO_L1P:
                return constants::GLO_L1_BASE_FREQ +
                       observation.glonass_frequency_channel * constants::GLO_L1_STEP_FREQ;
            case SignalType::GLO_L2CA:
            case SignalType::GLO_L2P:
                return constants::GLO_L2_BASE_FREQ +
                       observation.glonass_frequency_channel * constants::GLO_L2_STEP_FREQ;
            default:
                break;
        }
    }
    return signalFrequencyHz(observation.signal);
}

/// Return the carrier wavelength in meters for an Observation.
inline double signalWavelengthMeters(const Observation& observation) {
    const double frequency = signalFrequencyHz(observation);
    return frequency > 0.0 ? constants::SPEED_OF_LIGHT / frequency : 0.0;
}

}  // namespace libgnss
