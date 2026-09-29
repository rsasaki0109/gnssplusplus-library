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

/// Map a signal ID as transmitted in RTCM 10403.3 SSR code/phase bias
/// messages (GPS Table 3.5-91, GLONASS 3.5-96, Galileo 3.5-100; the same
/// assignment RTKLIB and cssrlib use) to the coarse libgnss++ SignalType.
///
/// Note that rtcmSsrSignalId() above is the internal bias-map key (MSM-style
/// numbering) used by the PPP bias lookup, not the RTCM SSR wire ID handled
/// here. Several wire IDs collapse onto one SignalType (e.g. Galileo E1 B/C/X);
/// @p preference_rank (lower is preferred) lets callers keep the tracking mode
/// a geodetic receiver most likely reports when several are broadcast.
/// Returns SignalType::SIGNAL_TYPE_COUNT for IDs without a SignalType.
inline SignalType signalTypeFromRtcmSsrSignalId(GNSSSystem system,
                                                uint8_t rtcm_ssr_signal_id,
                                                int* preference_rank = nullptr) {
    struct Entry {
        SignalType signal;
        int rank;
    };
    Entry entry{SignalType::SIGNAL_TYPE_COUNT, 0};
    switch (system) {
        case GNSSSystem::GPS:
            switch (rtcm_ssr_signal_id) {
                case 0: entry = {SignalType::GPS_L1CA, 0}; break;   // L1 C/A
                case 2: entry = {SignalType::GPS_L1P, 0}; break;    // L1 Z-tracking (W)
                case 1: entry = {SignalType::GPS_L1P, 1}; break;    // L1 P
                case 8: entry = {SignalType::GPS_L2C, 0}; break;    // L2C (L)
                case 9: entry = {SignalType::GPS_L2C, 1}; break;    // L2C (M+L)
                case 7: entry = {SignalType::GPS_L2C, 2}; break;    // L2C (M)
                case 5: entry = {SignalType::GPS_L2C, 3}; break;    // L2 C/A
                case 6: entry = {SignalType::GPS_L2C, 4}; break;    // L2 semi-codeless (D)
                case 11: entry = {SignalType::GPS_L2P, 0}; break;   // L2 Z-tracking (W)
                case 10: entry = {SignalType::GPS_L2P, 1}; break;   // L2 P
                case 15: entry = {SignalType::GPS_L5, 0}; break;    // L5 Q
                case 16: entry = {SignalType::GPS_L5, 1}; break;    // L5 I+Q
                case 14: entry = {SignalType::GPS_L5, 2}; break;    // L5 I
                default: break;
            }
            break;
        case GNSSSystem::GLONASS:
            switch (rtcm_ssr_signal_id) {
                case 0: entry = {SignalType::GLO_L1CA, 0}; break;
                case 1: entry = {SignalType::GLO_L1P, 0}; break;
                case 2: entry = {SignalType::GLO_L2CA, 0}; break;
                case 3: entry = {SignalType::GLO_L2P, 0}; break;
                default: break;
            }
            break;
        case GNSSSystem::Galileo:
            switch (rtcm_ssr_signal_id) {
                case 2: entry = {SignalType::GAL_E1, 0}; break;     // E1 C
                case 3: entry = {SignalType::GAL_E1, 1}; break;     // E1 B+C
                case 1: entry = {SignalType::GAL_E1, 2}; break;     // E1 B
                case 0: entry = {SignalType::GAL_E1, 3}; break;     // E1 A
                case 4: entry = {SignalType::GAL_E1, 4}; break;     // E1 A+B+C
                case 6: entry = {SignalType::GAL_E5A, 0}; break;    // E5a Q
                case 7: entry = {SignalType::GAL_E5A, 1}; break;    // E5a I+Q
                case 5: entry = {SignalType::GAL_E5A, 2}; break;    // E5a I
                case 9: entry = {SignalType::GAL_E5B, 0}; break;    // E5b Q
                case 10: entry = {SignalType::GAL_E5B, 1}; break;   // E5b I+Q
                case 8: entry = {SignalType::GAL_E5B, 2}; break;    // E5b I
                case 16: entry = {SignalType::GAL_E6, 0}; break;    // E6 C
                case 17: entry = {SignalType::GAL_E6, 1}; break;    // E6 B+C
                case 15: entry = {SignalType::GAL_E6, 2}; break;    // E6 B
                case 14: entry = {SignalType::GAL_E6, 3}; break;    // E6 A
                case 18: entry = {SignalType::GAL_E6, 4}; break;    // E6 A+B+C
                default: break;                                     // 11-13: E5 AltBOC
            }
            break;
        default:
            break;
    }
    if (preference_rank != nullptr) {
        *preference_rank = entry.rank;
    }
    return entry.signal;
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
