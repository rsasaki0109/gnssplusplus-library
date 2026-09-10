#pragma once

/**
 * @file phase131_canonical_correction_key.hpp
 * @brief Source-locked physical-frequency key for raw-base correction joins.
 *
 * Phase131 changes only the lookup boundary between an already prepared rover
 * pseudorange factor and an already prepared raw-base correction stream.  The
 * estimator continues to retain the original typed SignalType; this helper
 * provides a second, explicit key for the correction stream so literal RINEX
 * tracking-code/Android alias text cannot make equivalent physical bands
 * appear unrelated.
 */

#include <libgnss++/core/types.hpp>
#include <libgnss++/core/signal_policy.hpp>

#include <string>

namespace libgnss::phase131_canonical {

/** Frequency families represented by the frozen official FGO fields. */
enum class PhysicalFrequencyFamily : unsigned char {
    L1,
    L5,
    Unknown,
};

inline const char* familyName(PhysicalFrequencyFamily family) {
    switch (family) {
        case PhysicalFrequencyFamily::L1: return "L1";
        case PhysicalFrequencyFamily::L5: return "L5";
        default: return "unknown";
    }
}

/**
 * @brief Canonical correction stream key.
 *
 * GLONASS FCN is part of the key because it changes the physical wavelength.
 * Non-GLONASS callers leave `has_glonass_frequency_channel` false.
 */
struct Key {
    SatelliteId satellite;
    PhysicalFrequencyFamily family = PhysicalFrequencyFamily::Unknown;
    bool has_glonass_frequency_channel = false;
    int glonass_frequency_channel = 0;

    bool operator<(const Key& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        if (family != other.family) {
            return static_cast<unsigned char>(family) <
                   static_cast<unsigned char>(other.family);
        }
        if (has_glonass_frequency_channel !=
            other.has_glonass_frequency_channel) {
            return has_glonass_frequency_channel <
                   other.has_glonass_frequency_channel;
        }
        return glonass_frequency_channel < other.glonass_frequency_channel;
    }

    bool operator==(const Key& other) const {
        return !(*this < other) && !(other < *this);
    }
};

struct Canonicalization {
    bool accepted = false;
    Key key;
    int source_priority = 1000;
    std::string reason;
};

/** Map the existing typed SignalType to a frozen official FGO family. */
inline PhysicalFrequencyFamily familyForSignal(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::GPS_L1P:
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
        case SignalType::GAL_E1:
        case SignalType::BDS_B1I:
        case SignalType::BDS_B1C:
        case SignalType::QZS_L1CA:
            return PhysicalFrequencyFamily::L1;
        case SignalType::GPS_L5:
        case SignalType::GAL_E5A:
        case SignalType::BDS_B2A:
        case SignalType::QZS_L5:
            return PhysicalFrequencyFamily::L5;
        // The frozen official FGO source fields are L1/L5.  These signals
        // remain explicit misses here; Phase131 must not silently extend the
        // source contract to native-only L2/E5b/E6/B2I/B3I streams.
        default:
            return PhysicalFrequencyFamily::Unknown;
    }
}

/**
 * Map a native RINEX observation code through the existing typed policy.
 *
 * The observation-code suffix is deliberately not part of the returned
 * family.  `signal_policy` remains the single source-locked system/band
 * mapping, while callers retain the original observation text separately for
 * provenance and conflict diagnostics.
 */
inline PhysicalFrequencyFamily familyForRinexObservationType(
    GNSSSystem system, const std::string& observation_type) {
    SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
    if (!signal_policy::trySignalForObservationType(
            system, observation_type, signal)) {
        return PhysicalFrequencyFamily::Unknown;
    }
    return familyForSignal(signal);
}

/** Android rows are already typed by the raw loader; no frequency guess. */
inline PhysicalFrequencyFamily familyForAndroidSignalType(SignalType signal) {
    return familyForSignal(signal);
}

/** Source-defined tie-breaking priority within one physical family. */
inline int sourcePriority(GNSSSystem system, SignalType signal) {
    switch (system) {
        case GNSSSystem::GLONASS:
            if (signal == SignalType::GLO_L1CA) return 0;
            if (signal == SignalType::GLO_L1P) return 1;
            break;
        case GNSSSystem::GPS:
            if (signal == SignalType::GPS_L1CA) return 0;
            if (signal == SignalType::GPS_L1P) return 1;
            break;
        case GNSSSystem::BeiDou:
            if (signal == SignalType::BDS_B1I) return 0;
            if (signal == SignalType::BDS_B1C) return 1;
            break;
        default:
            break;
    }
    return 10 + static_cast<int>(signal);
}

/**
 * Convert a typed observation to the source-defined correction key.
 *
 * No signal text, carrier-frequency guess, fixed GLONASS channel, or
 * external table is consulted.  A GLONASS row without certified FCN is
 * rejected before any correction can be requested.
 */
inline Canonicalization canonicalize(
    const SatelliteId& satellite,
    SignalType signal,
    bool has_glonass_frequency_channel = false,
    int glonass_frequency_channel = 0) {
    Canonicalization result;
    result.key.satellite = satellite;
    result.key.family = familyForSignal(signal);
    result.source_priority = sourcePriority(satellite.system, signal);
    if (result.key.family == PhysicalFrequencyFamily::Unknown) {
        result.reason = "unknown-physical-frequency-family";
        return result;
    }
    if (satellite.system == GNSSSystem::GLONASS) {
        if (!has_glonass_frequency_channel) {
            result.reason = "glonass-fcn-missing";
            return result;
        }
        if (glonass_frequency_channel < -7 ||
            glonass_frequency_channel > 6) {
            result.reason = "glonass-fcn-out-of-range";
            return result;
        }
        result.key.has_glonass_frequency_channel = true;
        result.key.glonass_frequency_channel = glonass_frequency_channel;
    } else if (has_glonass_frequency_channel) {
        result.reason = "non-glonass-fcn-present";
        return result;
    }
    result.accepted = true;
    return result;
}

inline std::string keyString(const Key& key) {
    return std::to_string(static_cast<int>(key.satellite.system)) + ":" +
           std::to_string(static_cast<int>(key.satellite.prn)) + ":" +
           familyName(key.family) +
           (key.has_glonass_frequency_channel
                ? ":fcn=" + std::to_string(key.glonass_frequency_channel)
                : "");
}

}  // namespace libgnss::phase131_canonical
