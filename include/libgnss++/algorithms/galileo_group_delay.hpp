#pragma once

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>

#include <cmath>

namespace libgnss::galileo_group_delay {

// RINEX 4 Galileo data-source word bits retained by RINEXReader.  The clock
// reference is E5a for F/NAV and E5b for I/NAV; this is also the distinction
// made by the pinned RTKLIB prange() single-frequency E1 branch.
constexpr int kFNavClockSourceBit = 1 << 8;
constexpr int kINavClockSourceBit = 1 << 9;

enum class Reference {
    LegacyPrimary,
    FNavE1E5a,
    INavE1E5b,
    Ambiguous,
};

struct Selection {
    double delay_seconds = 0.0;
    Reference reference = Reference::LegacyPrimary;
    bool valid = true;

    bool sourceSpecific() const {
        return reference == Reference::FNavE1E5a ||
               reference == Reference::INavE1E5b;
    }

    bool usedSecondaryField() const {
        return reference == Reference::INavE1E5b;
    }

    bool fellBackToPrimaryField() const {
        return reference == Reference::Ambiguous;
    }
};

/**
 * Select the Galileo E1 broadcast group delay without guessing an absent
 * navigation-message source.
 *
 * The historical path always uses Ephemeris::tgd.  In the opt-in path only
 * Galileo E1 is source-sensitive: an unambiguous I/NAV clock-source bit uses
 * tgd_secondary (BGD E1/E5b), while an unambiguous F/NAV clock-source bit uses
 * tgd (BGD E1/E5a).  Missing or contradictory source bits retain the legacy
 * primary field and are reported as Ambiguous.  Other signals and systems
 * always retain the legacy field; no E5a single-frequency rule is invented.
 */
inline Selection select(const Observation& observation,
                        const Ephemeris& eph,
                        bool enable_source_specific_e1) {
    Selection result;
    result.delay_seconds = eph.tgd;

    if (enable_source_specific_e1 &&
        observation.satellite.system == GNSSSystem::Galileo &&
        observation.signal == SignalType::GAL_E1) {
        const bool fnav = (eph.data_source_code & kFNavClockSourceBit) != 0;
        const bool inav = (eph.data_source_code & kINavClockSourceBit) != 0;
        if (inav && !fnav) {
            result.reference = Reference::INavE1E5b;
            result.delay_seconds = eph.tgd_secondary;
        } else if (fnav && !inav) {
            result.reference = Reference::FNavE1E5a;
        } else {
            result.reference = Reference::Ambiguous;
        }
    }

    result.valid = std::isfinite(result.delay_seconds);
    return result;
}

inline double correctionMeters(const Observation& observation,
                               const Ephemeris& eph,
                               bool enable_source_specific_e1,
                               Selection* selection = nullptr) {
    const Selection chosen =
        select(observation, eph, enable_source_specific_e1);
    if (selection != nullptr) {
        *selection = chosen;
    }
    return chosen.delay_seconds * constants::SPEED_OF_LIGHT;
}

}  // namespace libgnss::galileo_group_delay
