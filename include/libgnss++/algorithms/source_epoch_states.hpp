#pragma once
#include "source_ephemeris_selection.hpp"
#include "source_transmission_time.hpp"

namespace libgnss::source_transmission_clock {
struct EpochSatelliteState {
    SelectedPseudorange selected_pseudorange;
    GNSSTime ephemeris_toe;
    int ephemeris_health;
    SelectedEphemerisState state;
};

// Strict single-epoch composition. No I/O, no receiver position, no fallback.
// Header tracking/quality selection and nav record ordering are caller-owned.
// Health is exposed, not silently ignored or mapped to another ephemeris.
inline std::map<SatelliteId, EpochSatelliteState> buildEpochStates(
    const GNSSTime& receive_time, const std::vector<Observation>& observations,
    const NavigationData& navigation) {
    if (!std::isfinite(receive_time.tow) || receive_time.tow < 0 ||
        receive_time.tow >= 604800)
        throw std::invalid_argument("invalid epoch state receive time");
    const auto selected = selectNativeEpoch(observations);
    std::map<SatelliteId, EpochSatelliteState> result;
    for (const auto& [satellite, pseudorange] : selected) {
        const auto records = navigation.ephemeris_data.find(satellite);
        if (records == navigation.ephemeris_data.end())
            throw std::invalid_argument("missing epoch satellite ephemerides");
        const auto* eph = selectBroadcastMessage(records->second, satellite, receive_time);
        if (!eph) throw std::invalid_argument("no admissible epoch satellite ephemeris");
        result.emplace(satellite, EpochSatelliteState{
            pseudorange, eph->toe, static_cast<int>(eph->health),
            stateFromSelectedEphemeris(receive_time, pseudorange.metres, *eph)});
    }
    return result;
}
} // namespace libgnss::source_transmission_clock
