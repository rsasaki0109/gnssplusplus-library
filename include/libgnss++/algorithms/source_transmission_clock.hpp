#pragma once

#include <cmath>
#include <array>
#include <optional>
#include <map>
#include <vector>
#include <libgnss++/core/types.hpp>
#include <libgnss++/core/observation.hpp>
#include <stdexcept>

namespace libgnss::source_transmission_clock {

// MatRTKLIB obs2obs.c slot order, not SignalType enum or arrival order.
// Input is one epoch/satellite after upstream observation masking.
enum class Slot : std::size_t { L1, L2, L5, L6, L7, L8, L9 };
// Slot labels above are MEX storage names, not universal physical bands.
// Input band is the digit from a validated RINEX observation code.
inline std::optional<Slot> slotForRinexBand(GNSSSystem system, int band) {
    int index = -1;
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            if (band == 1) index = 0;
            if (band == 2) index = 1;
            if (band == 5) index = 2;
            if (band == 6 && system == GNSSSystem::QZSS) index = 3;
            break;
        case GNSSSystem::GLONASS:
            if (band == 1 || band == 4) index = 0;
            if (band == 2 || band == 6) index = 1;
            if (band == 3) index = 2;
            break;
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
            if (band == 1 || (band == 2 && system == GNSSSystem::BeiDou)) index = 0;
            if (band == 7) index = 1;
            if (band == 5) index = 2;
            if (band == 6) index = 3;
            if (band == 8) index = 4;
            break;
        case GNSSSystem::SBAS:
            if (band == 1) index = 0;
            if (band == 5) index = 1;
            break;
        case GNSSSystem::NavIC:
            if (band == 5) index = 0;
            if (band == 9) index = 1;
            break;
        default: break;
    }
    if (index < 0) return std::nullopt;
    return static_cast<Slot>(index);
}
struct SelectedPseudorange {
    Slot slot;
    double metres;
};
inline std::optional<SelectedPseudorange> selectPseudorange(
    const std::array<double, 7>& slots) {
    for (std::size_t i = 0; i < slots.size(); ++i) {
        // MEX converts NaN to zero; satposs skips zero. Negative finite
        // values are deliberately not reclassified here as missing.
        if (std::isnan(slots[i]) || slots[i] == 0.0) continue;
        if (!std::isfinite(slots[i])) {
            throw std::invalid_argument("nonfinite selected transmission pseudorange");
        }
        return SelectedPseudorange{static_cast<Slot>(i), slots[i]};
    }
    return std::nullopt;
}

struct SlottedPseudorange {
    SatelliteId satellite;
    Slot slot;
    double metres;
};

// One epoch only. Caller must resolve the source's constellation-dependent
// slot mapping and tracking-code selection first. Duplicate slots fail rather
// than choosing by arrival order. No native SignalType->slot guess is made.
inline std::map<SatelliteId, SelectedPseudorange> selectBySatellite(
    const std::vector<SlottedPseudorange>& rows) {
    struct Slots {
        std::array<double, 7> values{};
        std::array<bool, 7> present{};
    };
    std::map<SatelliteId, Slots> grouped;
    for (const auto& row : rows) {
        const auto index = static_cast<std::size_t>(row.slot);
        if (index >= 7 || row.satellite.prn == 0) {
            throw std::invalid_argument("invalid transmission satellite or slot");
        }
        auto& slots = grouped[row.satellite];
        if (slots.present[index]) {
            throw std::invalid_argument("duplicate transmission satellite slot");
        }
        slots.present[index] = true;
        slots.values[index] = row.metres;
    }
    std::map<SatelliteId, SelectedPseudorange> selected;
    for (const auto& [satellite, slots] : grouped) {
        if (const auto value = selectPseudorange(slots.values)) {
            selected.emplace(satellite, *value);
        }
    }
    return selected;
}

// Adapter for an already tracking-selected and quality-masked native epoch.
// Does not reconstruct source tracking priority from discarded observations.
inline std::map<SatelliteId, SelectedPseudorange> selectNativeEpoch(
    const std::vector<Observation>& observations) {
    std::vector<SlottedPseudorange> rows;
    for (const auto& observation : observations) {
        if (!observation.valid || !observation.has_pseudorange) continue;
        const auto& code = observation.pseudorange_observation_type;
        if (code.size() != 3 || code[0] != 'C' || code[1] < '1' ||
            code[1] > '9' || code[2] < 'A' || code[2] > 'Z') {
            throw std::invalid_argument("missing or malformed transmission code provenance");
        }
        const auto slot = slotForRinexBand(observation.satellite.system, code[1]-'0');
        if (!slot) {
            throw std::invalid_argument("unsupported transmission observation band");
        }
        rows.push_back({observation.satellite, *slot, observation.pseudorange});
    }
    return selectBySatellite(rows);
}

// Broadcast polynomial clock for the initial satellite-clock -> GNSS-time
// conversion, not the final relativistic satellite measurement clock.
// MALIB 159e150d4a54e6b7b15d81128289b8559523ca81 eph2clk algorithm.
// Caller supplies elapsed seconds from the selected ephemeris toc.
inline double polynomialClockSeconds(double elapsed_from_toc_s,
                                     double af0, double af1, double af2) {
    if (!std::isfinite(elapsed_from_toc_s) || !std::isfinite(af0) ||
        !std::isfinite(af1) || !std::isfinite(af2)) {
        throw std::invalid_argument("nonfinite source transmission clock input");
    }
    const auto polynomial = [=](double t) {
        return af0 + af1 * t + af2 * t * t;
    };
    double corrected = elapsed_from_toc_s;
    for (int iteration = 0; iteration < 2; ++iteration) {
        corrected = elapsed_from_toc_s - polynomial(corrected);
        if (!std::isfinite(corrected)) {
            throw std::invalid_argument("source transmission clock overflow");
        }
    }
    const double result = polynomial(corrected);
    if (!std::isfinite(result)) {
        throw std::invalid_argument("source transmission clock overflow");
    }
    return result;
}

} // namespace libgnss::source_transmission_clock
