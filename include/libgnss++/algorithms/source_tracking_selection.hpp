#pragma once

#include "source_transmission_clock.hpp"
#include <string>
#include <string_view>

namespace libgnss::source_transmission_clock {

// Fixed MALIB defaults; no runtime code-priority or RINEX option overrides.
// Caller supplies an obs2code-validated two-character code.
inline int defaultTrackingPriority(GNSSSystem system, std::string_view code) {
    if (code.size() != 2) return 0;
    const auto slot = slotForRinexBand(system, code[0] - '0');
    if (!slot) return 0;
    const std::array<std::string_view, 7>* priorities = nullptr;
    static constexpr std::array<std::string_view, 7> gps{"C","PYWCMNDLXS","QXI","","","",""};
    static constexpr std::array<std::string_view, 7> glo{"CP","PC","QXI","","","",""};
    static constexpr std::array<std::string_view, 7> gal{"CBX","QXI","QXI","CXB","QXI","",""};
    static constexpr std::array<std::string_view, 7> qzs{"CLXS","LXS","QXI","SEZ","","",""};
    static constexpr std::array<std::string_view, 7> sbs{"C","IQX","","","","",""};
    static constexpr std::array<std::string_view, 7> bds{"IQDPXSLZ","IQXDPZ","DPX","IQXDPZ","DPX","",""};
    static constexpr std::array<std::string_view, 7> irn{"ABCX","ABCX","","","","",""};
    switch (system) {
        case GNSSSystem::GPS: priorities = &gps; break;
        case GNSSSystem::GLONASS: priorities = &glo; break;
        case GNSSSystem::Galileo: priorities = &gal; break;
        case GNSSSystem::QZSS: priorities = &qzs; break;
        case GNSSSystem::SBAS: priorities = &sbs; break;
        case GNSSSystem::BeiDou: priorities = &bds; break;
        case GNSSSystem::NavIC: priorities = &irn; break;
        default: return 0;
    }
    const auto rank = (*priorities)[static_cast<std::size_t>(*slot)].find(code[1]);
    return rank == std::string_view::npos ? 0 : 14 - static_cast<int>(rank);
}

// Header order, not epoch measurement availability, breaks equal priorities.
// Input includes codes declared by all observation kinds, not only C fields.
inline std::map<Slot, std::string> selectHeaderTrackingCodes(
    GNSSSystem system, const std::vector<std::string>& codes) {
    std::map<Slot, std::string> result;
    for (const auto& code : codes) {
        const int priority = defaultTrackingPriority(system, code);
        if (priority == 0) continue;
        const auto slot = *slotForRinexBand(system, code[0]-'0');
        const auto existing = result.find(slot);
        if (existing == result.end() ||
            priority > defaultTrackingPriority(system, existing->second)) {
            result[slot] = code;
        }
    }
    return result;
}
} // namespace libgnss::source_transmission_clock
