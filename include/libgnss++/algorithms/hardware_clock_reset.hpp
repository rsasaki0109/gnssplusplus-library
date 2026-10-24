#pragma once

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace libgnss::hardware_clock_reset {

// Raw Android UTC keys of the epochs whose HardwareClockDiscontinuityCount
// differs from the previous epoch.  The raw loader rebases the receiver-clock
// reference at such an epoch, so the clock state steps by the FullBias drift
// accumulated since the previous reference (about 194 m over the first 148 s
// of 2023-06-06 sjc-he2 Pixel5).  The steps are far below the positive 100 km
// pseudorange-delta clock-jump detector, so callers that keep clock
// continuity factors must treat these epochs as clock jumps explicitly.
// The returned keys are strictly increasing.
inline std::vector<std::int64_t> utcKeys(const std::vector<int>& discontinuity_counts,
                                         const std::vector<std::int64_t>& utc_millis) {
    if (discontinuity_counts.size() != utc_millis.size())
        throw std::invalid_argument("hardware clock reset keys require one count per UTC epoch");
    std::vector<std::int64_t> keys;
    for (std::size_t i = 1; i < utc_millis.size(); ++i) {
        if (!(utc_millis[i] > utc_millis[i - 1]))
            throw std::invalid_argument("hardware clock reset keys require increasing UTC epochs");
        if (discontinuity_counts[i] != discontinuity_counts[i - 1]) keys.push_back(utc_millis[i]);
    }
    return keys;
}

}  // namespace libgnss::hardware_clock_reset
