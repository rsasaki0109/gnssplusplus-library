#pragma once

#include <libgnss++/core/types.hpp>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace libgnss::algorithms::madoca_core {

// MADOCA-PPP Compact SSR GNSS IDs (IS-QZSS-MDC-004 Table 4.2.2-7).
enum class CompactSsrSystemId : std::uint8_t {
    GPS = 0,
    GLONASS = 1,
    Galileo = 2,
    BeiDou = 3,
    QZSS = 4,
    BeiDou3 = 7,
};

// Compact SSR subtype ids used by MADOCA L6E.
enum class CompactSsrSubtype : std::uint8_t {
    Mask = 1,
    Orbit = 2,
    Clock = 3,
    CodeBias = 4,
    PhaseBias = 5,
    Ura = 7,
};

std::optional<CompactSsrSystemId> compactSsrSystemId(int id);
GNSSSystem gnssSystemForCompactSsrId(int id);
std::optional<int> compactSsrIdForGnssSystem(GNSSSystem system, bool beidou3 = false);
std::string_view compactSsrSystemName(int id);

std::optional<CompactSsrSubtype> compactSsrSubtype(int subtype);
std::string_view compactSsrSubtypeName(int subtype);

std::optional<double> ssrUpdateIntervalSeconds(int index);

// Return the RTKLIB CODE_* value for a Compact SSR signal-mask slot
// (0..15). Invalid systems or slots return 0 (CODE_NONE).
int compactSsrSignalCodeAt(int compact_ssr_system_id, int mask_slot);

// Expand a 16-bit Compact SSR signal mask to RTKLIB CODE_* values in the same
// MSB-first order used by MADOCALIB's sigmask2list().
std::vector<int> compactSsrSignalCodes(int compact_ssr_system_id, std::uint16_t mask);

// RTCM/MADOCA SSR URA indicator to one-sigma standard deviation in meters.
double ssrUraIndicatorToSigmaMeters(int ura_index);

// MADOCA-PPP code/phase-bias code selection. The system argument uses the
// RTKLIB SYS_* bit constants mirrored in madoca_parity.hpp.
int selectBiasCode(int rtklib_system, int rtklib_code);

}  // namespace libgnss::algorithms::madoca_core
