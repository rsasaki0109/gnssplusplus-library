#pragma once

/**
 * @file phase128_glonass_provenance.hpp
 * @brief Phase128 parser/admission overlay for Phase127 GLONASS provenance.
 *
 * This adapter is default-off and deliberately delegates the actual
 * query-time/header-vs-geph decision to the sealed Phase127 helper.  Its only
 * additional authority is the canonical data[0..14] validity and the
 * distinction between an absent/valid-empty header label and a malformed
 * label.  No carrier-frequency metadata is consulted.
 */

#include <libgnss++/algorithms/phase127_glonass_channel_provenance.hpp>
#include <libgnss++/core/glonass_provenance.hpp>

namespace libgnss::phase128_glonass {

using HeaderStatus = GlonassFrequencyChannelHeaderStatus;
using Result = phase127_glonass::Result;

/** Stable selector/diagnostic label. */
constexpr const char* kSelector =
    "--native-phase128-glonass-provenance-parser-admission";

/**
 * Resolve a typed SatelliteId at the exact native GPST query boundary.
 * Absent and valid-empty header labels do not short-circuit broadcast-geph
 * resolution; malformed labels remain fail-closed.  Every selected/tied
 * geph record must have passed canonical fifteen-field validation.
 */
Result resolve(const SatelliteId& satellite,
               const GNSSTime& query_time,
               const NavigationData& navigation,
               HeaderStatus header_status,
               const std::vector<GlonassFrequencyChannelEntry>& header_entries,
               std::size_t header_malformed_entries = 0U);

/** Annotate only the caller's observation copy with the admitted geph FCN. */
Result resolveAndAnnotate(
    Observation& observation,
    const GNSSTime& query_time,
    const NavigationData& navigation,
    HeaderStatus header_status = HeaderStatus::Absent,
    const std::vector<GlonassFrequencyChannelEntry>& header_entries = {},
    std::size_t header_malformed_entries = 0U);

}  // namespace libgnss::phase128_glonass
