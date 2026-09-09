#pragma once

/**
 * @file phase127_glonass_channel_provenance.hpp
 * @brief Strict, read-only GLONASS FDMA channel provenance for Phase127.
 *
 * The adapter is deliberately independent of files, factors, and solver
 * state.  Callers pass the already parsed RINEX header ledger and the exact
 * query time used by the existing ephemeris boundary.  A successful result
 * only certifies the channel metadata that the existing signal-frequency
 * helpers already consume; it does not add a factor or synthesize a value.
 */

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/types.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace libgnss::phase127_glonass {

constexpr int kMinFrequencyChannel = -7;
constexpr int kMaxFrequencyChannel = 6;
constexpr double kMaxEphemerisAgeSeconds = 1800.0;

enum class ChannelSource {
    None,
    Header,
    BroadcastEphemeris,
};

struct Diagnostics {
    bool header_present = false;
    bool ephemeris_selected = false;
    bool header_match = false;
    bool used_ephemeris_fallback = false;

    std::size_t header_entries_seen = 0U;
    std::size_t header_duplicate_entries = 0U;
    std::size_t header_conflict_entries = 0U;
    std::size_t header_malformed_entries = 0U;
    std::size_t invalid_channel_entries = 0U;

    std::size_t ephemeris_candidates = 0U;
    std::size_t ephemeris_ties = 0U;
    std::size_t ephemeris_duplicate_entries = 0U;
    std::size_t ephemeris_conflict_entries = 0U;
    std::size_t query_time_coverage_gaps = 0U;

    double selected_ephemeris_age_s = 0.0;
    int header_channel = 0;
    int ephemeris_channel = 0;
    std::string failure_code;
    std::string failure;
};

struct Result {
    bool accepted = false;
    int channel = 0;
    ChannelSource source = ChannelSource::None;
    Diagnostics diagnostics;
};

/** Return true only for the source-supported signed GLONASS FCN domain. */
bool isValidFrequencyChannel(int channel) noexcept;

/** Return the source frequency in Hz, or NaN for an unsupported FCN/signal. */
double frequencyHz(SignalType signal, int channel) noexcept;

/** Return c/f in metres, or NaN when frequencyHz() is not finite-positive. */
double wavelengthMeters(SignalType signal, int channel) noexcept;

/** Stable compact source label for diagnostics/result metadata. */
const char* channelSourceName(ChannelSource source) noexcept;

/**
 * Resolve one GLONASS channel at the existing exact query-time boundary.
 *
 * Header entries have primary provenance, but a selected time-valid broadcast
 * ephemeris is always required as corroboration.  When no exact header entry
 * exists, the selected broadcast FCN is the only permitted fallback.  Header
 * conflicts, different-FCN minimum-age ties, absent/invalid FCNs, and a
 * missing query-time-valid ephemeris all fail closed.
 */
Result resolve(const SatelliteId& satellite,
               const GNSSTime& query_time,
               const NavigationData& navigation,
               const std::vector<GlonassFrequencyChannelEntry>& header_entries,
               std::size_t header_malformed_entries = 0U);

/**
 * Resolve and annotate a local observation copy.  The source observation is
 * never changed; for GLONASS the returned FCN is written only to the caller's
 * copy so existing frequency/wavelength helpers cannot fall back to Android
 * carrier-frequency metadata while this selector is enabled.
 */
Result resolveAndAnnotate(
    Observation& observation,
    const GNSSTime& query_time,
    const NavigationData& navigation,
    const std::vector<GlonassFrequencyChannelEntry>& header_entries = {},
    std::size_t header_malformed_entries = 0U);

}  // namespace libgnss::phase127_glonass
