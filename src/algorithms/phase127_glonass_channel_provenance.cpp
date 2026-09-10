#include <libgnss++/algorithms/phase127_glonass_channel_provenance.hpp>

#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::phase127_glonass {
namespace {

constexpr double kTieToleranceSeconds = 1.0e-9;

void fail(Result& result, const char* code, const char* message) {
    result.accepted = false;
    result.source = ChannelSource::None;
    result.diagnostics.failure_code = code;
    result.diagnostics.failure = message;
}

}  // namespace

bool isValidFrequencyChannel(int channel) noexcept {
    return channel >= kMinFrequencyChannel &&
           channel <= kMaxFrequencyChannel;
}

double frequencyHz(SignalType signal, int channel) noexcept {
    if (!isValidFrequencyChannel(channel)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    switch (signal) {
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            return constants::GLO_L1_BASE_FREQ +
                   static_cast<double>(channel) * constants::GLO_L1_STEP_FREQ;
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            return constants::GLO_L2_BASE_FREQ +
                   static_cast<double>(channel) * constants::GLO_L2_STEP_FREQ;
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

double wavelengthMeters(SignalType signal, int channel) noexcept {
    const double frequency = frequencyHz(signal, channel);
    if (!(frequency > 0.0) || !std::isfinite(frequency)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double wavelength = constants::SPEED_OF_LIGHT / frequency;
    return std::isfinite(wavelength) && wavelength > 0.0
               ? wavelength
               : std::numeric_limits<double>::quiet_NaN();
}

const char* channelSourceName(ChannelSource source) noexcept {
    switch (source) {
        case ChannelSource::Header:
            return "rinex-header";
        case ChannelSource::BroadcastEphemeris:
            return "broadcast-geph-frq";
        case ChannelSource::None:
        default:
            return "none";
    }
}

Result resolve(const SatelliteId& satellite,
               const GNSSTime& query_time,
               const NavigationData& navigation,
               const std::vector<GlonassFrequencyChannelEntry>& header_entries,
               std::size_t header_malformed_entries) {
    Result result;
    auto& diagnostics = result.diagnostics;

    // The caller may share this helper with a mixed-constellation stream.  A
    // non-GLONASS row has no FDMA FCN provenance obligation and is accepted
    // without changing any existing signal path.
    if (satellite.system != GNSSSystem::GLONASS) {
        result.accepted = true;
        return result;
    }

    diagnostics.header_malformed_entries = header_malformed_entries;
    int header_channel = 0;
    bool have_header_channel = false;
    for (const auto& entry : header_entries) {
        if (entry.satellite != satellite) {
            continue;
        }
        ++diagnostics.header_entries_seen;
        if (!isValidFrequencyChannel(entry.channel)) {
            ++diagnostics.invalid_channel_entries;
            fail(result, "header-fcn-invalid", "RINEX header FCN is out of range");
            return result;
        }
        if (!have_header_channel) {
            have_header_channel = true;
            header_channel = entry.channel;
            diagnostics.header_channel = entry.channel;
        } else if (entry.channel == header_channel) {
            ++diagnostics.header_duplicate_entries;
        } else {
            ++diagnostics.header_conflict_entries;
            fail(result, "header-fcn-conflict",
                 "RINEX header contains conflicting FCNs");
            return result;
        }
    }
    diagnostics.header_present = have_header_channel;
    if (header_malformed_entries != 0U) {
        fail(result, "header-fcn-malformed",
             "RINEX GLONASS FCN header ledger contains a malformed entry");
        return result;
    }

    if (!std::isfinite(query_time.tow)) {
        fail(result, "query-time-invalid", "GLONASS FCN query time is non-finite");
        return result;
    }

    const std::vector<Ephemeris> ephemerides =
        navigation.getEphemeris(satellite);
    double minimum_age = std::numeric_limits<double>::infinity();
    std::size_t valid_candidate_count = 0U;
    for (const auto& ephemeris : ephemerides) {
        if (ephemeris.satellite != satellite || !ephemeris.valid ||
            !ephemeris.isValid(query_time)) {
            continue;
        }
        const double age = ephemeris.getAge(query_time);
        if (!std::isfinite(age) || age > kMaxEphemerisAgeSeconds) {
            continue;
        }
        ++valid_candidate_count;
        minimum_age = std::min(minimum_age, age);
    }
    diagnostics.ephemeris_candidates = valid_candidate_count;
    if (valid_candidate_count == 0U ||
        !std::isfinite(minimum_age)) {
        if (ephemerides.empty()) {
            fail(result, "ephemeris-missing",
                 "no GLONASS broadcast ephemeris exists for the satellite");
        } else {
            ++diagnostics.query_time_coverage_gaps;
            fail(result, "query-time-coverage-gap",
                 "no GLONASS ephemeris is valid at the exact query time");
        }
        return result;
    }

    std::vector<const Ephemeris*> minimum_age_candidates;
    minimum_age_candidates.reserve(valid_candidate_count);
    for (const auto& ephemeris : ephemerides) {
        if (ephemeris.satellite != satellite || !ephemeris.valid ||
            !ephemeris.isValid(query_time)) {
            continue;
        }
        const double age = ephemeris.getAge(query_time);
        if (std::isfinite(age) && age <= kMaxEphemerisAgeSeconds &&
            std::abs(age - minimum_age) <= kTieToleranceSeconds) {
            minimum_age_candidates.push_back(&ephemeris);
        }
    }
    diagnostics.ephemeris_ties = minimum_age_candidates.size() > 1U
                                     ? minimum_age_candidates.size()
                                     : 0U;
    if (minimum_age_candidates.empty()) {
        ++diagnostics.query_time_coverage_gaps;
        fail(result, "query-time-coverage-gap",
             "selected GLONASS ephemeris is not time-valid");
        return result;
    }

    const Ephemeris* selected = navigation.getEphemeris(satellite, query_time);
    if (selected == nullptr) {
        ++diagnostics.query_time_coverage_gaps;
        fail(result, "query-time-coverage-gap",
             "native ephemeris selection has no exact query-time result");
        return result;
    }
    diagnostics.ephemeris_selected = true;
    diagnostics.selected_ephemeris_age_s = selected->getAge(query_time);

    // Every equally fresh record is part of the provenance decision.  A
    // missing FCN cannot silently tie with a valid one, and different FCNs
    // are an unresolved source conflict even though the legacy selector keeps
    // the first record.
    int selected_channel = 0;
    bool have_ephemeris_channel = false;
    for (const Ephemeris* candidate : minimum_age_candidates) {
        if (!candidate->glonass_frequency_channel_present) {
            ++diagnostics.invalid_channel_entries;
            fail(result, "ephemeris-fcn-missing",
                 "selected GLONASS ephemeris lacks an FCN field");
            return result;
        }
        if (!isValidFrequencyChannel(candidate->glonass_frequency_channel)) {
            ++diagnostics.invalid_channel_entries;
            fail(result, "ephemeris-fcn-invalid",
                 "selected GLONASS ephemeris FCN is out of range");
            return result;
        }
        if (!have_ephemeris_channel) {
            have_ephemeris_channel = true;
            selected_channel = candidate->glonass_frequency_channel;
        } else if (candidate->glonass_frequency_channel == selected_channel) {
            ++diagnostics.ephemeris_duplicate_entries;
        } else {
            ++diagnostics.ephemeris_conflict_entries;
            fail(result, "ephemeris-tie-fcn-conflict",
                 "equally fresh GLONASS ephemerides contain different FCNs");
            return result;
        }
    }
    diagnostics.ephemeris_channel = selected_channel;
    if (minimum_age_candidates.size() > 1U &&
        diagnostics.ephemeris_duplicate_entries == 0U) {
        // The only other possible multi-record outcome was returned above as
        // a different-FCN conflict.  Keep this invariant explicit for the
        // compact ledger and future callers.
        diagnostics.ephemeris_conflict_entries =
            minimum_age_candidates.size() - 1U;
    }

    if (selected->glonass_frequency_channel != selected_channel) {
        fail(result, "ephemeris-selection-inconsistent",
             "native selected GLONASS ephemeris disagrees with its minimum-age set");
        return result;
    }

    if (have_header_channel && header_channel != selected_channel) {
        fail(result, "header-geph-fcn-mismatch",
             "RINEX header FCN disagrees with selected broadcast geph.frq");
        return result;
    }

    result.accepted = true;
    result.channel = have_header_channel ? header_channel : selected_channel;
    result.source = have_header_channel
                        ? ChannelSource::Header
                        : ChannelSource::BroadcastEphemeris;
    diagnostics.header_match = have_header_channel;
    diagnostics.used_ephemeris_fallback = !have_header_channel;
    return result;
}

Result resolveAndAnnotate(
    Observation& observation,
    const GNSSTime& query_time,
    const NavigationData& navigation,
    const std::vector<GlonassFrequencyChannelEntry>& header_entries,
    std::size_t header_malformed_entries) {
    Result result = resolve(observation.satellite, query_time, navigation,
                            header_entries, header_malformed_entries);
    if (result.accepted &&
        observation.satellite.system == GNSSSystem::GLONASS) {
        observation.has_glonass_frequency_channel = true;
        observation.glonass_frequency_channel = result.channel;
    }
    return result;
}

}  // namespace libgnss::phase127_glonass
