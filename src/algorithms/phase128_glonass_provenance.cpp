#include <libgnss++/algorithms/phase128_glonass_provenance.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace libgnss::phase128_glonass {
namespace {

constexpr double kTieToleranceSeconds = 1.0e-9;

void fail(phase127_glonass::Result& result,
          const char* code,
          const char* message) {
    result.accepted = false;
    result.source = phase127_glonass::ChannelSource::None;
    result.diagnostics.failure_code = code;
    result.diagnostics.failure = message;
}

bool canonicalAtQuery(const Ephemeris& ephemeris,
                      const GNSSTime& query_time,
                      double minimum_age) {
    return ephemeris.satellite.system != GNSSSystem::GLONASS ||
           !ephemeris.valid || !ephemeris.isValid(query_time) ||
           !std::isfinite(ephemeris.getAge(query_time)) ||
           ephemeris.getAge(query_time) >
               phase127_glonass::kMaxEphemerisAgeSeconds ||
           std::abs(ephemeris.getAge(query_time) - minimum_age) >
               kTieToleranceSeconds ||
           ephemeris.glonass_canonical_geph_data_valid;
}

}  // namespace

Result resolve(const SatelliteId& satellite,
               const GNSSTime& query_time,
               const NavigationData& navigation,
               HeaderStatus header_status,
               const std::vector<GlonassFrequencyChannelEntry>& header_entries,
               std::size_t header_malformed_entries) {
    // A label which is absent or valid-but-empty is not a malformed ledger.
    // Preserve any explicit malformed count, and let the old helper retain
    // all mismatch/tie/range/query-time fail-closed behavior.
    const bool malformed_label =
        header_status == HeaderStatus::Malformed ||
        header_malformed_entries != 0U;
    const std::size_t effective_malformed = malformed_label
                                                ? std::max<std::size_t>(
                                                      1U,
                                                      header_malformed_entries)
                                                : 0U;
    Result result = phase127_glonass::resolve(
        satellite, query_time, navigation, header_entries, effective_malformed);
    if (!result.accepted || satellite.system != GNSSSystem::GLONASS) {
        return result;
    }

    // NavigationData exposes the exact records used by the existing native
    // GPST selector.  Recheck the minimum-age set so a malformed record cannot
    // be hidden by getEphemeris() choosing another record in a tie.
    const auto records = navigation.getEphemeris(satellite);
    double minimum_age = std::numeric_limits<double>::infinity();
    for (const auto& record : records) {
        if (record.satellite == satellite && record.valid &&
            record.isValid(query_time)) {
            const double age = record.getAge(query_time);
            if (std::isfinite(age) &&
                age <= phase127_glonass::kMaxEphemerisAgeSeconds) {
                minimum_age = std::min(minimum_age, age);
            }
        }
    }
    for (const auto& record : records) {
        if (!canonicalAtQuery(record, query_time, minimum_age)) {
            fail(result, "geph-canonical-invalid",
                 "selected GLONASS geph data[0..14] failed canonical admission");
            return result;
        }
    }
    return result;
}

Result resolveAndAnnotate(
    Observation& observation,
    const GNSSTime& query_time,
    const NavigationData& navigation,
    HeaderStatus header_status,
    const std::vector<GlonassFrequencyChannelEntry>& header_entries,
    std::size_t header_malformed_entries) {
    Result result = resolve(observation.satellite, query_time, navigation,
                            header_status, header_entries,
                            header_malformed_entries);
    if (result.accepted &&
        observation.satellite.system == GNSSSystem::GLONASS) {
        // Do not derive FCN from source_carrier_frequency_hz.  Only the
        // typed header/geph provenance result reaches the signal helper.
        observation.has_glonass_frequency_channel = true;
        observation.glonass_frequency_channel = result.channel;
    }
    return result;
}

}  // namespace libgnss::phase128_glonass
