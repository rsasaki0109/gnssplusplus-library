#pragma once

/**
 * @file phase129_glonass_local_miss.hpp
 * @brief Shared, read-only classification for Phase129 GLONASS admission.
 *
 * The base correction stream and the rover FGO builder must make the same
 * decision for an exact-query Phase127/128 provenance result.  Keeping the
 * reason-code and local-miss predicate here prevents one side from silently
 * retaining an uncertified row or inventing a different failure taxonomy.
 */

#include <libgnss++/algorithms/phase127_glonass_channel_provenance.hpp>

#include <cstddef>
#include <string>

namespace libgnss::phase129_glonass_local_miss {

struct Classification {
    bool accepted = false;
    bool local_miss = false;
    std::string reason_code;
};

inline Classification classify(const phase127_glonass::Result& result,
                               bool local_miss_enabled) {
    if (result.accepted) {
        return Classification{true, false, {}};
    }
    const std::string reason_code =
        result.diagnostics.failure_code.empty()
            ? "unknown"
            : result.diagnostics.failure_code;
    return Classification{false, local_miss_enabled, reason_code};
}

inline bool rowLedgerConsistent(std::size_t source_rows,
                                std::size_t certified_rows,
                                std::size_t local_miss_rows) noexcept {
    return source_rows == certified_rows + local_miss_rows;
}

}  // namespace libgnss::phase129_glonass_local_miss
