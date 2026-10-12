#pragma once

#include <optional>
#include <vector>

#include "../core/observation.hpp"

namespace libgnss {
namespace io {

/**
 * @brief Assembles per-message observation parts into complete epochs.
 *
 * RTCM receivers send one MSM message per constellation for the same time tag
 * and flag every message except the last with the "multiple message bit"
 * (legacy 1001-1004 / 1009-1012: synchronous GNSS flag). Treating each message
 * as an epoch yields partial single-constellation epochs. This class merges
 * the parts of one epoch:
 *
 *  - If a pending epoch exists and the part's time differs from it by more
 *    than kTimeToleranceSeconds, the pending epoch is emitted first (a lost
 *    last message must not merge two epochs).
 *  - Otherwise the part is merged into the pending epoch; a (satellite,
 *    signal) already present is not duplicated (the first one is kept).
 *  - When more_follows is false the pending epoch is emitted immediately, so
 *    no latency is added. A stream whose messages all carry bit 0 passes
 *    through unchanged.
 *  - flush() emits a pending epoch at end of stream.
 */
class ObservationEpochAssembler {
public:
    static constexpr double kTimeToleranceSeconds = 1e-3;

    /** Feed one decoded observation message; returns zero, one or two completed epochs. */
    std::vector<ObservationData> push(const ObservationData& part, bool more_follows);

    /** Emit the pending epoch (end of stream); empty when nothing is pending. */
    std::vector<ObservationData> flush();

    bool hasPending() const { return pending_.has_value(); }
    void clear() { pending_.reset(); }

private:
    std::optional<ObservationData> pending_;
};

}  // namespace io
}  // namespace libgnss
