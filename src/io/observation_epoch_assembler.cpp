#include <libgnss++/io/observation_epoch_assembler.hpp>

#include <cmath>
#include <set>
#include <utility>

namespace libgnss {
namespace io {

namespace {

void mergePart(ObservationData& pending, const ObservationData& part) {
    std::set<std::pair<SatelliteId, SignalType>> present;
    for (const auto& obs : pending.observations) {
        present.emplace(obs.satellite, obs.signal);
    }
    for (const auto& obs : part.observations) {
        if (present.emplace(obs.satellite, obs.signal).second) {
            pending.observations.push_back(obs);
        }
    }
    // The RTCM decoder also keeps every tracking code of a satellite here (used by the
    // RINEX writer); carry them over so that a merged epoch loses nothing a part had.
    for (const auto& [key, obs] : part.rinex_tracking_observations) {
        pending.rinex_tracking_observations.emplace(key, obs);
    }
    for (const auto& [key, code] : part.rinex_frequency_slots) {
        pending.rinex_frequency_slots.emplace(key, code);
    }
    if (pending.receiver_position.norm() == 0.0 && part.receiver_position.norm() > 0.0) {
        pending.receiver_position = part.receiver_position;
    }
}

}  // namespace

std::vector<ObservationData> ObservationEpochAssembler::push(const ObservationData& part,
                                                             bool more_follows) {
    std::vector<ObservationData> completed;
    if (pending_.has_value() &&
        std::abs(part.time - pending_->time) > kTimeToleranceSeconds) {
        completed.push_back(std::move(*pending_));
        pending_.reset();
    }
    if (pending_.has_value()) {
        mergePart(*pending_, part);
    } else {
        pending_ = part;
    }
    if (!more_follows) {
        completed.push_back(std::move(*pending_));
        pending_.reset();
    }
    return completed;
}

std::vector<ObservationData> ObservationEpochAssembler::flush() {
    std::vector<ObservationData> completed;
    if (pending_.has_value()) {
        completed.push_back(std::move(*pending_));
        pending_.reset();
    }
    return completed;
}

}  // namespace io
}  // namespace libgnss
