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
