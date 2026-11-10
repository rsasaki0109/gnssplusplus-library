#include <libgnss++/algorithms/forward_backward.hpp>

#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <cmath>

namespace libgnss::forward_backward {

namespace {

struct Entry {
    const PositionSolution* solution = nullptr;
    Eigen::Matrix3d weight = Eigen::Matrix3d::Identity();
};

std::vector<Entry> validEntries(const Pass& pass) {
    std::vector<Entry> entries;
    entries.reserve(pass.solutions.size());
    for (std::size_t index = 0; index < pass.solutions.size(); ++index) {
        const PositionSolution& solution = pass.solutions[index];
        if (statusRank(solution) < 0) {
            continue;
        }
        Entry entry;
        entry.solution = &solution;
        entry.weight = index < pass.weights.size() ? pass.weights[index]
                                                   : solution.position_covariance;
        entries.push_back(entry);
    }
    std::stable_sort(entries.begin(), entries.end(), [](const Entry& lhs, const Entry& rhs) {
        return lhs.solution->time < rhs.solution->time;
    });
    return entries;
}

// A FIX the other pass did not solve with RTK (FLOAT or FIX) is reported as
// FLOAT; other_rank is -1 when the other pass has no solution.
void confirmFix(PositionSolution& solution, int other_rank, const CombineConfig& config,
                CombineStats& counts) {
    if (!config.require_rtk_confirmation || !solution.isFixed() || other_rank >= 3) {
        return;
    }
    solution.status = solution.status == SolutionStatus::PPP_FIXED ? SolutionStatus::PPP_FLOAT
                                                                    : SolutionStatus::FLOAT;
    ++counts.fix_unconfirmed;
}

void setPosition(PositionSolution& solution, const Eigen::Vector3d& position) {
    solution.position_ecef = position;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(position, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
}

// Covariance-weighted average of two estimates of the same position (RTKLIB
// smoother()); a plain mean when the summed covariance is not usable.
void average(const Entry& forward, const Entry& backward,
             Eigen::Vector3d& position, Eigen::Matrix3d& covariance) {
    const Eigen::Vector3d& xf = forward.solution->position_ecef;
    const Eigen::Vector3d& xb = backward.solution->position_ecef;
    const Eigen::Matrix3d sum = forward.weight + backward.weight;
    const Eigen::FullPivLU<Eigen::Matrix3d> lu(sum);
    if (forward.weight.allFinite() && backward.weight.allFinite() && lu.isInvertible()) {
        const Eigen::Matrix3d inverse = lu.inverse();
        position = backward.weight * inverse * xf + forward.weight * inverse * xb;
        covariance = forward.weight * inverse * backward.weight;
        covariance = 0.5 * (covariance + covariance.transpose());
        if (position.allFinite() && covariance.allFinite()) {
            return;
        }
    }
    position = 0.5 * (xf + xb);
    covariance = 0.25 * (forward.weight + backward.weight);
}

}  // namespace

void BackwardLossOfLock::apply(ObservationData& epoch) {
    std::map<std::pair<SatelliteId, SignalType>, bool> own;
    for (const auto& observation : epoch.observations) {
        own[{observation.satellite, observation.signal}] = (observation.lli & 0x01) != 0;
    }
    for (auto& observation : epoch.observations) {
        const auto it = later_slips_.find({observation.satellite, observation.signal});
        const bool slip = it != later_slips_.end() && it->second;
        observation.lli = static_cast<uint8_t>((observation.lli & ~0x01) | (slip ? 0x01 : 0x00));
        observation.loss_of_lock = slip;
    }
    // A slip of a signal this epoch does not have lies before its next
    // (earlier) observation: keep it until then.
    for (const auto& [key, slip] : later_slips_) {
        if (slip && own.find(key) == own.end()) {
            own[key] = true;
        }
    }
    later_slips_ = std::move(own);
}

int statusRank(const PositionSolution& solution) {
    if (!solution.isValid() || !solution.position_ecef.allFinite()) {
        return -1;
    }
    switch (solution.status) {
        case SolutionStatus::FIXED:
        case SolutionStatus::PPP_FIXED:
            return 4;
        case SolutionStatus::FLOAT:
        case SolutionStatus::PPP_FLOAT:
            return 3;
        case SolutionStatus::DGPS:
            return 2;
        case SolutionStatus::SPP:
            return 1;
        case SolutionStatus::PROPAGATED:
            return 0;
        case SolutionStatus::NONE:
            break;
    }
    return -1;
}

std::vector<PositionSolution> combine(const Pass& forward,
                                      const Pass& backward,
                                      const CombineConfig& config,
                                      CombineStats* stats) {
    CombineStats local_stats;
    CombineStats& counts = stats != nullptr ? *stats : local_stats;
    counts = CombineStats{};

    const std::vector<Entry> forward_entries = validEntries(forward);
    const std::vector<Entry> backward_entries = validEntries(backward);
    std::vector<PositionSolution> combined;
    combined.reserve(std::max(forward_entries.size(), backward_entries.size()));

    std::size_t f = 0;
    std::size_t b = 0;
    while (f < forward_entries.size() || b < backward_entries.size()) {
        const Entry* fe = f < forward_entries.size() ? &forward_entries[f] : nullptr;
        const Entry* be = b < backward_entries.size() ? &backward_entries[b] : nullptr;
        const double dt = fe != nullptr && be != nullptr
                              ? fe->solution->time - be->solution->time
                              : 0.0;
        if (be == nullptr || (fe != nullptr && dt < -config.time_tolerance_s)) {
            PositionSolution chosen = *fe->solution;
            confirmFix(chosen, -1, config, counts);
            combined.push_back(chosen);
            ++counts.forward_only;
            ++f;
            continue;
        }
        if (fe == nullptr || dt > config.time_tolerance_s) {
            PositionSolution chosen = *be->solution;
            confirmFix(chosen, -1, config, counts);
            combined.push_back(chosen);
            ++counts.backward_only;
            if (chosen.isFixed()) {
                ++counts.fix_upgrades;
            }
            ++b;
            continue;
        }

        ++counts.both;
        const int forward_rank = statusRank(*fe->solution);
        const int backward_rank = statusRank(*be->solution);
        if (forward_rank > backward_rank) {
            PositionSolution chosen = *fe->solution;
            confirmFix(chosen, backward_rank, config, counts);
            combined.push_back(chosen);
            ++counts.from_forward;
        } else if (backward_rank > forward_rank) {
            PositionSolution chosen = *be->solution;
            chosen.time = fe->solution->time;
            confirmFix(chosen, forward_rank, config, counts);
            combined.push_back(chosen);
            ++counts.from_backward;
            if (chosen.isFixed()) {
                ++counts.fix_upgrades;
            }
        } else {
            PositionSolution averaged = *fe->solution;
            Eigen::Vector3d position;
            Eigen::Matrix3d covariance;
            average(*fe, *be, position, covariance);
            setPosition(averaged, position);
            averaged.position_covariance = covariance;
            if (!averaged.has_velocity && be->solution->has_velocity) {
                averaged.velocity_ecef = be->solution->velocity_ecef;
                averaged.velocity_covariance = be->solution->velocity_covariance;
                averaged.has_velocity = true;
            }
            if (fe->solution->isFixed() &&
                (fe->solution->position_ecef - be->solution->position_ecef).norm() >
                    config.max_fix_difference_m) {
                // The passes fixed different ambiguities: neither is trusted.
                averaged.status = fe->solution->status == SolutionStatus::PPP_FIXED
                                      ? SolutionStatus::PPP_FLOAT
                                      : SolutionStatus::FLOAT;
                ++counts.fix_inconsistent;
            }
            combined.push_back(averaged);
            ++counts.averaged;
        }
        ++f;
        ++b;
    }
    return combined;
}

}  // namespace libgnss::forward_backward
