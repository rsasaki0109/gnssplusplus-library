#include "libgnss++/algorithms/integrity_consensus.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace libgnss {

IntegrityConsensusManager::IntegrityConsensusManager()
    : IntegrityConsensusManager(Config{}) {}

IntegrityConsensusManager::IntegrityConsensusManager(Config config)
    : config_(config) {}

void IntegrityConsensusManager::reset() {
    state_ = State::NORMAL;
    suspect_count_ = 0;
    disagreement_count_ = 0;
    recovery_count_ = 0;
    has_independent_reset_generation_ = false;
}

IntegrityConsensusManager::Decision IntegrityConsensusManager::update(const Input& input) {
    Decision decision;
    const bool primary_healthy = input.primary.valid &&
        input.primary.position_ecef.allFinite() &&
        std::isfinite(input.primary.covariance_trace_m2) &&
        input.primary.covariance_trace_m2 <= config_.max_primary_covariance_trace_m2;
    const bool independent_available = input.independent.valid &&
        input.independent.position_ecef.allFinite() &&
        std::isfinite(input.independent_age_s) && input.independent_age_s >= 0.0 &&
        input.independent_age_s <= config_.max_independent_age_s;
    const bool independent_healthy = independent_available &&
        std::isfinite(input.independent.covariance_trace_m2) &&
        input.independent.covariance_trace_m2 <=
            config_.max_independent_covariance_trace_m2;

    if (!independent_available) {
        decision.reasons |= INDEPENDENT_UNAVAILABLE;
    } else if (!independent_healthy) {
        decision.reasons |= INDEPENDENT_UNCERTAIN;
    }

    if (primary_healthy && independent_healthy) {
        decision.disagreement_m =
            (input.primary.position_ecef - input.independent.position_ecef).norm();
        const double combined_trace = input.primary.covariance_trace_m2 +
            input.independent.covariance_trace_m2;
        decision.aperture_m = std::clamp(
            config_.aperture_sigma_multiplier * std::sqrt(std::max(0.0, combined_trace)),
            config_.aperture_min_m,
            config_.aperture_max_m);
        decision.estimators_agree = decision.disagreement_m <= decision.aperture_m;
        if (!decision.estimators_agree) decision.reasons |= ESTIMATOR_DISAGREEMENT;
    }

    if (input.primary_suspect || input.hard_primary_suspect) {
        decision.reasons |= PRIMARY_SUSPECT;
    }
    const bool soft_suspect_evidence_available = independent_healthy ||
        !config_.soft_suspect_requires_independent;
    const bool effective_primary_suspect = input.primary_suspect &&
        soft_suspect_evidence_available &&
        (!independent_healthy || !decision.estimators_agree);

    const bool generation_changed = has_independent_reset_generation_ &&
        input.independent_reset_generation != independent_reset_generation_;
    if (generation_changed) decision.reasons |= RESET_GENERATION_CHANGED;
    if (independent_available) {
        independent_reset_generation_ = input.independent_reset_generation;
        has_independent_reset_generation_ = true;
    }

    const bool disagreement = input.fixed_candidate && independent_healthy && primary_healthy &&
        !decision.estimators_agree &&
        (!config_.disagreement_requires_primary_suspect || input.primary_suspect);
    suspect_count_ = effective_primary_suspect ? suspect_count_ + 1 : 0;
    disagreement_count_ = disagreement ? disagreement_count_ + 1 : 0;

    switch (state_) {
        case State::NORMAL:
            if (input.hard_primary_suspect) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
                decision.request_primary_reset = true;
            } else if (suspect_count_ >= std::max(1, config_.suspect_streak) ||
                       disagreement_count_ >= std::max(1, config_.suspect_streak)) {
                state_ = State::SUSPECT;
            }
            break;
        case State::SUSPECT:
            if (input.hard_primary_suspect ||
                suspect_count_ >= std::max(1, config_.quarantine_streak) ||
                disagreement_count_ >= std::max(1, config_.quarantine_streak)) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
                decision.request_primary_reset = true;
            } else if (!effective_primary_suspect && !disagreement) {
                state_ = State::NORMAL;
            }
            break;
        case State::QUARANTINE:
            if (input.fixed_candidate && decision.estimators_agree &&
                !effective_primary_suspect && !generation_changed) {
                state_ = State::RECOVERY;
                recovery_count_ = 1;
            }
            break;
        case State::RECOVERY:
            if (generation_changed || effective_primary_suspect ||
                (input.fixed_candidate && !decision.estimators_agree)) {
                state_ = State::QUARANTINE;
                recovery_count_ = 0;
            } else if (input.fixed_candidate && decision.estimators_agree) {
                ++recovery_count_;
                if (recovery_count_ >= std::max(1, config_.recovery_streak)) {
                    state_ = State::NORMAL;
                    recovery_count_ = 0;
                    suspect_count_ = 0;
                    disagreement_count_ = 0;
                    decision.promote_joint_anchor = true;
                }
            }
            break;
    }

    decision.state = state_;
    decision.recovery_streak = recovery_count_;
    const bool allow_provisional_recovery_fixed =
        state_ == State::RECOVERY && input.fixed_candidate &&
        decision.estimators_agree && input.recovery_candidate_healthy;
    decision.allow_fixed = !input.fixed_candidate || state_ == State::NORMAL ||
        allow_provisional_recovery_fixed;
    return decision;
}

MultiShadowPositionConsensus::MultiShadowPositionConsensus()
    : MultiShadowPositionConsensus(Config{}) {}

MultiShadowPositionConsensus::MultiShadowPositionConsensus(Config config)
    : config_(config) {
    if (config_.min_independent_shadows < 2) {
        throw std::invalid_argument("min_independent_shadows must be at least two");
    }
    if (!(config_.shadow_agreement_aperture_m > 0.0) ||
        config_.primary_separation_min_m < 0.0 ||
        config_.candidate_max_prediction_error_m < 0.0) {
        throw std::invalid_argument("invalid multi-shadow consensus distance gate");
    }
}

MultiShadowPositionConsensus::Decision MultiShadowPositionConsensus::evaluate(
    const Input& input) const {
    Decision decision;
    if (!input.primary_valid || !input.primary_fixed ||
        !input.primary_position_ecef.allFinite()) {
        decision.reasons |= PRIMARY_NOT_FIXED;
        return decision;
    }

    std::vector<const Shadow*> eligible;
    std::set<std::uint64_t> source_ids;
    for (const auto& shadow : input.shadows) {
        if (!shadow.valid || !shadow.fixed || !shadow.healthy ||
            !shadow.position_ecef.allFinite()) {
            continue;
        }
        if (!source_ids.insert(shadow.source_id).second) {
            decision.reasons |= DUPLICATE_SHADOW_SOURCE;
            return decision;
        }
        eligible.push_back(&shadow);
    }
    if (eligible.size() < static_cast<std::size_t>(config_.min_independent_shadows)) {
        decision.reasons |= INSUFFICIENT_SHADOWS;
        return decision;
    }

    auto diameter = [](const std::vector<const Shadow*>& cluster) {
        double value = 0.0;
        for (std::size_t i = 0; i < cluster.size(); ++i) {
            for (std::size_t j = i + 1; j < cluster.size(); ++j) {
                value = std::max(
                    value,
                    (cluster[i]->position_ecef - cluster[j]->position_ecef).norm());
            }
        }
        return value;
    };

    std::vector<const Shadow*> best;
    double best_diameter = std::numeric_limits<double>::infinity();
    bool ambiguous = false;
    for (std::size_t target_size = eligible.size();
         target_size >= static_cast<std::size_t>(config_.min_independent_shadows);
         --target_size) {
        std::vector<const Shadow*> candidate;
        std::function<void(std::size_t)> visit = [&](std::size_t begin) {
            if (candidate.size() == target_size) {
                const bool fresh = config_.fresh_shadow_max_age_epochs == 0 ||
                    std::any_of(candidate.begin(), candidate.end(), [&](const Shadow* shadow) {
                        return shadow->age_epochs < config_.fresh_shadow_max_age_epochs;
                    });
                const double candidate_diameter = diameter(candidate);
                if (!fresh || candidate_diameter > config_.shadow_agreement_aperture_m) {
                    return;
                }
                if (best.empty()) {
                    best = candidate;
                    best_diameter = candidate_diameter;
                } else {
                    ambiguous = true;
                    if (candidate_diameter < best_diameter) {
                        best = candidate;
                        best_diameter = candidate_diameter;
                    }
                }
                return;
            }
            const std::size_t needed = target_size - candidate.size();
            for (std::size_t i = begin; i + needed <= eligible.size(); ++i) {
                candidate.push_back(eligible[i]);
                visit(i + 1);
                candidate.pop_back();
            }
        };
        visit(0);
        if (!best.empty()) break;
        if (target_size == static_cast<std::size_t>(config_.min_independent_shadows)) break;
    }
    if (best.empty()) {
        decision.reasons |= NO_UNIQUE_CLUSTER;
        return decision;
    }
    decision.cluster_diameter_m = best_diameter;
    if (ambiguous) {
        decision.reasons |= AMBIGUOUS_CLUSTERS;
        return decision;
    }

    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        std::vector<double> values;
        values.reserve(best.size());
        for (const auto* shadow : best) values.push_back(shadow->position_ecef[axis]);
        std::sort(values.begin(), values.end());
        const std::size_t middle = values.size() / 2;
        decision.consensus_position_ecef[axis] = values.size() % 2 == 0
            ? 0.5 * (values[middle - 1] + values[middle])
            : values[middle];
    }
    decision.primary_separation_m =
        (decision.consensus_position_ecef - input.primary_position_ecef).norm();
    if (decision.primary_separation_m < config_.primary_separation_min_m) {
        decision.reasons |= PRIMARY_TOO_CLOSE;
        return decision;
    }
    if (config_.candidate_max_prediction_error_m > 0.0) {
        if (!input.prediction_valid || !input.predicted_position_ecef.allFinite()) {
            decision.reasons |= PREDICTION_UNAVAILABLE;
            return decision;
        }
        decision.prediction_error_m =
            (decision.consensus_position_ecef - input.predicted_position_ecef).norm();
        if (decision.prediction_error_m > config_.candidate_max_prediction_error_m) {
            decision.reasons |= PREDICTION_DISAGREEMENT;
            return decision;
        }
    }
    for (const auto* shadow : best) decision.source_ids.push_back(shadow->source_id);
    std::sort(decision.source_ids.begin(), decision.source_ids.end());
    decision.replace_primary_position = true;
    return decision;
}

}  // namespace libgnss
